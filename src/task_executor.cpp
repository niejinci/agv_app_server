/**
 * @file task_executor.cpp
 * @author niejinci
 * @brief 任务执行框架，保证任务按照顺序执行，处理动作和导航任务
 *  通过 TaskParser 对象解析 gui 传入的 json 参数，记录动作和任务的顺序到执行队列中，然后启动处理下一个项目。
 *  在 plc 回调函数 plc_subscript_callback() 中，执行 等待 di 的动作，如果收到了对应的 di 信息，就清除等待 di 状态，继续处理下一个项目；
 *  在状态回调函数 state_subscript_callback() 中，处理任务或动作完成的状态，当判断任务或动作完成时，清除任务执行状态(task_executing)，继续处理下一个项目。
 *
 * @version 0.1
 * @date 2025-12-22
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "agv_app_server/task_executor.h"

#include "LogManager.hpp"
#include "agv_service/msg/order.hpp"
#include "agv_service/msg/instant_actions.hpp"
#include "agv_service/msg/digital_input.hpp"
#include "agv_service/msg/digital_output.hpp"

#include <queue>
#include <optional>
#include <variant>

namespace agv_app_server
{

int TaskExecutor::round_ = 1;

TaskExecutor::TaskExecutor(rclcpp::Node* node, rclcpp::Publisher< agv_service::msg::Order >::SharedPtr order_publisher,
        rclcpp::Publisher< agv_service::msg::InstantActions >::SharedPtr instant_publiser,
        rclcpp::Publisher<agv_service::msg::DigitalOutput>::SharedPtr plc_do_publisher)
        : node_(node)
        , order_publisher_(order_publisher)
        , instant_publisher_(instant_publiser)
        , plc_do_publisher_(plc_do_publisher)
        , head_id_(0)
        , current_instant_action_id_("")
    {
    }

void TaskExecutor::submitTask(const agv_app_msgs::msg::AppRequest::SharedPtr msg) {
    initializeFromAppRequest(msg);
}

void TaskExecutor::initializeFromAppRequest(const agv_app_msgs::msg::AppRequest::SharedPtr msg) {
    std::lock_guard<std::recursive_mutex> lock(mtx_);
    TaskParser parser;
    execution_plan_ = parser.parseTaskChain(msg);

    // 获取执行队列
    execution_queue_ = parser.getExecutionQueue();
    execution_queue_bakup_ = execution_queue_; // 备份执行队列，用于循环任务
    show_items_ = parser.getShowItems();
    LogManager::getInstance().getLogger()->info("execution_queue_.size={}, tasks.size={}, instant_actions.size={}",
        execution_queue_.size(), execution_plan_.tasks.size(), execution_plan_.instant_actions.size());

    // gui 提交新的任务时，清除之前的状态
    di_states.clear();
    current_instant_action_id_.clear();
    current_execute_order_id_.clear();
    task_executing = false;
    has_pending_wait = false;

    di_map_.clear();
    do_map_.clear();
    round_ = 1;

    // 开始执行
    processNextItem();
}

void TaskExecutor::processNextItem() {
    std::lock_guard<std::recursive_mutex> lock(mtx_);

    // 使用循环代替递归来处理非阻塞任务
    while (true) {
        // 如果有等待中的 wait_di 动作，不处理新项目
        if (has_pending_wait) {
            LogManager::getInstance().getLogger()->info("has pending task, waiting for DI condition to be met");
            return;
        }

        // 如果有任务正在执行，则退出循环等待其完成
        if (task_executing) {
            if (!current_execute_order_id_.empty()) {
                // 如果当前任务正在执行，等待完成
                LogManager::getInstance().getLogger()->info("Task[{}] is already executing, waiting for completion", current_execute_order_id_);
            }
            if (!current_instant_action_id_.empty()) {
                // 如果没有当前任务ID，说明是动作在执行
                LogManager::getInstance().getLogger()->info("Action[{}] is already executing, waiting for completion", current_instant_action_id_);
            }
            return; // 等待当前任务完成
        }

        if (execution_queue_.empty()) {
            di_map_.clear();
            do_map_.clear();
            if (execution_plan_.loop_) {
                execution_queue_ = execution_queue_bakup_; // 恢复备份的执行队列
                di_states.clear();
                LogManager::getInstance().getLogger()->info("start [{}] round task, size {}", ++round_, execution_queue_.size());
                if (execution_queue_.empty()) {
                    LogManager::getInstance().getLogger()->info("No tasks to execute in the loop.");
                    return;
                }
                // 通过命令行参数控制是否在两次任务循环之间等待: ./your_program sleep_time=3
                const char* sleep_time_env = std::getenv("sleep_time");
                if (sleep_time_env) {
                    try {
                        int sleep_seconds = std::stoi(sleep_time_env);
                        if (sleep_seconds > 0) {
                            LogManager::getInstance().getLogger()->info("Waiting for {} seconds before next loop...", sleep_seconds);
                            if (loop_delay_timer_) {
                                loop_delay_timer_->cancel();
                            }
                            loop_delay_timer_ = node_->create_wall_timer(
                                std::chrono::seconds(sleep_seconds),
                                [this]() {
                                    this->processNextItem();
                                });
                            return; // 退出等待定时器回调
                        }
                    } catch (const std::exception& e) {
                        LogManager::getInstance().getLogger()->warn("Invalid sleep_time value: {}. Not waiting.", sleep_time_env);
                    }
                }
                continue; // 继续循环以开始下一轮
            } else {
                LogManager::getInstance().getLogger()->info("All tasks and actions completed!");
                return; // 所有任务完成，退出
            }
        }

        auto current_item = execution_queue_.front();
        execution_queue_.pop();
        current_item_ = current_item; // 保存当前处理的项目

        if (current_item.first == "action") {
            // 处理动作
            if (current_item.second < execution_plan_.instant_actions.size()) {
                if (!processAction(execution_plan_.instant_actions[current_item.second])) {
                    return; // 如果动作是阻塞的，退出循环等待其完成
                }
                // 否则，继续循环处理下一个非阻塞动作
            }
        } else if (current_item.first == "task") {
            // 处理导航任务
            if (current_item.second < execution_plan_.tasks.size()) {
                executeNavigationTask(current_item.second);
                return; // 导航任务是阻塞的，退出循环等待其完成
            }
        }
    }
}

bool TaskExecutor::processAction(const agv_app_msgs::msg::ActionDetail& action) {
    if (action.name == "wait_di") {
        int index = action.index;
        // 检查DI状态
        if (di_states.count(index) && di_states[index]) {
            // DI已经为true，直接继续下一项
            LogManager::getInstance().getLogger()->info("DI {}-{} already true, continuing", index, action.desc);
            return true;
        }
        // 需要等待DI变为true
        LogManager::getInstance().getLogger()->info("Waiting for DI {}-{} to become true", index, action.desc);
        pending_wait_action = action;
        has_pending_wait = true;
        return false;
    }

    if (action.name == "set_do") {
        executeSetDo(action);
        di_states.clear(); // 清除DI状态，避免重复触发
        return true; // set_do 是非阻塞的，继续
    }

    if (action.category == "instant") {
        executeInstantAction(action);
        // 即时动作需要等待完成状态，是阻塞的
        return false;
    }
    // 对于未知或未处理的动作，默认为非阻塞
    return true;
}

void TaskExecutor::plc_subscript_callback(int32_t di_id, uint8_t di_value) {
    std::lock_guard<std::recursive_mutex> lock(mtx_);
    // 更新DI状态
    di_states[di_id] = di_value;
    LogManager::getInstance().getLogger()->info("Received DI[{}]={}", di_id, di_value);
    di_map_[di_id] = di_value; // 更新 di_map_ 状态, 显示到前端

    // 检查是否有等待该DI的动作
    if (has_pending_wait && pending_wait_action.has_value()) {
        const auto& action = pending_wait_action.value();
        if (action.name == "wait_di") {
            int index = action.index;
            if (index == di_id && di_value) {
                LogManager::getInstance().getLogger()->info("DI condition met for action {}[{}], proceeding to next item",
                    action.action_id, action.name);

                // 清除等待状态，继续下一项
                has_pending_wait = false;
                pending_wait_action.reset();
                processNextItem();
            }
        }
    } else if (di_id == 6 && di_value == 1) {
        // 暂停任务
        pauseOrResumeTask(true);
    } else if (di_id == 7 && di_value == 1) {
        // 恢复任务
        pauseOrResumeTask(false);
    }
}

bool TaskExecutor::check_instant_action_finish(const agv_app_server::StateLite& state_lite) {
    std::lock_guard<std::recursive_mutex> lock(mtx_);
    for (auto& item: state_lite.action_states) {
        if (item.action_status == "FINISHED" &&
            !current_instant_action_id_.empty() && current_instant_action_id_ == item.action_id) {
            current_instant_action_id_.clear();
            LogManager::getInstance().getLogger()->info("action finished: {}[{}]", item.action_id, item.action_type);

            task_executing = false;

            // 任务或动作完成后，处理下一项
            processNextItem();
            return true;
        }
    }
    return false;
}

bool TaskExecutor::check_order_finish(const agv_app_server::StateLite& state_lite) {
    std::lock_guard<std::recursive_mutex> lock(mtx_);
    const std::string& order_id = state_lite.order_id;
    if (!state_lite.order_id.empty() && (!state_lite.node_states.empty() || !state_lite.edegs_empty)) {
        // 记录任务为执行中
        task_id2status_[order_id] = TaskStatus::RUNNING;
        return true;
    }

    // 如果没有节点状态和边状态，说明任务已经完成了, 也有可能还未来得及设置节点和边，就发布了状态
    if (order_id.empty() && state_lite.node_states.empty() && state_lite.edegs_empty) {
        // 检查任务是否完成
        if (task_id2status_.find(order_id) != task_id2status_.end() && task_id2status_[order_id] == TaskStatus::RUNNING) {
            LogManager::getInstance().getLogger()->info("task[{}] finished", order_id);
            // 清除已完成的任务状态
            task_id2status_.erase(order_id);

            if (!current_execute_order_id_.empty() && current_execute_order_id_ == order_id) {
                current_execute_order_id_.clear();

                LogManager::getInstance().getLogger()->info("plc task finished: {}", order_id);

                task_executing = false;

                // 任务或动作完成后，处理下一项
                processNextItem();
                return true;
            }
        }
    }
    return false;
}

agv_service::msg::Order TaskExecutor::create_order(const TaskInfo& task_info)
{
    agv_service::msg::Order order;
    order.header_id = ++head_id_;
    order.timestamp = get_current_time_as_string();
    order.version = "2.0.0";            //VDA 标准版本，固定传 2.0.0
    order.manufacturer = "BYD";
    order.serial_number = "";           //唯一序列号，采用车号作为唯一序列号，应该通过前端传入，待肖工确定 robot_id, 通过 get_sysinfo api 获取
    order.data_origin = "app";
    order.order_id = TaskParser::generateUUID();    // task_info.order_id;  如果用相同的 order_id, 对于循环任务，两次任务之间的间隔太短时会导致循环任务不会执行
    order.order_update_id = 0;
    order.zone_set_id = execution_plan_.map_id_;

    std::map<std::string, agv_service::msg::Action> edge_action_buffer_;

    // 构造节点
    for (const auto& node : task_info.nodes) {
        agv_service::msg::Node order_node;
        order_node.node_id = node.node_id;
        order_node.sequence_id = node.sequence_id;
        order_node.released = true;

        order_node.node_position.x = node.x;
        order_node.node_position.y = node.y;
        order_node.node_position.theta = node.angle;
        order_node.node_position.map_id = execution_plan_.map_id_ ;

        // 添加动作
        for (const auto& action : node.bound_actions) {
            agv_service::msg::Action order_action;
            order_action.action_id = action.action_id;
            order_action.action_type = action.name;                 // 从前端传入的参数获取
            order_action.blocking_type = action.blocking_type;      // 从前端传入的参数获取  HARD or NONE

            // 添加动作参数
            // 顶升
            if (action.name == agv_app_msgs::msg::ActionDetail::PICK || action.name == agv_app_msgs::msg::ActionDetail::DROP) {
                order_action.action_description = action.name + (action.name == "pick" ? "-取货" : "-放货");
                order_action.action_parameters.push_back(agv_service::msg::ActionParameter().set__key("height").set__double_value(action.height));
            } else if (action.name == agv_app_msgs::msg::ActionDetail::START_CHARGING) {
                order_action.action_description = action.name + "-开始充电";
                order_action.action_parameters.push_back(agv_service::msg::ActionParameter().set__key("charging_time").set__int_value(action.charging_time));
            } else if (action.name == agv_app_msgs::msg::ActionDetail::SET_DO) {
                order_action.action_description = action.name + "-设置数字量输出";
                order_action.action_parameters.push_back(agv_service::msg::ActionParameter().set__key("index").set__int_value(action.index));
                order_action.action_parameters.push_back(agv_service::msg::ActionParameter().set__key("value").set__int_value(action.value));
            } else {
                order_action.action_description = action.name + "-未知动作";
            }

            if (order_action.blocking_type == "NONE") {
                // 如果是边动作，先记录下来，后面再添加到边中
                edge_action_buffer_.insert({action.bind_edge, order_action});
            } else {
                // 添加到节点动作中
                order_node.actions.push_back(order_action);
            }
        }

        // 添加避障参数
        agv_service::msg::Security security;
        security.avoid.avoid_id = node.node_id;
        security.avoid.forward = node.avoid.forward;
        security.avoid.back = node.avoid.back;
        security.avoid.left = node.avoid.left;
        security.avoid.right = node.avoid.right;
        order_node.security = security;

        order.nodes.push_back(order_node);
    }

    // 构造边
    for (const auto& edge : task_info.edges) {
        if (edge.start_point.node_id == edge.end_point.node_id) {
            LogManager::getInstance().getLogger()->warn("skip self-loop {}-{}", edge.start_point.node_id, edge.end_point.node_id);
            continue;
        }
        agv_service::msg::Edge order_edge;

        order_edge.edge_id = edge.start_point.node_id + "-" + edge.end_point.node_id;
        order_edge.sequence_id = edge.sequence_id;
        order_edge.released = true; // 边默认是 released 的
        order_edge.start_node_id = edge.start_point.node_id;
        order_edge.end_node_id = edge.end_point.node_id;
        order_edge.max_speed = edge.line_speed;
        order_edge.orientation = edge.orient;
        order_edge.orientation_type = edge.orient_type;
        order_edge.load_orientation = edge.load_orient;

        // 添加贝塞尔曲线的控制点
        if (!edge.control_points.empty()) {
            order_edge.trajectory.degree = edge.control_points.size();
            order_edge.trajectory.knot_vector = std::move(std::vector<double>{0,0,0,0,1,1,1,1});   // 4 阶节点向量
            for (const auto& point : edge.control_points) {
                agv_service::msg::Point tmp_point;
                tmp_point.x = point.x;
                tmp_point.y = point.y;
                tmp_point.weight = point.weight;
                order_edge.trajectory.control_points.push_back(tmp_point);
            }

            // 把起始节点也加入到 control_points 中
            agv_service::msg::Point start_point;
            start_point.x = edge.start_point.x;
            start_point.y = edge.start_point.y;
            start_point.weight = 1.0; // 起始点权重为1.0
            order_edge.trajectory.control_points.insert(order_edge.trajectory.control_points.begin(), start_point);

            // 把结束节点也加入到 control_points 中
            agv_service::msg::Point end_point;
            end_point.x = edge.end_point.x;
            end_point.y = edge.end_point.y;
            end_point.weight = 1.0; // 结束点权重为1.0
            order_edge.trajectory.control_points.push_back(end_point);
        }

        // 添加避障参数
        agv_service::msg::Security security;
        security.avoid.avoid_id = order_edge.edge_id;
        security.avoid.forward = edge.avoid.forward;
        security.avoid.back = edge.avoid.back;
        security.avoid.left = edge.avoid.left;
        security.avoid.right = edge.avoid.right;
        order_edge.security = security;

        // 添加边动作
        if (edge_action_buffer_.count(order_edge.edge_id + std::to_string(edge.original_index))) {
            order_edge.actions.push_back(edge_action_buffer_[order_edge.edge_id + std::to_string(edge.original_index)]);
        }

        order.edges.push_back(order_edge);
    }

    task_id2status_.insert({order.order_id, TaskStatus::PENDING});
    return order;
}

void TaskExecutor::executeNavigationTask(int task_index) {
    // 从执行计划中的任务列表中获取任务
    auto& task = execution_plan_.tasks[task_index];
    auto order = create_order(task);

    // 发布任务
    order_publisher_->publish(order);
    task_executing = true;
    current_execute_order_id_ = order.order_id;
    LogManager::getInstance().getLogger()->info("published order {}", order.order_id);
}

void TaskExecutor::executeSetDo(const agv_app_msgs::msg::ActionDetail& action) {
    agv_service::msg::DigitalOutput plc_do;
    int index = action.index;
    int value = action.value;
    plc_do.id = index;
    plc_do.value = value;
    plc_do_publisher_->publish(plc_do);
    do_map_[index] = value; // 更新 do_map_ 状态，显示到前端
    LogManager::getInstance().getLogger()->info("Set DO {}={}", index, value);
}

void TaskExecutor::executeInstantAction(const agv_app_msgs::msg::ActionDetail& action_info) {
    agv_service::msg::InstantActions instantActions;
    // 设置头部
    instantActions.data_origin = "app";
    instantActions.header_id = ++head_id_;
    instantActions.timestamp = get_current_time_as_string();
    instantActions.version = "2.0.0";
    instantActions.manufacturer = "BYD";
    instantActions.serial_number = "agv01";

    agv_service::msg::Action action;
    if (action_info.name == "pick" || action_info.name == "drop") {
        action = create_lifting_action(action_info.name, action_info.blocking_type, action_info.height);
    } else if (action_info.name == "start_charging") {
        action = create_start_charging_action(action_info.charging_time, action_info.blocking_type);
    }
    current_instant_action_id_ = std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
    action.action_id = current_instant_action_id_;
    instantActions.actions.push_back(action);
    instant_publisher_->publish(instantActions);

    task_executing = true;

    LogManager::getInstance().getLogger()->info("Executing instant action: {}[{}]", action.action_id, action_info.name);
}

/**
 * @brief 获取当前时间的字符串表示
 *
 * 此函数获取系统当前时间，并将其格式化为"年-月-日 时:分:秒"的字符串形式。
 * 时间基于本地时区设置。
 *
 * @return std::string 格式化后的当前时间字符串，格式为"YYYY-MM-DD HH:MM:SS"
 */
    std::string TaskExecutor::get_current_time_as_string()
{
    // 获取当前时间点
    auto now = std::chrono::system_clock::now();
    // 转换为time_t类型
    std::time_t now_time_t = std::chrono::system_clock::to_time_t(now);
    // 使用localtime转换为本地时间结构
    std::tm now_tm = *std::localtime(&now_time_t);

    // 使用ostringstream进行格式化
    std::ostringstream oss;
    oss << std::put_time(&now_tm, "%Y-%m-%d %H:%M:%S");
    return oss.str();
}

/**
 * @brief 创建一个 lifting 动作
 *
 * 动作结构体定义: /usr/include/agv_service/msg/detail/action__struct.hpp
*/
agv_service::msg::Action TaskExecutor::create_lifting_action(const std::string& action_type, const std::string& blocking_type, double height)
{
    agv_service::msg::Action action;
    action.action_id = std::to_string(head_id_++);
    action.action_type = action_type;
    action.action_description = (action_type == "pick" ? "取货" : "放货");
    action.blocking_type = blocking_type;

    // 添加高度参数
    agv_service::msg::ActionParameter height_param;
    height_param.key = "height";
    height_param.double_value = height; // 高度值
    action.action_parameters.push_back(height_param);
    return action;
}

agv_service::msg::Action TaskExecutor::create_start_charging_action(int charging_time, const std::string& blocking_type)
{
    agv_service::msg::Action action;
    action.action_id = std::to_string(head_id_++);
    action.action_type = "startCharging";
    action.action_description = "启动充电";
    action.blocking_type = blocking_type;

    action.action_parameters.push_back(agv_service::msg::ActionParameter().set__key("time").set__int_value(charging_time)); // 充电时间，单位为分钟

    return action;
}

void TaskExecutor::clear_status() {
    std::lock_guard<std::recursive_mutex> lock(mtx_);
    current_instant_action_id_.clear();
    current_execute_order_id_.clear();
    task_executing = false;
    has_pending_wait = false;

    while (!execution_queue_.empty()) {
        execution_queue_.pop();
    }
    while (!execution_queue_bakup_.empty()) {
        execution_queue_bakup_.pop();
    }
    di_states.clear();
    di_map_.clear();
    do_map_.clear();
    show_items_.clear();

    if (loop_delay_timer_) {
        loop_delay_timer_->cancel();
    }
    LogManager::getInstance().getLogger()->info("clear execution status");
}

std::vector<TaskParser::ShowItem> TaskExecutor::getShowItems() const {
    return show_items_;
}

// 获取当前执行的索引
// 这里的索引是基于 show_items_ 的索引
int TaskExecutor::getCurrentExecutionIndex() {
    std::pair<std::string, size_t> current_item;
    {
        std::lock_guard<std::recursive_mutex> lock(mtx_);
        current_item = current_item_;
    }
    int action_index = -1;
    int task_index = -1;
    int current_index = -1;
    for (auto& item: show_items_) {
        if (item.type == TaskParser::ShowItem::ACTION) {
            action_index++;
        } else if (item.type == TaskParser::ShowItem::NAVIGATION) {
            task_index++;
        }
        current_index++;
        if (current_item.first == "action" && (int)current_item.second == action_index) {
            break;
        } else if (current_item.first == "task" && (int)current_item.second == task_index) {
            break;
        }
    }
    return current_index;
}


void TaskExecutor::pauseOrResumeTask(bool isPauseTask) {
    agv_service::msg::InstantActions instantActions;
    // 设置头部
    instantActions.data_origin = "app";
    instantActions.header_id = ++head_id_;
    instantActions.timestamp = get_current_time_as_string();
    instantActions.version = "2.0.0";
    instantActions.manufacturer = "BYD";
    instantActions.serial_number = "agv01";

    agv_service::msg::Action action;
    action.action_id = std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
    if (isPauseTask) {
        action.action_type = "startPause";  // 暂停任务
    } else {
        action.action_type = "stopPause";   // 恢复任务
    }
    action.action_description = "Manual reset";
    action.blocking_type = "HARD";
    instantActions.actions.push_back(action);

    instant_publisher_->publish(instantActions);
}

}   // namespace agv_app_server