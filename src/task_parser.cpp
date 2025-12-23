#include "agv_app_server/task_parser.h"
#include "agv_app_msgs/msg/app_request.hpp"

#include <random>
#include <sstream>
#include <queue>
#include <chrono>
#include <iostream>

namespace agv_app_server
{

std::string TaskParser::generateUUID() {
    std::random_device rd;
    std::mt19937 gen(rd() ^ std::chrono::system_clock::now().time_since_epoch().count());
    std::uniform_int_distribution<int> dis(0, 15);
    std::uniform_int_distribution<int> dis2(8, 11);

    std::stringstream ss;
    int i;
    ss << std::hex;
    for (i = 0; i < 8; i++) {
        ss << dis(gen);
    }
    ss << "-";
    for (i = 0; i < 4; i++) {
        ss << dis(gen);
    }
    ss << "-4"; // version 4
    for (i = 0; i < 3; i++) {
        ss << dis(gen);
    }
    ss << "-";
    ss << dis2(gen); // variant bits
    for (i = 0; i < 3; i++) {
        ss << dis(gen);
    }
    ss << "-";
    for (i = 0; i < 12; i++) {
        ss << dis(gen);
    }

return ss.str();
}

std::string TaskParser::generateActionId() {
    return std::to_string(std::chrono::system_clock::now().time_since_epoch().count());
}


ExecutionPlan TaskParser::parseTaskChain(const agv_app_msgs::msg::AppRequest::SharedPtr msg) {
    execution_plan_ = ExecutionPlan{};
    current_sequence_id_ = 0;

    double max_speed = msg->task_chain.max_speed;
    if (max_speed <= 0) {
        max_speed = 0.1;
    } else if (max_speed > 2) {
        max_speed = 2;
    }

    std::string map_id = msg->task_chain.map_id;
    bool loop = msg->task_chain.loop;

    if (msg->task_chain.tasks.empty()) {
        return execution_plan_;
    }
    execution_plan_.max_speed_ = max_speed;
    execution_plan_.map_id_ = map_id;
    execution_plan_.loop_ = loop;

    // 首先预处理所有任务，找出所有的导航任务和动作
    std::vector<ParsedItem> parsed_items;
    parseAllItems(msg->task_chain.tasks, parsed_items);

    // 处理绑定逻辑
    processBindingLogic(parsed_items);

    // 生成最终的执行计划
    generateExecutionPlan(parsed_items);

    // 生成显示用的项目列表
    generateShowItems();

    return execution_plan_;
}

void TaskParser::parseAllItems(const std::vector<agv_app_msgs::msg::Task>& tasks, std::vector<ParsedItem>& parsed_items) {
    for (size_t i = 0; i < tasks.size(); ++i) {
        auto task = tasks[i];
        ParsedItem item;
        item.original_index = i;

        std::string type = task.type;
        if (type == agv_app_msgs::msg::Task::PATH_NAVIGATION) {
            // 导航任务
            item.type = ParsedItem::NAVIGATION;
            parseNavigationEdges(task, item.edges, i);
        } else if (type == agv_app_msgs::msg::Task::ACTION) {
            // 动作
            item.type = ParsedItem::ACTION;
            item.action_info = parseAction(task);
        }

        parsed_items.push_back(item);
    }
}

void TaskParser::parseNavigationEdges(const agv_app_msgs::msg::Task& nav_task, std::vector<agv_app_msgs::msg::EdgeDetail>& edges, int original_index) {
    for (const auto& edge : nav_task.edges) {
        agv_app_msgs::msg::EdgeDetail edge_detail;

        // 设置开始接节点
        auto sp = edge.start_point;
        edge_detail.start_point.set__angle(sp.angle)
                                .set__name(sp.name)
                                .set__x(sp.x)
                                .set__y(sp.y)
                                .set__avoid(sp.avoid);

        // 设置结束节点
        auto ep = edge.end_point;
        edge_detail.end_point.set__angle(ep.angle)
                                .set__name(ep.name)
                                .set__x(ep.x)
                                .set__y(ep.y)
                                .set__avoid(ep.avoid);

        // 设置边属性
        edge_detail.line_speed = edge.line_speed;
        edge_detail.load_orient = edge.load_orient;
        edge_detail.orient = edge.orient;
        edge_detail.orient_type = edge.orient_type;

        edge_detail.set__control_points(edge.control_points);
        edge_detail.set__avoid(edge.avoid);

        edge_detail.original_index = original_index;
        edges.push_back(edge_detail);
        std::cout << "push edge " << edge_detail.start_point.name << "-" << edge_detail.end_point.name << "\n";
    }
}

agv_app_msgs::msg::ActionDetail TaskParser::parseAction(const agv_app_msgs::msg::Task& task) {
    agv_app_msgs::msg::ActionDetail action_detail;
    action_detail.action_id = generateActionId();
    action_detail.name = task.action.name;
    action_detail.category = task.action.category;

    // plc di/do
    action_detail.index = task.action.index;
    action_detail.value= task.action.value;
    // 顶升
    action_detail.height = task.action.height;
    // 充电
    action_detail.charging_time= task.action.charging_time;
    // 站点二维码纠正动作用
    action_detail.type = task.action.type;

    // blocking_type = HARD 表示动作限制在 NODE 上执行;
    // blocking_type = NONE 表示动作现在 EDGE 上执行;
    action_detail.blocking_type = task.action.blocking_type;

    return action_detail;
}

void TaskParser::processBindingLogic(std::vector<ParsedItem>& parsed_items) {
    for (size_t i = 0; i < parsed_items.size(); ++i) {
        if (parsed_items[i].type == ParsedItem::ACTION &&
            parsed_items[i].action_info.category == "binding") {

            std::string bind_node_name = findBindingNode(parsed_items, i);
            parsed_items[i].action_info.bind_node_name = bind_node_name;
            std::cout << "bind_node_name=" << bind_node_name << "\n";
        }
    }
}

/**
 * @brief 找到绑定节点的名称
 * 
 * @param parsed_items 解析后的任务链，其元素要么是一个导航任务，要么是一个动作
 * @param action_index 动作在解析后的任务链中的索引
 * @return std::string 绑定到的节点名称
 */
std::string TaskParser::findBindingNode(std::vector<ParsedItem>& parsed_items, size_t action_index) {
    // 规则1：动作如果需要绑定站点，默认绑定前面边的终点
    // 从当前位置往前找最近的导航任务
    for (int i = action_index - 1; i >= 0; --i) {
        if (parsed_items[i].type == ParsedItem::NAVIGATION && !parsed_items[i].edges.empty()) {
            parsed_items[i].edges.back().end_point_bind_action.push_back(action_index);    // 记录节点要绑定的动作索引
            std::cout << "bind action " << action_index << " to edge end point " << parsed_items[i].edges.back().end_point.name << "\n";
            // 找到前面的导航任务，绑定到最后一条边的终点
            return parsed_items[i].edges.back().end_point.name;
        }

        // 如果中间有非 binding 的动作，停止向前查找
        if (parsed_items[i].type == ParsedItem::ACTION &&
            parsed_items[i].action_info.category != "binding") {
            break;
        }
    }

    // 规则2：动作如果需要绑定站点，并且前面没有站点的话就帮绑定后面边的起点
    // 从当前位置往后找最近的导航任务
    for (size_t i = action_index + 1; i < parsed_items.size(); ++i) {
        if (parsed_items[i].type == ParsedItem::NAVIGATION && !parsed_items[i].edges.empty()) {
            parsed_items[i].edges[0].start_point_bind_action.push_back(action_index);   // 记录节点要绑定的动作的索引
            std::cout << "bind action " << action_index << " to edge start point " << parsed_items[i].edges[0].start_point.name << "\n";
            // 找到后面的导航任务，绑定到第一条边的起点
            return parsed_items[i].edges.front().start_point.name;
        }

        // 如果中间有非binding的动作，停止向后查找
        if (parsed_items[i].type == ParsedItem::ACTION &&
            parsed_items[i].action_info.category != "binding") {
            break;
        }
    }

    return "";
}

void TaskParser::generateExecutionPlan(std::vector<ParsedItem>& parsed_items) {
    // 按照连续的导航任务分组
    std::vector<std::vector<size_t>> navigation_groups;
    std::vector<size_t> current_group;

    int task_index = 0;
    int action_index = 0;

    for (size_t i = 0; i < parsed_items.size(); ++i) {
        if (parsed_items[i].type == ParsedItem::NAVIGATION) {
            current_group.push_back(i); // 记录连续路径导航元素的索引
        } else {
            if (parsed_items[i].action_info.category != "binding") {    // 默认是即时动作
                // instant动作，结束当前组
                if (!current_group.empty()) {
                    navigation_groups.push_back(current_group);
                    current_group.clear();
                    execution_queue_.push({"task", task_index++});  // 记录任务的索引
                }

                // 处理instant动作
                processInstantAction(parsed_items[i].action_info);
                execution_queue_.push({"action", action_index++});  // 记录动作的索引
            }
        }
    }

    // 处理最后一组
    if (!current_group.empty()) {
        navigation_groups.push_back(current_group);
        execution_queue_.push({"task", task_index++});  // 记录任务的索引
    }

    // 为边的节点绑定动作
    for (size_t i = 0; i < parsed_items.size(); ++i) {
        if (parsed_items[i].type == ParsedItem::NAVIGATION) {
            // 要处理多条边
            for (auto& edge : parsed_items[i].edges) {
                // 绑定起始节点的动作
                for (size_t action_index : edge.start_point_bind_action) {
                    if (action_index < parsed_items.size() &&
                        parsed_items[action_index].type == ParsedItem::ACTION) {
                            // 记录边动作绑定的边名称
                            if (parsed_items[action_index].action_info.blocking_type == "NONE") {
                                parsed_items[action_index].action_info.bind_edge = edge.start_point.node_id + "-" + edge.end_point.node_id + std::to_string(edge.original_index);
                            }
                            edge.start_point.bound_actions.push_back(
                                parsed_items[action_index].action_info);
                    }
                }

                // 绑定结束节点的动作
                for (size_t action_index : edge.end_point_bind_action) {
                    if (action_index < parsed_items.size() &&
                        parsed_items[action_index].type == ParsedItem::ACTION) {
                        // 记录边动作绑定的边名称
                        if (parsed_items[action_index].action_info.blocking_type == "NONE") {
                            parsed_items[action_index].action_info.bind_edge = edge.start_point.node_id + "-" + edge.end_point.node_id + std::to_string(edge.original_index);
                        }
                        edge.end_point.bound_actions.push_back(
                            parsed_items[action_index].action_info);
                            std::cout << "bind " << parsed_items[action_index].action_info.name << " to " << edge.end_point.node_id << "\n";
                    }
                }
            }
        }
    }

    // 为每个导航组生成任务
    for (const auto& group : navigation_groups) {
        generateTaskFromGroup(parsed_items, group);
    }

}

void TaskParser::generateTaskFromGroup(const std::vector<ParsedItem>& parsed_items,
                            const std::vector<size_t>& group_indices) {
    TaskInfo task;
    task.order_id = generateUUID();

    std::string pre_node_name;
    std::vector<agv_app_msgs::msg::NodeDetail> nodes;

    // 节点从 0 开始递增，步长为 2
    int node_sequence_id = 0;

    // 边从 1 开始递增，步长为 2
    int edge_sequence_id = 1;

    // 收集所有导航边
    for (size_t idx : group_indices) {
        if (parsed_items[idx].type == ParsedItem::NAVIGATION) {
            for (const auto& edge : parsed_items[idx].edges) {
                task.edges.push_back(edge);
                task.edges.back().sequence_id = edge_sequence_id;

                edge_sequence_id += 2; // 每条边的序列号递增2

                if (pre_node_name != edge.start_point.name) {
                    // 添加起始节点
                    nodes.emplace_back(edge.start_point);
                    nodes.back().sequence_id = node_sequence_id;
                    nodes.back().node_id = edge.start_point.name;
                    node_sequence_id += 2;
                    pre_node_name = edge.start_point.name;
                }
                // 对于 LM0-LM0 这种自环边，不重复添加节点
                if (pre_node_name != edge.end_point.name) {
                    // 添加结束节点
                    nodes.emplace_back(edge.end_point);
                    nodes.back().sequence_id = node_sequence_id;
                    nodes.back().node_id = edge.end_point.name;
                    node_sequence_id += 2;
                    pre_node_name = edge.end_point.name;
                } else {
                    // 把自环边结束节点的绑定动作添加到开始节点上
                    for (const auto& action : edge.end_point.bound_actions) {
                        nodes.back().bound_actions.push_back(action);
                    }
                }
            }
        }
    }

    // 收集节点
    task.nodes = std::move(nodes);

    execution_plan_.tasks.push_back(task);
}

void TaskParser::processInstantAction(const agv_app_msgs::msg::ActionDetail& action) {
    // 保存即时动作到数组
    execution_plan_.instant_actions.push_back(action);
}

void TaskParser::generateShowItems() {
    // 根据 execution_plan_ 和 execution_queue_ 生成可视化的显示项
    auto execution_queue_tmp = execution_queue_;
    while (!execution_queue_tmp.empty()) {
        auto qitem = execution_queue_tmp.front();
        execution_queue_tmp.pop();

        ShowItem item;
        item.type = (qitem.first == "task") ? ShowItem::NAVIGATION : ShowItem::ACTION;

        if (item.type == ShowItem::NAVIGATION) {
            auto & task = execution_plan_.tasks[qitem.second];
            for (const auto & node : task.nodes) {
                item.task += node.name + " -> ";
            }
            item.name = task.order_id;
        } else {
            auto & action_info = execution_plan_.instant_actions[qitem.second];
            item.action = action_info.name;
            if(action_info.index) {
                item.action += action_info.index;
            }
            item.name = action_info.name + "-" + action_info.action_id;
        }

        show_items_.push_back(item);
    }
}

std::vector<TaskParser::ShowItem> TaskParser::getShowItems() const {
    return show_items_;
}

} // namespace agv_app_server

#ifdef TEST

#include <iostream>
#include <fstream>
#include <unordered_map>
#include "ini_parser.h"
#include <string>

std::unordered_map<std::string, uint16_t> load_file(const std::string& file_path) {
	INIParser ini(file_path);
    std::unordered_map<std::string, uint16_t> requestname2cmd_;

    auto section = ini.getSection("config");
    for (const auto& [key, value] : section) {
        uint16_t cmd = std::stoi(value, nullptr, 16);
        requestname2cmd_.insert({key, cmd});
    }
    // std::cout << "requestname2cmd_.size=" << requestname2cmd_.size() << "\n";

    return requestname2cmd_;
}


int main()
{
    using namespace agv_app_server;

    // 读取并解析JSON文件
    // std::ifstream file("/mnt/d/byd_agv_in_gitee/agv_server_pubsub/src/task-20250627.json");
    std::ifstream file("/mnt/d/byd_agv_in_gitee/agv_server_pubsub/src/test-robot.json");
    json json_data;
    file >> json_data;

    std::unordered_map<std::string, uint16_t> requestname2cmd_ = load_file("/mnt/d/byd_agv_in_gitee/agv_server_pubsub/config/requestname2cmd.ini");

    TaskParser tp(requestname2cmd_);
    ExecutionPlan ep = tp.parseTaskChain(json_data);

    for (auto& task : ep.tasks) {
        std::cout << "Task Order ID: " << task.order_id << std::endl;
        for (auto& node : task.nodes) {
            std::cout << "  Node: (" << node.point.x << ", " << node.point.y
                      << ", " << node.point.a << ") Name: " << node.name
                      << ", sequence_id: " << node.sequence_id << std::endl;

            std::cout << "    Bound Actions: " << std::endl;
            for (auto& action : node.bound_actions) {
                std::cout << "      Action: " << action.action_type
                          << ", Command: " << action.cmd
                          << ", Category: " << action.category
                          << ", Action ID: " << action.action_id
                          << ", Sequence ID: " << action.sequence_id
                          << ", Bind Node ID: " << action.bind_node_name
                          << std::endl;
            }
        }
        for (auto& edge : task.edges) {
            std::cout << "  Edge from " << edge.start_point.name
                      << " to " << edge.end_point.name
                      << " with speed " << edge.line_speed
                      << ", sequence_id: " << edge.sequence_id << std::endl;
        }
    }

    // 输出即时动作
    std::cout << "Instant Actions:" << std::endl;
    for (const auto& action : ep.instant_actions) {
        std::cout << "  Action: " << action.action_type
                  << ", Command: " << action.cmd
                  << ", Category: " << action.category
                  << ", Action ID: " << action.action_id
                  << ", Sequence ID: " << action.sequence_id
                  << ", Bind Node ID: " << action.bind_node_name
                  << std::endl;
    }

    // 输出即时动作和任务的顺序
    while (!tp.execution_queue_.empty()) {
        auto item = tp.execution_queue_.front();
        tp.execution_queue_.pop();
        if (item.first == "task") {
            std::cout << "Execution Item: Type = " << item.first
                      << ", Index = " << item.second
                      << " (Task Order ID: " << ep.tasks[item.second].order_id << ")"
                      << std::endl;
        }
        else if (item.first == "action") {
            std::cout << "Execution Item: Type = " << item.first
                      << ", Index = " << item.second
                      << " (Action ID: " << ep.instant_actions[item.second].action_id << ")"
                      << std::endl;
        }
        else
        std::cout << "Execution Item: Type = " << item.first
                  << ", Index = " << item.second << std::endl;
    }

}


/*
g++ task_parser.cpp -I/mnt/d/robotics/3rd/json-3.11.3/json-3.11.3/include/ -DTEST
*/

#endif // TEST