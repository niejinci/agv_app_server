/**
 * @file task_executor.cpp
 * @author your name (you@domain.com)
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

#pragma once

#include "agv_app_server/task_parser.h"
#include "agv_app_server/state_timer.hpp"

#include "agv_service/msg/order.hpp"
#include "agv_service/msg/instant_actions.hpp"
#include "agv_service/msg/digital_input.hpp"
#include "agv_service/msg/digital_output.hpp"
#include "rclcpp/rclcpp.hpp"

#include <queue>
#include <mutex>

namespace agv_app_server
{

class TaskExecutor {
private:
    ExecutionPlan execution_plan_;
    std::queue<std::pair<std::string, size_t>> execution_queue_; // <type, index> type可以是"action"或"task"
    std::queue<std::pair<std::string, size_t>> execution_queue_bakup_; // 备份执行队列，用于循环任务
    std::map<int, uint8_t> di_states;   // di index, value
    std::map<int32_t, uint8_t> di_map_; // key: index, value: value
    std::map<int32_t, uint8_t> do_map_; // key: index, value: value
    bool task_executing = false;
    std::optional<agv_app_msgs::msg::ActionDetail> pending_wait_action; // 当前等待的wait_di动作
    bool has_pending_wait = false;
    std::vector<TaskParser::ShowItem> show_items_;
    std::pair<std::string, size_t> current_item_;
    rclcpp::Node* node_ = nullptr;
    rclcpp::TimerBase::SharedPtr loop_delay_timer_;
    rclcpp::Publisher< agv_service::msg::Order >::SharedPtr order_publisher_;
    rclcpp::Publisher< agv_service::msg::InstantActions >::SharedPtr instant_publisher_;
    rclcpp::Publisher<agv_service::msg::DigitalOutput>::SharedPtr plc_do_publisher_;
    std::recursive_mutex mtx_;

    size_t head_id_ = 0;
    std::string current_instant_action_id_;
    std::string current_execute_order_id_;

    enum TaskStatus {
        PENDING = 0,
        RUNNING = 1,
        FINISHED = 2,
        FAILED = 3,
        CANCELLED = 4,
        PAUSED = 5,
    };
    std::map<std::string, TaskStatus> task_id2status_;
    static int round_;

public:
    TaskExecutor(rclcpp::Node* node, rclcpp::Publisher< agv_service::msg::Order >::SharedPtr order_publisher,
        rclcpp::Publisher< agv_service::msg::InstantActions >::SharedPtr instant_publiser,
        rclcpp::Publisher<agv_service::msg::DigitalOutput>::SharedPtr plc_do_publisher);

    void submitTask(const agv_app_msgs::msg::AppRequest::SharedPtr msg);
    void initializeFromAppRequest(const agv_app_msgs::msg::AppRequest::SharedPtr msg);
    void plc_subscript_callback(int32_t di_id, uint8_t di_value);
    bool check_instant_action_finish(const agv_app_server::StateLite& state_lite);
    bool check_order_finish(const agv_app_server::StateLite& state_lite);
    void clear_status();
    std::vector<TaskParser::ShowItem> getShowItems() const;
    int getCurrentExecutionIndex();
    std::map<int32_t, uint8_t> getDiMap() const { return di_map_; }
    std::map<int32_t, uint8_t> getDoMap() const { return do_map_; }

private:
    void processNextItem();
    bool processAction(const agv_app_msgs::msg::ActionDetail& action);
    agv_service::msg::Order create_order(const TaskInfo& task_info);
    void executeNavigationTask(int task_index);

    void executeSetDo(const agv_app_msgs::msg::ActionDetail& action);

    void executeInstantAction(const agv_app_msgs::msg::ActionDetail& action);

    /**
     * @brief 获取当前时间的字符串表示
     *
     * 此函数获取系统当前时间，并将其格式化为"年-月-日 时:分:秒"的字符串形式。
     * 时间基于本地时区设置。
     *
     * @return std::string 格式化后的当前时间字符串，格式为"YYYY-MM-DD HH:MM:SS"
     */
    static std::string get_current_time_as_string();

    /**
     * @brief 创建一个 lifting 动作
     *
     * 动作结构体定义: /usr/include/agv_service/msg/detail/action__struct.hpp
    */
    agv_service::msg::Action create_lifting_action(const std::string& action_type, const std::string& blocking_type, double height);

    agv_service::msg::Action create_start_charging_action(int charging_time, const std::string& blocking_type);
    void pauseOrResumeTask(bool isPauseTask);
};

}   // namespace agv_app_server