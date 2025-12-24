#ifndef AGV_APP_SERVER__STATE_TIMER_HPP_
#define AGV_APP_SERVER__STATE_TIMER_HPP_

#include <string>
#include <memory>
#include <functional>

#include "agv_service/msg/agv_position.hpp"
#include "agv_service/msg/node_state.hpp"
#include "agv_service/msg/action_state.hpp"
#include "agv_service/msg/error.hpp"

#include "rclcpp/rclcpp.hpp"
#include "LogManager.hpp"

namespace agv_app_server
{

struct StateLite
{
    bool is_task_running = false;
    bool edegs_empty = false;
    std::string order_id;
    std::string last_node_id;
    int64_t last_node_sequence_id;
    agv_service::msg::AgvPosition agv_position;
    std::vector<agv_service::msg::NodeState> node_states;
    std::vector<agv_service::msg::ActionState> action_states;
    std::vector<agv_service::msg::Error> errors;
    std::string operating_mode;
    std::string e_stop;
};

/**
 * @brief 状态定时器类
 * 用于定时发布状态相关信息
 */
class StateTimer
{
public:
    StateTimer(
        rclcpp::Node* node,
        const std::string& topic_name,
        const std::chrono::milliseconds& interval,
        std::function<void()> callback,
        const rclcpp::CallbackGroup::SharedPtr callback_group = nullptr)
    : node_(node)
        , topic_name_(topic_name)
        , callback_(callback)
        , interval_(interval)
        , is_active_(false)
        , callback_group_(callback_group)
    {
    }

    void start()
    {
        LogManager::getInstance().getLogger()->info("attempt to start stream [{}] current state: {}", topic_name_, is_active_);
        if (is_active_) return;

        LogManager::getInstance().getLogger()->info("Starting state timer: " + topic_name_);

        // 创建定时器
        state_relate_timer_ = node_->create_wall_timer(interval_, callback_, callback_group_);
        is_active_ = true;
    }

    void stop()
    {
        LogManager::getInstance().getLogger()->info("attempt to stop stream [{}] current state: {}", topic_name_, is_active_);
        if (!is_active_) return;

        LogManager::getInstance().getLogger()->info("Stopping state timer: " + topic_name_);
        state_relate_timer_.reset(); // 销毁定时器
        is_active_ = false;
    }

    bool is_active() const { return is_active_; }
    std::string get_topic_name() const { return topic_name_; }

private:
    rclcpp::Node* node_;
    std::string topic_name_;
    rclcpp::TimerBase::SharedPtr state_relate_timer_;
    std::function<void()> callback_;
    std::chrono::milliseconds interval_;
    bool is_active_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
};

} // namespace agv_app_server

#endif // AGV_APP_SERVER__STATE_TIMER_HPP_