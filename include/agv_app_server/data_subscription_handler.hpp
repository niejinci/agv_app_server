#pragma once

#include "LogManager.hpp"
#include "agv_app_server/command_handler.hpp"
#include "agv_app_server/data_stream_handler.hpp"
#include "agv_app_server/state_timer.hpp"
#include <map>
#include <memory>
#include <variant>

namespace agv_app_server
{

class DataSubscriptionHandler : public BaseCommandHandler
{
public:
    using DataStreamMap = std::map<std::string, std::shared_ptr<IDataStreamHandler>>;
    using StateTimerMap = std::map<std::string, std::shared_ptr<StateTimer>>;

    DataSubscriptionHandler(
        rclcpp::Publisher<agv_app_msgs::msg::AppData>::SharedPtr appDataPublisher,
        DataStreamMap& data_streams,
        StateTimerMap& state_timers)
        : BaseCommandHandler(appDataPublisher)
        , data_streams_(data_streams)
        , state_timers_(state_timers)
    {}

    void handle(const agv_app_msgs::msg::AppRequest::SharedPtr msg) override
    {
        const std::string& topic = msg->manage_data_subscription.topic;
        const std::string& action = msg->manage_data_subscription.action;

        bool success = false;
        std::string response_msg;

        // 尝试在数据流中查找
        auto it = data_streams_.find(topic);
        if (it != data_streams_.end()) {
            success = execute_action(it->second, action, topic, response_msg);
        } else {
            // 尝试在状态定时器中查找
            auto it2 = state_timers_.find(topic);
            if (it2 != state_timers_.end()) {
                success = execute_action(it2->second, action, topic, response_msg);
            } else {
                response_msg = "Unknown data stream: " + topic;
            }
        }

        // 成功时候的日志在策略对象里面打印
        if (!success) {
            LogManager::getInstance().getLogger()->info("false, {}", response_msg);
        }

        publish_response(msg->request_id, msg->command_type, success, response_msg);
    }

    std::string get_command_type() const override { return "MANAGE_DATA_SUBSCRIPTION"; }

private:
    template<typename T>
    bool execute_action(T& handler, const std::string& action,
                       const std::string& topic, std::string& response_msg)
    {
        if (action == "start") {
            handler->start();
            response_msg = "Started stream:  " + topic;
            return true;
        } else if (action == "stop") {
            handler->stop();
            response_msg = "Stopped stream: " + topic;
            return true;
        } else {
            response_msg = "Invalid action. Use start/stop for: " + topic;
            return false;
        }
    }

    DataStreamMap& data_streams_;
    StateTimerMap& state_timers_;
};

} // namespace agv_app_server