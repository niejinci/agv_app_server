#pragma once

#include "LogManager.hpp"
#include "agv_app_server/command_handler.hpp"
#include <functional>
#include <optional>

namespace agv_app_server
{

class SetRcsOnlineHandler : public BaseCommandHandler
{
public:

    SetRcsOnlineHandler(
        rclcpp::Publisher<agv_app_msgs::msg::AppData>::SharedPtr appDataPublisher,
        rclcpp::Publisher<agv_service::msg::MqttState>::SharedPtr mqttStatePublisher)
        : BaseCommandHandler(appDataPublisher)
        , mqtt_state_publisher_(mqttStatePublisher)
    {}

    void handle(const agv_app_msgs::msg::AppRequest::SharedPtr msg) override
    {
        // 内部发布
        agv_service::msg::MqttState mqtt_state;
        mqtt_state.online = msg->mqtt_state.online;
        mqtt_state_publisher_->publish(mqtt_state);

        LogManager::getInstance().getLogger()->info("attempt to set rcs online: {}", mqtt_state.online);

        // 响应发布
        publish_response(msg->request_id, msg->command_type, true, "RCS online state set successfully");
    }

    std::string get_command_type() const override { return "SET_RCS_ONLINE"; }

private:
    rclcpp::Publisher<agv_service::msg::MqttState>::SharedPtr mqtt_state_publisher_;
};

} // namespace agv_app_server