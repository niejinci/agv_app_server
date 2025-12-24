#pragma once

#include "LogManager.hpp"
#include "agv_app_server/command_handler.hpp"
#include <functional>
#include <optional>

namespace agv_app_server
{

class GetOperatingModeHandler : public BaseCommandHandler
{
public:
    using GetStateFn = std::function<std::string()>;

    GetOperatingModeHandler(
        rclcpp::Publisher<agv_app_msgs::msg::AppData>::SharedPtr appDataPublisher,
        GetStateFn get_operating_mode_fn)
        : BaseCommandHandler(appDataPublisher)
        , get_operating_mode_fn_(std::move(get_operating_mode_fn))
    {}

    void handle(const agv_app_msgs::msg::AppRequest::SharedPtr msg) override
    {
        agv_app_msgs::msg::AppData response;
        response.source_type = "cmd_response";
        response.request_id = msg->request_id;
        response.command_type = msg->command_type;
        response.success = true;
        response. message = "Operating mode retrieved successfully";

        response.operating_mode.mode = get_operating_mode_fn_();

        LogManager::getInstance().getLogger()->info("get operating mode: {}", response.operating_mode.mode);

        appDataPublisher_->publish(response);
    }

    std::string get_command_type() const override { return "GET_OPERATING_MODE"; }

private:
    GetStateFn get_operating_mode_fn_;
};

} // namespace agv_app_server