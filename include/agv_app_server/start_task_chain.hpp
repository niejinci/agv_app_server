#pragma once

#include "agv_app_server/command_handler.hpp"
#include "agv_app_msgs/msg/operating_mode.hpp"
#include "LogManager.hpp"
#include <functional>

namespace agv_app_server
{

class StartTaskChainHandler : public BaseCommandHandler
{
public:
    using GetModeFn = std::function<std::string()>;
    using SubmitTaskFn = std::function<void(const agv_app_msgs::msg::AppRequest::SharedPtr)>;

    StartTaskChainHandler(
        rclcpp::Publisher<agv_app_msgs::msg::AppData>::SharedPtr appDataPublisher,
        GetModeFn get_mode_fn,
        SubmitTaskFn submit_task_fn)
        : BaseCommandHandler(appDataPublisher)
        , get_mode_fn_(std::move(get_mode_fn))
        , submit_task_fn_(std::move(submit_task_fn))
    {}

    void handle(const agv_app_msgs::msg::AppRequest::SharedPtr msg) override
    {
        // 检查操作模式
        if (get_mode_fn_() != agv_app_msgs::msg::OperatingMode::MANUAL) {
            publish_response(msg->request_id, msg->command_type, false,
                           "Cannot start task chain:  AGV is not in MANUAL mode.");
            return;
        }
        LogManager::getInstance().getLogger()->info("start task chain");

        // 提交任务
        submit_task_fn_(msg);
        publish_response(msg->request_id, msg->command_type, true, "");
    }

    std::string get_command_type() const override { return "START_TASK_CHAIN"; }

private:
    GetModeFn get_mode_fn_;
    SubmitTaskFn submit_task_fn_;
};

} // namespace agv_app_server