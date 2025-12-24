#pragma once

#include "agv_app_msgs/msg/app_request.hpp"
#include "agv_app_msgs/msg/app_data.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>

namespace agv_app_server
{

/**
 * @brief 命令处理器基类 - 所有命令类型的统一接口
 */
class CommandHandler
{
public:
    virtual ~CommandHandler() = default;

    /**
     * @brief 处理命令请求
     * @param msg 请求消息
     */
    virtual void handle(const agv_app_msgs::msg::AppRequest::SharedPtr msg) = 0;

    /**
     * @brief 获取命令类型标识
     */
    virtual std::string get_command_type() const = 0;
};

using CommandHandlerPtr = std::shared_ptr<CommandHandler>;

/**
 * @brief 命令处理器基类 - 提供通用功能
 */
class BaseCommandHandler :  public CommandHandler
{
public:
    BaseCommandHandler(rclcpp::Publisher<agv_app_msgs::msg::AppData>::SharedPtr appDataPublisher)
        : appDataPublisher_(appDataPublisher) {}

protected:
    void publish_response(const std::string& request_id,
                         const std::string& cmd_type,
                         bool success,
                         const std::string& message)
    {
        agv_app_msgs::msg::AppData response;
        response.source_type = "cmd_response";
        response.request_id = request_id;
        response.command_type = cmd_type;
        response.success = success;
        response.message = message;
        appDataPublisher_->publish(response);
    }

    rclcpp::Publisher<agv_app_msgs::msg::AppData>::SharedPtr appDataPublisher_;
};

} // namespace agv_app_server