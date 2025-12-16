/**
 * @file instant_action.hpp
 * @author niejinci
 * @brief 定义即时动作处理基类及其派生类
 * @version 0.1
 * @date 2025-12-10
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once

// 项目相关头文件
#include "agv_app_msgs/msg/app_request.hpp"
#include "agv_app_msgs/msg/app_data.hpp"
#include "agv_service/msg/instant_actions.hpp"

// ROS 2 相关头文件
#include "rclcpp/rclcpp.hpp"

// c++ 标准库头文件
#include <string>
#include <memory>
#include <algorithm>
#include <chrono>

namespace agv_app_server
{

struct Result {
    bool success;
    std::string error_message;

    operator bool() const {return success;}

    static Result ok(const std::string err_msg="") {
        return {true, err_msg};
    }

    static Result fail(const std::string err_msg="") {
        return {false, err_msg};
    }
};

class BaseInstantActionHandler
{
public:
    BaseInstantActionHandler(rclcpp::Publisher<agv_service::msg::InstantActions>::SharedPtr instantPublisher = nullptr,
        rclcpp::Publisher<agv_app_msgs::msg::AppData>::SharedPtr appDataPublisher = nullptr,
        std::function<std::string()> get_mode_func = nullptr);
    ~BaseInstantActionHandler() = default;
    virtual void handle(const agv_app_msgs::msg::AppRequest::SharedPtr msg);

protected:
    // 虚方法，子类必须重写以自定义行为
    virtual Result validate_args(const agv_app_msgs::msg::AppRequest::SharedPtr msg) const { (void)msg; return Result::ok(); }
    virtual bool check_operating_mode() const {return true;}
    virtual std::string get_action_type() const = 0;
    virtual std::string get_action_description() const = 0;
    virtual void create_action_parameters(const agv_app_msgs::msg::AppRequest::SharedPtr msg, agv_service::msg::Action& action) {(void)action; (void)msg;}
    virtual void after_instant_action_sent() {}

protected:
    template<class T>
    agv_service::msg::ActionParameter set_action_parameter(const std::string& key, T&& value);
    std::string get_current_time_as_string();

private:
    // 生成即时控制消息的消息头
    void set_header(agv_service::msg::InstantActions& msg);
    int headId_;
    rclcpp::Publisher< agv_service::msg::InstantActions >::SharedPtr instantPublisher_;
    rclcpp::Publisher<agv_app_msgs::msg::AppData>::SharedPtr appDataPublisher_;

protected:
    std::function<std::string()> get_mode_func_;
};

//重定位
class RelocationHandler : public BaseInstantActionHandler
{
public:
    using BaseInstantActionHandler::BaseInstantActionHandler;

protected:
    virtual Result validate_args(const agv_app_msgs::msg::AppRequest::SharedPtr msg) const override;
    virtual bool check_operating_mode() const override;
    virtual std::string get_action_type() const override { return "initPosition";}
    virtual std::string get_action_description() const override { return "重定位";}
    virtual void create_action_parameters(const agv_app_msgs::msg::AppRequest::SharedPtr msg, agv_service::msg::Action& action)  override;
};

//平动
class TranslationHandler : public BaseInstantActionHandler
{
public:
    using BaseInstantActionHandler::BaseInstantActionHandler;

protected:
    virtual Result validate_args(const agv_app_msgs::msg::AppRequest::SharedPtr msg) const override;
    virtual bool check_operating_mode() const override;
    virtual std::string get_action_type() const override { return "translateAgv";}
    virtual std::string get_action_description() const override { return "平动";}
    virtual void create_action_parameters(const agv_app_msgs::msg::AppRequest::SharedPtr msg, agv_service::msg::Action& action)  override;
};

//旋转
class RotationHandler : public BaseInstantActionHandler
{
public:
    using BaseInstantActionHandler::BaseInstantActionHandler;

protected:
    virtual Result validate_args(const agv_app_msgs::msg::AppRequest::SharedPtr msg) const override;
    virtual bool check_operating_mode() const override;
    virtual std::string get_action_type() const override { return "rotateAgv";}
    virtual std::string get_action_description() const override { return "转动AGV";}
    virtual void create_action_parameters(const agv_app_msgs::msg::AppRequest::SharedPtr msg, agv_service::msg::Action& action)  override;
};

//托盘旋转
class PalletRotationHandler : public BaseInstantActionHandler
{
public:
    using BaseInstantActionHandler::BaseInstantActionHandler;

protected:
    virtual Result validate_args(const agv_app_msgs::msg::AppRequest::SharedPtr msg) const override;
    virtual bool check_operating_mode() const override;
    virtual std::string get_action_type() const override { return "rotateLoad";}
    virtual std::string get_action_description() const override { return "AGV托盘旋转";}
    virtual void create_action_parameters(const agv_app_msgs::msg::AppRequest::SharedPtr msg, agv_service::msg::Action& action)  override;
};

//举升
class LiftingHandler : public BaseInstantActionHandler
{
public:
    using BaseInstantActionHandler::BaseInstantActionHandler;

protected:
    virtual Result validate_args(const agv_app_msgs::msg::AppRequest::SharedPtr msg) const override;
    virtual bool check_operating_mode() const override;
    virtual std::string get_action_type() const override { return "pick";}
    virtual std::string get_action_description() const override { return "托盘顶升";}
    virtual void create_action_parameters(const agv_app_msgs::msg::AppRequest::SharedPtr msg, agv_service::msg::Action& action)  override;
};

//取消任务
class CancelTaskHandler : public BaseInstantActionHandler
{
public:
    using BaseInstantActionHandler::BaseInstantActionHandler;

protected:
    virtual bool check_operating_mode() const override;
    virtual std::string get_action_type() const override { return "cancelOrder";}
    virtual std::string get_action_description() const override { return "取消任务";}
    virtual void create_action_parameters(const agv_app_msgs::msg::AppRequest::SharedPtr msg, agv_service::msg::Action& action)  override;
    virtual void after_instant_action_sent() override;
};

//暂停任务
class PauseTaskHandler : public BaseInstantActionHandler
{
public:
    using BaseInstantActionHandler::BaseInstantActionHandler;

protected:
    virtual bool check_operating_mode() const override;
    virtual std::string get_action_type() const override { return "startPause";}
    virtual std::string get_action_description() const override { return "暂停任务";}
};

//恢复任务
class ResumeTaskHandler : public BaseInstantActionHandler
{
public:
    using BaseInstantActionHandler::BaseInstantActionHandler;

protected:
    virtual bool check_operating_mode() const override;
    virtual std::string get_action_type() const override { return "stopPause";}
    virtual std::string get_action_description() const override { return "恢复任务";}
};

// 遥控小车
class RemoteControlHandler : public BaseInstantActionHandler
{
public:
    using BaseInstantActionHandler::BaseInstantActionHandler;

protected:
    virtual bool check_operating_mode() const override;
    virtual std::string get_action_type() const override { return "cmd_vel"; }
    virtual std::string get_action_description() const override { return "遥控agv"; }
    virtual void create_action_parameters(const agv_app_msgs::msg::AppRequest::SharedPtr msg, agv_service::msg::Action& action)  override;
};

//急停命令处理程序
class EmergencyStopHandler : public BaseInstantActionHandler
{
public:
    using BaseInstantActionHandler::BaseInstantActionHandler;
protected:
    virtual std::string get_action_type() const override { return "softEstop"; }
    virtual std::string get_action_description() const override { return "软急停命令"; }
    virtual void create_action_parameters(const agv_app_msgs::msg::AppRequest::SharedPtr msg, agv_service::msg::Action& action)  override;
};


// 清除错误信息，直到底层再次上报错误
class ClearErrorsHandler : public BaseInstantActionHandler
{
public:
    using BaseInstantActionHandler::BaseInstantActionHandler;
protected:
    virtual std::string get_action_type() const override { return "clearErrors"; }
    virtual std::string get_action_description() const override { return "清除异常错误信息"; }
};

// 软复位
// 无 ActionParameter
class SoftResetHandler : public BaseInstantActionHandler
{
public:
    using BaseInstantActionHandler::BaseInstantActionHandler;

protected:
    virtual std::string get_action_type() const override { return "resetAgv";}
    virtual std::string get_action_description() const override { return "软复位";}
};


//设置操作模式
class SetOperatingModeHandler : public BaseInstantActionHandler
{
public:
    using BaseInstantActionHandler::BaseInstantActionHandler;

    virtual Result validate_args(const agv_app_msgs::msg::AppRequest::SharedPtr msg) const override;
    virtual std::string get_action_type() const override { return "controlMode";}
    virtual std::string get_action_description() const override { return "设置操作模式";}
    virtual void create_action_parameters(const agv_app_msgs::msg::AppRequest::SharedPtr msg, agv_service::msg::Action& action)  override;
};

}  // namespace agv_app_server