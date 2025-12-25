/**
 * @file instant_action.cpp
 * @author niejinci
 * @brief 定义即时动作处理基类及其派生类的实现
 * @version 0.1
 * @date 2025-12-10
 *
 * @copyright Copyright (c) 2025
 *
 */

// 项目相关头文件
#include "LogManager.hpp"
#include "agv_app_msgs/msg/translation.hpp"
#include "agv_app_msgs/msg/pallet_rotation.hpp"
#include "agv_app_msgs/msg/lifting.hpp"
#include "agv_app_server/instant_action.hpp"
#include "agv_app_msgs/msg/operating_mode.hpp"

namespace agv_app_server
{

BaseInstantActionHandler::BaseInstantActionHandler(
    rclcpp::Publisher<agv_service::msg::InstantActions>::SharedPtr instantPublisher,
    rclcpp::Publisher<agv_app_msgs::msg::AppData>::SharedPtr appDataPublisher,
    std::function<std::string()> get_mode_func,
    std::function<void()> after_instant_action_sent_func)
        : headId_(0)
        , instantPublisher_(instantPublisher)
        , appDataPublisher_(appDataPublisher)
        , get_mode_func_(get_mode_func)
        , after_instant_action_sent_func_(after_instant_action_sent_func)
        {}

std::string BaseInstantActionHandler::get_current_time_as_string()
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
 * @brief 设置ActionParameter对象
 *
 * @tparam T 参数值的类型，支持int、double和std::string
 * @param key 后端ActionParameter中使用的键名
 * @param value 键对应的值
 * @return agv_service::msg::ActionParameter 封装了键值对的ActionParameter对象
 */
template<class T>
agv_service::msg::ActionParameter BaseInstantActionHandler::set_action_parameter(const std::string& key, T&& value)
{
    agv_service::msg::ActionParameter action_parameter;
    action_parameter.key = key;

    // 替代方案：统一处理所有整数/浮点类型
    // DecayedT = std::decay_t<T> 用于去除引用和cv限定符
    if constexpr (std::is_integral_v<std::decay_t<T>>) {
        action_parameter.int_value = value;
    } else if constexpr (std::is_floating_point_v<std::decay_t<T>>) {
        action_parameter.double_value = value;
    } else if constexpr (std::is_same_v<std::decay_t<T>, std::string>) {
        action_parameter.string_value = value;
    } else if constexpr (std::is_same_v<std::decay_t<T>, const char*> || std::is_same_v<std::decay_t<T>, char*>) {
        action_parameter.string_value = std::string(value);
    } else {
        // 如果需要处理其他类型，可以在这里添加更多分支
        LogManager::getInstance().getLogger()->warn("Unsupported ActionParameter type for key: {}", key);
    }

    return action_parameter;
}


void BaseInstantActionHandler::set_header(agv_service::msg::InstantActions& instantActions)
{
    instantActions.data_origin = "app";
    instantActions.header_id = ++headId_;
    instantActions.timestamp = get_current_time_as_string();
    instantActions.version = "2.0.0";
    instantActions.manufacturer = "BYD";
    instantActions.serial_number = "agv01";
}

void BaseInstantActionHandler::handle(const agv_app_msgs::msg::AppRequest::SharedPtr msg)
{
    agv_app_msgs::msg::AppData response;
    response.source_type = "cmd_response";
    response.request_id = msg->request_id;
    response.command_type = msg->command_type;

    // 检查操作模式是否允许
    if (!check_operating_mode()) {
        response.success = false;
        response.message = "operating mode is not valid ";
        appDataPublisher_->publish(response);
        return;
    }

    // 请求参数校验
    auto result = validate_args(msg);
    if (!result) {
        response.success = false;
        response.message = result.error_message;
        appDataPublisher_->publish(response);
        return;
    }

    // 创建即时动作
    agv_service::msg::InstantActions instantActions;
    set_header(instantActions);

    agv_service::msg::Action action;
    action.action_id = std::to_string(headId_++);
    action.blocking_type = "HARD";
    action.action_type = get_action_type();
    action.action_description = get_action_description();

    // 创建即时动作参数
    create_action_parameters(msg, action);
    instantActions.actions.push_back(action);

    // 发布即时动作
    instantPublisher_->publish(instantActions);

    // 发布即时动作后需要的处理
    after_instant_action_sent();

    // 发布响应数据
    response.success = true;
    appDataPublisher_->publish(response);

    LogManager::getInstance().getLogger()->info("Send instant action [{}] of type [{}]", msg->command_type, action.action_type);
}

//重定位
Result RelocationHandler::validate_args(const agv_app_msgs::msg::AppRequest::SharedPtr msg) const
{
    if (msg->relocation.theta < -M_PI || msg->relocation.theta > M_PI) {
        return Result::fail("theta must be in range [-π, π]");
    }
    return Result::ok();
}

bool RelocationHandler::check_operating_mode() const
{
    // 抢占模式下才能发送重定位操作
    return (get_mode_func_() == agv_app_msgs::msg::OperatingMode::MANUAL);
}

void RelocationHandler::create_action_parameters(const agv_app_msgs::msg::AppRequest::SharedPtr msg, agv_service::msg::Action& action)
{
    action.action_parameters.push_back(set_action_parameter( "x", msg->relocation.x));
    action.action_parameters.push_back(set_action_parameter("y", msg->relocation.y));
    action.action_parameters.push_back(set_action_parameter("theta", msg->relocation.theta));
    action.action_parameters.push_back(set_action_parameter("mapId", msg->relocation.map_id));
    action.action_parameters.push_back(set_action_parameter("lastNodeId", msg->relocation.last_nodeid));
}

//平动
Result TranslationHandler::validate_args(const agv_app_msgs::msg::AppRequest::SharedPtr msg) const
{
    if (msg->translation.mode != agv_app_msgs::msg::Translation::MODE_LOCALIZATION && msg->translation.mode != agv_app_msgs::msg::Translation::MODE_ODOM) {
        return Result::fail("invalid mode, must be 0 or 1");
    }
    return Result::ok();
}

bool TranslationHandler::check_operating_mode() const
{
    // 抢占模式下才能发送平动操作
    return (get_mode_func_() == agv_app_msgs::msg::OperatingMode::MANUAL);
}
void TranslationHandler::create_action_parameters(const agv_app_msgs::msg::AppRequest::SharedPtr msg, agv_service::msg::Action& action)
{
    action.action_parameters.push_back(set_action_parameter( "vx", msg->translation.vx));
    action.action_parameters.push_back(set_action_parameter("vy", msg->translation.vy));
    action.action_parameters.push_back(set_action_parameter("dist", msg->translation.dist));
    action.action_parameters.push_back(set_action_parameter("mode", msg->translation.mode));
}

//旋转
Result RotationHandler::validate_args(const agv_app_msgs::msg::AppRequest::SharedPtr msg) const
{
    if (msg->rotation.angle < -M_PI || msg->rotation.angle > M_PI) {
        return Result::fail("angle must be in range [-π, π]");
    }
    return Result::ok();
}

bool RotationHandler::check_operating_mode() const
{
    // 抢占模式下才能发送平动操作
    return (get_mode_func_() == agv_app_msgs::msg::OperatingMode::MANUAL);
}

void RotationHandler::create_action_parameters(const agv_app_msgs::msg::AppRequest::SharedPtr msg, agv_service::msg::Action& action)
{
    action.action_parameters.push_back(set_action_parameter( "vw", msg->rotation.vw));
    action.action_parameters.push_back(set_action_parameter("angle", msg->rotation.angle));
}

//托盘旋转
Result PalletRotationHandler::validate_args(const agv_app_msgs::msg::AppRequest::SharedPtr msg) const
{
    if (msg->pallet_rotation.angle < -M_PI || msg->pallet_rotation.angle > M_PI) {
        return Result::fail("angle must be in range [-π, π]");
    }
    if (msg->pallet_rotation.mode != agv_app_msgs::msg::PalletRotation::MODE_INCREMENTAL &&
        msg->pallet_rotation.mode != agv_app_msgs::msg::PalletRotation::MODE_ABSOLUTE) {
        return Result::fail("invalid mode, must be 0 or 1");
    }
    return Result::ok();
}

bool PalletRotationHandler::check_operating_mode() const
{
    // 示教/手动模式才能操作
    return (get_mode_func_() == agv_app_msgs::msg::OperatingMode::TEACHIN || get_mode_func_() == agv_app_msgs::msg::OperatingMode::MANUAL);
}

void PalletRotationHandler::create_action_parameters(const agv_app_msgs::msg::AppRequest::SharedPtr msg, agv_service::msg::Action& action)
{
    action.action_parameters.push_back(set_action_parameter( "mode", msg->pallet_rotation.mode));
    action.action_parameters.push_back(set_action_parameter("angle", msg->pallet_rotation.angle));
}

//举升
Result LiftingHandler::validate_args(const agv_app_msgs::msg::AppRequest::SharedPtr msg) const
{
    if (msg->lifting.action != agv_app_msgs::msg::Lifting::PICK && msg->lifting.action != agv_app_msgs::msg::Lifting::DROP) {
        return Result::fail("invalid action, must be `pick` or `drop`");
    }
    return Result::ok();
}

bool LiftingHandler::check_operating_mode() const
{
    // 示教/手动模式才能操作
    return (get_mode_func_() == agv_app_msgs::msg::OperatingMode::TEACHIN || get_mode_func_() == agv_app_msgs::msg::OperatingMode::MANUAL);
}

void LiftingHandler::create_action_parameters(const agv_app_msgs::msg::AppRequest::SharedPtr msg, agv_service::msg::Action& action)
{
    action.action_type = msg->lifting.action;
    action.action_description = (action.action_type == "pick" ? "取货" : "放货");
    action.action_parameters.push_back(set_action_parameter("height", msg->lifting.height));
}

//取消任务
bool CancelTaskHandler::check_operating_mode() const
{
    // 抢占了模式/示教模式才能操作
    return (get_mode_func_() == agv_app_msgs::msg::OperatingMode::MANUAL || get_mode_func_() == agv_app_msgs::msg::OperatingMode::TEACHIN);
}

void CancelTaskHandler::create_action_parameters(const agv_app_msgs::msg::AppRequest::SharedPtr msg, agv_service::msg::Action& action)
{
    // 默认停到下一个节点
    if (msg->cancel_task.stop_right) {
        // 当前位置立马停车
        action.action_parameters.push_back(set_action_parameter("stopRight", 1));
    }
}

void CancelTaskHandler::after_instant_action_sent()
{
    // 取消任务
    if (after_instant_action_sent_func_) {
        after_instant_action_sent_func_();
    }
}

//暂停任务
bool PauseTaskHandler::check_operating_mode() const
{
    // 抢占了模式才能操作
    return (get_mode_func_() == agv_app_msgs::msg::OperatingMode::MANUAL);
}

//恢复任务
bool ResumeTaskHandler::check_operating_mode() const
{
    // 抢占了模式才能操作
    return (get_mode_func_() == agv_app_msgs::msg::OperatingMode::MANUAL);
}

// 遥控小车
bool RemoteControlHandler::check_operating_mode() const
{
    // 示教模式才能操作
    return (get_mode_func_() == agv_app_msgs::msg::OperatingMode::TEACHIN);
}

void RemoteControlHandler::create_action_parameters(const agv_app_msgs::msg::AppRequest::SharedPtr msg, agv_service::msg::Action& action)
{
    action.action_parameters.push_back(set_action_parameter("linear_x", msg->remote_control.linear_x));
    action.action_parameters.push_back(set_action_parameter("angular_z", msg->remote_control.angular_z));
    action.action_parameters.push_back(set_action_parameter("linear_y", msg->remote_control.linear_y));
}

// 急停命令处理程序
void EmergencyStopHandler::create_action_parameters(const agv_app_msgs::msg::AppRequest::SharedPtr msg, agv_service::msg::Action& action)
{
    action.action_parameters.push_back(set_action_parameter("status", msg->emergency_stop.status ? "true" : "false"));
}

// 设置操作模式
Result SetOperatingModeHandler::validate_args(const agv_app_msgs::msg::AppRequest::SharedPtr msg) const
{
    auto mode = msg->operating_mode.mode;
    if (mode != agv_app_msgs::msg::OperatingMode::AUTOMATIC &&
        mode != agv_app_msgs::msg::OperatingMode::SEMIAUTOMATIC &&
        mode != agv_app_msgs::msg::OperatingMode::MANUAL &&
        mode != agv_app_msgs::msg::OperatingMode::SERVICE &&
        mode != agv_app_msgs::msg::OperatingMode::TEACHIN) {
        return Result::fail("invalid mode, must be `AUTOMATIC`, `SEMIAUTOMATIC`, `MANUAL`, `SERVICE`, or `TEACHIN`");
    }
    return Result::ok();
}

void SetOperatingModeHandler::create_action_parameters(const agv_app_msgs::msg::AppRequest::SharedPtr msg, agv_service::msg::Action& action)
{
    action.action_parameters.push_back(set_action_parameter("mode", msg->operating_mode.mode));
}


}  // namespace agv_app_server