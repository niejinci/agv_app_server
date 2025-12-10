// 项目相关头文件
#include "LogManager.hpp"
#include "agv_app_server/agv_app_server.hpp"

namespace agv_app_server
{

AgvAppServer::AgvAppServer(const rclcpp::NodeOptions & options)
: Node("agv_app_server", options)
{
    // 1. 外部接口
    sub_app_request_ = this->create_subscription<agv_app_msgs::msg::AppRequest>("app_request_topic", 10,
        std::bind(&AgvAppServer::handle_app_request, this, std::placeholders::_1));

    pub_app_data_ = this->create_publisher<agv_app_msgs::msg::AppData>("app_data_topic", 10);

    // 2. 内部接口 (Publishers)
    pub_agv_instant_ = this->create_publisher<agv_service::msg::InstantActions>("agv_instant_topic", 10);
    pub_agv_order_ = this->create_publisher<agv_service::msg::Order>("agv_order_topic", 10);
    pub_mqtt_operate_ = this->create_publisher<agv_service::msg::MqttState>("mqtt_operate_topic", 10);

    // 3. 内部接口 (Subscribers，数据源)
    // 示例：仅展示部分核心订阅，其他类似
    sub_filte_scan_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "filte_scan", rclcpp::SensorDataQoS(),
        std::bind(&AgvAppServer::on_filte_scan, this, std::placeholders::_1));

    // 初始化订阅状态
    data_subscription_status_["filte_scan"] = false;
    // ... init others to false

    register_instant_action_handlers();

    LogManager::getInstance().getLogger()->info("AgvAppServer initialized.");
}

AgvAppServer::~AgvAppServer() {}

// 注册各种即时动作处理程序
void AgvAppServer::register_instant_action_handlers()
{
    // RelocationHandler
    instant_action_handlers_["RELOCATION"] = std::make_shared<RelocationHandler>(pub_agv_instant_, pub_app_data_);
    // TranslationHandler
    instant_action_handlers_["TRANSLATION"] = std::make_shared<TranslationHandler>(pub_agv_instant_, pub_app_data_);
    // RotationHandler
    instant_action_handlers_["ROTATION"] = std::make_shared<RotationHandler>(pub_agv_instant_, pub_app_data_);
    // PalletRotationHandler
    instant_action_handlers_["PALLET_ROTATION"] = std::make_shared<PalletRotationHandler>(pub_agv_instant_, pub_app_data_);
    // LiftingHandler
    instant_action_handlers_["LIFTING"] = std::make_shared<LiftingHandler>(pub_agv_instant_, pub_app_data_);
    // CancelTaskHandler
    instant_action_handlers_["CANCEL_TASK"] = std::make_shared<CancelTaskHandler>(pub_agv_instant_, pub_app_data_);
    // PauseTaskHandler
    instant_action_handlers_["PAUSE_TASK"] = std::make_shared<PauseTaskHandler>(pub_agv_instant_, pub_app_data_);
    // ResumeTaskHandler
    instant_action_handlers_["RESUME_TASK"] = std::make_shared<ResumeTaskHandler>(pub_agv_instant_, pub_app_data_);
    // RemoteControlHandler
    instant_action_handlers_["REMOTE_CONTROL"] = std::make_shared<RemoteControlHandler>(pub_agv_instant_, pub_app_data_);
    // EmergencyStopHandler
    instant_action_handlers_["EMERGENCY_STOP"] = std::make_shared<EmergencyStopHandler>(pub_agv_instant_, pub_app_data_);
    // ClearErrorsHandler
    instant_action_handlers_["CLEAR_ERRORS"] = std::make_shared<ClearErrorsHandler>(pub_agv_instant_, pub_app_data_);
    // SoftResetHandler
    instant_action_handlers_["SOFT_RESET"] = std::make_shared<SoftResetHandler>(pub_agv_instant_, pub_app_data_);
    // ClearErrorsHandler
    instant_action_handlers_["CLEAR_ERRORS"] = std::make_shared<ClearErrorsHandler>(pub_agv_instant_, pub_app_data_);
    // SoftResetHandler
    instant_action_handlers_["SOFT_RESET"] = std::make_shared<SoftResetHandler>(pub_agv_instant_, pub_app_data_);
}

void AgvAppServer::handle_app_request(const agv_app_msgs::msg::AppRequest::SharedPtr msg)
{
    auto handler = instant_action_handlers_.find(msg->command_type);
    if (handler != instant_action_handlers_.end()) {
        handler->second->handle(msg);
        return;
    }

    bool success = true;
    std::string response_msg = "Command executed successfully";
    if (msg->command_type == "MANAGE_DATA_SUBSCRIPTION") {
        // 处理订阅开关
        std::string topic = msg->manage_data_subscription.topic;
        std::string action = msg->manage_data_subscription.action;

        if (action == "start") {
            data_subscription_status_[topic] = true;
            LogManager::getInstance().getLogger()->info("Started subscription for: " + topic);
        } else if (action == "stop") {
            data_subscription_status_[topic] = false;
            LogManager::getInstance().getLogger()->info("Stopped subscription for: " + topic);
        } else {
            success = false;
            response_msg = "Invalid MANAGE_DATA_SUBSCRIPTION action: " + action;
            LogManager::getInstance().getLogger()->warn(action);
        }
    } else {
        response_msg = "Unknown command type";
        LogManager::getInstance().getLogger()->warn("Unknown command: " + msg->command_type);
    }

    // 发送指令响应
    publish_cmd_response(msg->request_id, msg->command_type, success, response_msg);
}

void AgvAppServer::publish_cmd_response(const std::string & request_id, const std::string & cmd_type,
                          bool success, const std::string & message)
{
    agv_app_msgs::msg::AppData response;
    response.source_type = "cmd_response";
    response.request_id = request_id;
    response.command_type = cmd_type;
    response.success = success;
    response.message = message;
    pub_app_data_->publish(response);
}

void AgvAppServer::on_filte_scan(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // 检查订阅是否开启
    if (data_subscription_status_["filte_scan"]) {
        // 构造 AppData
        // 注意：这里需要确保 AppData.msg 中有适合存放 scan 的字段
        // 如果 AppData 定义中没有 LaserScan 类型字段，通常的做法是转换成 JSON
        // 或者如果 AppData 有 `sensor_msgs/LaserScan scan` 字段

        // 示例：假设不直接转发大流量数据到 JSON，而是通过单独字段或转义
        // 这里只是做逻辑演示
        /*
        agv_app_msgs::msg::AppData app_data;
        app_data.source_type = "filte_scan";
        // app_data.scan = *msg; // 如果有这个字段
        pub_app_data_->publish(app_data);
        */
        LogManager::getInstance().getLogger()->info("Forwarding filte_scan data. width={}", msg->width);
    }
}

} // namespace agv_app_server