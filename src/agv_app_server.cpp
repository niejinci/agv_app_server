// 源代码文件对应头文件
#include "agv_app_server/agv_app_server.hpp"

// 项目相关头文件
#include "LogManager.hpp"

// ros2 相关头文件
#include "sensor_msgs/point_cloud2_iterator.hpp"

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
    register_instant_action_handlers();

    // pub_agv_order_ = this->create_publisher<agv_service::msg::Order>("agv_order_topic", 10);
    // pub_mqtt_operate_ = this->create_publisher<agv_service::msg::MqttState>("mqtt_operate_topic", 10);

    LogManager::getInstance().getLogger()->info("AgvAppServer initialized.");
}

AgvAppServer::~AgvAppServer() {}

// 注册各种即时动作处理程序
void AgvAppServer::register_instant_action_handlers()
{
    // 1. RelocationHandler
    instant_action_handlers_["RELOCATION"] = std::make_shared<RelocationHandler>(pub_agv_instant_, pub_app_data_);
    // 2. TranslationHandler
    instant_action_handlers_["TRANSLATION"] = std::make_shared<TranslationHandler>(pub_agv_instant_, pub_app_data_);
    // 3. RotationHandler
    instant_action_handlers_["ROTATION"] = std::make_shared<RotationHandler>(pub_agv_instant_, pub_app_data_);
    // 4. PalletRotationHandler
    instant_action_handlers_["PALLET_ROTATION"] = std::make_shared<PalletRotationHandler>(pub_agv_instant_, pub_app_data_);
    // 5. LiftingHandler
    instant_action_handlers_["LIFTING"] = std::make_shared<LiftingHandler>(pub_agv_instant_, pub_app_data_);
    // 6. CancelTaskHandler
    instant_action_handlers_["CANCEL_TASK"] = std::make_shared<CancelTaskHandler>(pub_agv_instant_, pub_app_data_);
    // 7. PauseTaskHandler
    instant_action_handlers_["PAUSE_TASK"] = std::make_shared<PauseTaskHandler>(pub_agv_instant_, pub_app_data_);
    // 8. ResumeTaskHandler
    instant_action_handlers_["RESUME_TASK"] = std::make_shared<ResumeTaskHandler>(pub_agv_instant_, pub_app_data_);
    // 9. RemoteControlHandler
    instant_action_handlers_["REMOTE_CONTROL"] = std::make_shared<RemoteControlHandler>(pub_agv_instant_, pub_app_data_);
    // 10. EmergencyStopHandler
    instant_action_handlers_["EMERGENCY_STOP"] = std::make_shared<EmergencyStopHandler>(pub_agv_instant_, pub_app_data_);
    // 11. ClearErrorsHandler
    instant_action_handlers_["CLEAR_ERRORS"] = std::make_shared<ClearErrorsHandler>(pub_agv_instant_, pub_app_data_);
    // 12. SoftResetHandler
    instant_action_handlers_["SOFT_RESET"] = std::make_shared<SoftResetHandler>(pub_agv_instant_, pub_app_data_);
    // 13. ClearErrorsHandler
    instant_action_handlers_["CLEAR_ERRORS"] = std::make_shared<ClearErrorsHandler>(pub_agv_instant_, pub_app_data_);
    // 14. SoftResetHandler
    instant_action_handlers_["SOFT_RESET"] = std::make_shared<SoftResetHandler>(pub_agv_instant_, pub_app_data_);
}

void AgvAppServer::register_data_stream_handlers()
{
    // 1. 注册 filte_scan - 点云数据流
    data_stream_handlers_["filte_scan"] = std::make_shared<DataStreamHandler<sensor_msgs::msg::PointCloud2>>(
        this,
        "filte_scan",
        rclcpp::SensorDataQoS(), // 点云通常使用 SensorDataQoS(BestEffort)
        std::bind(&AgvAppServer::process_filte_scan, this, std::placeholders::_1)
    );

    // 2. 注册 locationInfo - 位置信息数据流
    data_stream_handlers_["locationInfo"] = std::make_shared<DataStreamHandler<agv_service::msg::SlamLocationInfo>>(
        this,
        "locationInfo",
        rclcpp::SystemDefaultsQoS(),
        std::bind(&AgvAppServer::process_locationInfo, this, std::placeholders::_1)
    );

    // 3. 注册 scan2pointcloud - 点云数据流
    data_stream_handlers_["scan2pointcloud"] = std::make_shared<DataStreamHandler<sensor_msgs::msg::PointCloud2>>(
        this,
        "scan2pointcloud",
        rclcpp::SensorDataQoS(),
        std::bind(&AgvAppServer::process_scan2pointcloud, this, std::placeholders::_1)
    );

    // 4. 注册 obst_pcl - 障碍物点云数据流
    data_stream_handlers_["obst_pcl"] = std::make_shared<DataStreamHandler<sensor_msgs::msg::PointCloud2>>(
        this,
        "obst_pcl",
        rclcpp::SensorDataQoS(),
        std::bind(&AgvAppServer::process_obst_pcl, this, std::placeholders::_1)
    );

    // 5. 注册 obst_polygon - 障碍物多边形数据流
    data_stream_handlers_["obst_polygon"] = std::make_shared<DataStreamHandler<geometry_msgs::msg::PolygonStamped>>(
        this,
        "obst_polygon",
        rclcpp::SystemDefaultsQoS(),
        std::bind(&AgvAppServer::process_obst_polygon, this, std::placeholders::_1)
    );

    // 6. 注册 model_polygon - 模型多边形数据流
    data_stream_handlers_["model_polygon"] = std::make_shared<DataStreamHandler<geometry_msgs::msg::PolygonStamped>>(
        this,
        "model_polygon",
        rclcpp::SystemDefaultsQoS(),
        std::bind(&AgvAppServer::process_model_polygon, this, std::placeholders::_1)
    );

    // 7. 注册 qr_pos_data  - 二维码位置数据流
    data_stream_handlers_["qr_pos_data"] = std::make_shared<DataStreamHandler<agv_service::msg::MCUToPC>>(
        this,
        "qr_pos_data",
        rclcpp::SystemDefaultsQoS(),
        std::bind(&AgvAppServer::process_qr_pos_data, this, std::placeholders::_1)
    );

    // 8. 注册 qr_rack_data - 二维码货架数据流
    data_stream_handlers_["qr_rack_data"] = std::make_shared<DataStreamHandler<agv_service::msg::MCUToPC>>(
        this,
        "qr_rack_data",
        rclcpp::SystemDefaultsQoS(),
        std::bind(&AgvAppServer::process_qr_rack_data, this, std::placeholders::_1)
    );

    LogManager::getInstance().getLogger()->info("All data stream handlers registered.");
}

void AgvAppServer::handle_app_request(const agv_app_msgs::msg::AppRequest::SharedPtr msg)
{
    // 检查是否有对应的即时动作处理程序
    auto handler = instant_action_handlers_.find(msg->command_type);
    if (handler != instant_action_handlers_.end()) {
        handler->second->handle(msg);
        return;
    }

    bool success = true;
    std::string response_msg = "Command executed successfully";

    if (msg->command_type == "MANAGE_DATA_SUBSCRIPTION") {
        // 管理数据流推送的启动和停止
        std::string topic = msg->manage_data_subscription.topic;
        std::string action = msg->manage_data_subscription.action;

        auto it = data_stream_handlers_.find(topic);
        if (it != data_stream_handlers_.end()) {
            if (action == "start") {
                it->second->start();
                response_msg = "Started stream: " + topic;
            } else if (action == "stop") {
                it->second->stop();
                response_msg = "Stopped stream: " + topic;
            } else {
                success = false;
                response_msg = "Invalid action. Use start/stop.";
            }
        } else {
            success = false;
            response_msg = "Unknown data stream: " + topic;
        }
    } else {
        success = false;
        response_msg = "Unknown command type";
    }

    if (success) {
        LogManager::getInstance().getLogger()->info(response_msg);
    } else {
        LogManager::getInstance().getLogger()->warn(response_msg);
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

// 具体处理函数的实现示例
void AgvAppServer::process_filte_scan(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // 构造 AppData 并发布
    agv_app_msgs::msg::AppData response;
    response.source_type = "point_cloud";
    response.command_type = "filte_scan";

    bool has_intensity = false;
    bool has_x = false;
    bool has_y = false;
    bool has_z = false;
    for (const auto &field : msg->fields) {
        if (field.name == "intensity") {
            has_intensity = true;
        }
        if (field.name == "x") {
            has_x = true;
        }
        if (field.name == "y") {
            has_y = true;
        }
        if (field.name == "z") {
            has_z = true;
        }
    }
    if (!has_x || !has_y || !has_z) {
        response.success = false;
        response.message = "PointCloud2 is invalid: missing x/y/z fields";
        pub_app_data_->publish(response);
        return;
    }

    // 使用迭代器遍历点云数据
    sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");

    // 如果没有 intensity，临时借用 "x" 字段初始化迭代器，保证循环中 ++iter 操作安全
    // 实际取值时会根据 has_intensity 判断，不会使用这个借用的值
    std::string intensity_field_name = has_intensity ? "intensity" : "x";
    sensor_msgs::PointCloud2Iterator<float> iter_intensity(*msg, intensity_field_name);

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_intensity) {
        response.points.emplace_back(agv_app_msgs::msg::PointXyzi()
                                        .set__x(*iter_x)
                                        .set__y(*iter_y)
                                        .set__z(*iter_z)
                                        .set__intensity(has_intensity ? *iter_intensity : 0.0f));
    }
    pub_app_data_->publish(response);
}

void AgvAppServer::process_locationInfo(const agv_service::msg::SlamLocationInfo::SharedPtr msg)
{

}

void AgvAppServer::process_scan2pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{

}
void AgvAppServer::process_obst_pcl(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{

}
void AgvAppServer::process_obst_polygon(const geometry_msgs::msg::PolygonStamped::SharedPtr msg)
{

}
void AgvAppServer::process_model_polygon(const geometry_msgs::msg::PolygonStamped::SharedPtr msg)
{

}
void AgvAppServer::process_qr_pos_data(const agv_service::msg::MCUToPC::SharedPtr msg)
{

}
void AgvAppServer::process_qr_rack_data(const agv_service::msg::MCUToPC::SharedPtr msg)
{

}

} // namespace agv_app_server