// 源代码文件对应头文件
#include "agv_app_server/agv_app_server.hpp"

// 项目相关头文件
#include "agv_app_server/start_task_chain.hpp"
#include "agv_app_server/get_operating_mode_handler.hpp"
#include "agv_app_server/set_rcs_online.hpp"
#include "agv_app_server/data_subscription_handler.hpp"

#include "LogManager.hpp"
#include "agv_app_server/point_cloud_util.hpp"
#include "agv_app_msgs/msg/operating_mode.hpp"

// ros2 相关头文件
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace agv_app_server
{

AgvAppServer::AgvAppServer(const rclcpp::NodeOptions & options)
: Node("agv_app_server", options)
{
    // 1. 外部接口，订阅者
    sub_app_request_ = this->create_subscription<agv_app_msgs::msg::AppRequest>("app_request_topic", 10,
        std::bind(&AgvAppServer::handle_app_request, this, std::placeholders::_1));

    // 2. 外部接口，发布者
    pub_app_data_ = this->create_publisher<agv_app_msgs::msg::AppData>("app_data_topic", 50);

    // 33 定义回调组
    callback_group_pointcloud_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_state_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // 4. 内部接口，发布者
    pub_agv_instant_ = this->create_publisher<agv_service::msg::InstantActions>("agv_instant_topic", 10);

    // 5. 内部接口，订阅者
    // 小车状态信息必须订阅，因为很多即时动作需要根据当前操作模式决定是否允许执行
    sub_agv_state_ = this->create_subscription<agv_service::msg::State>("agv_state_topic", 10,
        [this](const agv_service::msg::State::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(agv_state_mutex_);
            agv_app_server::StateLite state_lite;
            state_lite.is_task_running = msg->is_task_running;
            state_lite.edegs_empty = msg->edge_states.empty();
            state_lite.order_id = msg->order_id;
            state_lite.last_node_id = msg->last_node_id;
            state_lite.last_node_sequence_id = msg->last_node_sequence_id;
            state_lite.agv_position = msg->agv_position;
            state_lite.node_states = msg->node_states;
            state_lite.action_states = msg->action_states;
            state_lite.errors = msg->errors;
            state_lite.operating_mode = msg->operating_mode;
            state_lite.e_stop = msg->safety_state.e_stop;
            latest_agv_state_lite_ = std::move(state_lite);
        });

    // 6. mqtt 有关
    // 连接rcs的状态，订阅者，主题发布频率: 1/3hz = 3s
    mqtt_state_ = this->create_subscription<agv_service::msg::MqttState>("mqtt_state_topic", 10,
                            [this](const agv_service::msg::MqttState::SharedPtr msg) {
                                mqtt_state_online_.store(msg->online, std::memory_order_release);

                                // 日志限频 (使用 atomic 保证线程安全)
                                static std::atomic<uint32_t> count{0};
                                // 20 次打印一次
                                if (count.fetch_add(1) % 20 != 0) return;
                                LogManager::getInstance().getLogger()->info("Received MQTT state update: {}", msg->online);
                            });
    // 设置上下线，发布者
    mqtt_state_publisher_ = this->create_publisher<agv_service::msg::MqttState>("mqtt_operate_topic", 10);

    // 7. 内部接口，发布者
    pub_agv_order_ = this->create_publisher<agv_service::msg::Order>("agv_order_topic", 10);

    // 8. plc 联动
    pub_plc_do = this->create_publisher<agv_service::msg::DigitalOutput>("agv_do_topic", 10);
    task_executor_ = std::make_shared<agv_app_server::TaskExecutor>(this, pub_agv_order_, pub_agv_instant_, pub_plc_do);

    // 创建更可靠的QoS配置
    // rclcpp::QoS(rclcpp::KeepLast(10))：初始化一个 QoS 配置文件，其历史策略设置为 “保留最新”，用于存储最近 10 条消息。
    // .reliability(rclcpp::ReliabilityPolicy::Reliable)：将可靠性策略设置为 “可靠”，这意味着通信将保证消息传递（类似于 TCP）。
    // .durability(rclcpp::DurabilityPolicy::TransientLocal)：将持久性策略设置为 “TransientLocal”，即发布者会存储消息以供后续加入的订阅者使用。
    // 这确保订阅者即使在消息发布后才订阅，也能接收到最新的消息。
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
        .reliability(rclcpp::ReliabilityPolicy::Reliable)
        .durability(rclcpp::DurabilityPolicy::TransientLocal)
        .history(rclcpp::HistoryPolicy::KeepLast);
    sub_plc_di = this->create_subscription<agv_service::msg::DigitalInput>("agv_di_topic", qos,
                                                        [this](const agv_service::msg::DigitalInput::SharedPtr msg) {
                                                            task_executor_->plc_subscript_callback(msg->id, msg->value);
                                                        });

    // 9. 注册状态定时器
    register_state_timer();

    // 10. 注册数据流处理程序
    register_data_stream_handlers();

    // 11. 注册各种即时动作处理程序
    register_instant_action_handlers();

    // 12. 注册命令处理器
    register_command_handlers();

    LogManager::getInstance().getLogger()->info("AgvAppServer initialized.");
}

AgvAppServer::~AgvAppServer() {}

void AgvAppServer::register_state_timer()
{
    state_timers_["agv_state_topic"] = std::make_shared<StateTimer>(
        this,
        "agv_state_topic",
        std::chrono::milliseconds(500),     // 2hz
        std::bind(&AgvAppServer::state_relate_timer_cb, this),
        callback_group_state_
    );

    state_timers_["mqtt_state_topic"] = std::make_shared<StateTimer>(
        this,
        "mqtt_state_topic",
        std::chrono::milliseconds(3000),    // mqtt_state_topic 3s 才发布一次，这里也3秒发布一次
        std::bind(&AgvAppServer::mqtt_state_timer_cb, this),
        callback_group_state_
    );

    LogManager::getInstance().getLogger()->info("Registered state timers: agv_state_topic, mqtt_state_topic.");
}

void AgvAppServer::mqtt_state_timer_cb()
{
    // 发布 MQTT 状态信息
    agv_app_msgs::msg::AppData response;
    response.source_type = "state";
    response.command_type = "mqtt_state_topic";
    response.mqtt_state.online = mqtt_state_online_.load(std::memory_order_acquire);
    pub_app_data_->publish(response);
}

void AgvAppServer::state_relate_timer_cb()
{
    std::optional<agv_app_server::StateLite> agv_state_lite_copy;
    {
        // 锁只保护共享资源的读写
        std::lock_guard<std::mutex> lock(agv_state_mutex_);
        agv_state_lite_copy = latest_agv_state_lite_;
    }
    if (!agv_state_lite_copy.has_value()) {
        static uint32_t count{0};
        if (count++ % 10 == 0)
            LogManager::getInstance().getLogger()->warn("AGV state is not available yet. Skipping processing.");
        return;
    }

    // 填充错误信息
    agv_app_msgs::msg::AppData response;
    response.source_type = "state";
    response.command_type = "agv_state_topic";
    for (const auto& item : agv_state_lite_copy.value().errors) {
        response.errors.emplace_back(agv_app_msgs::msg::Error()
                                    .set__error_type(item.error_type)
                                    .set__error_description(item.error_description)
                                    .set__error_hint(item.error_hint)
                                    .set__error_level(item.error_level));
    }

    // 填充运行任务信息
    if (agv_state_lite_copy->is_task_running && !agv_state_lite_copy->edegs_empty) {
        response.running_task.set__order_id(agv_state_lite_copy->order_id)
                            .set__last_node_id(agv_state_lite_copy->last_node_id);

        // 查找 next_node_id
        int64_t next_node_id = agv_state_lite_copy->last_node_sequence_id + 2;
        for (auto& node : agv_state_lite_copy->node_states) {
            if (node.sequence_id == next_node_id) {
                response.running_task.set__next_node_id(node.node_id);
                break;
            }
        }
    }

    // 填充操作模式
    response.operating_mode.mode = agv_state_lite_copy->operating_mode;

    // 发布状态信息
    // 一次发布包含错误信息、运行任务信息和操作模式，这样做的原因是，上面三个信息都是来自同一个订阅，为了节省带宽，把他们放到一个 AppData 中返回。
    pub_app_data_->publish(response);

    // 检查动作或任务的完成情况
    task_executor_->check_instant_action_finish(agv_state_lite_copy.value());
    task_executor_->check_order_finish(agv_state_lite_copy.value());
}

// 注册各种即时动作处理程序
void AgvAppServer::register_instant_action_handlers()
{
    auto get_mode_func = [this]() -> std::string {
        std::lock_guard<std::mutex> lock(agv_state_mutex_);
        if (latest_agv_state_lite_.has_value()) {
            return latest_agv_state_lite_->operating_mode;
        }
        return agv_app_msgs::msg::OperatingMode::AUTOMATIC;
    };

    // 1. RelocationHandler
    instant_action_handlers_["RELOCATION"] = std::make_shared<RelocationHandler>(pub_agv_instant_, pub_app_data_, get_mode_func);
    // 2. TranslationHandler
    instant_action_handlers_["TRANSLATION"] = std::make_shared<TranslationHandler>(pub_agv_instant_, pub_app_data_, get_mode_func);
    // 3. RotationHandler
    instant_action_handlers_["ROTATION"] = std::make_shared<RotationHandler>(pub_agv_instant_, pub_app_data_, get_mode_func);
    // 4. PalletRotationHandler
    instant_action_handlers_["PALLET_ROTATION"] = std::make_shared<PalletRotationHandler>(pub_agv_instant_, pub_app_data_, get_mode_func);
    // 5. LiftingHandler
    instant_action_handlers_["LIFTING"] = std::make_shared<LiftingHandler>(pub_agv_instant_, pub_app_data_, get_mode_func);
    // 6. CancelTaskHandler
    instant_action_handlers_["CANCEL_TASK"] = std::make_shared<CancelTaskHandler>(pub_agv_instant_, pub_app_data_, get_mode_func, [this](){
        task_executor_->clear_status();
    });
    // 7. PauseTaskHandler
    instant_action_handlers_["PAUSE_TASK"] = std::make_shared<PauseTaskHandler>(pub_agv_instant_, pub_app_data_, get_mode_func);
    // 8. ResumeTaskHandler
    instant_action_handlers_["RESUME_TASK"] = std::make_shared<ResumeTaskHandler>(pub_agv_instant_, pub_app_data_, get_mode_func);
    // 9. RemoteControlHandler
    instant_action_handlers_["REMOTE_CONTROL"] = std::make_shared<RemoteControlHandler>(pub_agv_instant_, pub_app_data_, get_mode_func);
    // 10. EmergencyStopHandler
    instant_action_handlers_["EMERGENCY_STOP"] = std::make_shared<EmergencyStopHandler>(pub_agv_instant_, pub_app_data_);
    // 11. ClearErrorsHandler
    instant_action_handlers_["CLEAR_ERRORS"] = std::make_shared<ClearErrorsHandler>(pub_agv_instant_, pub_app_data_);
    // 12. SoftResetHandler
    instant_action_handlers_["SOFT_RESET"] = std::make_shared<SoftResetHandler>(pub_agv_instant_, pub_app_data_);
    // 13. SetOperatingModeHandler
    instant_action_handlers_["SET_OPERATING_MODE"] = std::make_shared<SetOperatingModeHandler>(pub_agv_instant_, pub_app_data_);

    LogManager::getInstance().getLogger()->info("Registered instant action handlers.");
}

void AgvAppServer::register_data_stream_handlers()
{
    // 注册订阅时指定组(点云组)
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = callback_group_pointcloud_;

    // 1. 注册 filte_scan - 点云数据流， 20hz
    data_stream_handlers_["filte_scan"] = std::make_shared<DataStreamHandler<sensor_msgs::msg::PointCloud2>>(
        this,
        "filte_scan",
        rclcpp::SensorDataQoS(), // 点云通常使用 SensorDataQoS(BestEffort)
        std::bind(&AgvAppServer::process_filte_scan, this, std::placeholders::_1),
        sub_opt
    );

    // 2. 注册 locationInfo - 位置信息数据流，20hz
    data_stream_handlers_["locationInfo"] = std::make_shared<DataStreamHandler<agv_service::msg::SlamLocationInfo>>(
        this,
        "locationInfo",
        rclcpp::SystemDefaultsQoS(),
        std::bind(&AgvAppServer::process_locationInfo, this, std::placeholders::_1),
        sub_opt
    );

    // 3. 注册 scan2pointcloud - 点云数据流, 20hz
    data_stream_handlers_["scan2pointcloud"] = std::make_shared<DataStreamHandler<sensor_msgs::msg::PointCloud2>>(
        this,
        "scan2pointcloud",
        rclcpp::SensorDataQoS(),
        std::bind(&AgvAppServer::process_scan2pointcloud, this, std::placeholders::_1),
        sub_opt
    );

    // 4. 注册 obst_pcl - 障碍物点云数据流，遇到障碍物时发布, 20hz
    data_stream_handlers_["obst_pcl"] = std::make_shared<DataStreamHandler<sensor_msgs::msg::PointCloud2>>(
        this,
        "obst_pcl",
        rclcpp::SensorDataQoS(),
        std::bind(&AgvAppServer::process_obst_pcl, this, std::placeholders::_1),
        sub_opt
    );

    // 5. 注册 obst_polygon - 障碍物多边形数据流
    data_stream_handlers_["obst_polygon"] = std::make_shared<DataStreamHandler<geometry_msgs::msg::PolygonStamped>>(
        this,
        "obst_polygon",
        rclcpp::SystemDefaultsQoS(),
        std::bind(&AgvAppServer::process_obst_polygon, this, std::placeholders::_1),
        sub_opt
    );

    // 6. 注册 model_polygon - 模型多边形数据流
    data_stream_handlers_["model_polygon"] = std::make_shared<DataStreamHandler<geometry_msgs::msg::PolygonStamped>>(
        this,
        "model_polygon",
        rclcpp::SystemDefaultsQoS(),
        std::bind(&AgvAppServer::process_model_polygon, this, std::placeholders::_1),
        sub_opt
    );

    // 注册订阅时指定组(状态组)
    auto sub_opt2 = rclcpp::SubscriptionOptions();
    sub_opt2.callback_group = callback_group_state_;

    // 7. 注册 qr_pos_data  - 二维码位置数据流
    data_stream_handlers_["qr_pos_data"] = std::make_shared<DataStreamHandler<agv_service::msg::QrCameraData>>(
        this,
        "qr_pos_data",
        rclcpp::SystemDefaultsQoS(),
        std::bind(&AgvAppServer::process_qr_pos_data, this, std::placeholders::_1),
        sub_opt2
    );

    // 8. 注册 qr_rack_data - 二维码货架数据流
    data_stream_handlers_["qr_rack_data"] = std::make_shared<DataStreamHandler<agv_service::msg::QrCameraData>>(
        this,
        "qr_rack_data",
        rclcpp::SystemDefaultsQoS(),
        std::bind(&AgvAppServer::process_qr_rack_data, this, std::placeholders::_1),
        sub_opt2
    );

    // 9. 注册 mcu_to_pc - MCU 到 PC 的数据流， 50hz
    data_stream_handlers_["mcu_to_pc"] = std::make_shared<DataStreamHandler<agv_service::msg::MCUToPC>>(
        this,
        "mcu_to_pc",
        rclcpp::SystemDefaultsQoS(),
        std::bind(&AgvAppServer::process_mcu_to_pc, this, std::placeholders::_1),
        sub_opt2
    );

    // 10. 注册 sys_info - 系统信息数据流, 1hz
    data_stream_handlers_["sys_info"] = std::make_shared<DataStreamHandler<agv_service::msg::SysInfo>>(
        this,
        "sys_info",
        rclcpp::SystemDefaultsQoS(),
        std::bind(&AgvAppServer::process_sys_info, this, std::placeholders::_1),
        sub_opt2
    );

    LogManager::getInstance().getLogger()->info("All data stream handlers registered.");
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

// 处理 小车点云(稀疏，显示用) 数据流，发布频率限制为 5Hz
void AgvAppServer::process_filte_scan(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // rclcpp::Time 默认构造的时间类型是 RCL_SYSTEM_TIME (Type 2)
    // this->now() 返回的时间类型是 RCL_ROS_TIME  (Type 1)
    // 两者不能直接比较，需要统一类型
    static rclcpp::Time last_process_time(0, 0, RCL_ROS_TIME);

    // 检查频率限制 (5Hz = 0.2s)
    if (!should_process(last_process_time, 0.2)) {
        return;
    }

    // 构造 AppData 并发布
    agv_app_msgs::msg::AppData response;
    response.source_type = "data_stream";
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

// 处理位置信息数据流
void AgvAppServer::process_locationInfo(const agv_service::msg::SlamLocationInfo::SharedPtr msg)
{
    static rclcpp::Time last_process_time(0, 0, RCL_ROS_TIME);

    // 检查频率限制 (5Hz = 0.2s)
    if (!should_process(last_process_time, 0.2)) {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(agv_pos_mutex_);
        latest_agv_pos_ = msg->agv_position;
    }

    // 构造 AppData 并发布
    agv_app_msgs::msg::AppData response;
    response.source_type = "data_stream";
    response.command_type = "locationInfo";
    response.agv_position.set__x(msg->agv_position.x)
                          .set__y(msg->agv_position.y)
                          .set__z(msg->agv_position.z)
                          .set__yaw(msg->agv_position.yaw)
                          .set__pitch(msg->agv_position.pitch)
                          .set__roll(msg->agv_position.roll)
                          .set__map_id(msg->agv_position.map_id)
                          .set__position_initialized(msg->agv_position.position_initialized)
                          .set__map_description(msg->agv_position.map_description)
                          .set__localization_score(std::round(msg->agv_position.localization_score * 10000) / 100)
                          .set__deviation_range(msg->agv_position.deviation_range);
    pub_app_data_->publish(response);
}

// 处理 小车点云(避障用)数据流
void AgvAppServer::process_scan2pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    static rclcpp::Time last_process_time(0, 0, RCL_ROS_TIME);

    // 检查频率限制 (5Hz = 0.2s)
    if (!should_process(last_process_time, 0.2)) {
        return;
    }

    std::optional<agv_service::msg::AgvPosition> agv_pos_copy;
    {
        // 锁只保护共享资源的读写
        std::lock_guard<std::mutex> lock(agv_pos_mutex_);
        agv_pos_copy = latest_agv_pos_;
    }
    if (!agv_pos_copy.has_value()) {
        static uint32_t count{0};
        if (count++ % 10 == 0)
            LogManager::getInstance().getLogger()->warn("AGV position is not available yet. Skipping scan2pointcloud processing.");
        return;
    }

    // 构造 AppData 并发布
    agv_app_msgs::msg::AppData response;
    response.source_type = "data_stream";
    response.command_type = "scan2pointcloud";
    processPointCloud(msg, agv_pos_copy.value(), response.points);
    pub_app_data_->publish(response);
}

// 处理 障碍物点云 数据流
void AgvAppServer::process_obst_pcl(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    static rclcpp::Time last_process_time(0, 0, RCL_ROS_TIME);

    // 检查频率限制 (5Hz = 0.2s)
    if (!should_process(last_process_time, 0.2)) {
        return;
    }

    std::optional<agv_service::msg::AgvPosition> agv_pos_copy;
    {
        std::lock_guard<std::mutex> lock(agv_pos_mutex_);
        agv_pos_copy = latest_agv_pos_;
    }
    if (!agv_pos_copy.has_value()) {
        static uint32_t count{0};
        if (count++ % 10 == 0)
            LogManager::getInstance().getLogger()->warn("AGV position is not available yet. Skipping obst_pcl processing.");
        return;
    }

    // 构造 AppData 并发布
    agv_app_msgs::msg::AppData response;
    response.source_type = "data_stream";
    response.command_type = "obst_pcl";
    processPointCloud(msg, agv_pos_copy.value(), response.points);
    pub_app_data_->publish(response);
}

// 处理 障碍物多边形 数据流
void AgvAppServer::process_obst_polygon(const geometry_msgs::msg::PolygonStamped::SharedPtr msg)
{
    static rclcpp::Time last_process_time(0, 0, RCL_ROS_TIME);

    // 检查频率限制 (5Hz = 0.2s)
    if (!should_process(last_process_time, 0.2)) {
        return;
    }

    std::optional<agv_service::msg::AgvPosition> agv_pos_copy;
    {
        std::lock_guard<std::mutex> lock(agv_pos_mutex_);
        agv_pos_copy = latest_agv_pos_;
    }
    if (!agv_pos_copy.has_value()) {
        static uint32_t count{0};
        if (count++ % 10 == 0)
            LogManager::getInstance().getLogger()->warn("AGV position is not available yet. Skipping obst_pcl processing.");
        return;
    }

    // 构造 AppData 并发布
    agv_app_msgs::msg::AppData response;
    response.source_type = "data_stream";
    response.command_type = "obst_polygon";
    processPolygon(msg, agv_pos_copy.value(), response.points);
    pub_app_data_->publish(response);
}

// 处理 模型多边形 数据流
void AgvAppServer::process_model_polygon(const geometry_msgs::msg::PolygonStamped::SharedPtr msg)
{
    static rclcpp::Time last_process_time(0, 0, RCL_ROS_TIME);

    // 检查频率限制 (5Hz = 0.2s)
    if (!should_process(last_process_time, 0.2)) {
        return;
    }

    std::optional<agv_service::msg::AgvPosition> agv_pos_copy;
    {
        std::lock_guard<std::mutex> lock(agv_pos_mutex_);
        agv_pos_copy = latest_agv_pos_;
    }
    if (!agv_pos_copy.has_value()) {
        static uint32_t count{0};
        if (count++ % 10 == 0)
            LogManager::getInstance().getLogger()->warn("AGV position is not available yet. Skipping obst_pcl processing.");
        return;
    }

    // 构造 AppData 并发布
    agv_app_msgs::msg::AppData response;
    response.source_type = "data_stream";
    response.command_type = "model_polygon";
    processPolygon(msg, agv_pos_copy.value(), response.points);
    pub_app_data_->publish(response);
}

// 处理二维码位置数据流
void AgvAppServer::process_qr_pos_data(const agv_service::msg::QrCameraData::SharedPtr msg)
{
    static rclcpp::Time last_process_time(0, 0, RCL_ROS_TIME);

    // 检查频率限制 (5Hz = 0.2s)
    if (!should_process(last_process_time, 0.2)) {
        return;
    }

    // 构造 AppData 并发布
    agv_app_msgs::msg::AppData response;
    response.source_type = "data_stream";
    response.command_type = "qr_pos_data";
    response.qr_pos_camera_data.set__stamp(msg->stamp)
                                 .set__x(msg->x)
                                 .set__y(msg->y)
                                 .set__angle(msg->angle)
                                 .set__tag_number(msg->tag_number)
                                 .set__is_matrix(msg->is_matrix);
    pub_app_data_->publish(response);
}

// 处理二维码货架数据流
void AgvAppServer::process_qr_rack_data(const agv_service::msg::QrCameraData::SharedPtr msg)
{
    static rclcpp::Time last_process_time(0, 0, RCL_ROS_TIME);

    // 检查频率限制 (5Hz = 0.2s)
    if (!should_process(last_process_time, 0.2)) {
        return;
    }

    // 构造 AppData 并发布
    agv_app_msgs::msg::AppData response;
    response.source_type = "data_stream";
    response.command_type = "qr_rack_data";
    response.qr_rack_camera_data.set__stamp(msg->stamp)
                                 .set__x(msg->x)
                                 .set__y(msg->y)
                                 .set__angle(msg->angle)
                                 .set__tag_number(msg->tag_number)
                                 .set__is_matrix(msg->is_matrix);
    pub_app_data_->publish(response);
}

// 发布频率 50hz
void AgvAppServer::process_mcu_to_pc(const agv_service::msg::MCUToPC::SharedPtr msg)
{
    static uint32_t count{1};
    if (count++ % 50) { // 降低发布频率到1hz：如果不能被50整除，则返回
        return;
    }

    // 构造 AppData 并发布
    agv_app_msgs::msg::AppData response;
    response.source_type = "data_stream";
    response.command_type = "mcu_to_pc";
    response.mcu_data.battery.set__battery_charge(msg->battery_charge)
                           .set__battery_voltage(msg->battery_voltage)
                           .set__charging(msg->charging);

    // 设置电机信息
    response.mcu_data.left_servo.set__driver_state(msg->left_servo.driver_state)
                           .set__encoder_data(msg->left_servo.encoder_data)
                           .set__error_code(msg->left_servo.error_code)
                           .set__time_gap(msg->left_servo.time_gap)
                           .set__motor_speed(msg->left_servo.motor_speed);
    response.mcu_data.right_servo.set__driver_state(msg->right_servo.driver_state)
                           .set__encoder_data(msg->right_servo.encoder_data)
                           .set__error_code(msg->right_servo.error_code)
                           .set__time_gap(msg->right_servo.time_gap)
                           .set__motor_speed(msg->right_servo.motor_speed);
    response.mcu_data.lift_servo.set__driver_state(msg->lift_servo.driver_state)
                           .set__encoder_data(msg->lift_servo.encoder_data)
                           .set__error_code(msg->lift_servo.error_code)
                           .set__time_gap(msg->lift_servo.time_gap)
                           .set__motor_speed(msg->lift_servo.motor_speed);
    response.mcu_data.rotation_servo.set__driver_state(msg->rotation_servo.driver_state)
                           .set__encoder_data(msg->rotation_servo.encoder_data)
                           .set__error_code(msg->rotation_servo.error_code)
                           .set__time_gap(msg->rotation_servo.time_gap)
                           .set__motor_speed(msg->rotation_servo.motor_speed);

    // 设置GPIO输入位
    response.mcu_data.gpio_input_bits.reserve(24);
    for (uint32_t i = 0; i < 8 * sizeof(uint8_t); i++) {
        response.mcu_data.gpio_input_bits.emplace_back((msg->gpio_input_1 >> i) & 0x01);
    }
    for (uint32_t i = 0; i < 8 * sizeof(uint8_t); i++) {
        response.mcu_data.gpio_input_bits.emplace_back((msg->gpio_input_2 >> i) & 0x01);
    }
        for (uint32_t i = 0; i < 8 * sizeof(uint8_t); i++) {
        response.mcu_data.gpio_input_bits.emplace_back((msg->gpio_input_3 >> i) & 0x01);
    }

    // 设置GPIO输出位
    response.mcu_data.gpio_output_bits.reserve(24);
    for (uint32_t i = 0; i < 8 * sizeof(uint8_t); i++) {
        response.mcu_data.gpio_output_bits.emplace_back((msg->output_1 >> i) & 0x01);
    }
    for (uint32_t i = 0; i < 8 * sizeof(uint8_t); i++) {
        response.mcu_data.gpio_output_bits.emplace_back((msg->output_2 >> i) & 0x01);
    }
        for (uint32_t i = 0; i < 8 * sizeof(uint8_t); i++) {
        response.mcu_data.gpio_output_bits.emplace_back((msg->output_3 >> i) & 0x01);
    }
    pub_app_data_->publish(response);
}

// 发布频率 1hz
void AgvAppServer::process_sys_info(const agv_service::msg::SysInfo::SharedPtr msg)
{
    // 构造 AppData 并发布
    agv_app_msgs::msg::AppData response;
    response.source_type = "data_stream";
    response.command_type = "sys_info";

    // 版本信息
    response.sys_info.version.set__firmware_ver(msg->firmware_ver)
                      .set__map_ver(msg->map_ver)
                      .set__model_ver(msg->model_ver)
                      .set__robot_id(msg->robot_id)
                      .set__robot_name(msg->robot_name)
                      .set__robot_type(msg->robot_type)
                      .set__sensor_ver(msg->sensor_ver);

    // 系统资源
    response.sys_info.resource.set__cpu(msg->cpu)
        .set__idle(msg->idle)
        .set__mem(msg->mem)
        .set__mem_unused(msg->mem_unused)
        .set__mem_used(msg->mem_used)
        .set__swapd(msg->swapd)
        .set__temp(msg->temp);

    // 进程列表信息
    response.sys_info.process_infos.reserve(msg->process_infos.size());
    for (const auto& item : msg->process_infos) {
        agv_app_msgs::msg::ProcessInfo process_info;
        process_info.pid = item.pid;
        process_info.name = item.name;
        process_info.cpu = item.cpu;
        process_info.mem = item.mem;
        process_info.rss = item.rss;
        process_info.vss = item.vss;
        process_info.pss = item.pss;
        process_info.threads = item.threads;
        process_info.fds = item.fds;
        process_info.cswch = item.cswch;
        process_info.nvcswch = item.nvcswch;
        process_info.model_ver = item.model_ver;
        process_info.state = item.state;
        process_info.us = item.us;
        process_info.sy = item.sy;
        response.sys_info.process_infos.emplace_back(std::move(process_info));
    }
    pub_app_data_->publish(response);
}

/**
 * @brief 检查自上次处理后是否过了足够的时间
 * 如果满足时间间隔，则更新 last_process_time 并返回 true，否则返回 false
 *
 * @param last_time 指向存储上一次处理时间的变量的引用
 * @param interval_seconds 时间间隔，单位为秒
 * @return true 如果满足时间间隔，允许处理
 * @return false 调用频率过高，跳过此次处理
 */
bool AgvAppServer::should_process(rclcpp::Time& last_time, double interval_seconds)
{
    rclcpp::Time now = this->now();
    if ((now - last_time).seconds() < interval_seconds) {
        return false;
    }
    last_time = now;
    return true;
}

void AgvAppServer::register_command_handlers()
{
    // 获取操作模式的 lambda
    auto get_mode_str_fn = [this]() -> std::string {
        std::lock_guard<std::mutex> lock(agv_state_mutex_);
        if (latest_agv_state_lite_.has_value()) {
            return latest_agv_state_lite_->operating_mode;
        }
        return agv_app_msgs::msg::OperatingMode::AUTOMATIC;
    };

    // 1. 注册 GET_OPERATING_MODE
    command_handlers_["GET_OPERATING_MODE"] =
        std::make_shared<GetOperatingModeHandler>(pub_app_data_, get_mode_str_fn);

    // 2. 注册 START_TASK_CHAIN
    command_handlers_["START_TASK_CHAIN"] =
        std::make_shared<StartTaskChainHandler>(
            pub_app_data_,
            get_mode_str_fn,
            [this](const auto& msg) { task_executor_->submitTask(msg); }
        );

    // 3. 注册 MANAGE_DATA_SUBSCRIPTION
    command_handlers_["MANAGE_DATA_SUBSCRIPTION"] =
        std::make_shared<DataSubscriptionHandler>(
            pub_app_data_,
            data_stream_handlers_,
            state_timers_
        );

    // 4. 注册 SET_RCS_ONLINE
    command_handlers_["SET_RCS_ONLINE"] =
        std::make_shared<SetRcsOnlineHandler>(
            pub_app_data_,
            mqtt_state_publisher_
        );

    LogManager::getInstance().getLogger()->info("All command handlers registered.");
}

// 重构后的 handle_app_request - 简洁统一
void AgvAppServer::handle_app_request(const agv_app_msgs::msg::AppRequest::SharedPtr msg)
{
    // 1. 首先检查即时动作处理器
    auto instant_handler = instant_action_handlers_.find(msg->command_type);
    if (instant_handler != instant_action_handlers_.end()) {
        instant_handler->second->handle(msg);
        return;
    }

    // 2. 检查命令处理器
    auto cmd_handler = command_handlers_.find(msg->command_type);
    if (cmd_handler != command_handlers_.end()) {
        cmd_handler->second->handle(msg);
        return;
    }

    // 3. 未知命令类型
    LogManager::getInstance().getLogger()->warn("Unknown command type: {}", msg->command_type);
    publish_cmd_response(msg->request_id, msg->command_type, false, "Unknown command type");
}

} // namespace agv_app_server