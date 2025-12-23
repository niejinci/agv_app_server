#ifndef AGV_APP_SERVER__AGV_APP_SERVER_HPP_
#define AGV_APP_SERVER__AGV_APP_SERVER_HPP_

#include "agv_app_server/data_stream_handler.hpp"
#include "agv_app_server/state_timer.hpp"
#include "agv_app_server/task_parser.h"
#include "agv_app_server/task_executor.h"

// 引入 AGV 相关消息
// 内部发布/订阅使用
#include "agv_app_server/instant_action.hpp"
#include "agv_service/msg/agv_position.hpp"
#include "agv_service/msg/order.hpp"
#include "agv_service/msg/slam_location_info.hpp"
#include "agv_service/msg/mcu_to_pc.hpp"
#include "agv_service/msg/sys_info.hpp"
#include "agv_service/msg/qr_camera_data.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "agv_service/msg/mqtt_state.hpp"
#include "agv_service/msg/state.hpp"
#include "agv_service/msg/mcu_to_pc.hpp"
#include "agv_service/msg/sys_info.hpp"
#include "agv_service/msg/mqtt_state.hpp"
#include "agv_service/msg/digital_input.hpp"
#include "agv_service/msg/digital_output.hpp"

// ros2 相关头文件
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"

// 引入自定义消息
#include "agv_app_msgs/msg/app_request.hpp"
#include "agv_app_msgs/msg/app_data.hpp"

// c++ 标准库头文件
#include <memory>
#include <string>
#include <map>
#include <vector>
#include <atomic>

// JSON 支持
#include <nlohmann/json.hpp>
using json = nlohmann::json;

namespace agv_app_server
{

class AgvAppServer : public rclcpp::Node
{
public:
    explicit AgvAppServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    virtual ~AgvAppServer();

private:
    // 外部接口
    rclcpp::Subscription<agv_app_msgs::msg::AppRequest>::SharedPtr sub_app_request_;
    rclcpp::Publisher<agv_app_msgs::msg::AppData>::SharedPtr pub_app_data_;

    // 内部接口：发布者
    // 任务订单
    rclcpp::Publisher<agv_service::msg::Order>::SharedPtr pub_agv_order_;
    // 即时动作
    rclcpp::Publisher<agv_service::msg::InstantActions>::SharedPtr pub_agv_instant_;

    // 内部接口：订阅者
    // MQTT 连接状态
    rclcpp::Subscription<agv_service::msg::MqttState>::SharedPtr mqtt_state_;
    // 小车状态
    rclcpp::Subscription<agv_service::msg::State>::SharedPtr sub_agv_state_;

    // 即时动作管理
    // 注册各种即时动作处理程序
    void register_instant_action_handlers();
    // Key: command_type, Value: handler
    std::map<std::string, std::shared_ptr<BaseInstantActionHandler>> instant_action_handlers_;

    // 数据流管理
    // 使用 Map 统一管理所有动态订阅
    // Key: 业务层面的 topic 标识 (如 "filte_scan"), Value: 对应的处理器
    std::map<std::string, std::shared_ptr<IDataStreamHandler>> data_stream_handlers_;
    // 初始化注册所有的 handler
    void register_data_stream_handlers();

    // 具体的处理函数，负责将收到的数据打包发布到 app_data_topic
    void process_filte_scan(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void process_locationInfo(const agv_service::msg::SlamLocationInfo::SharedPtr msg);
    void process_scan2pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void process_obst_pcl(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void process_obst_polygon(const geometry_msgs::msg::PolygonStamped::SharedPtr msg);
    void process_model_polygon(const geometry_msgs::msg::PolygonStamped::SharedPtr msg);
    void process_qr_pos_data(const agv_service::msg::QrCameraData::SharedPtr msg);
    void process_qr_rack_data(const agv_service::msg::QrCameraData::SharedPtr msg);
    void process_mcu_to_pc(const agv_service::msg::MCUToPC::SharedPtr msg);
    void process_sys_info(const agv_service::msg::SysInfo::SharedPtr msg);

    // 外部请求的回调函数(app_request_topic)
    void handle_app_request(const agv_app_msgs::msg::AppRequest::SharedPtr msg);
    // 发布指令响应
    void publish_cmd_response(const std::string & request_id, const std::string & cmd_type,
                            bool success, const std::string & message);

    // 互斥锁
    std::mutex agv_pos_mutex_;
    // 默认构造时它是 nullopt (无值状态)
    std::optional<agv_service::msg::AgvPosition> latest_agv_pos_;

    // 状态相关处理
    void state_relate_timer_cb();
    void register_state_timer();
    void mqtt_state_timer_cb();
    std::map<std::string, std::shared_ptr<StateTimer>> state_timers_;

    // 小车状态
    std::mutex agv_state_mutex_;
    std::optional<agv_app_server::StateLite> latest_agv_state_lite_;

    // rcs连接状态
    std::atomic<bool> mqtt_state_online_{false};

    // rcs 上下线处理
    rclcpp::Publisher<agv_service::msg::MqttState>::SharedPtr mqtt_state_publisher_;
    void set_rcs_online(const agv_app_msgs::msg::AppRequest::SharedPtr msg);

    // 检查自上次处理后是否过了足够的时间
    // 如果满足时间间隔，则更新 last_process_time 并返回 true，否则返回 false
    bool should_process(rclcpp::Time& last_time, double interval_seconds);

    void get_operating_mode(const agv_app_msgs::msg::AppRequest::SharedPtr msg);

    // plc 订阅发布
    // agv 订阅 di, 发布 do
    rclcpp::Publisher<agv_service::msg::DigitalOutput>::SharedPtr pub_plc_do;
    rclcpp::Subscription<agv_service::msg::DigitalInput>::SharedPtr sub_plc_di;

    // 任务执行器
    std::shared_ptr<agv_app_server::TaskExecutor> task_executor_;
    void start_task_chain(const agv_app_msgs::msg::AppRequest::SharedPtr msg);
};

} // namespace agv_app_server

#endif // AGV_APP_SERVER__AGV_APP_SERVER_HPP_