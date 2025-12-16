#ifndef AGV_APP_SERVER__DATA_STREAM_HANDLER_HPP_
#define AGV_APP_SERVER__DATA_STREAM_HANDLER_HPP_

#include <string>
#include <memory>
#include <functional>

#include "agv_service/msg/agv_position.hpp"
#include "agv_service/msg/node_state.hpp"
#include "agv_service/msg/error.hpp"
#include "rclcpp/rclcpp.hpp"
#include "LogManager.hpp"

namespace agv_app_server
{

struct StateLite
{
    bool is_task_running = false;
    bool edegs_empty = false;
    std::string order_id;
    std::string last_node_id;
    int64_t last_node_sequence_id;
    agv_service::msg::AgvPosition agv_position;
    std::vector<agv_service::msg::NodeState> node_states;
    std::vector<agv_service::msg::Error> errors;
    std::string operating_mode;
    std::string e_stop;
};

/**
 * @brief 数据流处理器的抽象基类
 * 用于在 Map 中统一管理不同类型的 Handler
 */
class IDataStreamHandler
{
public:
    virtual ~IDataStreamHandler() = default;

    // 开启订阅
    virtual void start() = 0;

    // 停止订阅
    virtual void stop() = 0;

    // 获取当前状态
    virtual bool is_active() const = 0;

    // 获取 Topic 名称
    virtual std::string get_topic_name() const = 0;
};

/**
 * @brief 具体的泛型数据流处理器
 * T: ROS 消息类型 (如 sensor_msgs::msg::PointCloud2)
 */
template <typename T>
class DataStreamHandler : public IDataStreamHandler
{
public:
    // 定义转换函数的签名：输入是 ROS 消息，无返回值（内部直接 publish）
    // 或者：输入 ROS 消息，返回 AppData（由 Handler 负责 publish）
    // 这里采用 Handler 负责 publish 的方式，更加灵活
    using ConvertAndPublishFunc = std::function<void(const typename T::SharedPtr)>;

    // rclcpp::SensorDataQoS()（通常对应 Best Effort, Volatile），而对控制/状态类数据使用了
    // rclcpp::SystemDefaultsQoS()（通常是 Reliable）
    // 请根据你实际的系统配置调整 QoS，如果发布者是 Reliable 而你订阅用 BestEffort，可能会导致连不上。通常点云推荐 BestEffort
    DataStreamHandler(
        rclcpp::Node* node,
        const std::string& topic_name,
        const rclcpp::QoS& qos,
        ConvertAndPublishFunc callback)
    : node_(node), topic_name_(topic_name), qos_(qos), callback_(callback), is_active_(false)
    {
    }

    void start() override
    {
        if (is_active_) return;

        LogManager::getInstance().getLogger()->info("Starting data stream: " + topic_name_);

        // 创建订阅
        subscription_ = node_->create_subscription<T>(
            topic_name_,
            qos_,
            [this](const typename T::SharedPtr msg) {
                // 收到消息后，调用传入的转换逻辑
                if (this->callback_) {
                    this->callback_(msg);
                }
            }
        );
        is_active_ = true;
    }

    void stop() override
    {
        if (!is_active_) return;

        LogManager::getInstance().getLogger()->info("Stopping data stream: " + topic_name_);
        subscription_.reset(); // 销毁订阅对象
        is_active_ = false;
    }

    bool is_active() const override { return is_active_; }
    std::string get_topic_name() const override { return topic_name_; }

private:
    rclcpp::Node* node_; // 持有 Node 的裸指针用于创建订阅（注意生命周期，Handler 通常属于 Node，所以安全）
    std::string topic_name_;
    rclcpp::QoS qos_;
    ConvertAndPublishFunc callback_;
    bool is_active_;

    typename rclcpp::Subscription<T>::SharedPtr subscription_;
};

} // namespace agv_app_server

#endif // AGV_APP_SERVER__DATA_STREAM_HANDLER_HPP_