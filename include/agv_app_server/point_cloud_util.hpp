#pragma once

// 项目相关头文件
#include "LogManager.hpp"
#include "agv_app_msgs/msg/app_data.hpp"

// ros2 相关头文件
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

// c++ 标准库头文件
#include <cmath>
#include <array>


namespace agv_app_server
{
    struct Point3Dd {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };
    using RotationMatrix = std::array<double, 9>;   // 3x3 旋转矩阵
    using TranslationVector = Point3Dd;             // 平移向量和点结构相同

    RotationMatrix eulerAnglesToRotationMatrix(double yaw, double pitch, double roll)
    {
        RotationMatrix rot;
        double c1 = std::cos(yaw);
        double s1 = std::sin(yaw);
        double c2 = std::cos(roll);
        double s2 = std::sin(roll);
        double c3 = std::cos(pitch);
        double s3 = std::sin(pitch);

        rot[0] = c1*c2;
        rot[1] = -s1*c3 + c1*s2*s3;
        rot[2] = s1*s3 + c1*s2*c3;
        rot[3] = s1*c2;
        rot[4] = c1*c3 + s1*s2*s3;
        rot[5] = -c1*s3 + s1*s2*c3;
        rot[6] = -s2;
        rot[7] = c2*s3;
        rot[8] = c2*c3;
        return rot;
    }

    Point3Dd transformPoint(const Point3Dd& point, const RotationMatrix& rot, const TranslationVector& t)
    {
        Point3Dd tran_point;
        tran_point.x = rot[0] * point.x + rot[1] * point.y + rot[2] * point.z + t.x;
        tran_point.y = rot[3] * point.x + rot[4] * point.y + rot[5] * point.z + t.y;
        tran_point.z = rot[6] * point.x + rot[7] * point.y + rot[8] * point.z + t.z;
        return tran_point;
    }

/*
    base_link 转换为 map 的旋转矩阵：
    1. 先旋转
    2. 再平移
*/
inline agv_app_msgs::msg::AppData::_points_type processPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr& input_cloud, const agv_service::msg::AgvPosition& agv_pos)
{
    agv_app_msgs::msg::AppData::_points_type output_points;

    // 从AGV位姿创建平移向量和旋转矩阵
    const TranslationVector t = {agv_pos.x, agv_pos.y, agv_pos.z};
    const RotationMatrix rot = eulerAnglesToRotationMatrix(agv_pos.yaw, agv_pos.pitch, agv_pos.roll);

    bool has_intensity = false;
    bool has_x = false;
    bool has_y = false;
    bool has_z = false;
    for (const auto &field : input_cloud->fields) {
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
        LogManager::getInstance().getLogger()->warn("PointCloud2 is invalid: missing x/y/z fields");
        return output_points;
    }

    // 预分配内存，避免 vector 动态扩容带来的性能开销
    size_t point_count = input_cloud->width * input_cloud->height;
    output_points.reserve(point_count);

    // 使用迭代器遍历点云数据
    sensor_msgs::PointCloud2Iterator<float> iter_x(*input_cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*input_cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*input_cloud, "z");

    // 如果没有 intensity，临时借用 "x" 字段初始化迭代器，保证循环中 ++iter 操作安全
    // 实际取值时会根据 has_intensity 判断，不会使用这个借用的值
    std::string intensity_field_name = has_intensity ? "intensity" : "x";
    sensor_msgs::PointCloud2Iterator<float> iter_intensity(*input_cloud, intensity_field_name);

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_intensity) {
        // 创建当前点并进行坐标转换
        const Point3Dd current_point = {static_cast<double>(*iter_x), static_cast<double>(*iter_y), static_cast<double>(*iter_z)};
        const Point3Dd trans_point = transformPoint(current_point, rot, t);

        output_points.emplace_back(agv_app_msgs::msg::PointXyzi()
                                .set__x(trans_point.x)
                                .set__y(trans_point.y)
                                .set__z(trans_point.z)
                                .set__intensity(has_intensity ? *iter_intensity : 0.0f));
    }
    return output_points;
}

inline agv_app_msgs::msg::AppData::_points_type processPolygon(const geometry_msgs::msg::PolygonStamped::SharedPtr& polygon_msg, const agv_service::msg::AgvPosition& agv_pos)
{
    const TranslationVector t = {agv_pos.x, agv_pos.y, agv_pos.z};      // 平移向量
    const RotationMatrix rot = eulerAnglesToRotationMatrix(agv_pos.yaw, agv_pos.pitch, agv_pos.roll);   // 计算旋转矩阵

    agv_app_msgs::msg::AppData::_points_type output_points;
    output_points.reserve(polygon_msg->polygon.points.size());

    for (const auto& p : polygon_msg->polygon.points) {
        // 创建当前点并进行坐标转换
        const Point3Dd current_point = {p.x, p.y, p.z};
        const Point3Dd trans_point = transformPoint(current_point, rot, t);

        output_points.emplace_back(agv_app_msgs::msg::PointXyzi()
                                        .set__x(trans_point.x)
                                        .set__y(trans_point.y)
                                        .set__z(trans_point.z));

    }
    return output_points;
}

}  // namespace agv_app_server