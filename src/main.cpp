// 项目相关头文件
#include "LogManager.hpp"
#include "agv_app_server/agv_app_server.hpp"

// ROS 2 相关头文件
#include "rclcpp/rclcpp.hpp"

// C++ 标准库头文件
#include <exception>
#include <iostream>

int main(int argc, char * argv[])
{
    LogManager::getInstance().initialize("/home/byd/log/agv_app_server", "agvappserver");
    LogManager::getInstance().getLogger()->info("log instance initialize success");
    try {
        // 1. 初始化 ROS 2 上下文
        rclcpp::init(argc, argv);

        // 2. 创建节点
        // 使用 make_shared 创建智能指针，确保资源管理安全
        auto node = std::make_shared<agv_app_server::AgvAppServer>();

        // 使用多线程执行器
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(node);

        // 3. 运行节点
        // spin 会阻塞在这里，直到收到 shutdown 信号 (Ctrl+C)
        // rclcpp::spin() 默认使用的是 SingleThreadedExecutor (单线程执行器)
        // 这意味着该节点下的所有回调函数（Subscription callbacks, Timer callbacks, Service callbacks 等）都会被添加到一个队列中，
        // 并由调用 spin 的那个线程（即主线程 main）串行地取出并执行。
        // rclcpp::spin(node);

        // 这里会开启多个线程（默认它是 CPU 核心数）并行处理回调
        executor.spin();

        // 4. 关闭上下文
        rclcpp::shutdown();
    }
    catch (const rclcpp::exceptions::RCLError & e) {
        // 捕获 ROS 特有的运行时错误
        std::cerr << "ROS 2 RCLError detected: " << e.what() << std::endl;
        LogManager::getInstance().getLogger()->critical("ROS 2 RCLError detected: {}", e.what());
        return 1;
    }
    catch (const std::exception & e) {
        // 捕获标准 C++ 异常
        std::cerr << "Standard exception detected: " << e.what() << std::endl;
        LogManager::getInstance().getLogger()->critical("Standard exception detected: {}", e.what());
        return 2;
    }
    catch (...) {
        // 捕获所有其他未知异常
        std::cerr << "Unknown exception detected." << std::endl;
        LogManager::getInstance().getLogger()->critical("Unknown exception detected");
        return 3;
    }

    return 0;
}