/**
 * @file task_parser.h
 * @author niejinci
 * @brief 解析应该遵循的规则：
    1. 动作如果需要绑定节点，默认绑定前面边的终点
    2. 动作如果需要绑定节点，并且前面没有节点的话就帮绑定后面边的起点。
    3. 动作需要区分即时动作还是非即时动作。
 * @version 0.1
 * @date 2025-12-22
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once

// 项目相关头文件
#include "agv_app_msgs/msg/node_detail.hpp"
#include "agv_app_msgs/msg/edge_detail.hpp"
#include "agv_app_msgs/msg/action_detail.hpp"
#include "agv_app_msgs/msg/task.hpp"
#include "agv_app_msgs/msg/app_request.hpp"

// 标准库头文件
#include <string>
#include <vector>
#include <queue>
#include <variant>
#include <map>
#include <optional>

namespace agv_app_server
{

struct TaskInfo {
    std::string order_id;
    std::vector<agv_app_msgs::msg::NodeDetail> nodes;
    std::vector<agv_app_msgs::msg::EdgeDetail> edges;
};

struct ExecutionPlan {
    double max_speed_ = 0.4;
    std::string map_id_;
    bool loop_ = false;
    std::vector<TaskInfo> tasks;
    std::vector<agv_app_msgs::msg::ActionDetail> instant_actions;
};

class TaskParser {
private:
    ExecutionPlan execution_plan_;
    int current_sequence_id_ = 0;

    std::string generateActionId();

public:
    TaskParser()
    {}
    ExecutionPlan parseTaskChain(const agv_app_msgs::msg::AppRequest::SharedPtr msg);
    std::queue<std::pair<std::string, size_t>> getExecutionQueue() const { return execution_queue_; }
    std::queue<std::pair<std::string, size_t>> execution_queue_; // <type, index> type 可以是 "action" 或 "task"

    struct ShowItem {
        enum Type { NAVIGATION, ACTION };
        // 类型：任务-0，动作-1
        Type type;
        // 动作名称，如果是任务，为空，例如： wait_di8
        std::string action;
        // 对于动作：空字符串，对于任务：节点路径信息
        std::string task;
        // 对于动作:动作名称，对于任务：order_id
        std::string name;
    };
    std::vector<ShowItem> getShowItems() const;

public:
    static std::string generateUUID();

private:
    struct ParsedItem {
        enum Type { NAVIGATION, ACTION };
        Type type;
        agv_app_msgs::msg::ActionDetail action_info;             // 只对ACTION类型有效
        std::vector<agv_app_msgs::msg::EdgeDetail> edges;    // 只对NAVIGATION类型有效
        int original_index;
    };

    void parseAllItems(const std::vector<agv_app_msgs::msg::Task>& tasks, std::vector<ParsedItem>& parsed_items);

    void parseNavigationEdges(const agv_app_msgs::msg::Task& nav_task, std::vector<agv_app_msgs::msg::EdgeDetail>& edges, int original_index);

    void processBindingLogic(std::vector<ParsedItem>& parsed_items);

    std::string findBindingNode(std::vector<ParsedItem>& parsed_items, size_t action_index);

    void generateExecutionPlan(std::vector<ParsedItem>& parsed_items);

    void generateTaskFromGroup(const std::vector<ParsedItem>& parsed_items, const std::vector<size_t>& group_indices);

    agv_app_msgs::msg::ActionDetail parseAction(const agv_app_msgs::msg::Task& task);

    void processInstantAction(const agv_app_msgs::msg::ActionDetail& action);

    std::vector<ShowItem> show_items_;
    void generateShowItems();
};

}   // namespace agv_app_server