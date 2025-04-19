#include "VDA5050.hpp"
#include "utils.hpp"
#include <iostream>
#include <chrono>
#include <stdexcept>

// 超时保护时间定义
constexpr std::chrono::seconds TIMEOUT_WAIT_ACTION = std::chrono::seconds(30);

// 允许偏差值定义
constexpr double ALLOWED_DEVIATION_THETA = 0.1; // 角度允许偏差（弧度）
constexpr double ALLOWED_DEVIATION_XY = 0.5;    // 位置允许偏差（米）

namespace deviceagv
{

    struct VDA5050Agv::Imp
    {
        // MQTT客户端
        std::unique_ptr<MqttClient> mqtt_;

        // AGV信息
        std::string version_;
        std::string sendVersion_;
        std::string manufacturer_;
        std::string serialNumber_;
        static std::string orderId_;
        static std::string actionId_;
        uint32_t orderUpdateId_ = 0;
        std::vector<vda5050::Edge> unreleaseEdges_;
        std::vector<vda5050::Node> unreleaseNodes_;
        static std::string headerIds_;
        int tempCur;
        int currentActionId = -1;
        // 实例状态标志
        static int connect_num;
        static std::mutex connect_mutex;
        static bool is_connected_;
        // 回调函数
        StateCallback stateCallback_;
        ConnectionCallback connectionCallback_;
        FactsheetCallback factsheetCallback_;
        // 当前状态
        vda5050::StateMessage currentState_;
        std::optional<vda5050::ConnectionState> connectionState_;
        // 节点管理相关
        struct NodeInfo
        {
            int nodeId;
            double x;
            double y;
            double theta;
            std::string mapId;
        };
        std::vector<NodeInfo> mapManager_{
            // {1, -0.16, 0.45, 1.61, "test1"},
            // {2, -0.25, 2.75, 1.61, "test1"},
            // {3, -0.41, 6.17, 1.62, "test1"}

            {1, 3.8, -0.77, 1.57, "HC"},
            {2, 7.83, 2.05, 3.141, "HC"},
            {3, 4.5, 2.05, 3.141, "HC"},
            {4, 7.83, 4.97, 3.132, "HC"},
            {5, 4.5, 4.97, 3.141, "HC"},
            {6, 7.2, 7.58, 3.141, "HC"},
            {7, 4.5, 7.58, 3.124, "HC"},
            {8, 18.8, 5.74, -1.58, "HC"},
            {9, 18.8, 3.05, -1.58, "HC"},
            {10, 3.8, 0.77, 1.57, "HC"},
            {11, 3.8, 6.77, 1.57, "HC"}};

        // 连接监控相关
        std::atomic<bool> is_monitoring_{false};
        std::thread monitor_thread_;
        std::mutex monitor_mutex_;
        std::condition_variable monitor_cv_;
        const std::chrono::seconds MONITOR_INTERVAL{5}; // 监控间隔5秒
        const int MAX_RECONNECT_ATTEMPTS{3};            // 最大重连次数

        // 全局监控相关
        static std::mutex global_monitor_mutex;
        static bool global_monitoring;
    };
    // 初始化静态成员
    int VDA5050Agv::Imp::connect_num = 0;
    std::mutex VDA5050Agv::Imp::connect_mutex;
    std::string VDA5050Agv::Imp::headerIds_ = "1";
    std::string VDA5050Agv::Imp::orderId_ = "1";
    std::string VDA5050Agv::Imp::actionId_ = "1";
    bool VDA5050Agv::Imp::is_connected_ = false; // 添加静态成员变量跟踪连接状态
    std::mutex VDA5050Agv::Imp::global_monitor_mutex;
    bool VDA5050Agv::Imp::global_monitoring = false;

    VDA5050Agv::VDA5050Agv(const std::string &broker, int port,
                           const std::string &manufacturer,
                           const std::string &serialNumber,
                           const std::string &version,
                           const std::string &username,
                           const std::string &password)
        : imp_(new Imp)
    {
        try
        {
            // 设置MQTT认证信息
            imp_->mqtt_ = std::make_unique<MqttClient>(broker, port);
            imp_->manufacturer_ = manufacturer;
            imp_->serialNumber_ = serialNumber;
            imp_->version_ = version;
            if (version[0] == 'v' || version[0] == 'V')
            {
                imp_->sendVersion_ = version.substr(1);
            }
            else
            {
                imp_->sendVersion_ = version;
            }
            imp_->tempCur = -1;
            setLastStationInt(-1);
            setFinialGoalInt(-1);
            DeviceAgv::moveToStationInt(-1);

            if (!username.empty() && !password.empty())
            {
                imp_->mqtt_->setCredentials(username, password);
            }
        }
        catch (const std::exception &e)
        {
            throw std::runtime_error("初始化VDA5050Agv实例失败: " + std::string(e.what()));
        }
    }

    VDA5050Agv::~VDA5050Agv()
    {
        try
        {
            stopConnectionMonitor(); // 停止监控线程

            std::lock_guard<std::mutex> lock(imp_->connect_mutex);
            imp_->connect_num--;

            // 只有当所有实例都被销毁时才断开连接
            if (imp_->connect_num <= 0 && imp_->is_connected_)
            {
                disconnect();
                imp_->is_connected_ = false;
                imp_->connect_num = 0; // 重置连接计数
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << "析构函数中发生错误: " << e.what() << std::endl;
        }
    }

    auto VDA5050Agv::init() -> void
    {
        try
        {
            std::lock_guard<std::mutex> lock(imp_->connect_mutex);
            imp_->connect_num++;
            // 如果已经连接，直接返回
            if (imp_->is_connected_)
            {
                return;
            }
            // 尝试连接
            if (!connect())
            {
                if (imp_->mqtt_->reconnect(imp_->MAX_RECONNECT_ATTEMPTS, 5))
                {
                    imp_->is_connected_ = true;
                    subscribeToTopics();
                    std::cerr << "重连成功" << std::endl;
                }
                else
                {
                    std::cerr << "重连失败，已达到最大重试次数" << std::endl;
                    imp_->connect_num--;
                    throw std::runtime_error("MQTT连接失败");
                }
            }
            // imp_->is_connected_ = true;
            subscribeToTopics();
            update();
            startConnectionMonitor();
            return;
        }
        catch (const std::exception &e)
        {
            throw std::runtime_error("初始化失败: " + std::string(e.what()));
        }
    }

    bool VDA5050Agv::connect()
    {
        try
        {
            if (imp_->is_connected_)
            {
                return true; // 如果已经连接，直接返回
            }
            imp_->mqtt_->connect();
            imp_->mqtt_->setClientId(imp_->manufacturer_ + "_" + imp_->serialNumber_);
            imp_->is_connected_ = true;
            return true;
        }
        catch (const std::exception &e)
        {
            std::cerr << "连接错误: " << e.what() << std::endl;
            return false;
        }
    }

    void VDA5050Agv::disconnect()
    {
        if (imp_->mqtt_ && imp_->is_connected_)
        {
            imp_->mqtt_->disconnect();
            imp_->is_connected_ = false;
        }
    }

    bool VDA5050Agv::isConnected() const
    {
        return imp_->mqtt_ && imp_->is_connected_;
    }

    bool VDA5050Agv::sendOrder(const vda5050::OrderMessage &order)
    {
        try
        {
            if (!isConnected())
            {
                throw std::runtime_error("MQTT未连接");
            }

            std::string topic = getTopic("order");
            if (!imp_->mqtt_->publish(topic, order.toJson()))
            {
                throw std::runtime_error("发送订单消息失败");
            }

            std::cout << "\n=== 发送订单消息 ===" << std::endl;
            std::cout << "话题: " << topic << std::endl;
            std::cout << "消息内容: " << order.toJson() << std::endl;
            std::cout << "==================\n"
                      << std::endl;
            return true;
        }
        catch (const std::exception &e)
        {
            std::cerr << "发送订单错误: " << e.what() << std::endl;
            return false;
        }
    }

    bool VDA5050Agv::sendInstantActions(const vda5050::InstantActionsMessage &actions)
    {
        try
        {
            if (!isConnected())
            {
                throw std::runtime_error("MQTT未连接");
            }

            std::string topic = getTopic("instantActions");
            if (!imp_->mqtt_->publish(topic, actions.toJson()))
            {
                throw std::runtime_error("发送即时动作消息失败");
            }

            // 输出订单消息和话题
            std::cout << "\n=== 发送动作消息 ===" << std::endl;
            std::cout << "话题: " << topic << std::endl;
            std::cout << "消息内容: " << actions.toJson() << std::endl;
            std::cout << "==================\n"
                      << std::endl;

            imp_->currentActionId = std::stoi(actions.actions.back().actionId);
            const auto timeout = std::chrono::steady_clock::now() + TIMEOUT_WAIT_ACTION;
            while (imp_->currentActionId != -1)
            {
                if (std::chrono::steady_clock::now() > timeout)
                {
                    std::cerr << "等待动作完成更新超时" << std::endl;
                    std::cout << imp_->currentState_.actionStates.back().actionId << " " << std::to_string(imp_->currentActionId) << std::endl;
                    std::cout << imp_->currentState_.actionStates.back().actionStatus << std::endl;
                    return false;
                }
            }
            return true;
        }
        catch (const std::exception &e)
        {
            std::cerr << "发送即时动作错误: " << e.what() << std::endl;
            return false;
        }
    }

    void VDA5050Agv::setStateCallback(StateCallback callback)
    {
        imp_->stateCallback_ = std::move(callback);
    }

    void VDA5050Agv::setConnectionCallback(ConnectionCallback callback)
    {
        imp_->connectionCallback_ = std::move(callback);
    }

    void VDA5050Agv::setFactsheetCallback(FactsheetCallback callback)
    {
        imp_->factsheetCallback_ = std::move(callback);
    }

    std::string VDA5050Agv::getTopic(const std::string &type) const
    {
        return "VDA/" + imp_->version_ + "/" + imp_->manufacturer_ + "/" + imp_->serialNumber_ + "/" + type;
    }

    void VDA5050Agv::subscribeToTopics()
    {
        // 订阅状态主题
        std::string stateTopic = "VDA/" + imp_->version_ + "/" + imp_->manufacturer_ + "/" + imp_->serialNumber_ + "/state";
        setStateCallback([this](const vda5050::StateMessage &state)
                         { this->handleStateUpdate(state); });
        imp_->mqtt_->subscribe(stateTopic, 1, [this](const std::string &payload)
                               { this->handleStateMessage(payload); });

        // 订阅连接主题
        std::string connectionTopic = "VDA/" + imp_->version_ + "/" + imp_->manufacturer_ + "/" + imp_->serialNumber_ + "/connection";
        setConnectionCallback([this](const vda5050::ConnectionMessage &msg)
                              { this->handleConnectionState(msg.connectionState); });
        imp_->mqtt_->subscribe(connectionTopic, 1, [this](const std::string &payload)
                               { this->handleConnectionMessage(payload); });

        // 订阅factsheet主题
        std::string factsheetTopic = "VDA/" + imp_->version_ + "/" + imp_->manufacturer_ + "/" + imp_->serialNumber_ + "/factsheet";
        imp_->mqtt_->subscribe(factsheetTopic, 1, [this](const std::string &payload)
                               { this->handleFactsheetMessage(payload); });
    }

    void VDA5050Agv::update()
    {
        try
        {
            if (!isConnected())
            {
                std::cerr << "MQTT未连接，无法更新状态" << std::endl;
                return;
            }
            std::cout << "更新状态" << std::endl;
            if (!sendInstantActions(createInstantActionsMessage("stateRequest")))
            {
                std::cerr << "发送状态请求失败" << std::endl;
                return;
            }
            return;
        }
        catch (const std::exception &e)
        {
            std::cerr << "更新状态错误: " << e.what() << std::endl;
            return;
        }
        catch (...)
        {
            std::cerr << "更新状态时发生未知错误" << std::endl;
            return;
        }
    }

    void VDA5050Agv::handleStateMessage(const std::string &payload)
    {
        try
        {
            auto stateMsg = vda5050::MessageProcessor::parseStateMessage(payload);
            if (stateMsg)
            {
                imp_->currentState_ = *stateMsg;
                if (imp_->stateCallback_)
                {
                    imp_->stateCallback_(*stateMsg);
                }
            }
            else
            {
                std::cerr << "状态消息解析失败" << std::endl;
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << "处理连接消息时出错: " << e.what() << std::endl;
        }
    }

    void VDA5050Agv::handleConnectionState(const vda5050::ConnectionState &state)
    {
        if (state == vda5050::ConnectionState::OFFLINE)
        {
            std::cerr << "收到断开连接消息，尝试重连" << std::endl;
            // if (imp_->mqtt_->reconnect(imp_->MAX_RECONNECT_ATTEMPTS, 5))
            // {
            //     imp_->is_connected_ = true;
            //     subscribeToTopics();
            //     std::cerr << "重连成功" << std::endl;
            // }
            // else
            // {
            //     imp_->is_connected_ = false;
            //     throw std::runtime_error("重连失败，已达到最大重试次数");
            // }
            // imp_->is_connected_ = false;
        }
        else if (state == vda5050::ConnectionState::ONLINE)
        {
            // std::cerr << "收到连接成功消息" << std::endl;
            imp_->is_connected_ = true;
        }
    }

    void VDA5050Agv::handleConnectionMessage(const std::string &payload)
    {
        try
        {
            auto connMsg = vda5050::MessageProcessor::parseConnectionMessage(payload);
            if (connMsg)
            {
                imp_->connectionState_ = connMsg->connectionState;

                if (imp_->connectionCallback_)
                {
                    imp_->connectionCallback_(*connMsg);
                }
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << "处理连接消息时出错: " << e.what() << std::endl;
        }
    }

    void VDA5050Agv::handleFactsheetMessage(const std::string &payload)
    {
        try
        {
            auto connMsg = vda5050::MessageProcessor::parseVisualizationMessage(payload);
            if (connMsg)
            {
                if (imp_->factsheetCallback_)
                {
                    imp_->factsheetCallback_(*connMsg);
                }
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << "处理factsheet消息时出错: " << e.what() << std::endl;
        }
    }

    void VDA5050Agv::handleStateUpdate(const vda5050::StateMessage &state)
    {
        try
        {
            // 更新位置信息
            setX(state.agvPosition.value().x);
            setY(state.agvPosition.value().y);
            setTheta(state.agvPosition.value().theta);
            setBatteryPercent(static_cast<int>(state.batteryState.batteryCharge));

            // 设置错误消息
            if (state.errors.empty())
            {
                setErrMsg("");
            }
            else
            {
                const auto &lastError = state.errors.back();
                std::string errorMsg = "error level: " + lastError.errorLevel + "\n";
                errorMsg += "errorDescription: " + lastError.errorDescription.value_or("") + "\n";
                if (!lastError.errorReferences.empty())
                {
                    errorMsg += "error code: " + lastError.errorReferences.back().referenceValue + "\n";
                }
                setErrMsg(errorMsg);
            }

            setAgvMap(state.agvPosition.value().mapId);

            // 更新站点信息
            int currentName = positionToName(x(), y());
            setCurrentStationInt(currentName);

            // 更新上一个站点
            if (currentName != -1)
            {
                imp_->tempCur = currentName;
            }
            else
            {
                setLastStationInt(imp_->tempCur);
            }

            // 更新路径信息
            if (!state.nodeStates.empty())
            {
                std::vector<int> restpath;
                if (currentStationInt() == std::stoi(state.nodeStates[0].nodeId))
                {
                    // 到达点上在点上旋转
                    if (state.nodeStates.size() > 1)
                    {
                        // 实际还有下一个点
                        setNextStationInt(std::stoi(state.nodeStates[1].nodeId));
                        restpath.reserve(state.nodeStates.size() - 1);
                        for (size_t i = 1; i < state.nodeStates.size(); ++i)
                        {
                            restpath.push_back(std::stoi(state.nodeStates[i].nodeId));
                        }
                    }
                    else
                    {
                        // 实际没有下一个点
                        setNextStationInt(-1);
                        restpath = {};
                    }
                }
                else
                {
                    // 在边上未到达点
                    setNextStationInt(std::stoi(state.nodeStates[0].nodeId));
                    restpath.reserve(state.nodeStates.size());
                    for (const auto &node : state.nodeStates)
                    {
                        restpath.push_back(std::stoi(node.nodeId));
                    }
                }
                setRestPathInt(restpath);
            }
            else
            {
                // 到达终点
                setNextStationInt(-1);
                setRestPathInt({});
            }

            // 更新移动状态
            if (state.paused.value_or(false) == true)
            {
                setAgvMoveState(AgvMoveState::kPaused);
            }
            else
            {
                if (state.driving == true)
                {
                    setAgvMoveState(AgvMoveState::kMoving);
                }
                else
                {
                    setAgvMoveState(AgvMoveState::kIdle);
                }
            }

            // 检查动作完成状态
            if (imp_->currentActionId != -1)
            {
                for (int i = state.actionStates.size() - 1; i >= 0; --i)
                {
                    if (state.actionStates[i].actionId == std::to_string(imp_->currentActionId) &&
                        (state.actionStates[i].actionStatus == "FINISHED" ||
                         state.actionStates[i].actionStatus == "RUNNING"))
                    {
                        imp_->currentActionId = -1;
                        std::cout << "动作完成" << std::endl;
                    }
                }
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << "更新状态时发生错误: " << e.what() << std::endl;
        }
        catch (...)
        {
            std::cerr << "更新状态时发生未知错误" << std::endl;
        }
    }

    // 订单消息创建辅助函数实现
    vda5050::OrderMessage VDA5050Agv::createOrderMessage(bool updateOrderId)
    {
        vda5050::OrderMessage orderMsg;
        orderMsg.headerId = std::stoi(Imp::headerIds_);
        Imp::headerIds_ = std::to_string(std::stoi(Imp::headerIds_) + 1);
        orderMsg.timestamp = getCurrentIsoTimestamp();
        orderMsg.version = imp_->sendVersion_;
        orderMsg.manufacturer = imp_->manufacturer_;
        orderMsg.serialNumber = imp_->serialNumber_;
        if (updateOrderId)
        {
            imp_->orderUpdateId_++;
        }
        else
        {
            Imp::orderId_ = std::to_string(std::stoi(Imp::orderId_) + 1);
            imp_->orderUpdateId_ = 0;
        }
        orderMsg.orderId = Imp::orderId_;
        orderMsg.orderUpdateId = imp_->orderUpdateId_;
        return orderMsg;
    }

    vda5050::Edge VDA5050Agv::createEdge(std::string edgeId, const std::string &startNodeId, const std::string &endNodeId, int sequenceId, bool release)
    {
        vda5050::Edge edge;
        edge.edgeId = edgeId;
        edge.sequenceId = sequenceId;
        edge.released = release;
        edge.startNodeId = startNodeId;
        edge.endNodeId = endNodeId;
        edge.edgeDescription = "";
        edge.orientation = 0;

        /*
        贝塞尔
        edge.trajectory = vda5050::Trajectory{};
        edge.trajectory->controlPoints.push_back(vda5050::ControlPoint{x[0],y[0],w[0]});
        edge.trajectory->controlPoints.push_back(vda5050::ControlPoint{x[1],y[1],w[1]});
        edge.trajectory->degree = 1;
        edge.trajectory->knotVector = {};
        */

        return edge;
    }

    vda5050::NodePosition VDA5050Agv::createNodePosition(const std::string &mapId, double x, double y, double theta)
    {
        vda5050::NodePosition pos;
        pos.x = x;
        pos.y = y;
        pos.theta = theta;
        pos.mapId = mapId;
        pos.allowedDeviationTheta = ALLOWED_DEVIATION_THETA;
        pos.allowedDeviationXY = ALLOWED_DEVIATION_XY;
        return pos;
    }

    vda5050::Node VDA5050Agv::createNode(const std::string &nodeId, double x, double y, std::string mapId, double theta)
    {
        vda5050::Node node;
        node.nodeId = nodeId;
        node.nodePosition = createNodePosition(mapId, x, y, theta);
        return node;
    }

    vda5050::InstantActionsMessage VDA5050Agv::createInstantActionsMessage(const std::string &actionType, const std::string &blockingType)
    {
        vda5050::InstantActionsMessage msg;
        msg.headerId = std::stoi(Imp::headerIds_);
        msg.timestamp = getCurrentIsoTimestamp();
        msg.version = imp_->sendVersion_;
        msg.manufacturer = imp_->manufacturer_;
        msg.serialNumber = imp_->serialNumber_;

        vda5050::Action action;
        action.actionId = Imp::actionId_;
        action.actionType = actionType;
        action.blockingType = blockingType;
        msg.actions.push_back(action);

        Imp::headerIds_ = std::to_string(std::stoi(Imp::headerIds_) + 1);
        Imp::actionId_ = std::to_string(std::stoi(Imp::actionId_) + 1);

        return msg;
    }

    auto VDA5050Agv::setPathInt(const std::vector<int> &nodepath, bool append) -> void
    {
        try
        {
            if (!append)
            {
                update();
                if (currentStationInt() == -1)
                {
                    throw std::runtime_error("当前不在站点上，设置路径失败");
                }
                if (nodepath.empty())
                {
                    throw std::invalid_argument("路径不能为空，设置路径失败");
                }
                if (nodepath[0] != currentStationInt())
                {
                    throw std::runtime_error("起始点不是当前站点：" + std::to_string(currentStationInt()) + "，设置路径失败");
                }
            }

            std::vector<vda5050::Node> path;
            for (int nodeId : nodepath)
            {
                vda5050::Node node = nameToNode(nodeId);
                path.push_back(node);
            }

            if (!restPathInt().empty())
            {
                std::cout << "已有路径，更新路径" << std::endl;
                int temp = currentStationInt();
                cancelMove();
                if(append){
                    DeviceAgv::moveToStationInt(temp);
                }
            }
            DeviceAgv::setPathInt(nodepath);

            imp_->unreleaseNodes_.clear();
            imp_->unreleaseEdges_.clear();
            setFinialGoalInt(std::stoi(path.back().nodeId));

            // 创建新的订单消息
            auto orderMsg = createOrderMessage(false);
            // 处理节点和边
            for (size_t i = 0; i < path.size(); ++i)
            {
                // 添加节点
                vda5050::Node pathNode = path[i];
                pathNode.sequenceId = static_cast<uint32_t>(i * 2); // 节点使用偶数sequenceId，从2开始
                pathNode.released = i == 0 ? true : false;
                orderMsg.nodes.push_back(pathNode);
                imp_->unreleaseNodes_.push_back(pathNode);

                // 如果不是最后一个节点，添加边
                if (i < path.size() - 1)
                {
                    vda5050::Edge edge = createEdge(std::to_string(i + 1), path[i].nodeId, path[i + 1].nodeId,
                                                    static_cast<int>(i * 2 + 1), false);
                    imp_->unreleaseEdges_.push_back(edge);
                    orderMsg.edges.push_back(edge);
                }
            }
            if (!sendOrder(orderMsg))
            {
                throw std::runtime_error("发送路径订单失败");
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << "设置路径错误: " << e.what() << std::endl;
            // throw;
        }
    }

    auto VDA5050Agv::moveToStationInt(const int &nodename) -> void
    {
        try
        {
            // 检查目标节点是否在剩余路径中
            bool found = false;
            for (const auto &node : imp_->unreleaseNodes_)
            {
                if (std::stoi(node.nodeId) == nodename)
                {
                    found = true;
                    break;
                }
            }
            
            if (!found)
            {
                throw std::runtime_error("目标节点 " + std::to_string(nodename) + " 不在剩余路径中");
            }

            auto orderMsg = createOrderMessage(true);
            bool nodeover = false;
            bool edgeover = false;
            std::vector<vda5050::Node> newRestPath;
            std::vector<vda5050::Edge> newEdgesPath;
            std::string name = std::to_string(nodename);
            DeviceAgv::moveToStationInt(nodename);

            try
            {
                // 处理所有节点
                for (size_t i = 0; i < imp_->unreleaseNodes_.size(); ++i)
                {
                    vda5050::Node currentNode = imp_->unreleaseNodes_[i];

                    if (currentNode.nodeId == name && nodeover == false)
                    {
                        nodeover = true;
                        currentNode.released = true;
                        newRestPath.push_back(currentNode);
                        orderMsg.nodes.push_back(currentNode);
                    }
                    else if (nodeover)
                    {
                        // 目标节点之后的节点
                        currentNode.released = false;
                        newRestPath.push_back(currentNode);
                        orderMsg.nodes.push_back(currentNode);
                    }
                    else
                    {
                        // 目标节点之前的节点
                        currentNode.released = true;
                        orderMsg.nodes.push_back(currentNode);
                    }
                }

                // 处理边
                for (size_t i = 0; i < imp_->unreleaseEdges_.size(); ++i)
                {
                    vda5050::Edge edge = imp_->unreleaseEdges_[i];
                    if (edge.startNodeId == name)
                    {
                        edgeover = true;
                        edge.released = false;
                        newEdgesPath.push_back(edge);
                        orderMsg.edges.push_back(edge);
                    }
                    else if (edgeover)
                    {
                        // 目标节点之后的边
                        edge.released = false;
                        newEdgesPath.push_back(edge);
                        orderMsg.edges.push_back(edge);
                    }
                    else
                    {
                        // 目标节点之前的边
                        edge.released = true;
                        orderMsg.edges.push_back(edge);
                    }
                }
            }
            catch (const std::exception &e)
            {
                throw std::runtime_error("处理路径节点和边失败: " + std::string(e.what()));
            }

            // 更新路径
            imp_->unreleaseNodes_ = std::move(newRestPath);
            imp_->unreleaseEdges_ = std::move(newEdgesPath);

            // 发送订单消息
            if (!sendOrder(orderMsg))
            {
                throw std::runtime_error("发送移动订单失败");
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << "移动错误: " << e.what() << std::endl;
            //throw;
        }
    }

auto VDA5050Agv::goOn() -> void
{
    sendInstantActions(createInstantActionsMessage("stopPause", "NONE"));
}

auto VDA5050Agv::pause() -> void
{
    sendInstantActions(createInstantActionsMessage("startPause", "HARD"));
}

auto VDA5050Agv::cancelMove() -> void
{
    try
    {
        if (currentStationInt() == -1)
        {
            throw std::runtime_error("当前不在站点上");
        }
        imp_->unreleaseNodes_.clear();
        imp_->unreleaseEdges_.clear();
        setFinialGoalInt(-1);
        DeviceAgv::moveToStationInt(-1);
        setNextStationInt(-1);
        DeviceAgv::setPathInt({});
        setRestPathInt({});
        setErrMsg("");
        sendInstantActions(createInstantActionsMessage("cancelOrder", "HARD"));
    }
    catch (const std::exception &e)
    {
        std::cerr << "取消订单失败: " << e.what() << std::endl;
        // throw;
    }
}

auto VDA5050Agv::eStop() -> void
{
    throw std::runtime_error("错误调用: VDA5050不支持");
}

auto VDA5050Agv::cancelEStop() -> void
{
    throw std::runtime_error("错误调用: VDA5050不支持");
}

auto VDA5050Agv::appendPathInt(const std::vector<int> &path) -> void
{
    try
    {
        update();
        if (currentStationInt() == -1)
        {
            throw std::runtime_error("当前不在站点上，追加路径失败");
        }
        if (path.empty())
        {
            throw std::invalid_argument("追加路径不能为空，追加路径失败");
        }
        if (path[0] != finialGoalInt())
        {
            throw std::runtime_error("追加路径与原路径不连续，追加路径失败");
        }
        if (currentStationInt() != currentGoalInt())
        {
            throw std::runtime_error("当前路径未结束，追加路径失败");
        }
        auto oldpath = pathInt();
        oldpath.insert(oldpath.end(), path.begin() + 1, path.end());
        DeviceAgv::setPathInt(oldpath);

        auto currentPath = restPathInt();
        currentPath.insert(currentPath.begin(), currentStationInt());
        currentPath.insert(currentPath.end(), path.begin() + 1, path.end());

        setPathInt(currentPath, true);
    }
    catch (const std::exception &e)
    {
        std::cerr << "追加路径错误: " << e.what() << std::endl;
        // throw;
    }
}

auto VDA5050Agv::nameToNode(const int &name) -> vda5050::Node
{
    /*
    std::string devicename{};
    std::set<std::string> stationname{};
    auto &map = ControlServer::instance().mapManager().map(agvMap());
    for (const auto &devicestation : map.deviceStations()) {
        for(const auto & station: devicestation.stations){
            if(station.stationid == name){
                Node node = createNode(name, station.x, station.y, std::to_string(devicestation.mapid), station.theta);
                return std::vector<Node>{node};
            }
        }
    }
    return std::vector<Node>();
    */
    vda5050::Node result;
    for (const auto &nodeInfo : imp_->mapManager_)
    {
        if (nodeInfo.nodeId == name)
        {
            result = createNode(std::to_string(name), nodeInfo.x, nodeInfo.y, nodeInfo.mapId, nodeInfo.theta);
            return result;
        }
    }
    throw std::runtime_error("找不到节点: " + std::to_string(name));
}

auto VDA5050Agv::positionToName(const double &posX, const double &posY) -> int
{
    for (const auto &nodeInfo : imp_->mapManager_)
    {
        double dx = std::abs(nodeInfo.x - posX);
        double dy = std::abs(nodeInfo.y - posY);
        if (dx < ALLOWED_DEVIATION_XY && dy < ALLOWED_DEVIATION_XY)
        {
            return nodeInfo.nodeId;
        }
    }
    return -1;
}

void VDA5050Agv::setHeaderIds(const std::string &headerIds)
{
    Imp::headerIds_ = headerIds;
}

void VDA5050Agv::setOrderId(const std::string &orderId)
{
    Imp::orderId_ = orderId;
}

void VDA5050Agv::setActionId(const std::string &actionId)
{
    Imp::actionId_ = actionId;
}

// 添加监控线程的启动和停止函数
void VDA5050Agv::startConnectionMonitor()
{
    // 尝试获取全局监控权
    std::unique_lock<std::mutex> global_lock(imp_->global_monitor_mutex);
    if (imp_->global_monitoring)
    {
        imp_->monitor_cv_.wait(global_lock, [this](){ 
            return !imp_->global_monitoring && !imp_->is_monitoring_; });
    }

    if (!imp_->is_monitoring_)
    {
        imp_->global_monitoring = true;
        imp_->is_monitoring_ = true;
        imp_->monitor_thread_ = std::thread([this](){
            while (imp_->is_monitoring_) {
                {
                    std::unique_lock<std::mutex> lock(imp_->monitor_mutex_);
                    imp_->monitor_cv_.wait_for(lock, imp_->MONITOR_INTERVAL, 
                    [this]() { return !imp_->is_monitoring_; });      
                    if (!imp_->is_monitoring_) {
                        break;
                    }
                }
                if (!imp_->mqtt_->isConnected()) {
                    std::cerr << "检测到连接断开，尝试重连..." << std::endl;
                    if (imp_->mqtt_->reconnect(imp_->MAX_RECONNECT_ATTEMPTS, 5)) {
                        imp_->is_connected_ = true;
                        subscribeToTopics();
                        std::cerr << "重连成功" << std::endl;
                    } else {
                        imp_->is_connected_ = false;
                        throw std::runtime_error("重连失败，已达到最大重试次数" );
                    }
                }
            }
        // 释放全局监控权
        {
            std::lock_guard<std::mutex> lock(imp_->global_monitor_mutex);
            imp_->global_monitoring = false;
        }
        imp_->monitor_cv_.notify_all(); });
    }
}

void VDA5050Agv::stopConnectionMonitor()
{
    if (!imp_->is_monitoring_)
    {
        return;
    }

    imp_->is_monitoring_ = false;
    imp_->monitor_cv_.notify_all();

    if (imp_->monitor_thread_.joinable())
    {
        imp_->monitor_thread_.join();
    }
}

} // namespace vda5050