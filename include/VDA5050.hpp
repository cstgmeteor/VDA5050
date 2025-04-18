#pragma once

#include "VDA5050_json.hpp"
#include "mqtt.hpp"
#include "device_agv.hpp"
#include <string>
#include <functional>
#include <memory>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <atomic>

namespace deviceagv {
    //using namespace vda5050;
    class VDA5050Agv : public DeviceAgv {
    public:
        // 控制接口
        auto setPathInt(const std::vector<int>& path, bool append = false, int goal = -1)->void override;
        auto moveToStationInt(const int& name)->void override;
        auto appendPathInt(const std::vector<int>& path)->void override;

        auto goOn() -> void override;
        auto pause() -> void override;
        auto cancelMove() -> void override;
        auto eStop() -> void override;
        auto cancelEStop() -> void override;

        auto init()->void override;
        auto update()->void override;

        // 设置headerIds和orderId
        void setHeaderIds(const std::string& headerIds);
        void setOrderId(const std::string& orderId);
        void setActionId(const std::string& actionId);

    private:
        struct Imp;
        std::unique_ptr<Imp> imp_;

        // 连接状态处理
        void handleConnectionState(const vda5050::ConnectionState& state);

        // 连接监控相关
        void startConnectionMonitor();
        void stopConnectionMonitor();

        // 获取主题名称
        std::string getTopic(const std::string& type) const;

        // 订阅主题
        void subscribeToTopics();

        // 消息处理函数
        void handleStateMessage(const std::string& payload);
        void handleConnectionMessage(const std::string& payload);
        void handleFactsheetMessage(const std::string& payload);
        void handleStateUpdate(const vda5050::StateMessage& state);

        // 订单消息创建辅助函数
        vda5050::OrderMessage createOrderMessage(bool updateOrderId = false);
        vda5050::NodePosition createNodePosition(const std::string& mapId = "Map1", double x = 0.0, double y = 0.0, double theta = 0.0);
        vda5050::InstantActionsMessage createInstantActionsMessage(const std::string& actionType, const std::string& blockingType = "NONE");

        // 创建节点和边的辅助函数
        vda5050::Node createNode(const std::string& nodeId, double x, double y, std::string mapId, double theta);
        vda5050::Edge createEdge(std::string edgeId, const std::string& startNodeId, const std::string& endNodeId, int sequenceId, bool release);

        // 连接相关
        bool connect();
        void disconnect();
        bool isConnected() const;

        // 设置MQTT认证信息
        void setMqttCredentials(const std::string& username, const std::string& password);

        // 发送消息接口
        bool sendOrder(const vda5050::OrderMessage& order);
        bool sendInstantActions(const vda5050::InstantActionsMessage& actions);

        // 回调函数类型定义
        using StateCallback = std::function<void(const vda5050::StateMessage&)>;
        using ConnectionCallback = std::function<void(const vda5050::ConnectionMessage&)>;
        using FactsheetCallback = std::function<void(const vda5050::VisualizationMessage&)>;

        // 设置回调函数
        void setStateCallback(StateCallback callback);
        void setConnectionCallback(ConnectionCallback callback);
        void setFactsheetCallback(FactsheetCallback callback);

        auto nameToNode(const int &name) -> vda5050::Node;
        auto positionToName(const double &, const double &) -> int;

    public:
        // 构造函数
        VDA5050Agv(const std::string& broker, int port,
            const std::string& manufacturer,
            const std::string& serialNumber,
            const std::string& version = "2.0.0",
            const std::string& username = "admin",
            const std::string& password = "admin");

        // 析构函数
        ~VDA5050Agv();

    };
} // namespace vda5050 