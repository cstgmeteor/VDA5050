#pragma once

#include <string>
#include <vector>
#include <optional>
#include <memory>
#include <functional>
#include "json.hpp"

using json = nlohmann::json;

namespace vda5050 {
    // 操作模式枚举
    enum class OperatingMode {
        AUTOMATIC,
        SEMIAUTOMATIC,
        MANUAL,
        SERVICE,
        TEACHIN
    };

    // 连接状态枚举
    enum class ConnectionState {
        ONLINE,
        OFFLINE,
        CONNECTIONBROKEN
    };

    // 消息类型枚举
    enum class MessageType {
        CONNECTION,
        ORDER,
        STATE,
        INSTANTACTIONS,
        VISUALIZATION,
        UNKNOWN
    };

    // 基础消息类
    class VDA5050Message {
    public:
        virtual void fromJson(const std::string& jsonStr);
        virtual std::string toJson() const;
        virtual void fromJsonObj(const json& jsonObj);
        virtual json toJsonObj() const;
        virtual ~VDA5050Message() = default;

        std::optional<uint32_t> headerId;
        std::string timestamp;
        std::string version;
        std::string manufacturer;
        std::string serialNumber;
    };

    // 通用数据结构
    struct NodePosition {
        double x;
        double y;
        std::optional<double> theta;
        std::string mapId;
        std::optional<std::string> mapDescription;
        std::optional<double> allowedDeviationXY;
        std::optional<double> allowedDeviationTheta;

        void fromJson(const json& j);
        json toJson() const;
    };

    struct ActionParameter {
        std::string key;
        json value;

        void fromJson(const json& j);
        json toJson() const;
    };

    struct Action {
        std::string actionId;
        std::string actionType;
        std::optional<std::string> actionDescription;
        std::string blockingType;
        std::optional<std::vector<ActionParameter>> actionParameters;

        void fromJson(const json& j);
        json toJson() const;
    };

    // 轨迹相关结构
    struct ControlPoint {
        double x;
        double y;
        std::optional<double> weight;

        void fromJson(const json& j);
        json toJson() const;
    };

    struct Trajectory {
        int degree;
        std::vector<double> knotVector;
        std::vector<ControlPoint> controlPoints;

        void fromJson(const json& j);
        json toJson() const;
    };

    // 走廊相关结构
    struct Corridor {
        double leftWidth;
        double rightWidth;
        std::optional<std::string> corridorRefPoint;

        void fromJson(const json& j);
        json toJson() const;
    };

    struct Node {
        std::string nodeId;
        uint32_t sequenceId;
        std::optional<std::string> nodeDescription;
        bool released;
        std::optional<NodePosition> nodePosition;
        std::vector<Action> actions;

        void fromJson(const json& j);
        json toJson() const;
    };

    struct Edge {
        std::string edgeId;
        uint32_t sequenceId;
        std::optional<std::string> edgeDescription;
        bool released;
        std::string startNodeId;
        std::string endNodeId;
        std::optional<double> maxSpeed;
        std::optional<double> maxHeight;
        std::optional<double> minHeight;
        std::optional<double> orientation;
        std::optional<std::string> orientationType;
        std::optional<std::string> direction;
        std::optional<bool> rotationAllowed;
        std::optional<double> maxRotationSpeed;
        std::optional<double> length;
        std::optional<Trajectory> trajectory;
        std::optional<Corridor> corridor;
        std::vector<Action> actions;

        void fromJson(const json& j);
        json toJson() const;
    };

    struct ActionState {
        std::string actionId;
        std::optional<std::string> actionType;
        std::optional<std::string> actionDescription;
        std::string actionStatus;
        std::optional<std::string> resultDescription;

        void fromJson(const json& j);
        json toJson() const;
    };

    struct BatteryState {
        double batteryCharge;
        std::optional<double> batteryVoltage;
        std::optional<double> batteryHealth;
        bool charging;
        std::optional<double> reach;

        void fromJson(const json& j);
        json toJson() const;
    };

    struct ErrorReference {
        std::string referenceKey;
        std::string referenceValue;

        void fromJson(const json& j);
        json toJson() const;
    };

    struct Error {
        std::string errorType;
        std::string errorLevel;
        std::optional<std::string> errorDescription;
        std::optional<std::string> errorHint;
        std::vector<ErrorReference> errorReferences;

        void fromJson(const json& j);
        json toJson() const;
    };

    struct SafetyState {
        std::string eStop;
        bool fieldViolation;

        void fromJson(const json& j);
        json toJson() const;
    };

    struct AgvPosition {
        double x;
        double y;
        double theta;
        std::string mapId;
        std::optional<std::string> mapDescription;
        bool positionInitialized;
        std::optional<double> localizationScore;
        std::optional<double> deviationRange;

        void fromJson(const json& j);
        json toJson() const;
    };

    struct Velocity {
        std::optional<double> vx;
        std::optional<double> vy;
        std::optional<double> omega;

        void fromJson(const json& j);
        json toJson() const;
    };

    struct NodeState {
        std::string nodeId;
        uint32_t sequenceId;
        std::optional<std::string> nodeDescription;
        bool released;
        std::optional<NodePosition> nodePosition;

        void fromJson(const json& j);
        json toJson() const;
    };

    struct EdgeState {
        std::string edgeId;
        uint32_t sequenceId;
        std::optional<std::string> edgeDescription;
        bool released;
        std::optional<Trajectory> trajectory;

        void fromJson(const json& j);
        json toJson() const;
    };

    struct Load {
        std::optional<std::string> loadId;
        std::optional<std::string> loadType;
        std::optional<std::string> loadPosition;
        struct BoundingBoxReference {
            double x;
            double y;
            double z;
            std::optional<double> theta;

            void fromJson(const json& j);
            json toJson() const;
        } boundingBoxReference;
        struct LoadDimensions {
            double length;
            double width;
            std::optional<double> height;

            void fromJson(const json& j);
            json toJson() const;
        } loadDimensions;
        std::optional<double> weight;

        void fromJson(const json& j);
        json toJson() const;
    };

    struct Info {
        std::string infoType;
        std::string infoLevel;
        std::optional<std::string> infoDescription;
        struct InfoReference {
            std::string referenceKey;
            std::string referenceValue;

            void fromJson(const json& j);
            json toJson() const;
        };
        std::vector<InfoReference> infoReferences;

        void fromJson(const json& j);
        json toJson() const;
    };

    struct Map {
        std::string mapId;
        std::string mapVersion;
        std::optional<std::string> mapDescription;
        std::string mapStatus;

        void fromJson(const json& j);
        json toJson() const;
    };

    // 具体消息类
    class ConnectionMessage : public VDA5050Message {
    public:
        void fromJsonObj(const json& jsonObj) override;
        json toJsonObj() const override;

        static ConnectionState stringToConnectionState(const std::string& state);
        static std::string connectionStateToString(ConnectionState state);

        ConnectionState connectionState;
    };

    class OrderMessage : public VDA5050Message {
    public:
        void fromJsonObj(const json& jsonObj) override;
        json toJsonObj() const override;

        std::string orderId;
        uint32_t orderUpdateId;
        std::optional<std::string> zoneSetId;
        std::vector<Node> nodes;
        std::vector<Edge> edges;
    };

    class StateMessage : public VDA5050Message {
    public:
        void fromJsonObj(const json& jsonObj) override;
        json toJsonObj() const override;

        std::string orderId;
        uint32_t orderUpdateId;
        std::optional<std::string> zoneSetId;
        std::string lastNodeId;
        int lastNodeSequenceId;
        bool driving;
        std::optional<bool> paused;
        std::optional<bool> newBaseRequest;
        std::optional<double> distanceSinceLastNode;
        OperatingMode operatingMode;
        std::vector<NodeState> nodeStates;
        std::vector<EdgeState> edgeStates;
        std::optional<AgvPosition> agvPosition;
        std::optional<Velocity> velocity;
        std::vector<Load> loads;
        std::vector<ActionState> actionStates;
        BatteryState batteryState;
        std::vector<Error> errors;
        std::vector<Info> information;
        SafetyState safetyState;
        std::vector<Map> maps;
    };

    class InstantActionsMessage : public VDA5050Message {
    public:
        void fromJsonObj(const json& jsonObj) override;
        json toJsonObj() const override;

        std::vector<Action> actions;
    };

    class VisualizationMessage : public VDA5050Message {
    public:
        void fromJsonObj(const json& jsonObj) override;
        json toJsonObj() const override;

        std::optional<AgvPosition> agvPosition;
        std::optional<Velocity> velocity;
    };

    // 消息处理器类
    class MessageProcessor {
    public:
        static MessageType detectMessageType(const std::string& jsonStr);
        static std::shared_ptr<ConnectionMessage> parseConnectionMessage(const std::string& jsonStr);
        static std::shared_ptr<OrderMessage> parseOrderMessage(const std::string& jsonStr);
        static std::shared_ptr<StateMessage> parseStateMessage(const std::string& jsonStr);
        static std::shared_ptr<InstantActionsMessage> parseInstantActionsMessage(const std::string& jsonStr);
        static std::shared_ptr<VisualizationMessage> parseVisualizationMessage(const std::string& jsonStr);
        static std::shared_ptr<VDA5050Message> parseMessage(const std::string& jsonStr);
    };

    // 工具函数
    inline OperatingMode stringToOperatingMode(const std::string& mode) {
        if (mode == "AUTOMATIC") return OperatingMode::AUTOMATIC;
        if (mode == "SEMIAUTOMATIC") return OperatingMode::SEMIAUTOMATIC;
        if (mode == "MANUAL") return OperatingMode::MANUAL;
        if (mode == "SERVICE") return OperatingMode::SERVICE;
        if (mode == "TEACHIN") return OperatingMode::TEACHIN;
        return OperatingMode::MANUAL;
    }

    inline std::string operatingModeToString(OperatingMode mode) {
        switch (mode) {
        case OperatingMode::AUTOMATIC: return "AUTOMATIC";
        case OperatingMode::SEMIAUTOMATIC: return "SEMIAUTOMATIC";
        case OperatingMode::MANUAL: return "MANUAL";
        case OperatingMode::SERVICE: return "SERVICE";
        case OperatingMode::TEACHIN: return "TEACHIN";
        default: return "MANUAL";
        }
    }

} // namespace vda5050 