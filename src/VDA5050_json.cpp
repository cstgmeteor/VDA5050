#include "VDA5050_json.hpp"

namespace vda5050 {

    // VDA5050Message实现
    void VDA5050Message::fromJson(const std::string& jsonStr) {
        json jsonObj = json::parse(jsonStr);
        fromJsonObj(jsonObj);
    }

    std::string VDA5050Message::toJson() const {
        return toJsonObj().dump(2);
    }

    void VDA5050Message::fromJsonObj(const json& jsonObj) {
        if (jsonObj.contains("headerId")) {
            headerId = jsonObj["headerId"].get<uint32_t>();
        }
        if (jsonObj.contains("timestamp")) {
            timestamp = jsonObj["timestamp"].get<std::string>();
        }
        if (jsonObj.contains("version")) {
            version = jsonObj["version"].get<std::string>();
        }
        if (jsonObj.contains("manufacturer")) {
            manufacturer = jsonObj["manufacturer"].get<std::string>();
        }
        if (jsonObj.contains("serialNumber")) {
            serialNumber = jsonObj["serialNumber"].get<std::string>();
        }
    }

    json VDA5050Message::toJsonObj() const {
        json jsonObj;

        if (headerId.has_value()) {
            jsonObj["headerId"] = headerId.value();
        }
        if (!timestamp.empty()) {
            jsonObj["timestamp"] = timestamp;
        }
        if (!version.empty()) {
            jsonObj["version"] = version;
        }
        if (!manufacturer.empty()) {
            jsonObj["manufacturer"] = manufacturer;
        }
        if (!serialNumber.empty()) {
            jsonObj["serialNumber"] = serialNumber;
        }

        return jsonObj;
    }

    // NodePosition实现
    void NodePosition::fromJson(const json& j) {
        x = j["x"].get<double>();
        y = j["y"].get<double>();

        if (j.contains("theta")) {
            theta = j["theta"].get<double>();
        }

        mapId = j["mapId"].get<std::string>();

        if (j.contains("mapDescription")) {
            mapDescription = j["mapDescription"].get<std::string>();
        }

        if (j.contains("allowedDeviationXY")) {
            allowedDeviationXY = j["allowedDeviationXY"].get<double>();
        }

        if (j.contains("allowedDeviationTheta")) {
            allowedDeviationTheta = j["allowedDeviationTheta"].get<double>();
        }
    }

    json NodePosition::toJson() const {
        json j;
        j["x"] = x;
        j["y"] = y;

        if (theta.has_value()) {
            j["theta"] = theta.value();
        }

        j["mapId"] = mapId;

        if (mapDescription.has_value()) {
            j["mapDescription"] = mapDescription.value();
        }

        if (allowedDeviationXY.has_value()) {
            j["allowedDeviationXY"] = allowedDeviationXY.value();
        }

        if (allowedDeviationTheta.has_value()) {
            j["allowedDeviationTheta"] = allowedDeviationTheta.value();
        }

        return j;
    }

    // ControlPoint实现
    void ControlPoint::fromJson(const json& j) {
        x = j["x"].get<double>();
        y = j["y"].get<double>();

        if (j.contains("weight")) {
            weight = j["weight"].get<double>();
        }
    }

    json ControlPoint::toJson() const {
        json j;
        j["x"] = x;
        j["y"] = y;

        if (weight.has_value()) {
            j["weight"] = weight.value();
        }

        return j;
    }

    // Trajectory实现
    void Trajectory::fromJson(const json& j) {
        degree = j["degree"].get<int>();

        knotVector.clear();
        for (const auto& knot : j["knotVector"]) {
            knotVector.push_back(knot.get<double>());
        }

        controlPoints.clear();
        for (const auto& point : j["controlPoints"]) {
            ControlPoint cp;
            cp.fromJson(point);
            controlPoints.push_back(cp);
        }
    }

    json Trajectory::toJson() const {
        json j;
        j["degree"] = degree;

        json knotsArray = json::array();
        for (const auto& knot : knotVector) {
            knotsArray.push_back(knot);
        }
        j["knotVector"] = knotsArray;

        json pointsArray = json::array();
        for (const auto& point : controlPoints) {
            pointsArray.push_back(point.toJson());
        }
        j["controlPoints"] = pointsArray;

        return j;
    }

    // Corridor实现
    void Corridor::fromJson(const json& j) {
        leftWidth = j["leftWidth"].get<double>();
        rightWidth = j["rightWidth"].get<double>();

        if (j.contains("corridorRefPoint")) {
            corridorRefPoint = j["corridorRefPoint"].get<std::string>();
        }
    }

    json Corridor::toJson() const {
        json j;
        j["leftWidth"] = leftWidth;
        j["rightWidth"] = rightWidth;

        if (corridorRefPoint.has_value()) {
            j["corridorRefPoint"] = corridorRefPoint.value();
        }

        return j;
    }

    // ActionParameter实现
    void ActionParameter::fromJson(const json& j) {
        key = j["key"].get<std::string>();
        value = j["value"];
    }

    json ActionParameter::toJson() const {
        json j;
        j["key"] = key;
        j["value"] = value;
        return j;
    }

    // Action实现
    void Action::fromJson(const json& j) {
        actionId = j["actionId"].get<std::string>();
        actionType = j["actionType"].get<std::string>();
        blockingType = j["blockingType"].get<std::string>();

        if (j.contains("actionDescription")) {
            actionDescription = j["actionDescription"].get<std::string>();
        }

        if (j.contains("actionParameters")) {
            std::vector<ActionParameter> params;
            for (const auto& param : j["actionParameters"]) {
                ActionParameter actionParam;
                actionParam.fromJson(param);
                params.push_back(actionParam);
            }
            actionParameters = params;
        }
    }

    json Action::toJson() const {
        json j;
        j["actionId"] = actionId;
        j["actionType"] = actionType;
        j["blockingType"] = blockingType;

        if (actionDescription.has_value()) {
            j["actionDescription"] = actionDescription.value();
        }

        if (actionParameters.has_value() && !actionParameters.value().empty()) {
            json paramsArray = json::array();
            for (const auto& param : actionParameters.value()) {
                paramsArray.push_back(param.toJson());
            }
            j["actionParameters"] = paramsArray;
        }

        return j;
    }

    // Node实现
    void Node::fromJson(const json& j) {
        nodeId = j["nodeId"].get<std::string>();
        sequenceId = j["sequenceId"].get<uint32_t>();
        released = j["released"].get<bool>();

        if (j.contains("nodeDescription")) {
            nodeDescription = j["nodeDescription"].get<std::string>();
        }

        if (j.contains("nodePosition")) {
            NodePosition position;
            position.fromJson(j["nodePosition"]);
            nodePosition = position;
        }

        actions.clear();
        for (const auto& action : j["actions"]) {
            Action a;
            a.fromJson(action);
            actions.push_back(a);
        }
    }

    json Node::toJson() const {
        json j;
        j["nodeId"] = nodeId;
        j["sequenceId"] = sequenceId;
        j["released"] = released;

        if (nodeDescription.has_value()) {
            j["nodeDescription"] = nodeDescription.value();
        }

        if (nodePosition.has_value()) {
            j["nodePosition"] = nodePosition.value().toJson();
        }

        json actionsArray = json::array();
        for (const auto& action : actions) {
            actionsArray.push_back(action.toJson());
        }
        j["actions"] = actionsArray;

        return j;
    }

    // Edge实现
    void Edge::fromJson(const json& j) {
        edgeId = j["edgeId"].get<std::string>();
        sequenceId = j["sequenceId"].get<uint32_t>();
        released = j["released"].get<bool>();
        startNodeId = j["startNodeId"].get<std::string>();
        endNodeId = j["endNodeId"].get<std::string>();

        if (j.contains("edgeDescription")) {
            edgeDescription = j["edgeDescription"].get<std::string>();
        }

        if (j.contains("maxSpeed")) {
            maxSpeed = j["maxSpeed"].get<double>();
        }

        if (j.contains("maxHeight")) {
            maxHeight = j["maxHeight"].get<double>();
        }

        if (j.contains("minHeight")) {
            minHeight = j["minHeight"].get<double>();
        }

        if (j.contains("orientation")) {
            orientation = j["orientation"].get<double>();
        }

        if (j.contains("orientationType")) {
            orientationType = j["orientationType"].get<std::string>();
        }

        if (j.contains("direction")) {
            direction = j["direction"].get<std::string>();
        }

        if (j.contains("rotationAllowed")) {
            rotationAllowed = j["rotationAllowed"].get<bool>();
        }

        if (j.contains("maxRotationSpeed")) {
            maxRotationSpeed = j["maxRotationSpeed"].get<double>();
        }

        if (j.contains("length")) {
            length = j["length"].get<double>();
        }

        if (j.contains("trajectory")) {
            Trajectory trajectory;
            trajectory.fromJson(j["trajectory"]);
            this->trajectory = trajectory;
        }

        if (j.contains("corridor")) {
            Corridor corridor;
            corridor.fromJson(j["corridor"]);
            this->corridor = corridor;
        }

        actions.clear();
        for (const auto& action : j["actions"]) {
            Action a;
            a.fromJson(action);
            actions.push_back(a);
        }
    }

    json Edge::toJson() const {
        json j;
        j["edgeId"] = edgeId;
        j["sequenceId"] = sequenceId;
        j["released"] = released;
        j["startNodeId"] = startNodeId;
        j["endNodeId"] = endNodeId;

        if (edgeDescription.has_value()) {
            j["edgeDescription"] = edgeDescription.value();
        }

        if (maxSpeed.has_value()) {
            j["maxSpeed"] = maxSpeed.value();
        }

        if (maxHeight.has_value()) {
            j["maxHeight"] = maxHeight.value();
        }

        if (minHeight.has_value()) {
            j["minHeight"] = minHeight.value();
        }

        if (orientation.has_value()) {
            j["orientation"] = orientation.value();
        }

        if (orientationType.has_value()) {
            j["orientationType"] = orientationType.value();
        }

        if (direction.has_value()) {
            j["direction"] = direction.value();
        }

        if (rotationAllowed.has_value()) {
            j["rotationAllowed"] = rotationAllowed.value();
        }

        if (maxRotationSpeed.has_value()) {
            j["maxRotationSpeed"] = maxRotationSpeed.value();
        }

        if (length.has_value()) {
            j["length"] = length.value();
        }

        if (trajectory.has_value()) {
            j["trajectory"] = trajectory.value().toJson();
        }

        if (corridor.has_value()) {
            j["corridor"] = corridor.value().toJson();
        }

        json actionsArray = json::array();
        for (const auto& action : actions) {
            actionsArray.push_back(action.toJson());
        }
        j["actions"] = actionsArray;

        return j;
    }

    // ActionState实现
    void ActionState::fromJson(const json& j) {
        actionId = j["actionId"].get<std::string>();
        actionStatus = j["actionStatus"].get<std::string>();

        if (j.contains("actionType")) {
            actionType = j["actionType"].get<std::string>();
        }

        if (j.contains("actionDescription")) {
            actionDescription = j["actionDescription"].get<std::string>();
        }

        if (j.contains("resultDescription")) {
            resultDescription = j["resultDescription"].get<std::string>();
        }
    }

    json ActionState::toJson() const {
        json j;
        j["actionId"] = actionId;
        j["actionStatus"] = actionStatus;

        if (actionType.has_value()) {
            j["actionType"] = actionType.value();
        }

        if (actionDescription.has_value()) {
            j["actionDescription"] = actionDescription.value();
        }

        if (resultDescription.has_value()) {
            j["resultDescription"] = resultDescription.value();
        }

        return j;
    }

    // BatteryState实现
    void BatteryState::fromJson(const json& j) {
        batteryCharge = j["batteryCharge"].get<double>();
        charging = j["charging"].get<bool>();

        if (j.contains("batteryVoltage")) {
            batteryVoltage = j["batteryVoltage"].get<double>();
        }

        if (j.contains("batteryHealth")) {
            batteryHealth = j["batteryHealth"].get<double>();
        }

        if (j.contains("reach")) {
            reach = j["reach"].get<double>();
        }
    }

    json BatteryState::toJson() const {
        json j;
        j["batteryCharge"] = batteryCharge;
        j["charging"] = charging;

        if (batteryVoltage.has_value()) {
            j["batteryVoltage"] = batteryVoltage.value();
        }

        if (batteryHealth.has_value()) {
            j["batteryHealth"] = batteryHealth.value();
        }

        if (reach.has_value()) {
            j["reach"] = reach.value();
        }

        return j;
    }

    // ErrorReference实现
    void ErrorReference::fromJson(const json& j) {
        referenceKey = j["referenceKey"].get<std::string>();
        referenceValue = j["referenceValue"].get<std::string>();
    }

    json ErrorReference::toJson() const {
        json j;
        j["referenceKey"] = referenceKey;
        j["referenceValue"] = referenceValue;
        return j;
    }

    // Error实现
    void Error::fromJson(const json& j) {
        errorType = j["errorType"].get<std::string>();
        errorLevel = j["errorLevel"].get<std::string>();

        if (j.contains("errorDescription")) {
            errorDescription = j["errorDescription"].get<std::string>();
        }

        if (j.contains("errorHint")) {
            errorHint = j["errorHint"].get<std::string>();
        }

        errorReferences.clear();
        if (j.contains("errorReferences")) {
            for (const auto& ref : j["errorReferences"]) {
                ErrorReference errRef;
                errRef.fromJson(ref);
                errorReferences.push_back(errRef);
            }
        }
    }

    json Error::toJson() const {
        json j;
        j["errorType"] = errorType;
        j["errorLevel"] = errorLevel;

        if (errorDescription.has_value()) {
            j["errorDescription"] = errorDescription.value();
        }

        if (errorHint.has_value()) {
            j["errorHint"] = errorHint.value();
        }

        json refsArray = json::array();
        for (const auto& ref : errorReferences) {
            refsArray.push_back(ref.toJson());
        }
        j["errorReferences"] = refsArray;

        return j;
    }

    // SafetyState实现
    void SafetyState::fromJson(const json& j) {
        eStop = j["eStop"].get<std::string>();
        fieldViolation = j["fieldViolation"].get<bool>();
    }

    json SafetyState::toJson() const {
        json j;
        j["eStop"] = eStop;
        j["fieldViolation"] = fieldViolation;
        return j;
    }

    // AgvPosition实现
    void AgvPosition::fromJson(const json& j) {
        x = j["x"].get<double>();
        y = j["y"].get<double>();
        theta = j["theta"].get<double>();
        mapId = j["mapId"].get<std::string>();
        positionInitialized = j["positionInitialized"].get<bool>();

        if (j.contains("mapDescription")) {
            mapDescription = j["mapDescription"].get<std::string>();
        }

        if (j.contains("localizationScore")) {
            localizationScore = j["localizationScore"].get<double>();
        }

        if (j.contains("deviationRange")) {
            deviationRange = j["deviationRange"].get<double>();
        }
    }

    json AgvPosition::toJson() const {
        json j;
        j["x"] = x;
        j["y"] = y;
        j["theta"] = theta;
        j["mapId"] = mapId;
        j["positionInitialized"] = positionInitialized;

        if (mapDescription.has_value()) {
            j["mapDescription"] = mapDescription.value();
        }

        if (localizationScore.has_value()) {
            j["localizationScore"] = localizationScore.value();
        }

        if (deviationRange.has_value()) {
            j["deviationRange"] = deviationRange.value();
        }

        return j;
    }

    // Velocity实现
    void Velocity::fromJson(const json& j) {
        if (j.contains("vx")) {
            vx = j["vx"].get<double>();
        }

        if (j.contains("vy")) {
            vy = j["vy"].get<double>();
        }

        if (j.contains("omega")) {
            omega = j["omega"].get<double>();
        }
    }

    json Velocity::toJson() const {
        json j;

        if (vx.has_value()) {
            j["vx"] = vx.value();
        }

        if (vy.has_value()) {
            j["vy"] = vy.value();
        }

        if (omega.has_value()) {
            j["omega"] = omega.value();
        }

        return j;
    }

    // NodeState实现
    void NodeState::fromJson(const json& j) {
        nodeId = j["nodeId"].get<std::string>();
        sequenceId = j["sequenceId"].get<uint32_t>();
        released = j["released"].get<bool>();

        if (j.contains("nodeDescription")) {
            nodeDescription = j["nodeDescription"].get<std::string>();
        }

        if (j.contains("nodePosition")) {
            NodePosition position;
            position.fromJson(j["nodePosition"]);
            nodePosition = position;
        }
    }

    json NodeState::toJson() const {
        json j;
        j["nodeId"] = nodeId;
        j["sequenceId"] = sequenceId;
        j["released"] = released;

        if (nodeDescription.has_value()) {
            j["nodeDescription"] = nodeDescription.value();
        }

        if (nodePosition.has_value()) {
            j["nodePosition"] = nodePosition.value().toJson();
        }

        return j;
    }

    // EdgeState实现
    void EdgeState::fromJson(const json& j) {
        edgeId = j["edgeId"].get<std::string>();
        sequenceId = j["sequenceId"].get<uint32_t>();
        released = j["released"].get<bool>();

        if (j.contains("edgeDescription")) {
            edgeDescription = j["edgeDescription"].get<std::string>();
        }

        if (j.contains("trajectory")) {
            Trajectory trajectory;
            trajectory.fromJson(j["trajectory"]);
            this->trajectory = trajectory;
        }
    }

    json EdgeState::toJson() const {
        json j;
        j["edgeId"] = edgeId;
        j["sequenceId"] = sequenceId;
        j["released"] = released;

        if (edgeDescription.has_value()) {
            j["edgeDescription"] = edgeDescription.value();
        }

        if (trajectory.has_value()) {
            j["trajectory"] = trajectory.value().toJson();
        }

        return j;
    }

    // Load实现
    void Load::fromJson(const json& j) {
        if (j.contains("loadId")) {
            loadId = j["loadId"].get<std::string>();
        }

        if (j.contains("loadType")) {
            loadType = j["loadType"].get<std::string>();
        }

        if (j.contains("loadPosition")) {
            loadPosition = j["loadPosition"].get<std::string>();
        }

        if (j.contains("boundingBoxReference")) {
            boundingBoxReference.fromJson(j["boundingBoxReference"]);
        }

        if (j.contains("loadDimensions")) {
            loadDimensions.fromJson(j["loadDimensions"]);
        }

        if (j.contains("weight")) {
            weight = j["weight"].get<double>();
        }
    }

    json Load::toJson() const {
        json j;

        if (loadId.has_value()) {
            j["loadId"] = loadId.value();
        }

        if (loadType.has_value()) {
            j["loadType"] = loadType.value();
        }

        if (loadPosition.has_value()) {
            j["loadPosition"] = loadPosition.value();
        }

        j["boundingBoxReference"] = boundingBoxReference.toJson();
        j["loadDimensions"] = loadDimensions.toJson();

        if (weight.has_value()) {
            j["weight"] = weight.value();
        }

        return j;
    }

    // Load::BoundingBoxReference实现
    void Load::BoundingBoxReference::fromJson(const json& j) {
        x = j["x"].get<double>();
        y = j["y"].get<double>();
        z = j["z"].get<double>();

        if (j.contains("theta")) {
            theta = j["theta"].get<double>();
        }
    }

    json Load::BoundingBoxReference::toJson() const {
        json j;
        j["x"] = x;
        j["y"] = y;
        j["z"] = z;

        if (theta.has_value()) {
            j["theta"] = theta.value();
        }

        return j;
    }

    // Load::LoadDimensions实现
    void Load::LoadDimensions::fromJson(const json& j) {
        length = j["length"].get<double>();
        width = j["width"].get<double>();

        if (j.contains("height")) {
            height = j["height"].get<double>();
        }
    }

    json Load::LoadDimensions::toJson() const {
        json j;
        j["length"] = length;
        j["width"] = width;

        if (height.has_value()) {
            j["height"] = height.value();
        }

        return j;
    }

    // Info实现
    void Info::fromJson(const json& j) {
        infoType = j["infoType"].get<std::string>();
        infoLevel = j["infoLevel"].get<std::string>();

        if (j.contains("infoDescription")) {
            infoDescription = j["infoDescription"].get<std::string>();
        }

        infoReferences.clear();
        if (j.contains("infoReferences")) {
            for (const auto& ref : j["infoReferences"]) {
                InfoReference infoRef;
                infoRef.fromJson(ref);
                infoReferences.push_back(infoRef);
            }
        }
    }

    json Info::toJson() const {
        json j;
        j["infoType"] = infoType;
        j["infoLevel"] = infoLevel;

        if (infoDescription.has_value()) {
            j["infoDescription"] = infoDescription.value();
        }

        json refsArray = json::array();
        for (const auto& ref : infoReferences) {
            refsArray.push_back(ref.toJson());
        }
        j["infoReferences"] = refsArray;

        return j;
    }

    // Info::InfoReference实现
    void Info::InfoReference::fromJson(const json& j) {
        referenceKey = j["referenceKey"].get<std::string>();
        referenceValue = j["referenceValue"].get<std::string>();
    }

    json Info::InfoReference::toJson() const {
        json j;
        j["referenceKey"] = referenceKey;
        j["referenceValue"] = referenceValue;
        return j;
    }

    // Map实现
    void Map::fromJson(const json& j) {
        mapId = j["mapId"].get<std::string>();
        mapVersion = j["mapVersion"].get<std::string>();
        mapStatus = j["mapStatus"].get<std::string>();

        if (j.contains("mapDescription")) {
            mapDescription = j["mapDescription"].get<std::string>();
        }
    }

    json Map::toJson() const {
        json j;
        j["mapId"] = mapId;
        j["mapVersion"] = mapVersion;
        j["mapStatus"] = mapStatus;

        if (mapDescription.has_value()) {
            j["mapDescription"] = mapDescription.value();
        }

        return j;
    }

    // ConnectionMessage实现
    ConnectionState ConnectionMessage::stringToConnectionState(const std::string& state) {
        if (state == "ONLINE") {
            return ConnectionState::ONLINE;
        }
        else if (state == "OFFLINE") {
            return ConnectionState::OFFLINE;
        }
        else if (state == "CONNECTIONBROKEN") {
            return ConnectionState::CONNECTIONBROKEN;
        }
        return ConnectionState::OFFLINE;
    }

    std::string ConnectionMessage::connectionStateToString(ConnectionState state) {
        switch (state) {
        case ConnectionState::ONLINE:
            return "ONLINE";
        case ConnectionState::OFFLINE:
            return "OFFLINE";
        case ConnectionState::CONNECTIONBROKEN:
            return "CONNECTIONBROKEN";
        default:
            return "OFFLINE";
        }
    }

    void ConnectionMessage::fromJsonObj(const json& jsonObj) {
        VDA5050Message::fromJsonObj(jsonObj);

        if (jsonObj.contains("connectionState")) {
            connectionState = stringToConnectionState(jsonObj["connectionState"].get<std::string>());
        }
        else {
            connectionState = ConnectionState::OFFLINE;
        }
    }

    json ConnectionMessage::toJsonObj() const {
        json j = VDA5050Message::toJsonObj();
        j["connectionState"] = connectionStateToString(connectionState);
        return j;
    }

    // OrderMessage实现
    void OrderMessage::fromJsonObj(const json& jsonObj) {
        VDA5050Message::fromJsonObj(jsonObj);

        if (jsonObj.contains("orderId")) {
            orderId = jsonObj["orderId"].get<std::string>();
        }

        if (jsonObj.contains("orderUpdateId")) {
            orderUpdateId = jsonObj["orderUpdateId"].get<uint32_t>();
        }

        if (jsonObj.contains("zoneSetId")) {
            zoneSetId = jsonObj["zoneSetId"].get<std::string>();
        }

        nodes.clear();
        if (jsonObj.contains("nodes") && jsonObj["nodes"].is_array()) {
            for (const auto& nodeJson : jsonObj["nodes"]) {
                Node node;
                node.fromJson(nodeJson);
                nodes.push_back(node);
            }
        }

        edges.clear();
        if (jsonObj.contains("edges") && jsonObj["edges"].is_array()) {
            for (const auto& edgeJson : jsonObj["edges"]) {
                Edge edge;
                edge.fromJson(edgeJson);
                edges.push_back(edge);
            }
        }
    }

    json OrderMessage::toJsonObj() const {
        json j = VDA5050Message::toJsonObj();

        j["orderId"] = orderId;
        j["orderUpdateId"] = orderUpdateId;

        if (zoneSetId.has_value()) {
            j["zoneSetId"] = zoneSetId.value();
        }

        json nodesArray = json::array();
        for (const auto& node : nodes) {
            nodesArray.push_back(node.toJson());
        }
        j["nodes"] = nodesArray;

        json edgesArray = json::array();
        for (const auto& edge : edges) {
            edgesArray.push_back(edge.toJson());
        }
        j["edges"] = edgesArray;

        return j;
    }

    // StateMessage实现
    void StateMessage::fromJsonObj(const json& jsonObj) {
        VDA5050Message::fromJsonObj(jsonObj);

        if (jsonObj.contains("orderId")) {
            orderId = jsonObj["orderId"].get<std::string>();
        }

        if (jsonObj.contains("orderUpdateId")) {
            orderUpdateId = jsonObj["orderUpdateId"].get<uint32_t>();
        }

        if (jsonObj.contains("zoneSetId")) {
            zoneSetId = jsonObj["zoneSetId"].get<std::string>();
        }

        if (jsonObj.contains("lastNodeId")) {
            lastNodeId = jsonObj["lastNodeId"].get<std::string>();
        }

        if (jsonObj.contains("lastNodeSequenceId")) {
            lastNodeSequenceId = jsonObj["lastNodeSequenceId"].get<uint32_t>();
        }

        if (jsonObj.contains("driving")) {
            driving = jsonObj["driving"].get<bool>();
        }

        if (jsonObj.contains("paused")) {
            paused = jsonObj["paused"].get<bool>();
        }

        if (jsonObj.contains("newBaseRequest")) {
            newBaseRequest = jsonObj["newBaseRequest"].get<bool>();
        }

        if (jsonObj.contains("distanceSinceLastNode")) {
            distanceSinceLastNode = jsonObj["distanceSinceLastNode"].get<double>();
        }

        if (jsonObj.contains("operatingMode")) {
            operatingMode = stringToOperatingMode(jsonObj["operatingMode"].get<std::string>());
        }
        else {
            operatingMode = OperatingMode::MANUAL;
        }

        nodeStates.clear();
        if (jsonObj.contains("nodeStates") && jsonObj["nodeStates"].is_array()) {
            for (const auto& nodeStateJson : jsonObj["nodeStates"]) {
                NodeState nodeState;
                nodeState.fromJson(nodeStateJson);
                nodeStates.push_back(nodeState);
            }
        }

        edgeStates.clear();
        if (jsonObj.contains("edgeStates") && jsonObj["edgeStates"].is_array()) {
            for (const auto& edgeStateJson : jsonObj["edgeStates"]) {
                EdgeState edgeState;
                edgeState.fromJson(edgeStateJson);
                edgeStates.push_back(edgeState);
            }
        }

        if (jsonObj.contains("agvPosition")) {
            AgvPosition position;
            position.fromJson(jsonObj["agvPosition"]);
            agvPosition = position;
        }

        if (jsonObj.contains("velocity")) {
            Velocity velocity;
            velocity.fromJson(jsonObj["velocity"]);
            this->velocity = velocity;
        }

        loads.clear();
        if (jsonObj.contains("loads") && jsonObj["loads"].is_array()) {
            for (const auto& loadJson : jsonObj["loads"]) {
                Load load;
                load.fromJson(loadJson);
                loads.push_back(load);
            }
        }

        actionStates.clear();
        if (jsonObj.contains("actionStates") && jsonObj["actionStates"].is_array()) {
            for (const auto& actionStateJson : jsonObj["actionStates"]) {
                ActionState actionState;
                actionState.fromJson(actionStateJson);
                actionStates.push_back(actionState);
            }
        }

        if (jsonObj.contains("batteryState")) {
            batteryState.fromJson(jsonObj["batteryState"]);
        }

        errors.clear();
        if (jsonObj.contains("errors") && jsonObj["errors"].is_array()) {
            for (const auto& errorJson : jsonObj["errors"]) {
                Error error;
                error.fromJson(errorJson);
                errors.push_back(error);
            }
        }

        information.clear();
        if (jsonObj.contains("information") && jsonObj["information"].is_array()) {
            for (const auto& infoJson : jsonObj["information"]) {
                Info info;
                info.fromJson(infoJson);
                information.push_back(info);
            }
        }

        if (jsonObj.contains("safetyState")) {
            safetyState.fromJson(jsonObj["safetyState"]);
        }

        maps.clear();
        if (jsonObj.contains("maps") && jsonObj["maps"].is_array()) {
            for (const auto& mapJson : jsonObj["maps"]) {
                Map map;
                map.fromJson(mapJson);
                maps.push_back(map);
            }
        }
    }

    json StateMessage::toJsonObj() const {
        json j = VDA5050Message::toJsonObj();

        j["orderId"] = orderId;
        j["orderUpdateId"] = orderUpdateId;

        if (zoneSetId.has_value()) {
            j["zoneSetId"] = zoneSetId.value();
        }

        j["lastNodeId"] = lastNodeId;
        j["lastNodeSequenceId"] = lastNodeSequenceId;
        j["driving"] = driving;

        if (paused.has_value()) {
            j["paused"] = paused.value();
        }

        if (newBaseRequest.has_value()) {
            j["newBaseRequest"] = newBaseRequest.value();
        }

        if (distanceSinceLastNode.has_value()) {
            j["distanceSinceLastNode"] = distanceSinceLastNode.value();
        }

        j["operatingMode"] = operatingModeToString(operatingMode);

        json nodeStatesArray = json::array();
        for (const auto& nodeState : nodeStates) {
            nodeStatesArray.push_back(nodeState.toJson());
        }
        j["nodeStates"] = nodeStatesArray;

        json edgeStatesArray = json::array();
        for (const auto& edgeState : edgeStates) {
            edgeStatesArray.push_back(edgeState.toJson());
        }
        j["edgeStates"] = edgeStatesArray;

        if (agvPosition.has_value()) {
            j["agvPosition"] = agvPosition.value().toJson();
        }

        if (velocity.has_value()) {
            j["velocity"] = velocity.value().toJson();
        }

        json loadsArray = json::array();
        for (const auto& load : loads) {
            loadsArray.push_back(load.toJson());
        }
        j["loads"] = loadsArray;

        json actionStatesArray = json::array();
        for (const auto& actionState : actionStates) {
            actionStatesArray.push_back(actionState.toJson());
        }
        j["actionStates"] = actionStatesArray;

        j["batteryState"] = batteryState.toJson();

        json errorsArray = json::array();
        for (const auto& error : errors) {
            errorsArray.push_back(error.toJson());
        }
        j["errors"] = errorsArray;

        json informationArray = json::array();
        for (const auto& info : information) {
            informationArray.push_back(info.toJson());
        }
        j["information"] = informationArray;

        j["safetyState"] = safetyState.toJson();

        json mapsArray = json::array();
        for (const auto& map : maps) {
            mapsArray.push_back(map.toJson());
        }
        j["maps"] = mapsArray;

        return j;
    }

    // InstantActionsMessage实现
    void InstantActionsMessage::fromJsonObj(const json& jsonObj) {
        VDA5050Message::fromJsonObj(jsonObj);

        actions.clear();
        if (jsonObj.contains("actions") && jsonObj["actions"].is_array()) {
            for (const auto& actionJson : jsonObj["actions"]) {
                Action action;
                action.fromJson(actionJson);
                actions.push_back(action);
            }
        }
    }

    json InstantActionsMessage::toJsonObj() const {
        json j = VDA5050Message::toJsonObj();

        json actionsArray = json::array();
        for (const auto& action : actions) {
            actionsArray.push_back(action.toJson());
        }
        j["actions"] = actionsArray;

        return j;
    }

    // VisualizationMessage实现
    void VisualizationMessage::fromJsonObj(const json& jsonObj) {
        VDA5050Message::fromJsonObj(jsonObj);

        if (jsonObj.contains("agvPosition")) {
            AgvPosition position;
            position.fromJson(jsonObj["agvPosition"]);
            agvPosition = position;
        }

        if (jsonObj.contains("velocity")) {
            Velocity velocity;
            velocity.fromJson(jsonObj["velocity"]);
            this->velocity = velocity;
        }
    }

    json VisualizationMessage::toJsonObj() const {
        json j = VDA5050Message::toJsonObj();

        if (agvPosition.has_value()) {
            j["agvPosition"] = agvPosition.value().toJson();
        }

        if (velocity.has_value()) {
            j["velocity"] = velocity.value().toJson();
        }

        return j;
    }

    // MessageProcessor实现
    MessageType MessageProcessor::detectMessageType(const std::string& jsonStr) {
        try {
            json jsonObj = json::parse(jsonStr);
            if (jsonObj.contains("connectionState")) {
                return MessageType::CONNECTION;
            }
            else if (jsonObj.contains("orderId") && jsonObj.contains("nodes")) {
                return MessageType::ORDER;
            }
            else if (jsonObj.contains("orderId") && jsonObj.contains("nodeStates")) {
                return MessageType::STATE;
            }
            else if (jsonObj.contains("actions")) {
                return MessageType::INSTANTACTIONS;
            }
            else if (jsonObj.contains("agvPosition")) {
                return MessageType::VISUALIZATION;
            }
        }
        catch (...) {
            return MessageType::UNKNOWN;
        }
        return MessageType::UNKNOWN;
    }

    std::shared_ptr<ConnectionMessage> MessageProcessor::parseConnectionMessage(const std::string& jsonStr) {
        auto msg = std::make_shared<ConnectionMessage>();
        msg->fromJson(jsonStr);
        return msg;
    }

    std::shared_ptr<OrderMessage> MessageProcessor::parseOrderMessage(const std::string& jsonStr) {
        auto msg = std::make_shared<OrderMessage>();
        msg->fromJson(jsonStr);
        return msg;
    }

    std::shared_ptr<StateMessage> MessageProcessor::parseStateMessage(const std::string& jsonStr) {
        auto msg = std::make_shared<StateMessage>();
        msg->fromJson(jsonStr);
        return msg;
    }

    std::shared_ptr<InstantActionsMessage> MessageProcessor::parseInstantActionsMessage(const std::string& jsonStr) {
        auto msg = std::make_shared<InstantActionsMessage>();
        msg->fromJson(jsonStr);
        return msg;
    }

    std::shared_ptr<VisualizationMessage> MessageProcessor::parseVisualizationMessage(const std::string& jsonStr) {
        auto msg = std::make_shared<VisualizationMessage>();
        msg->fromJson(jsonStr);
        return msg;
    }

    std::shared_ptr<VDA5050Message> MessageProcessor::parseMessage(const std::string& jsonStr) {
        switch (detectMessageType(jsonStr)) {
        case MessageType::CONNECTION:
            return parseConnectionMessage(jsonStr);
        case MessageType::ORDER:
            return parseOrderMessage(jsonStr);
        case MessageType::STATE:
            return parseStateMessage(jsonStr);
        case MessageType::INSTANTACTIONS:
            return parseInstantActionsMessage(jsonStr);
        case MessageType::VISUALIZATION:
            return parseVisualizationMessage(jsonStr);
        default:
            return nullptr;
        }
    }

} // namespace vda5050