#include "mqtt.hpp"
#include <mosquitto.hpp>
#include <iostream>
#include <cstring>
#include <chrono>

// 静态库初始化，确保只初始化一次
static bool mosquitto_initialized = false;

MqttClient::MqttClient(const std::string& broker, int port, int timeout_sec)
    : broker_(broker), port_(port), timeout_sec_(timeout_sec),
    connected_(false), response_received_(false), client_id_("mqtt_client_" + std::to_string(rand())),
    has_credentials_(false) {

    // 初始化mosquitto库
    if (!mosquitto_initialized) {
        mosquitto_lib_init();
        mosquitto_initialized = true;
    }

    // 创建mosquitto实例
    mosq_ = mosquitto_new(client_id_.c_str(), true, this);
    if (!mosq_) {
        throw std::runtime_error("Failed to create mosquitto instance");
    }

    // 设置回调函数
    mosquitto_connect_callback_set(mosq_, on_connect_wrapper);
    mosquitto_message_callback_set(mosq_, on_message_wrapper);
}

MqttClient::~MqttClient() {
    disconnect();

    if (mosq_) {
        mosquitto_destroy(mosq_);
        mosq_ = nullptr;
    }

    // 清理mosquitto库
    mosquitto_lib_cleanup();
}

void MqttClient::setCredentials(const std::string& username, const std::string& password) {
    username_ = username;
    password_ = password;
    has_credentials_ = true;

    // 如果已经连接，需要重新连接以应用新的认证信息
    if (connected_) {
        reconnect();
    }
}

bool MqttClient::publish(const std::string& topic, const std::string& message, int qos) {
    if (!ensure_connected()) {
        return false;
    }

    int ret = mosquitto_publish(mosq_, nullptr, topic.c_str(), message.length(),
        message.c_str(), qos, RETAIN_);
    return (ret == MOSQ_ERR_SUCCESS);
}

bool MqttClient::subscribe(const std::string& topic, int qos,
    std::function<void(const std::string&)> callback) {
    if (!ensure_connected()) {
        return false;
    }

    // 存储回调函数
    callbacks_[topic] = callback;

    int ret = mosquitto_subscribe(mosq_, nullptr, topic.c_str(), qos);
    return (ret == MOSQ_ERR_SUCCESS);
}

void MqttClient::connect() {
    if (connected_) {
        return;
    }

    // 设置用户名和密码
    if (has_credentials_) {
        int ret = mosquitto_username_pw_set(mosq_, username_.c_str(), password_.c_str());
        if (ret != MOSQ_ERR_SUCCESS) {
            throw std::runtime_error("Failed to set MQTT credentials");
        }
    }

    // 设置连接超时
    mosquitto_connect_async(mosq_, broker_.c_str(), port_, 60);

    // 启动网络循环
    mosquitto_loop_start(mosq_);

    // 等待连接成功
    std::unique_lock<std::mutex> lock(mtx_);
    if (!cv_.wait_for(lock, std::chrono::seconds(timeout_sec_),
        [this] { return connected_.load(); })) {
        mosquitto_loop_stop(mosq_, true);
        throw std::runtime_error("Connection to MQTT broker timed out");
    }
}

void MqttClient::disconnect() {
    if (!connected_) {
        return;
    }

    mosquitto_disconnect(mosq_);
    mosquitto_loop_stop(mosq_, true);
    connected_ = false;
}

bool MqttClient::isConnected() const {
    return connected_;
}

void MqttClient::processMessages(int timeout_ms) {
    if (!connected_) {
        return;
    }

    // 处理一次消息循环
    mosquitto_loop(mosq_, timeout_ms, 1);
}

void MqttClient::setClientId(const std::string& client_id) {
    if (connected_) {
        disconnect();
    }

    if (mosq_) {
        mosquitto_destroy(mosq_);
    }

    client_id_ = client_id;
    mosq_ = mosquitto_new(client_id_.c_str(), true, this);

    if (!mosq_) {
        throw std::runtime_error("Failed to create mosquitto instance with new client ID");
    }

    // 重新设置回调函数
    mosquitto_connect_callback_set(mosq_, on_connect_wrapper);
    mosquitto_message_callback_set(mosq_, on_message_wrapper);
}

void MqttClient::setBrokerAddress(const std::string& broker, int port) {
    broker_ = broker;
    port_ = port;

    if (connected_) {
        // 重新连接到新地址
        disconnect();
    }
}

void MqttClient::reconnect() {
    disconnect();
    connect();
}

bool MqttClient::ensure_connected() {
    if (!connected_) {
        try {
            connect();
            return true;
        }
        catch (const std::exception& e) {
            std::cerr << "Failed to connect to MQTT broker: " << e.what() << std::endl;
            return false;
        }
    }
    return true;
}

void MqttClient::on_connect_wrapper(struct mosquitto* mosq, void* obj, int rc) {
    MqttClient* client = static_cast<MqttClient*>(obj);
    client->on_connect(rc);
}

void MqttClient::on_connect(int rc) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (rc == 0) {
        connected_ = true;

        // 重新订阅所有主题
        for (const auto& [topic, callback] : callbacks_) {
            mosquitto_subscribe(mosq_, nullptr, topic.c_str(), QOS_);
        }
    }
    else {
        connected_ = false;
    }
    cv_.notify_all();
}

void MqttClient::on_message_wrapper(struct mosquitto* mosq, void* obj,
    const struct mosquitto_message* msg) {
    MqttClient* client = static_cast<MqttClient*>(obj);
    client->on_message(msg);
}

void MqttClient::on_message(const struct mosquitto_message* msg) {
    if (!msg || !msg->payload) {
        return;
    }

    std::string topic = msg->topic;
    std::string payload(static_cast<char*>(msg->payload), msg->payloadlen);

    // 检查是否有对应主题的回调函数
    auto it = callbacks_.find(topic);
    if (it != callbacks_.end() && it->second) {
        it->second(payload);
    }

    // 设置接收到消息的标志
    {
        std::lock_guard<std::mutex> lock(mtx_);
        received_payload_ = payload;
        response_received_ = true;
    }
    cv_.notify_all();
}