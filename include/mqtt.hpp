#ifndef MQTT_CLIENT_HPP_
#define MQTT_CLIENT_HPP_

#include <string>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <optional>
#include <functional>
#include <map>

struct mosquitto; // 前向声明

// 修改为与VDA5050Agv实现匹配的类名
class MqttClient {
public:
    // 构造函数
    MqttClient(const std::string& broker = "127.0.0.1", int port = 1883, int timeout_sec = 10);

    // 析构函数
    ~MqttClient();

    // 设置用户名和密码
    void setCredentials(const std::string& username, const std::string& password);

    // 发布消息方法
    bool publish(
        const std::string& topic,
        const std::string& message,
        int qos = 1);

    // 订阅主题方法
    bool subscribe(
        const std::string& topic,
        int qos,
        std::function<void(const std::string&)> callback);

    // 连接方法
    void connect();

    // 断开连接方法
    void disconnect();

    // 检查连接状态
    bool isConnected() const;

    // 处理消息
    void processMessages(int timeout_ms = 100);

    // 设置客户端ID
    void setClientId(const std::string& client_id);

    // 设置代理地址和端口
    void setBrokerAddress(const std::string& broker, int port = 1883);

    // 重新连接
    void reconnect();

private:
    // MQTT 优先级和保留标志
    static const int QOS_ = 1;
    static const bool RETAIN_ = false;

    // MQTT 实例
    struct mosquitto* mosq_;

    // 客户端ID
    std::string client_id_;

    // 连接信息
    std::string broker_;
    int port_;
    int timeout_sec_;

    // 认证信息
    std::string username_;
    std::string password_;
    bool has_credentials_ = false;

    // 线程同步和状态
    std::mutex mtx_;
    std::condition_variable cv_;
    std::atomic<bool> connected_;
    std::atomic<bool> response_received_;
    std::string received_payload_;  // 接收消息缓存

    // 存储主题订阅的回调函数
    std::map<std::string, std::function<void(const std::string&)>> callbacks_;

    // 确保连接
    bool ensure_connected();

    // 回调包装器
    static void on_connect_wrapper(struct mosquitto*, void*, int);
    void on_connect(int rc);

    static void on_message_wrapper(struct mosquitto*, void*,
        const struct mosquitto_message*);
    void on_message(const struct mosquitto_message* msg);
};

#endif // MQTT_CLIENT_HPP_