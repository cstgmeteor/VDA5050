#include "VDA5050.hpp"
#include <iostream>
#include <chrono>
#include <csignal>
#include <vector>
#include <string>
#include <sstream>

// 全局变量用于控制程序运行
std::atomic<bool> g_running(true);

// 显示AGV状态信息
void showStatus(deviceagv::VDA5050Agv& vda5050) {
    vda5050.update();
    std::cout << "\n=== AGV状态信息 ===" << std::endl;
    std::cout << "当前位置: (" << vda5050.x() << ", " << vda5050.y() << "), 角度: " << vda5050.theta() << std::endl;
    //std::cout << "地图ID: " << vda5050.getMapId() << std::endl;
    std::cout << "上一个节点: " << vda5050.lastStationInt() << std::endl;
    std::cout << "当前节点: " << vda5050.currentStationInt() << std::endl;
    std::cout << "下一个节点: " << vda5050.nextStationInt() << std::endl;
    std::cout << "路径目标节点: " << vda5050.finialGoalInt() << std::endl;
    std::cout << "当前目标节点: " << vda5050.currentGoalInt() << std::endl;
    
    std::cout << "完整路径: ";
    for(auto node : vda5050.pathInt()) {
        std::cout << node << " ";
    }
    std::cout << std::endl;
    
    std::cout << "剩余路径: ";
    for(auto node : vda5050.restPathInt()) {
        std::cout << node << " ";
    }
    std::cout << std::endl;
    std::cout << "==================\n" << std::endl;
}

// 解析用户输入的节点路径
std::vector<int> parsePath(const std::string& input) {
    std::vector<int> path;
    std::stringstream ss(input);
    int node;
    while (ss >> node) {
        path.push_back(node);
    }
    return path;
}

int main() {
    try {

        // 获取用户输入参数
        std::string broker, manufacturer, serialNumber, version;
        int port;
        
        std::cout<<"是否使用默认"<<std::endl;
        std::cout << "1. 是" << std::endl;
        std::cout << "2. 否" << std::endl;
        int fault;
        std::cin >> fault;
        if(fault == 2){
            std::cout << "请输入MQTT代理地址: "; // 127.0.0.1
            std::cin >> broker;
            std::cout << "请输入MQTT端口: "; // 1883
            std::cin >> port;
            std::cout << "请输入制造商名称: "; // HikRobot
            std::cin >> manufacturer;
            std::cout << "请输入序列号: "; // 1
            std::cin >> serialNumber;
            std::cout << "请输入版本号: "; // V2.0.0
            std::cin >> version;
        }
        else{
            // broker = "192.168.1.101";
            // manufacturer = "StandardRobots";
            // port = 1883;
            // serialNumber = "312532";
            // version = "v2.0.0";
            broker = "192.168.1.105";
            manufacturer = "HikRobot";
            port = 1883;
            serialNumber = "1";
            version = "V2.0.0";
        }

        // 创建VDA5050实例
        deviceagv::VDA5050Agv vda5050(broker, port, manufacturer, serialNumber,version);
        vda5050.init();
        std::cout << "VDA5050初始化成功" << std::endl;

        // 设置headerIds和orderId
        std::string headid;
        std::cout << "请输入headerIds: ";
        std::cin >> headid;
        vda5050.setHeaderIds(headid);

        std::string orderid;
        std::cout << "请输入orderId: ";
        std::cin >> orderid;
        vda5050.setActionId(orderid);

        std::string actionId;
        std::cout << "请输入actionId: ";
        std::cin >> actionId;
        vda5050.setActionId(actionId);

        int choice;
        std::string input;
        std::vector<int> path;
        int nodeId;
        bool stop = false;
        bool path_set = false;

        while (g_running) {
            std::cout << "\n=== 功能菜单 ===" << std::endl;
            std::cout << "1. 设置路径" << std::endl;
            std::cout << "2. 移动到点" << std::endl;
            std::cout << "3. 添加路径" << std::endl;
            if(stop){
                std::cout << "4. 解除急停" << std::endl;
            }
            else{
                std::cout << "4. 急停" << std::endl;
            }
            std::cout << "5. 显示状态" << std::endl;
            std::cout << "6. 取消当前任务" << std::endl;
            std::cout << "0. 退出" << std::endl;
            std::cout << "请选择功能: ";
            
            std::cin >> choice;
            std::cin.ignore(); // 清除输入缓冲区

            switch (choice) {
                case 1: {
                    std::cout << "请输入节点路径(用空格分隔): ";
                    std::getline(std::cin, input);
                    path = parsePath(input);
                    vda5050.setPathInt(path);
                    //showStatus(vda5050);
                    path_set = true;
                    break;
                }
                case 2: {
                    if(!path_set){
                        std::cout << "请先设置路径" << std::endl;
                        break;
                    }
                    std::cout << "请输入目标节点ID: ";
                    std::cin >> nodeId;
                    vda5050.moveToStationInt(nodeId);
                    //showStatus(vda5050);
                    break;
                }
                case 3: {
                    if(!path_set){
                        std::cout << "请先设置路径" << std::endl;
                        break;
                    }
                    std::cout << "请输入要添加的节点路径(用空格分隔): ";
                    std::getline(std::cin, input);
                    path = parsePath(input);
                    vda5050.appendPathInt(path);
                    //showStatus(vda5050);
                    break;
                }
                case 4: {
                    if(stop){
                        vda5050.goOn();
                        stop = false;
                    }
                    else{
                        vda5050.pause();
                        stop = true;
                    }
                    //showStatus(vda5050);
                    
                    break;
                }
                case 5: {
                    showStatus(vda5050);
                    break;
                }
                case 6: {
                    vda5050.cancelMove();
                    break;
                }
                case 0: {
                    g_running = false;
                    break;
                }
                default: {
                    std::cout << "无效的选择，请重试" << std::endl;
                    break;
                }
            }
        }

        std::cout << "程序已安全退出。" << std::endl;
    }
    catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return 1;
    }
    catch (...) {
        std::cerr << "发生未知错误" << std::endl;
        return 1;
    }

    return 0;
}