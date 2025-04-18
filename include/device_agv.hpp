#ifndef DEVICE_MANAGER_DEVICE_AGV_HPP_
#define DEVICE_MANAGER_DEVICE_AGV_HPP_

#include <vector>
#include <string>
#include <any>
#include <memory>
//new
#include <functional>
#include <set>

// #include "kaanhrcs_lib_export.h"
// #include "kaanhrcs/device/device.hpp"
// #include "kaanhrcs/core/json.hpp"
// #include "kaanhrcs/core/macro.hpp"
// #include "kaanhrcs/agv_manager/agv_tray.hpp"
// #include "kaanhcode/pgm_controller.hpp"
// #include "kaanhrcs/device_manager/device.hpp"

// 传参和返回值都是int
using StateCbkFuncT = std::function<int(int)>;

namespace deviceagv
{
    // 修改
    enum class AgvProductCode : int
    {
        kNull = 0,
        kStdhttp,
        kStdmodbus,
        KSeermodbus,
        kVDA5050
    };
    auto toAgvProductCode(const std::string &code) -> AgvProductCode; // 修改

    auto getAgvTypes() -> std::vector<std::string> &;
    auto getAgvProductCode() -> std::vector<std::string> &;

    // TODO: 删除了Device的继承，是否有风险
    // 原 class KAANHRCS_API DeviceAgv
    class DeviceAgv
    {
    public:
        // AGV系统状态：断连，离线，在线，充电
        enum class AgvSysState
        {
            kDisconnected = -2,
            kOffline = -1,
            kOnline = 0,
        };
        // AGV移动状态：急停，空闲，移动，避障，暂停
        enum class AgvMoveState
        {
            kEstop = -1,
            kIdle,
            kMoving,
            kAvoidObstacle,
            kPaused,
            kCancelMove
        };
        // AGV控制模式:手动，自动 ,失效
        enum class AgvControlMode
        {
            kInvalid = 0,
            kMannual = 1,
            kAuto = 2
        };
        enum class AgvActState
        {
            kError = -1,//失败
            kNull = 0,
            kExecuting = 1,//执行中
            kSuccess = 2,//成功
            kPaused = 3,//暂停
            kPending = 4,//挂起
        };
        DeviceAgv();
        virtual ~DeviceAgv();
        // pgm
        //修改auto agvPgmController() const -> kaanhcode::PgmController &; 
        // state func
        auto setCurrentStationId(const std::uint16_t stationId) -> void;
        auto currentStationId() const -> std::uint16_t;

        auto setAgvSysState(AgvSysState state) -> void;
        auto agvSysState() const -> AgvSysState;
        auto setAgvSysStateCbk(StateCbkFuncT cbk) -> void;
        auto setAgvMoveState(AgvMoveState state) -> void;
        auto agvMoveState() const -> AgvMoveState;
        auto setAgvMoveStateCbk(StateCbkFuncT cbk) -> void;
        auto setAgvControlMode(AgvControlMode state) -> void;
        auto agvControlMode() const -> AgvControlMode;
        auto setAgvControlModeCbk(StateCbkFuncT cbk) -> void;
        /* -------------------------reflection property */
        // name
        auto name() const -> const std::string &;
        auto setName(const std::string &agvname) -> void;
        auto id() const -> const std::string &;
        auto setId(const std::string &agvid) -> void;
        auto type() const ->const std::string&;
        auto setType(const std::string & agvtype) -> void;

        auto servedDevices() const -> const std::set<std::string> &;
        auto setServedDevices(const std::set<std::string> &devices) -> void;
        // physical model
        auto x() const -> double;
        auto setX(double x) -> void;
        auto y() const -> double;
        auto setY(double y) -> void;
        auto theta() const -> double;
        auto setTheta(double theta) -> void;

        auto width() const -> double;
        auto setWidth(double width) -> void;
        auto height() const -> double;
        auto setHeight(double height) -> void;

        // agv data
        auto setDeviceData(std::any data) -> void;
        auto deviceData() -> const std::any &;

        auto errMsg() const -> const std::string &;
        auto setErrMsg(const std::string &err_msg) -> void;

        // using map
        auto agvMap() const -> const std::string &;
        auto setAgvMap(const std::string &map_name) -> void;

        /* -------------------------reflection property */

        // agv state
        // auto state() const->State;

        // agv info
        // auto virtual mode() const -> AgvMode = 0;
        auto batteryPercent() const -> int;
        auto setBatteryPercent(int battery) const -> void;
        auto vel() const -> double;
        auto setVel(int vell) const -> void;
        auto lastStation() const -> std::string;
        auto currentStation() const -> std::string;
        auto nextStation() const -> std::string;
        auto setLastStation(const std::string & name)->void;
        auto setCurrentStation(const std::string & name)->void;
        auto setNextStation(const std::string & name)->void;

        // 新增int接口
        auto lastStationInt() const -> int;
        auto currentStationInt() const -> int;
        auto nextStationInt() const -> int;
        auto setLastStationInt(const int & name)->void;
        auto setCurrentStationInt(const int & name)->void;
        auto setNextStationInt(const int & name)->void;
        auto pathInt() const -> std::vector<int>;
        auto virtual setPathInt(const std::vector<int>& path, bool append = false, int goal = -1)->void;
        auto restPathInt() const -> std::vector<int>;
        auto setRestPathInt(const std::vector<int> &) -> void;
        auto finialGoalInt() const -> int;
        auto setFinialGoalInt(const int &) -> void;
        auto currentGoalInt() const -> int;
        auto setCurrentGoalInt(const int &) -> void;
        auto virtual moveToStationInt(const int &name) -> void = 0;
        auto virtual appendPathInt(const std::vector<int>& path)->void = 0;
        // ****
        auto virtual goOn() -> void = 0;
        auto virtual pause() -> void = 0;
        auto virtual cancelMove() -> void = 0;
        auto virtual eStop() -> void = 0;
        auto virtual cancelEStop() -> void = 0;
        //返回动作订单id
        auto virtual startAct(const std::string & actcode,const std::string & actparam1 = "null",const std::string & actparam2 = "null",std::vector<std::string> anyparams = {}) -> int{return 0;};
        auto virtual actStatus(int actid)->AgvActState{return AgvActState::kNull;};
        
        auto path() const -> std::vector<std::string>;
        auto setPath(const std::vector<std::string> &) -> void;
        auto targetStation() const ->std::string;
        //new auto setAgvTray(kaanhrcs::agvmanager::AgvTray *tray) -> void;
        //new auto agvTray() const -> kaanhrcs::agvmanager::AgvTray &;
        auto beginServer() -> void;
        auto finishServer() -> void;
        auto isServing() const -> bool;

        auto refVel() const -> double;
        auto setRefVel(double ref_vel) -> void;
        auto setDeviceTools(const std::set<std::string> &devtools) -> void;
        auto deviceTools() const -> std::set<std::string>;

        auto virtual init() -> void;
        auto virtual update() -> void = 0;

        auto setParkingStations(std::set<std::string> parking_stations) -> void;
        auto parkingStations() -> std::set<std::string>;

        // 临时让车动起来
        // auto getDeviceStatus() const -> const int;
        // auto setDeviceStatus(int status) -> void;
    protected:
        // auto setState(State state)->void;

    protected:
        struct Imp;
        std::unique_ptr<Imp> imp_;

        // 修改
        //KAANHRCS_DELETE_BIG_FOUR(DeviceAgv)
    };
    //new void to_json(nlohmann::json &j, const DeviceAgv &agv);

    // class KAANHRCS_API VirtualAgv : public DeviceAgv
    // {
    // public:
    //     VirtualAgv();
    //     ~VirtualAgv();

    //     auto accVel() const -> double;
    //     auto setAccVel(double vel) -> void;
    //     auto decVel() const -> double;
    //     auto setDecVel(double vel) -> void;
    //     auto rotationalRefVel() -> double;
    //     auto setRotationalRefVel(double vel) -> void;
    //     auto rotationalVel() -> double;
    //     auto setRotationalVel(double vel) -> void;
    //     auto rotationalAccVel() -> double;
    //     auto setrotationalAccVel(double vel) -> void;
    //     auto rotationalDecVel() -> double;
    //     auto setrotationalDecVel(double vel) -> void;

    //     auto popFrontPath() -> void;

    //     auto moveToStation(const std::string &name, const std::vector<std::string> &path = {}) -> void override;

    //     auto goOn() -> void override;
    //     auto pause() -> void override;
    //     auto cancelMove() -> void override;
    //     auto eStop() -> void override;
    //     auto cancelEStop() -> void override;

    //     auto init() -> void override;
    //     auto update() -> void override;

    // private:
    //     struct Imp;
    //     std::unique_ptr<Imp> imp_;

    //     KAANHRCS_DELETE_BIG_FOUR(VirtualAgv)
    // };
};

#endif // DEVICE_AGV_HPP_
