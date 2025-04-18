#include "device_agv.hpp"

#include <atomic>
#include <mutex>

// #include "aris/core/reflection.hpp"
// #include "kaanhrcs/core/tool_api.hpp"
// #include "kaanhrcs/core/macro.hpp"
// #include "kaanhrcs/server/control_server.hpp"
// #include "kaanhrcs/core/string_process.hpp"
// #include "kaanhrcs/order/order_manager.hpp"
#define MAX_VEL 1000.0
#define PI 3.14159265358979323846

#define JudgeTime(rmtime, dectime) \
    if (rmtime > dectime)          \
    {                              \
        rmtime -= dectime;         \
        continue;                  \
    }                              \
    else                           \
        break;
// new
// auto getAngel(std::pair<double, double> p1, std::pair<double, double> p2, std::pair<double, double> p3 = {0, 0}) -> double
// {
//     auto at1 = atan2(p1.second - p3.second, p1.first - p3.first);
//     auto at2 = atan2(p2.second - p3.second, p2.first - p3.first);
//     double theta = at2 - at1;
//     if (theta > PI)
//         theta -= 2 * PI;
//     else if (theta < -PI)
//         theta += 2 * PI;

//     return theta * 180 / PI; // 返回角度
// }
auto clockWise(double rttheta, double theta) -> double
{
    if (rttheta < 0)
        return -theta;
    return theta;
}

namespace deviceagv
{

auto toAgvProductCode(const std::string &code) -> AgvProductCode
{
    if (code == "Null")
    {
        return AgvProductCode::kNull;
    }
    else if (code == "Stdhttp")
    {
        return AgvProductCode::kStdhttp;
    }
    else if (code == "Stdmodbus")
    {
        return AgvProductCode::kStdmodbus;
    }
    else if (code == "Seermodbus")
    {
        return AgvProductCode::KSeermodbus;
    }
    else if (code == "VDA5050")
    {
        return AgvProductCode::kVDA5050;
    }
    return AgvProductCode::kNull;
}
auto getAgvModes() -> std::vector<std::string> &
{
    static std::vector<std::string> agvmodes{"Auto", "Invalid", "Manual"};
    return agvmodes;
}

auto getAgvProductCode() -> std::vector<std::string> &
{
    static std::vector<std::string> agvproducts{"null", "Stdhttp", "Stdmodbus","Seermodbus"};
    return agvproducts;
}

struct DeviceAgv::Imp
{
    std::string                                     name_{"null"}, agv_map_{"null"}, id_{"null"};
    std::atomic<double>                             x_{0}, y_{0}, theta_{0}, width_{0}, height_{0};
    std::any                                        data_;
    std::string                                     err_msg_;
    std::string                                     type_{"Agv"};
    // std::atomic<State> state_;
    std::atomic<double>                             virtual_vel_{100}; // ms/s
    //std::unique_ptr<kaanhrcs::agvmanager::AgvTray>  agvtray_{new kaanhrcs::agvmanager::AgvTray};
    std::atomic_bool                                is_serving_{false};
    std::set<std::string>                           serve_devices_{};
    std::set<std::string>                           device_tools_{};
    int                                             vel_{0};
    int                                             battery_{80};
    std::uint16_t                                   current_station_id_{0};
    std::vector<std::string>                          path_;
    std::recursive_mutex                            mtex_path_;
    std::string                                     laststation_{},currentstation_{},nextstation_{};
    // 新增变量
    int                                             laststationInt_{},currentstationInt_{},nextstationInt_{};
    std::vector<int>                                pathInt_,restPathInt_;
    int                                             finialGoalInt_{},currentGoalInt_{};
    //
    AgvSysState                                     sysstate_{AgvSysState::kOffline};
    AgvMoveState                                    movestate_{AgvMoveState::kIdle};
    AgvControlMode                                  controlmode_{AgvControlMode::kAuto};
    StateCbkFuncT                                   agvSysStateCbk_{nullptr};
    StateCbkFuncT                                   agvMoveStateCbk_{nullptr};
    StateCbkFuncT                                   agvControlModeCbk_{nullptr};
    double                                          ref_vel_{700};
    std::mutex                                      mtx_name_, mtx_map_, mtx_data_, mtx_msg_;
    //std::unique_ptr<kaanhcode::PgmController>       pgmcontroller_{new kaanhcode::PgmController};
    std::set<std::string>                           parking_stations;
};
// 新增接口
auto DeviceAgv::lastStationInt() const -> int{
    return imp_->laststationInt_;
}
auto DeviceAgv::currentStationInt() const -> int{
    return imp_->currentstationInt_;
}
auto DeviceAgv::nextStationInt() const -> int{
    return imp_->nextstationInt_;
}
auto DeviceAgv::setLastStationInt(const int & name)->void{
    imp_->laststationInt_ = name;
}
auto DeviceAgv::setCurrentStationInt(const int & name)->void{
    imp_->currentstationInt_ = name;
}
auto DeviceAgv::setNextStationInt(const int & name)->void{
    imp_->nextstationInt_ = name;
}
auto DeviceAgv::finialGoalInt() const -> int{
    return imp_->finialGoalInt_;
}
auto DeviceAgv::currentGoalInt() const -> int{
    return imp_->currentGoalInt_;
}
auto DeviceAgv::setFinialGoalInt(const int & name)->void{
    imp_->finialGoalInt_ = name;
}
auto DeviceAgv::setCurrentGoalInt(const int & name)->void{
    imp_->currentGoalInt_ = name;
}
auto DeviceAgv::pathInt() const -> std::vector<int>{
    std::lock_guard<std::recursive_mutex> guard(imp_->mtex_path_);
    return imp_->pathInt_;
}
auto DeviceAgv::setPathInt(const std::vector<int>& path, bool append, int goal) -> void
{
    std::lock_guard<std::recursive_mutex> guard(imp_->mtex_path_);
    imp_->pathInt_.clear();
    imp_->pathInt_.resize(path.size());
    std::copy(path.begin(), path.end(), imp_->pathInt_.begin());
}
auto DeviceAgv::restPathInt() const -> std::vector<int>{
    std::lock_guard<std::recursive_mutex> guard(imp_->mtex_path_);
    return imp_->restPathInt_;
}
auto DeviceAgv::setRestPathInt(const std::vector<int> &reqpath) -> void
{
    std::lock_guard<std::recursive_mutex> guard(imp_->mtex_path_);
    imp_->restPathInt_.clear();
    imp_->restPathInt_.resize(reqpath.size());
    std::copy(reqpath.begin(),reqpath.end(),imp_->restPathInt_.begin());
}
//*******


auto DeviceAgv::lastStation() const -> std::string{
    return imp_->laststation_;
}
auto DeviceAgv::currentStation() const -> std::string{
    return imp_->currentstation_;
}
auto DeviceAgv::nextStation() const -> std::string{
    return imp_->nextstation_;
}
auto DeviceAgv::setLastStation(const std::string & name)->void{
    imp_->laststation_ = name;
}
auto DeviceAgv::setCurrentStation(const std::string & name)->void{
    imp_->currentstation_ = name;
}

auto DeviceAgv::setCurrentStationId(const std::uint16_t stationId) -> void{
    imp_->current_station_id_ = stationId;
}
auto DeviceAgv::currentStationId() const -> std::uint16_t{
    return imp_->current_station_id_;
}
auto DeviceAgv::setNextStation(const std::string & name)->void{
    imp_->nextstation_ = name;
}
auto DeviceAgv::batteryPercent() const -> int{
    return imp_->battery_;
}
auto DeviceAgv::setBatteryPercent(int battery) const -> void{
    imp_->battery_ = battery;
}
auto DeviceAgv::vel() const -> double{
    return imp_->vel_;
}
auto DeviceAgv::setVel(int vell) const -> void{
    imp_->vel_ = vell;
}
//new
// auto DeviceAgv::agvPgmController() const->kaanhcode::PgmController&{
//     return *imp_->pgmcontroller_;
// }
 auto DeviceAgv::init() -> void{
//     kaanhcode::PgmConfig * agv_config = new kaanhcode::PgmConfig(ControlServer::instance().orderManager().pgmconfig());
//     imp_->pgmcontroller_->resetConfig(std::move(agv_config));
//     imp_->pgmcontroller_->resetGlobalDataPkg(ControlServer::instance().orderManager().globalDataPkg());
//     if(!std::filesystem::exists("./program/Agvs/" + name()))
//         std::filesystem::create_directories("./program/Agvs/" + name());
//     imp_->pgmcontroller_->resetPath("./program/Agvs/" + name());
//     imp_->pgmcontroller_->init();

//     if(name() != "" && name() != "null"){   
//         auto &dbManager = ControlServer::instance().dataBaseManager();
//         std::unique_ptr<Standard::AgvStatus> data{new Standard::AgvStatus};
//         data->setAgvName(name());
//         dbManager.setAgvStatusData(name(),std::move(data));
//         dbManager.agvStatusData(name()).setStartTime(dbManager.agvStatusManager().currentTimestamp());
//     }
 }
auto DeviceAgv::setAgvControlMode(AgvControlMode state) -> void
{
    if (imp_->agvSysStateCbk_)
        imp_->agvControlModeCbk_(int(state));
    imp_->controlmode_ = state;
}
auto DeviceAgv::agvControlMode() const -> DeviceAgv::AgvControlMode
{
    return imp_->controlmode_;
}
auto DeviceAgv::setAgvControlModeCbk(StateCbkFuncT cbk) -> void
{
    imp_->agvControlModeCbk_ = cbk;
}

auto DeviceAgv::setAgvSysState(AgvSysState state) -> void{
    imp_->sysstate_ = state;
}
auto DeviceAgv::agvSysState() const -> AgvSysState{
    return imp_->sysstate_;
}
auto DeviceAgv::setAgvSysStateCbk(StateCbkFuncT cbk) -> void{
    imp_->agvSysStateCbk_ = cbk;
}
auto DeviceAgv::setAgvMoveState(AgvMoveState state) -> void{
    imp_->movestate_ = state;
}
auto DeviceAgv::agvMoveState() const -> AgvMoveState{
    return imp_->movestate_;
}
auto DeviceAgv::setAgvMoveStateCbk(StateCbkFuncT cbk) -> void{
    imp_->agvMoveStateCbk_ = cbk;
}
auto DeviceAgv::path() const -> std::vector<std::string>{
    std::lock_guard<std::recursive_mutex> guard(imp_->mtex_path_);
    return imp_->path_;
}
auto DeviceAgv::setPath(const std::vector<std::string> &reqpath) -> void
{
    std::lock_guard<std::recursive_mutex> guard(imp_->mtex_path_);
    imp_->path_.clear();
    imp_->path_.resize(reqpath.size());
    std::copy(reqpath.begin(),reqpath.end(),imp_->path_.begin());
}
auto DeviceAgv::targetStation() const ->std::string{
    if(path().size() > 0) return path().back();
    return "null";
}
auto DeviceAgv::name() const -> const std::string &
{
    // std::lock_guard<std::mutex> guard(imp_->mtx_name_);
    return imp_->name_;
}

auto DeviceAgv::setName(const std::string &name) -> void
{
    std::lock_guard<std::mutex> guard(imp_->mtx_name_);
    imp_->name_ = name;
}

auto DeviceAgv::id() const -> const std::string &
{
    return imp_->id_;
}
auto DeviceAgv::setId(const std::string &agvid) -> void
{
    imp_->id_ = agvid;
}
auto DeviceAgv::type() const -> const std::string &
{
    return imp_->type_;
}

auto DeviceAgv::setType(const std::string &agvtype) -> void
{
    imp_->type_ = agvtype;
}
auto DeviceAgv::x() const -> double
{
    return imp_->x_.load();
}

auto DeviceAgv::setX(double x) -> void
{
    imp_->x_.store(x);
}

auto DeviceAgv::y() const -> double
{
    return imp_->y_.load();
}

auto DeviceAgv::setY(double y) -> void
{
    imp_->y_.store(y);
}

auto DeviceAgv::theta() const -> double
{
    return imp_->theta_.load();
}

auto DeviceAgv::setTheta(double theta) -> void
{
    imp_->theta_.store(theta);
}

auto DeviceAgv::width() const -> double
{
    return imp_->width_.load();
}

auto DeviceAgv::setWidth(double width) -> void
{
    imp_->width_.store(width);
}

auto DeviceAgv::height() const -> double
{
    return imp_->height_.load();
}

auto DeviceAgv::setHeight(double height) -> void
{
    imp_->height_.store(height);
}

auto DeviceAgv::setDeviceData(std::any data) -> void
{
    std::lock_guard<std::mutex> guard(imp_->mtx_data_);
    imp_->data_ = data;
}

auto DeviceAgv::deviceData() -> const std::any &
{
    // std::lock_guard<std::mutex> guard(imp_->mtx_data_);
    return imp_->data_;
}

auto DeviceAgv::errMsg() const -> const std::string &
{
    // std::lock_guard<std::mutex> guard(imp_->mtx_msg_);
    return imp_->err_msg_;
}

auto DeviceAgv::setErrMsg(const std::string &err_msg) -> void
{
    std::lock_guard<std::mutex> guard(imp_->mtx_msg_);
    imp_->err_msg_ = err_msg;
}

auto DeviceAgv::agvMap() const -> const std::string &
{
    // std::lock_guard<std::mutex> guard(imp_->mtx_map_);
    return imp_->agv_map_;
}

auto DeviceAgv::setAgvMap(const std::string &map_name) -> void
    {
        std::lock_guard<std::mutex> guard(imp_->mtx_map_);
        imp_->agv_map_ = map_name;
    }

// auto DeviceAgv::setAgvTray(kaanhrcs::agvmanager::AgvTray* tray)->void{
//     imp_->agvtray_.reset(tray);
// }
// auto DeviceAgv::agvTray()const ->kaanhrcs::agvmanager::AgvTray&{
//     return *imp_->agvtray_;
// }

// auto DeviceAgv::beginServer() -> void
// {
//     assert(!imp_->is_serving_.load());
//     imp_->is_serving_.store(true);
// }
// auto DeviceAgv::finishServer() -> void
// {
//     assert(imp_->is_serving_.load());

//     imp_->is_serving_.store(false);
// }
auto DeviceAgv::isServing() const -> bool
{
    return imp_->is_serving_;
}

auto DeviceAgv::servedDevices() const -> const std::set<std::string> &
{
    return imp_->serve_devices_;
}
auto DeviceAgv::setServedDevices(const std::set<std::string> &devices) -> void
{
    imp_->serve_devices_ = devices;
}

auto DeviceAgv::setParkingStations(std::set<std::string> parking_stations) -> void
{
    imp_->parking_stations = parking_stations;
}
auto DeviceAgv::parkingStations() -> std::set<std::string>
{
    return imp_->parking_stations;
}

auto DeviceAgv::refVel() const -> double { return imp_->ref_vel_; }

auto DeviceAgv::setRefVel(double ref_vel)->void { imp_->ref_vel_ =  ref_vel<=0? 150 : ref_vel; }
auto DeviceAgv::setDeviceTools(const std::set<std::string> &devtools) -> void{
    imp_->device_tools_ = devtools;
}
auto DeviceAgv::deviceTools() const -> std::set<std::string>{
    return imp_->device_tools_;
}
DeviceAgv::DeviceAgv()
    : imp_(new Imp) {
    }

DeviceAgv::~DeviceAgv()
{
    // if(name() != "" && name() != "null"){
    //     auto &dbManager = ControlServer::instance().dataBaseManager();
    //     dbManager.agvStatusData(name()).setEndTime(dbManager.agvStatusManager().currentTimestamp());
    //     dbManager.agvStatusManager().insertAgvStatus(dbManager.agvStatusData(name()));
    // }
}

// void to_json(nlohmann::json &j, const DeviceAgv &agv)
// {
//     j["id"] = agv.id();
//     j["devName"] = agv.name();
//     j["mapid"] = agv.agvMap();
//     j["servereddevices"] = agv.servedDevices();
//     j["type"] = agv.type();
//     j["x"] = agv.x() / ControlServer::instance().mapManager().map(agv.agvMap()).xRatio();
//     j["y"] = agv.y() / ControlServer::instance().mapManager().map(agv.agvMap()).yRatio();
//     j["theta"] = agv.theta();
//     j["width"] = agv.width() / fabs(ControlServer::instance().mapManager().map(agv.agvMap()).xRatio());
//     j["height"] = agv.height() / fabs(ControlServer::instance().mapManager().map(agv.agvMap()).yRatio());
//     // j["status"]         = agv.getDeviceStatus();
//     j["sysstatus"] = int(agv.agvSysState());
//     j["movestatus"] = int(agv.agvMoveState());
//     j["controlmode"] = int(agv.agvControlMode());
//     j["batteryPercent"] = agv.batteryPercent();
//     j["vel"] = agv.vel();
//     j["currentStation"] = agv.currentStation();
//     j["tray"] = agv.agvTray();
// }
// ARIS_REGISTRATION
// {
//     auto setParkingStation = [](deviceagv::DeviceAgv *object, std::string &str)
//     {
//         const auto &str_vec = splitString(str, ";");
//         std::set<std::string> stations(str_vec.begin(), str_vec.end());
//         object->setParkingStations(stations);
//     };
//     auto parkingStation = [](deviceagv::DeviceAgv *object) -> std::string
//     {
//         std::string ret;
//         for (const auto &machine : object->parkingStations())
//             ret += machine + ";";
//         if (ret.back() == ';')
//             ret.pop_back();
//         return ret;
//     };

//     auto setServedMachines = [](deviceagv::DeviceAgv *object, std::string &str)
//     {
//         const auto &str_vec = splitString(str, ";");
//         std::set<std::string> machines(str_vec.begin(), str_vec.end());
//         object->setServedDevices(machines);
//     };
//     auto servedMachines = [](deviceagv::DeviceAgv *object) -> std::string
//     {
//         std::string ret;
//         for (const auto &machine : object->servedDevices())
//             ret += machine + ";";
//         if (ret.back() == ';')
//             ret.pop_back();
//         return ret;
//     };
    // auto setDevTools= [](deviceagv::DeviceAgv *object, std::string &str) {
    //     const auto &str_vec = splitString(str, ";");
    //     std::set<std::string> machines(str_vec.begin(), str_vec.end());
    //     object->setDeviceTools(machines);
    // };
    // auto devTools = [](deviceagv::DeviceAgv *object)->std::string {
    //     std::string ret;
    //     for (const auto &machine : object->deviceTools())
    //         ret += machine + ";";
    //     if (ret.back()==';')
    //         ret.pop_back();
    //     return ret;
    // };

// aris::core::class_<deviceagv::DeviceAgv>("DeviceAgv")
//     .prop("name", &deviceagv::DeviceAgv::setName, &deviceagv::DeviceAgv::name)
//     .prop("id", &deviceagv::DeviceAgv::setId, &deviceagv::DeviceAgv::id)
//     .prop("type", &deviceagv::DeviceAgv::setType, &deviceagv::DeviceAgv::type)
//     .prop("x", &deviceagv::DeviceAgv::setX, &deviceagv::DeviceAgv::x)
//     .prop("y", &deviceagv::DeviceAgv::setY, &deviceagv::DeviceAgv::y)
//     .prop("serve_device_names", &setServedMachines, &servedMachines)
//     .prop("parking_station_names", &setParkingStation, &parkingStation)
//     .prop("theta", &deviceagv::DeviceAgv::setTheta, &deviceagv::DeviceAgv::theta)
//     .prop("width", &deviceagv::DeviceAgv::setWidth, &deviceagv::DeviceAgv::width)
//     .prop("height", &deviceagv::DeviceAgv::setHeight, &deviceagv::DeviceAgv::height)
//     .prop("map_name", &deviceagv::DeviceAgv::setAgvMap, &deviceagv::DeviceAgv::agvMap)
//     .prop("ref_vel", &deviceagv::DeviceAgv::setRefVel, &deviceagv::DeviceAgv::refVel)
//     .prop("agv_tray", &deviceagv::DeviceAgv::setAgvTray, &deviceagv::DeviceAgv::agvTray)
//     ;
}
