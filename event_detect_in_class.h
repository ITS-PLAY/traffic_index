#ifndef _EVENT_DETECT_IN_CLASS_H
#define _EVENT_DETECT_IN_CLASS_H

#include "traffic_incident.h"

using namespace std;

class Device_Detect {                                                                                  //交叉口事件汇总类
public:
    Device_Detect() {};
    Device_Detect(string json_File_Name) {
        read_Detect_JsonFile(json_File_Name, detect_Config);
        detect_Config_Points = get<0>(detect_Config);
        lanes_Num = get<1>(detect_Config);
        volume_test = Volume_Caculation(lanes_Num,true, detect_Config_Points);
        speed_test = Space_Speed_Caculation(lanes_Num, 1800, 60.0/3.6, detect_Config_Points); 
        capacity_test = Capacity_Caculation(lanes_Num, 4.0, 60.0/3.6, detect_Config_Points);
        max_queue_test = Max_Queue_Caculation(lanes_Num, true, 5.0/3.6, 20/3.6, 2, map_Lanes_Queue, detect_Config_Points);  
        //stops_test = Stops_Caculation(lanes_Num, window_Interval, speed_Start, min_Stop_Duration, detect_Config_Points);
    };

public:
    Volume_Caculation volume_test;                                                                     //定义流量指标
    Space_Speed_Caculation speed_test;                                                                 //定义平均空间速度指标
    Capacity_Caculation capacity_test;                                                                 //定义平均车头时距指标
    Max_Queue_Caculation max_queue_test;                                                               //定义排队长度指标
  //Stops_Caculation stops_test;                                                                     //定义停车次数指标

    tuple<map<string, vector<vector<Point>>>, int> detect_Config;                                      //配置文件        
    map<string, vector<vector<Point>>> detect_Config_Points;                                           //检测区间
    int lanes_Num;                                                                                     //进口道的车道总数
    map<int, Vehicleincident_Detection> map_Vehs_Entry, map_Vehs_Stop;                                 //记录entry-stop对中，车辆上一时刻的信息
    map<int, bool> map_Lanes_Queue;                                                                    //记录车道排队的状态

    map<int, Vehicleincident_Detection> map_Vehs_Entry_noparking, map_Vehs_Stop_noparking;
    map<int, Vehicleincident_Detection> map_Vehs_Entry_intersection, map_Vehs_Stop_intersection;
    map<int, Vehicleincident_Detection> illegal;
    map<int, int> map_trafficincident;

    vector<Vehicleincident_Detection> vehs_test;
    bool gatewaywarninglable = false;
    int link_congestionlabel = 0;
    map<int, int> lane_congestionlabel;
};

class Event_detect
{
  public:

    void recieve_data();  
    void update_Config_Info();

  public:
    Event_detect();
    ~Event_detect();

  public:
    tuple<int, double, double, double, double, double, int, int, double, double, double, double, double, double, int, double, double, double, double, int, double, int> intersection_Config;            //交叉口配置
    map<int, Device_Detect> intersection_Devices_Detect;                          //根据雷达数，建立对应的事件变量数组
    int eventID = 0;                                                             //事件ID编号
    bool flag = false;                                                           //时间间隔内，是否已统计
    bool test_flag = false;

    string config_File_Name = "";     //配置文件的目录

    int devices_num = 4;                                                                               //雷达数/进口道数
    double MaxSpeedUpper = 200 / 3.6;                                                                    //车辆最高速度                                                                                                                                        
    double MaxLimitedSpeed = 60.0 / 3.6;                                                                 //路段限速值
    double MinLimitedSpeed = 5.0 / 3.6;
    double ParkingSpeed = 2.0 / 3.6;
    double NegativeSpeed = 5.0 / 3.6;
    int MaxLimitedVehicleNum_Link = 8;
    int MaxLimitedVehicleNum_Lane = 3;
    double MaxLimitedPresenceTime = 20.0;
    double MaxLimitedAccidentTime = 20.0;
    double MaxLimitedTime = 50000.0;
    double CongestionSpeed_Slight = 20.0 / 3.6;
    double CongestionSpeed_Moderate = 10.0 / 3.6;
    double CongestionSpeed_Severe = 5.0 / 3.6;
    int window_Interval = 30 * 60;                                                                       //车辆数据保存的最大时间窗口                                                                       
    double time_Interval = 60.0;
    int pz = -1;

};
#endif
