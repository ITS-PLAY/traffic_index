#ifndef _EVENT_DETECT_IN_CLASS_H
#define _EVENT_DETECT_IN_CLASS_H

#include "traffic_incident.h"

using namespace std;

class Device_Detect{                                                           //交叉口事件汇总类
public:
      Device_Detect() {};
      Device_Detect(string json_File_Name){
            read_Detect_JsonFile(json_File_Name, detect_Config);
            detect_Config_Points = get<0>(detect_Config);
            lanes_Num = get<1>(detect_Config);
            volume_test = Volume_Caculation(lanes_Num,true, detect_Config_Points);
            speed_test = Space_Speed_Caculation(lanes_Num, window_Interval, MaxLimitedSpeed, detect_Config_Points); 
            capacity_test = Capacity_Caculation(lanes_Num, sat_Max_Headway, MaxLimitedSpeed, detect_Config_Points);
            max_queue_test = Max_Queue_Caculation(lanes_Num, true, speed_Start, speed_End, min_Vehs_Size, map_Lanes_Queue, detect_Config_Points);  //定义排队长度指标
	          //stops_test = Stops_Caculation(lanes_Num, window_Interval, speed_Start, min_Stop_Duration, detect_Config_Points);
	  }; 

public:
	  Volume_Caculation volume_test;  
      Space_Speed_Caculation speed_test;
      Capacity_Caculation capacity_test;                  
	  Max_Queue_Caculation max_queue_test;
      //Stops_Caculation stops_test;

      tuple<map<string, vector<vector<Point>>>, int> detect_Config;             //配置文件        
      map<string, vector<vector<Point>>> detect_Config_Points;                  //检测区间
      int lanes_Num;                                                            //进口道的车道总数
	  map<int, Vehicleincident_Detection> map_Vehs_Entry, map_Vehs_Stop;        //记录entry-stop对中，车辆上一时刻的信息
      map<int, bool> map_Lanes_Queue;                                           //记录车道排队的状态

	  map<int, Vehicleincident_Detection> map_Vehs_Entry_noparking, map_Vehs_Stop_noparking;
	  map<int, Vehicleincident_Detection> map_Vehs_Entry_intersection, map_Vehs_Stop_intersection;
	  map<int, Vehicleincident_Detection> illegal;
      map<int,int> map_trafficincident;

      vector<Vehicleincident_Detection> vehs_test;
      bool gatewaywarninglable = false;
	  int congestionlabel = 0;    
};

class Event_detect
{
  public:

    void recieve_data();  

  public:
    Event_detect();
    ~Event_detect();

  public:
    tuple<int> intersection_Config;                                                //交叉口配置
    map<int, Device_Detect> intersection_Devices_Detect;                          //根据雷达数，建立对应的事件变量数组
    int eventID = 0;                                                             //事件ID编号
    bool flag = false;                                                           //时间间隔内，是否已统计

};
#endif