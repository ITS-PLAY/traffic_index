#pragma once
#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <queue>
#include <deque>
#include <math.h>
#include <ostream>
#include <fstream>
#include <stdio.h>
#include "json/json.h"
#include <fstream>
#include <ostream>

using namespace std;
map<string, double> car_Type = { {"",1.0}, {"light",1.0},{"middle",1.5},{"oversize",2.5} };   //不同类型车辆的换算系数
static const double time_Reaction = 1.5, dist_Safe = 1.5, surface_Ratio = 0.8;       //刹车的反应时间，两车安全距离，沥青路面附着系数
map<string, double> speed_Deceleration_Value = { {"light",6.05},{"middle",5.6},{"oversize",4.75} };  //不同类型车辆的刹车加速度值
static const int lanes_Num = 4;                 //车道数

struct Point {                 //检测区域的二维坐标点
	Point() { x = 0.0; y = 0.0; }
	Point(double mx, double my) :x(mx), y(my) {};
	double x;
	double y;
};
 
class Vehicle {                  //车辆类
public:
	Vehicle() {}
	Vehicle(double msec, double mnsec, int mid, int mlane_Num, uint32_t mlabel, string mtype, double mx, double my, double mvx, double mvy, double mlength, double mwidth) :
		sec(msec), nsec(mnsec), id(mid), lane_Num(mlane_Num), label(mlabel), cartype(mtype), radar_px(mx), radar_py(my), radar_vx(mvx), radar_vy(mvy), dimensions_x(mlength), dimensions_y(mwidth) {
		caculation_Value();
	}
	double sec;                     //秒
	double nsec;                    //纳秒
	int id;                      //目标ID
	int lane_Num;                //获取车辆车道编号
	uint32_t label;                //目标类别 
	string cartype;              //车辆类型
	double radar_px;             //雷达x轴坐标
	double radar_py;             //雷达y轴坐标
	double radar_vx;             //雷达x轴速度
	double radar_vy;             //雷达y轴速度
	double dimensions_x;         //目标长度
	double dimensions_y;         //目标宽度
public:
	void caculation_Value();
	double speed;                //换算得到速度值
	double timestamp;            //换算得到时间
};

class Vehicle_Extent:public Vehicle {
public:
	Vehicle_Extent() {};
	Vehicle_Extent(double msec, double mnsec, int mid, int mlane_Num, uint32_t mlabel, string mtype, double mx, double my, double mvx, double mvy, double mlength, double mwidth) :
		Vehicle(msec, mnsec, mid, mlane_Num, mlabel, mtype, mx, my, mvx, mvy, mlength, mwidth) {
		veh_In_Zone = false;
		drive_In_Zone = false;
		drive_Out_Zone = false;
		veh_Is_Stop = false;
	};

public:
	bool veh_In_Zone;            //是否位于区域内
	bool drive_In_Zone;          //是否正在驶入区域
	double time_Drive_In;           //驶入区域的时间
	Point pos_Drive_In;          //驶入区域时的位置
	bool drive_Out_Zone;         //是否正在驶出区域
	double time_Drive_Out;          //驶出区域的时间
	Point pos_Drive_Out;         //驶出区域时的位置
	bool veh_Is_Stop;            //是否处于（不完全）停车状态
};

class Location_Detection {      //***车辆位置检测
public:
	Location_Detection() {}
	Location_Detection(string mline_type, map<string, vector<vector<Point>>> mdetect_Config):line_type(mline_type), detect_Config_Points(mdetect_Config){
		get_Boundary_Point();
	};
	virtual void detect_Location() = 0;
	void get_Boundary_Point();        //配置检测区域
	virtual Vehicle_Extent update_Veh() = 0;

public:
	string line_type;
	map<string, vector<vector<Point>>> detect_Config_Points;
	vector<vector<Point>> points;
};
//位置检测的详细函数
class Location_Cross_Line :public Location_Detection {        //**跨线检测
public:
	Location_Cross_Line() {}
	Location_Cross_Line(string mline_type, map<int, Vehicle_Extent> &mmap_Vehs, Vehicle_Extent mvehicle, map<string, vector<vector<Point>>> mdetect_Config,double mcurrent_Time, double mwindow_Interval) :
		Location_Detection(mline_type, mdetect_Config),map_Vehs(mmap_Vehs),vehicle(mvehicle),current_Time(mcurrent_Time),window_Interval(mwindow_Interval){
		detect_Location();                             //基于回归线两侧的正负号，进行判断
		mmap_Vehs = update_Map_Vehs();
	};
	void detect_Location();
	map<int, Vehicle_Extent> update_Map_Vehs();
	Vehicle_Extent update_Veh();
private:
	double current_Time;
	double window_Interval;
	Vehicle_Extent vehicle;
	map<int, Vehicle_Extent> map_Vehs;
};

class Location_In_Zone :public Location_Detection {      //**区域检测
public:
	Location_In_Zone() {};
	Location_In_Zone(string mline_type, Vehicle_Extent mvehicle, map<string, vector<vector<Point>>> mdetect_Config) :
		Location_Detection(mline_type,mdetect_Config), vehicle(mvehicle) {
		detect_Location();                             //基于向量内积法，进行判断
	};
	void detect_Location();
	Vehicle_Extent update_Veh();

private:
	Vehicle_Extent vehicle;
	double duration_Time;             //停留时间要求
};

class Location_In_Lane :public Location_Detection {      //**车道位置检测
public:
	Location_In_Lane() {};
	Location_In_Lane(string mline_type, Vehicle_Extent mvehicle, map<string, vector<vector<Point>>> mdetect_Config) :
		Location_Detection(mline_type,mdetect_Config), vehicle(mvehicle) {
		detect_Location();                             //基于向量内积法，进行判断
	};
	void detect_Location();
	Vehicle_Extent update_Veh();

private:
	Vehicle_Extent vehicle;
};

class Index_Caculation {        //***交通指标计算
public:
	Index_Caculation() {};
	Index_Caculation(double mtime_Interval, map<string, vector<vector<Point>>> mdetect_Config) :time_Interval(mtime_Interval) {
		get_Lanes_Info(mdetect_Config);
	};
	void get_Lanes_Info(map<string, vector<vector<Point>>> detect_Config);
	virtual void get_Vehicles_Info(Vehicle_Extent &veh) = 0;          //获取目标车辆的集合
	virtual void caculation_Index() = 0;             //指标计算
	virtual void update_Vehicles_Info() = 0;       //更新目标车辆的集合

public:
	double time_Interval = 1.0;                          //时间间隔
	vector<int> lane_Code;
public:
	double stop_Distance = 0.0;       //停止线到原点的距离
	double lanes_Length = 0.0;        //区域内所有车道的总长度
};

//指标计算的详细函数
class Volume_Caculation :public Index_Caculation {       //**统计流量
public:
	Volume_Caculation() {};
	Volume_Caculation(double mtime_Interval,bool msection_Flag, map<string, vector<vector<Point>>> mdetect_Config) :section_Flag(msection_Flag), Index_Caculation(mtime_Interval, mdetect_Config) {};
	void get_Vehicles_Info(Vehicle_Extent &veh);
	void caculation_Index();
	void update_Vehicles_Info();

private:
	bool section_Flag;
public:
	map<int, vector<Vehicle_Extent>> vehs_Set;
	double section_Volume=0.0;
	map<int, double> lanes_Volume;
};

class Time_Headway_Caculation:public Index_Caculation {     //**统计车头时距
public:
	Time_Headway_Caculation() {};
	Time_Headway_Caculation(double mtime_Interval, double msat_Max_Headway, map<string, vector<vector<Point>>> mdetect_Config): sat_Max_Headway(msat_Max_Headway),Index_Caculation(mtime_Interval, mdetect_Config) {}; //无前期流量统计结果
	Time_Headway_Caculation(map<int, vector<Vehicle_Extent>> mvehs_Set) :vehs_Set(mvehs_Set){
		caculation_Index();
	};                                          //已有流量统计结果
	void get_Vehicles_Info(Vehicle_Extent &veh);
	void caculation_Index();
	void update_Vehicles_Info();
public:
	map<int, vector<Vehicle_Extent>> vehs_Set;
	map<int, double> ave_Time_Headway;
	map<int, double> sat_Time_Headway;
	double sat_Max_Headway;                            //饱和车头时距的最大阈值
};

class Capacity_Caculation :public Time_Headway_Caculation {      //**计算车道通行能力
public:
	Capacity_Caculation() {};
	Capacity_Caculation(double mtime_Interval, double msat_Max_Headway,double mspeed_Max, map<string, vector<vector<Point>>> mdetect_Config) :Time_Headway_Caculation(mtime_Interval, msat_Max_Headway, mdetect_Config),speed_Max(mspeed_Max) {};
	Capacity_Caculation(map<int, vector<Vehicle_Extent>> mvehs_Set) :Time_Headway_Caculation(mvehs_Set) {};
	void caculation_Index();
	double time_Headway_Calibration(string cartype);        //标定车头时距值
public:
	map<int, double> lanes_Capactity;
	double speed_Max;                                       //路段的限速值
};

class Space_Speed_Caculation :public Index_Caculation {       //**计算区间平均速度、区间延误
public:
	Space_Speed_Caculation() {};
	Space_Speed_Caculation(double mtime_Interval,int mwindow_Interval,double mspeed_Max, map<string, vector<vector<Point>>> mdetect_Config) :Index_Caculation(mtime_Interval, mdetect_Config),window_Interval(mwindow_Interval),
		speed_Max(mspeed_Max){};
	void get_Vehicles_Info(Vehicle_Extent &veh);
	void caculation_Index();
	void update_Vehicles_Info();

private:
	map<int, Vehicle_Extent> vehs_Set;
	deque<int> vehs_ID_Set;
	double ave_Travel_Time = 0.0;
	double window_Interval=30;                                      //时间窗口的最大值，分钟
	double speed_Max;                                         //路段的限速值      

public:
	double current_Time = 0.0;                                         //当前时间
	double ave_Delay=0.0;
	double ave_Space_Speed=0.0;
};

class Headway_Density_Caculation :public Index_Caculation {     //**计算平均车头间距、平均车速和密度
public:
	Headway_Density_Caculation() {};
	Headway_Density_Caculation(double mtime_Interval,bool msection_Flag, map<string, vector<vector<Point>>> mdetect_Config) :section_Flag(msection_Flag), Index_Caculation(mtime_Interval, mdetect_Config) {};
	
	void get_Vehicles_Info(Vehicle_Extent &veh);
	void caculation_Index();
	void update_Vehicles_Info();

private:
	bool section_Flag = true;
	double section_Length = 50;    //区间长度
	map<int, double> lanes_Space_Headway_Sum;
	map<int, double> lanes_Time_Speed_Sum;
	map<int, double> lanes_Headway_Volume_Sum;
	map<int, int> lanes_Speed_Volume_Sum;
public:
	map<int, vector<Vehicle_Extent>> vehs_Set;
	map<int, double> lanes_Space_Headway;       //车道的平均车头间距
	map<int, double> lanes_Time_Speed;          //车道的平均速度
	double ave_Space_Headway=0.0;               //区域的平均车头间距 
	double ave_Time_Speed = 0.0;                 //区域的平均速度
	double section_density=0.0;                 //区域的密度
	int zone_volume = 0;                        //区域内的车辆数
};

class Max_Queue_Caculation :public Headway_Density_Caculation {         //**计算最大排队长度、最大排队车辆数（未换算）
public:
	Max_Queue_Caculation() {};
	Max_Queue_Caculation(double mtime_Interval,bool msection_Flag, double mspeed_Start, double mspeed_end, int mmin_Size, map<int, bool> mqueue_continue, map<string, vector<vector<Point>>> mdetect_Config) :Headway_Density_Caculation(mtime_Interval,msection_Flag, mdetect_Config),
		speed_Queue_Start(mspeed_Start), speed_Queue_End(mspeed_end), min_Queue_Size(mmin_Size), queue_Continue(mqueue_continue){
		max_Headway_Queue = speed_Queue_Start * time_Reaction + pow(speed_Queue_Start, 2.0) / (2 * speed_Deceleration_Value["light"]* surface_Ratio) + 5 + dist_Safe;  //vt+pow(v,2)/(2a)+length+dist_safe
		max_Headway_Slow = speed_Queue_End * time_Reaction + pow(speed_Queue_End, 2.0) / (2 * speed_Deceleration_Value["light"]* surface_Ratio) + 5 + dist_Safe;
		for (int i = 0; i < lane_Code.size(); i++) {
			queue_Continue.emplace(lane_Code[i], true);
		}
	};
	void create_Queue();
	void caculate_Queue(int lane_Num,int queue_Index,double speed);
	void caculation_Index();
	void update_Vehicles_Info();
	map<int, bool> update_Queue_Status();
	
private:
	map<int,queue<vector<Vehicle_Extent>>> queues_Set;
	map<int, vector<double>> queues_Density;
	double speed_Queue_Start = 5.0/3.6;            //排队形成的速度 m/s,默认值
	double speed_Queue_End = 20.0/3.6;              //排队消散的速度 m/s,默认值
	double max_Headway_Queue = 10.0;            //排队阶段的车队最大车头时距m,默认值
	double max_Headway_Slow = 20.0;             //缓行阶段的车队最大车头时距m,默认值
	int min_Queue_Size = 2;                    //车队的最小车辆数
	bool section_Flag=true;
public:
	map<int,double> lanes_Queue_Length;
	map<int, int> lanes_Queue_Num;
	double section_Queue_Length=0.0;
	int section_Queue_Num=0;
	map<int, bool> queue_Continue;
};

class Stops_Caculation :public Index_Caculation {                //**计算平均停车次数
public:
	Stops_Caculation() {};
	Stops_Caculation(double mtime_Interval,int mwindow_Interval,double mspeed_Stop_Start, double mmin_Stop_Duration, map<string, vector<vector<Point>>> mdetect_Config) :speed_Stop_Start(mspeed_Stop_Start), min_Stop_Duration(mmin_Stop_Duration),
		window_Interval(mwindow_Interval), Index_Caculation(mtime_Interval, mdetect_Config) {};
	void get_Vehicles_Info(Vehicle_Extent &veh);
	void caculation_Index();
	void update_Vehicles_Info();

private:
	map<int, vector<Vehicle_Extent>> vehs_Set;
	map<int, double> vehs_Stop_Start;
	map<int, double> vehs_Stops_Num;
	vector<int> vehs_ID;
	double window_Interval = 30;                                      //时间窗口的最大值，分钟
	double speed_Stop_Start = 5.0;                                 //车辆处于不完全停车的速度阈值
	double min_Stop_Duration = 3;                                     //停车持续时间的最小值
public:
	double current_Time = 0.0;                                         //当前时间
	double ave_Stops=0.0;
};

void ReadJsonFromFile(string filename, map<string, vector<vector<Point>>>& detect_Config_Points);