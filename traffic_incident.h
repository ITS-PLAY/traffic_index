#ifndef _TRAFFIC_INCIDENT_H
#define _TRAFFIC_INCIDENT_H

#pragma warning(disable:4996)
#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <math.h>
#include <queue>
#include <deque>
#include <tuple>
#include <stdlib.h>
#include <chrono>
#include <ctime>
#include "json/json.h" 
#include <ostream>
#include <fstream>

using namespace std;
using namespace std::chrono;

using second_Clock_Type = time_point<system_clock, seconds>;
static const map<string, double> car_Type = { {"",1.0}, {"light",1.0},{"middle",1.5},{"oversize",2.5} };          //不同类型车辆的换算系数
static const double time_Reaction = 1.5, dist_Safe = 1.5, surface_Ratio = 0.8;                                    //刹车的反应时间，两车安全距离，沥青路面附着系数
static const map<string, double> speed_Deceleration_Value = { {"light",6.05},{"middle",5.6},{"oversize",4.75} };  //不同类型车辆的刹车加速度值                                               

static const int window_Interval = 30 * 60;                                            //时间窗口
static const double time_Interval = 20.0;                                               //时间间隔
static const double sat_Max_Headway = 4.0;                                             //饱和车头时距的阈值
static const double speed_Start = 5.0 / 3.6, speed_End = 20.0 / 3.6;                   //排队形成的速度阈值，排队消散的速度阈值
static const int min_Vehs_Size = 2;                                                    //车队的最小车辆数
static const double min_Stop_Duration = 3.0;                                           //车辆停车判断的最小持续时间,秒

static const double MaxSpeedUpper = 200 / 3.6;                  //有效速度的最大值
static const double MaxLimitedSpeed = 60 / 3.6;					//默认60
static const double MinLimitedSpeed = 8 / 3.6;					//默认10
static const double ParkingSpeed = 4 / 3.6;						//默认2
static const double NegativeSpeed = -5 / 3.6;
static const int    MaxLimitedVehicleNum_Link = 5;				//默认10
static const int    MaxLimitedVehicleNum_Lane = 3;				//默认10
static const double MaxLimitedPresenceTime = 15;                //默认30
static const double MaxLimitedAccidentTime = 15;				//默认30
static const double MaxLimitedTime = 50000;						//默认50000
static const double Congestiondensity = 0.001;					//默认0.005
static const double CongestionSpeed_Slight = 20 / 3.6;			//s默认35
static const double CongestionSpeed_Moderate = 10 / 3.6;        //默认30
static const double CongestionSpeed_Severe = 5 / 3.6;			//默认20

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
	int lane_Num;				 //获取车辆车道编号
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


class Vehicle_Extent :public Vehicle {
public:
	Vehicle_Extent() {};
	Vehicle_Extent(double msec, double mnsec, int mid, int mlane_Num, uint32_t mlabel, string mtype, double mx, double my, double mvx, double mvy, double mlength, double mwidth) :
		Vehicle(msec, mnsec, mid, mlane_Num, mlabel, mtype, mx, my, mvx, mvy, mlength, mwidth) {
		veh_In_Zone = false;
		drive_In_Zone = false;
		drive_Out_Zone = false;

		veh_In_Intersection = false;
		drive_In_Intersection = false;
		drive_Out_Intersection = false;

		// veh_In_Zone_come = false;
		// drive_In_Zone_come = false;
		// drive_Out_Zone_come = false;

		// veh_In_Zone_leave = false;
		// drive_In_Zone_leave = false;
		// drive_Out_Zone_leave = false;


		// veh_In_Intersection_come = false;
		// drive_In_Intersection_come = false;
		// drive_Out_Intersection_come = false;

		// veh_In_Intersection_leave = false;
		// drive_In_Intersection_leave = false;
		// drive_Out_Intersection_leave = false;

		veh_Is_Stop = false;

		veh_In_NoParking = false;
		drive_In_NoParking = false;
		drive_Out_NoParking = false;
	};

public:
	bool veh_In_Zone;            			//是否位于区域内
	bool drive_In_Zone;          			//是否正在驶入区域
	bool drive_Out_Zone;         			//是否正在驶出区域
	double time_Veh_Recognize;              //车辆在区域内被首次检测到的时间
	double time_Drive_In;        			//驶入区域的时间
	double time_Drive_Out;       			//驶出区域的时间
	Point pos_Veh_Recognize;                //车辆在区域内被首次检测道的位置
	Point pos_Drive_In;          			//驶入区域时的位置
	Point pos_Drive_Out;         			//驶出区域时的位置

	// bool veh_In_Zone_come;					//是否位于来向区域内
	// bool drive_In_Zone_come;				//是否正在驶入来向区域
	// bool drive_Out_Zone_come;				//是否正在驶出来向区域
	// double time_Drive_In_come;				//驶入来向区域的时间
	// double time_Drive_Out_come;				//驶出来向区域的时间
	// Point pos_Drive_In_come;				//驶入来向区域的位置
	// Point pos_Drive_Out_come;				//驶出来向区域的位置

	// bool veh_In_Zone_leave;					//是否位于去向区域内
	// bool drive_In_Zone_leave;				//是否正在驶入去向区域
	// bool drive_Out_Zone_leave;				//是否正在驶出去向区域
	// double time_Drive_In_leave;				//驶入去向区域的时间
	// double time_Drive_Out_leave;			//驶出去向区域的时间
	// Point pos_Drive_In_leave;				//驶入去向区域的位置
	// Point pos_Drive_Out_leave;				//驶出去向区域的位置

	bool veh_Is_Stop;            			//是否处于（不完全）停车状态

public:
	bool veh_In_NoParking;					//是否在临时停车区域	
	bool drive_In_NoParking;				//是否正在驶入区域
	bool drive_Out_NoParking;				//是否正在驶出区域
	double time_Drive_In_NoParking;			//驶入临停区时间
	double time_Drive_Out_NoParking;		//驶出临停区时间
	Point pos_Drive_In_NoParking;			//驶入临停区位置
	Point pos_Drive_Out_NoParking;			//驶出临停区位置

	bool veh_In_Intersection;	 			//是否在交叉口区域
	bool drive_In_Intersection;				//是否正在驶入交叉口区域
	bool drive_Out_Intersection;			//是否正在驶出区域
	double time_Drive_In_Intersection;		//驶入交叉口时间
	double time_Drive_Out_Intersection;		//驶出交叉口时间
	Point pos_Drive_In_Intersection;		//驶入交叉口位置
	Point pos_Drive_Out_Intersection;		//驶出交叉口位置

	// bool veh_In_Intersection_come;	 			//是否在交叉口区域
	// bool drive_In_Intersection_come;			//是否正在驶入交叉口区域
	// bool drive_Out_Intersection_come;			//是否正在驶出区域
	// double time_Drive_In_Intersection_come;		//驶入交叉口时间
	// double time_Drive_Out_Intersection_come;	//驶出交叉口时间
	// Point pos_Drive_In_Intersection_come;		//驶入交叉口位置
	// Point pos_Drive_Out_Intersection_come;		//驶出交叉口位置

	// bool veh_In_Intersection_leave;	 			//是否在交叉口区域
	// bool drive_In_Intersection_leave;			//是否正在驶入交叉口区域
	// bool drive_Out_Intersection_leave;			//是否正在驶出区域
	// double time_Drive_In_Intersection_leave;	//驶入交叉口时间
	// double time_Drive_Out_Intersection_leave;	//驶出交叉口时间
	// Point pos_Drive_In_Intersection_leave;		//驶入交叉口位置
	// Point pos_Drive_Out_Intersection_leave;		//驶出交叉口位置

};

class Vehicleincident_Detection :public Vehicle_Extent {      //***车辆事件检测
public:
	Vehicleincident_Detection() {}
	Vehicleincident_Detection(double msec, double mnsec, int mid, int mlane_Num, uint32_t mlabel, string mtype, double mx, double my, double mvx, double mvy, double mlength, double mwidth) :
		Vehicle_Extent(msec, mnsec, mid, mlane_Num, mlabel, mtype, mx, my, mvx, mvy, mlength, mwidth) {};
	void Overspeed();
	void Lowspeed();
	void Retrograde();
	void Illegalparking();

public:
	bool overspeed = false;
	bool lowspeed = false;
	bool retrograde = false;
	bool illegalparking = false;
	bool illegallanechange = false;
	bool veh_In_Parking = false;
};


class Location_Detection {      //***车辆位置检测
public:
	Location_Detection() {}
	Location_Detection(string mline_type, map<string, vector<vector<Point>>> mdetect_Config) :line_type(mline_type), detect_Config_Points(mdetect_Config) {
		get_Boundary_Point();
	};
	virtual void detect_Location() = 0;
	void get_Boundary_Point();        //配置检测区域
	virtual Vehicleincident_Detection update_Veh() = 0;

public:
	string line_type;
	map<string, vector<vector<Point>>> detect_Config_Points;
	vector<vector<Point>> points;
};
//位置检测的详细函数
class Location_Cross_Line :public Location_Detection {        //**跨线检测
public:
	Location_Cross_Line() {}
	Location_Cross_Line(string mline_type, map<int, Vehicleincident_Detection>& mmap_Vehs, Vehicleincident_Detection mvehicle, map<string, vector<vector<Point>>> mdetect_Config, double mcurrent_Time, double mwindow_Interval) :
		Location_Detection(mline_type, mdetect_Config), map_Vehs(mmap_Vehs), vehicle(mvehicle), current_Time(mcurrent_Time), window_Interval(mwindow_Interval) {
		detect_Location();                             //基于回归线两侧的正负号，进行判断
		mmap_Vehs = update_Map_Vehs();
	};
	void detect_Location();
	map<int, Vehicleincident_Detection> update_Map_Vehs();
	Vehicleincident_Detection update_Veh();
public:
	double current_Time;
private:
	double window_Interval;
	Vehicleincident_Detection vehicle;
	map<int, Vehicleincident_Detection> map_Vehs;
};

class Location_In_Zone :public Location_Detection {      //**区域检测
public:
	Location_In_Zone() {};
	Location_In_Zone(string mline_type, Vehicleincident_Detection mvehicle, map<string, vector<vector<Point>>> mdetect_Config) :
		Location_Detection(mline_type, mdetect_Config), vehicle(mvehicle) {
		detect_Location();                             //基于向量内积法，进行判断
	};
	void detect_Location();
	Vehicleincident_Detection update_Veh();

private:
	Vehicleincident_Detection vehicle;
	double duration_Time;             //停留时间要求
};

class Location_In_Lane :public Location_Detection {      //**车道位置检测
public:
	Location_In_Lane() {};
	Location_In_Lane(string mline_type, Vehicleincident_Detection mvehicle, map<string, vector<vector<Point>>> mdetect_Config) :
		Location_Detection(mline_type, mdetect_Config), vehicle(mvehicle) {
		detect_Location();                             //基于向量内积法，进行判断
	};
	void detect_Location();
	Vehicleincident_Detection update_Veh();

private:
	Vehicleincident_Detection vehicle;
};

class Index_Caculation {        //***交通指标计算
public:
	Index_Caculation() {};
	Index_Caculation(int mlanes_Num, map<string, vector<vector<Point>>> mdetect_Config) :lanes_Num(mlanes_Num) {
		get_Lanes_Info(mdetect_Config);
	};
	void get_Lanes_Info(map<string, vector<vector<Point>>> detect_Config);
	virtual void get_Vehicles_Info(Vehicleincident_Detection& veh) = 0;          //获取目标车辆的集合
	virtual void caculation_Index() = 0;             //指标计算
	virtual void update_Vehicles_Info() = 0;       //更新目标车辆的集合

public:
	int lanes_Num = 1;                          //车道编号
	vector<int> lane_Code;
public:
	double stop_Distance = 0.0;       //停止线到原点的距离
	double lanes_Length = 0.0;        //区域内所有车道的总长度
};

//指标计算的详细函数
class Volume_Caculation :public Index_Caculation {       //**统计流量
public:
	Volume_Caculation() {};
	Volume_Caculation(int mlanes_Num, bool msection_Flag, map<string, vector<vector<Point>>> mdetect_Config) :section_Flag(msection_Flag), Index_Caculation(mlanes_Num, mdetect_Config) {};
	void get_Vehicles_Info(Vehicleincident_Detection& veh);
	void caculation_Index();
	void update_Vehicles_Info();

private:
	bool section_Flag;
public:
	map<int, vector<Vehicleincident_Detection>> vehs_Set;
	double section_Volume = 0.0;
	map<int, double> lanes_Volume;
};

class Time_Headway_Caculation :public Index_Caculation {     //**统计车头时距
public:
	Time_Headway_Caculation() {};
	Time_Headway_Caculation(int mlanes_Num, double msat_Max_Headway, map<string, vector<vector<Point>>> mdetect_Config) : sat_Max_Headway(msat_Max_Headway), Index_Caculation(mlanes_Num, mdetect_Config) {}; //无前期流量统计结果
	Time_Headway_Caculation(map<int, vector<Vehicleincident_Detection>> mvehs_Set) :vehs_Set(mvehs_Set) {
		caculation_Index();
	};                                          //已有流量统计结果
	void get_Vehicles_Info(Vehicleincident_Detection& veh);
	void caculation_Index();
	void update_Vehicles_Info();
public:
	map<int, vector<Vehicleincident_Detection>> vehs_Set;
	map<int, double> ave_Time_Headway;
	map<int, double> sat_Time_Headway;
	double sat_Max_Headway;                            //饱和车头时距的最大阈值
};

class Capacity_Caculation :public Time_Headway_Caculation {      //**计算车道通行能力
public:
	Capacity_Caculation() {};
	Capacity_Caculation(int mlanes_Num, double msat_Max_Headway, double mspeed_Max, map<string, vector<vector<Point>>> mdetect_Config) :Time_Headway_Caculation(mlanes_Num, msat_Max_Headway, mdetect_Config), speed_Max(mspeed_Max) {};
	Capacity_Caculation(map<int, vector<Vehicleincident_Detection>> mvehs_Set) :Time_Headway_Caculation(mvehs_Set) {};
	void caculation_Index();
	double time_Headway_Calibration(string cartype);        //标定车头时距值
public:
	map<int, double> lanes_Capactity;
	double speed_Max;                                       //路段的限速值
};

class Space_Speed_Caculation :public Index_Caculation {       //**计算区间平均速度、区间延误
public:
	Space_Speed_Caculation() {};
	Space_Speed_Caculation(int mlanes_Num, int mwindow_Interval, double mspeed_Max, map<string, vector<vector<Point>>> mdetect_Config) :Index_Caculation(mlanes_Num, mdetect_Config), window_Interval(mwindow_Interval),
		speed_Max(mspeed_Max) {};
	void get_Vehicles_Info(Vehicleincident_Detection& veh);
	void caculation_Index();
	void update_Vehicles_Info();

private:
	map<int, Vehicleincident_Detection> vehs_Set;
	deque<int> vehs_ID_Set;
	double ave_Travel_Time = 0.0;
	double window_Interval = 30;                                //时间窗口的最大值，分钟
	double speed_Max;                                         //路段的限速值      

public:
	double current_Time;                                      //当前时间
	double ave_Delay = 0.0;
	double ave_Space_Speed = 0.0;
	map<int, double> lanes_Space_Speed;                        //车道速度
	map<int, int> lanes_Space_Speed_Volume;                    //空间速度计算对应的车道流量
};

class Headway_Density_Caculation :public Index_Caculation {     //**计算平均车头间距、平均车速和密度
public:
	Headway_Density_Caculation() {};
	Headway_Density_Caculation(int mlanes_Num, bool msection_Flag, map<string, vector<vector<Point>>> mdetect_Config) :section_Flag(msection_Flag), Index_Caculation(mlanes_Num, mdetect_Config) {};

	void get_Vehicles_Info(Vehicleincident_Detection& veh);
	void caculation_Index();
	void update_Vehicles_Info();
	void linkcongestion();
private:
	bool section_Flag = true;
	double section_Length = 50;    //区间长度
	map<int, double> lanes_Space_Headway_Sum;
	map<int, double> lanes_Time_Speed_Sum;
	map<int, double> lanes_Headway_Volume_Sum;

public:
	map<int, int> lanes_Speed_Volume_Sum;		//车道的车辆数
	map<int, vector<Vehicleincident_Detection>> vehs_Set;
	map<int, double> lanes_Space_Headway;       //车道的平均车头间距
	map<int, double> lanes_Time_Speed;          //车道的平均速度
	double ave_Space_Headway = 0.0;               //区域的平均车头间距 
	double ave_Time_Speed = 0.0;                //区域的平均速度
	double section_density = 0.0;                 //区域的密度
	int zone_volume = 0;                         //区域内的车辆数
	map<int, int> lcongestion;					//拥堵
};

class Max_Queue_Caculation :public Headway_Density_Caculation {         //**计算最大排队长度、最大排队车辆数（未换算）
public:
	Max_Queue_Caculation() {};
	Max_Queue_Caculation(int mlanes_Num, bool msection_Flag, double mspeed_Start, double mspeed_end, int mmin_Size, map<int, bool> mqueue_continue, map<string, vector<vector<Point>>> mdetect_Config) :Headway_Density_Caculation(mlanes_Num, msection_Flag, mdetect_Config),
		speed_Queue_Start(mspeed_Start), speed_Queue_End(mspeed_end), min_Queue_Size(mmin_Size), queue_Continue(mqueue_continue) {
		max_Headway_Queue = speed_Queue_Start * time_Reaction + pow(speed_Queue_Start, 2.0) / (2 * speed_Deceleration_Value.at("light") * surface_Ratio) + 5 + dist_Safe;  //vt+pow(v,2)/(2a)+length+dist_safe
		max_Headway_Slow = speed_Queue_End * time_Reaction + pow(speed_Queue_End, 2.0) / (2 * speed_Deceleration_Value.at("light") * surface_Ratio) + 5 + dist_Safe;
		for (int i = 0; i < lane_Code.size(); i++) {
			queue_Continue.emplace(lane_Code[i], true);
		}
	};
	void create_Queue();
	void caculate_Queue(int lane_Num, int queue_Index, double speed);
	void caculation_Index();
	void update_Vehicles_Info();
	map<int, bool> update_Queue_Status();

private:
	map<int, queue<vector<Vehicleincident_Detection>>> queues_Set;
	map<int, vector<double>> queues_Density;
	double speed_Queue_Start = 5.0 / 3.6;            //排队形成的速度 m/s,默认值
	double speed_Queue_End = 20.0 / 3.6;              //排队消散的速度 m/s,默认值
	double max_Headway_Queue = 10.0;            //排队阶段的车队最大车头间距m,默认值
	double max_Headway_Slow = 20.0;             //缓行阶段的车队最大车头间距m,默认值
	int min_Queue_Size = 2;                    //车队的最小车辆数
	bool section_Flag = true;
public:
	map<int, double> lanes_Queue_Length;
	map<int, int> lanes_Queue_Num;
	double section_Queue_Length = 0.0;
	int section_Queue_Num = 0;
	map<int, bool> queue_Continue;
};

class Stops_Caculation :public Index_Caculation {                //**计算平均停车次数
public:
	Stops_Caculation() {};
	Stops_Caculation(int mlanes_Num, int mwindow_Interval, double mspeed_Stop_Start, double mmin_Stop_Duration, map<string, vector<vector<Point>>> mdetect_Config) :speed_Stop_Start(mspeed_Stop_Start), min_Stop_Duration(mmin_Stop_Duration),
		window_Interval(mwindow_Interval), Index_Caculation(mlanes_Num, mdetect_Config) {};
	void get_Vehicles_Info(Vehicleincident_Detection& veh);
	void caculation_Index();
	void update_Vehicles_Info();

private:
	map<int, vector<Vehicleincident_Detection>> vehs_Set;
	map<int, double> vehs_Stop_Start;
	map<int, double> vehs_Stops_Num;
	vector<int> vehs_ID;
	double window_Interval = 30;                                      //时间窗口的最大值，分钟
	double speed_Stop_Start = 5.0;                                 //车辆处于不完全停车的速度阈值
	double min_Stop_Duration = 3;                                     //停车持续时间的最小值

public:
	double current_Time = 0.0;                                         //当前时间
	double ave_Stops = 0.0;
};

void read_Detect_JsonFile(string filename, tuple<map<string, vector<vector<Point>>>, int>& detect_Config);                     //读取每个雷达对应进口道的json配置
void read_Intersection_JsonFile(string filename, tuple<int>& intersection_Config);                                            //读取每个交叉口的json配置  

Vehicleincident_Detection Illegallanechange(Vehicleincident_Detection test, map<int, Vehicleincident_Detection>& II);         //违法变道识别
bool Accident(Vehicleincident_Detection test);                                                                                //交通事件识别

bool Gateway(vector<Vehicleincident_Detection> vehs_test, int lanes_Num);                                                     //匝道汇入预警
int LinkCongestion(Max_Queue_Caculation max_queue_test);                             						  //路段拥堵识别
int LaneCongestion(Max_Queue_Caculation max_queue_test, int item);                             						  //车道拥堵识别
int lanenum_detector(float object_px, float object_py, float Vy, int source, int lane_id);
#endif