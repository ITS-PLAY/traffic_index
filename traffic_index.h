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
 
class Pedestrian {                  //行人类
public:
	Pedestrian() {}
	Pedestrian(double msec, double mnsec, int mid, int mlane_Num, uint32_t mlabel, string mtype, double mx, double my, double mvx, double mvy, double mlength, double mwidth) :
		sec(msec), nsec(mnsec), id(mid), lane_Num(mlane_Num), label(mlabel), cartype(mtype), radar_px(mx), radar_py(my), radar_vx(mvx), radar_vy(mvy), dimensions_x(mlength), dimensions_y(mwidth) {
		caculation_Value();
	}
	double sec;                     //秒
	double nsec;                    //纳秒
	int id;                      //目标ID
	int lane_Num;                //获取车道编号
	uint32_t label;                //目标类别 
	string cartype;              //行人类型
	double radar_px;             //雷达x轴坐标
	double radar_py;             //雷达y轴坐标
	double radar_vx;             //雷达x轴速度
	double radar_vy;             //雷达y轴速度
	double dimensions_x;         //长度
	double dimensions_y;         //宽度
public:
	void caculation_Value();
	double speed;                //换算得到速度值
	double timestamp;            //换算得到时间
};

class Pedestrian_Extent:public Pedestrian {
public:
	Pedestrian_Extent() {};
	Pedestrian_Extent(double msec, double mnsec, int mid, int mlane_Num, uint32_t mlabel, string mtype, double mx, double my, double mvx, double mvy, double mlength, double mwidth) :
		Pedestrian(msec, mnsec, mid, mlane_Num, mlabel, mtype, mx, my, mvx, mvy, mlength, mwidth) {
		ped_In_Zone = false;
		walk_In_Zone = false;
		walk_Out_Zone = false;
	};

public:
	bool ped_In_Zone = false;            //是否位于区域内
	bool walk_In_Zone = false;          //是否正在驶入区域
	double time_Walk_In = 0.0;           //驶入区域的时间
	Point pos_Walk_In;                   //驶入区域时的位置
	bool walk_Out_Zone = false;         //是否正在驶出区域
	double time_Walk_Out = 0.0;          //驶出区域的时间
	Point pos_Walk_Out;                 //驶出区域时的位置
};

class Location_Detection {      //***行人位置检测
public:
	Location_Detection() {}
	Location_Detection(string mline_type, map<string, vector<vector<Point>>> mdetect_Config):line_type(mline_type), detect_Config_Points(mdetect_Config){
		get_Boundary_Point();
	};
	virtual void detect_Location() = 0;
	void get_Boundary_Point();        //配置检测区域
	virtual Pedestrian_Extent update_Ped() = 0;

public:
	string line_type;
	map<string, vector<vector<Point>>> detect_Config_Points;
	vector<vector<Point>> points;
};
//位置检测的详细函数
class Location_Cross_Line :public Location_Detection {        //**跨线检测
public:
	Location_Cross_Line() {}
	Location_Cross_Line(string mline_type, map<int, Pedestrian_Extent> &mmap_Peds, Pedestrian_Extent mpedestrian, map<string, vector<vector<Point>>> mdetect_Config,double mcurrent_Time, double mwindow_Interval) :
		Location_Detection(mline_type, mdetect_Config),map_Peds(mmap_Peds),pedestrian(mpedestrian),current_Time(mcurrent_Time),window_Interval(mwindow_Interval){
		detect_Location();                             //基于回归线两侧的正负号，进行判断
		mmap_Peds = update_Map_Peds();
	};
	void detect_Location();
	map<int, Pedestrian_Extent> update_Map_Peds();
	Pedestrian_Extent update_Ped();
private:
	double current_Time;
	double window_Interval;
	Pedestrian_Extent pedestrian;
	map<int, Pedestrian_Extent> map_Peds;
};

class Location_In_Zone :public Location_Detection {      //**区域检测
public:
	Location_In_Zone() {};
	Location_In_Zone(string mline_type, Pedestrian_Extent mpedestrian, map<string, vector<vector<Point>>> mdetect_Config) :
		Location_Detection(mline_type,mdetect_Config), pedestrian(mpedestrian) {
		detect_Location();                             //基于向量内积法，进行判断
	};
	void detect_Location();
	Pedestrian_Extent update_Ped();

private:
	Pedestrian_Extent pedestrian;
	double duration_Time;             //停留时间要求
};

class Index_Caculation {        //***交通指标计算
public:
	Index_Caculation() {};
	Index_Caculation(double mtime_Interval, map<string, vector<vector<Point>>> mdetect_Config) :time_Interval(mtime_Interval) {
		get_Lanes_Info(mdetect_Config);
	};
	void get_Lanes_Info(map<string, vector<vector<Point>>> detect_Config);
	virtual void get_Pedestrians_Info(Pedestrian_Extent &ped) = 0;          //获取目标行人的集合
	virtual void caculation_Index() = 0;             //指标计算
	virtual void update_Pedestrians_Info() = 0;       //更新目标行人的集合

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
	void get_Pedestrians_Info(Pedestrian_Extent &ped);
	void caculation_Index();
	void update_Pedestrians_Info();

private:
	bool section_Flag;
public:
	map<int, vector<Pedestrian_Extent>> peds_Set;
	double section_Volume=0.0;
	map<int, double> lanes_Volume;
};

class Space_Speed_Caculation :public Index_Caculation {       //**计算区间平均速度、区间延误
public:
	Space_Speed_Caculation() {};
	Space_Speed_Caculation(double mtime_Interval,int mwindow_Interval,double mspeed_Max, map<string, vector<vector<Point>>> mdetect_Config) :Index_Caculation(mtime_Interval, mdetect_Config),window_Interval(mwindow_Interval),
		speed_Max(mspeed_Max){};
	void get_Pedestrians_Info(Pedestrian_Extent &ped);
	void caculation_Index();
	void update_Pedestrians_Info();

private:
	map<int, Pedestrian_Extent> peds_Set;
	double ave_Travel_Time = 0.0;
	double window_Interval=30;                                      //时间窗口的最大值，分钟
	double speed_Max;                                         //路段的限速值      

public:
	double current_Time = 0.0;                                         //当前时间
	double ave_Delay=0.0;
	double ave_Space_Speed=0.0;
};

void ReadJsonFromFile(string filename, map<string, vector<vector<Point>>>& detect_Config_Points);