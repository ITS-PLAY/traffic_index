#include "traffic_index.h"
using namespace std;

void Pedestrian::caculation_Value() {
	speed = sqrt(pow(radar_vx, 2.0) + pow(radar_vy, 2.0));
	timestamp = sec + nsec / pow(10.0, 9.0);
}

void Location_Detection::get_Boundary_Point() {                //读取配置文件
	points.swap(Location_Detection::detect_Config_Points[line_type]);
	return;
}

void Location_Cross_Line::detect_Location() {
	map<int, Pedestrian_Extent>::iterator it = map_Peds.find(pedestrian.id);
	if (it == map_Peds.end() && pedestrian.ped_In_Zone) {
		map_Peds.emplace(pedestrian.id, pedestrian);
		return;
	}
	if ((*it).second.walk_In_Zone || (*it).second.walk_Out_Zone ){
		pedestrian.walk_In_Zone = (*it).second.walk_In_Zone; pedestrian.walk_Out_Zone = (*it).second.walk_Out_Zone;
		pedestrian.pos_Walk_In = (*it).second.pos_Walk_In; pedestrian.pos_Walk_Out = (*it).second.pos_Walk_Out;
		pedestrian.time_Walk_In = (*it).second.time_Walk_In; pedestrian.time_Walk_Out = (*it).second.time_Walk_Out;
	}
	double y2 = pedestrian.radar_py, y1 = (*it).second.radar_py,
		   x2 = pedestrian.radar_px, x1 = (*it).second.radar_px;
	double a = points[0][1].y - points[0][0].y, b = points[0][0].x - points[0][1].x;      //计算直线的参数：斜率和截距
	double c = points[0][0].x * points[0][1].y - points[0][0].y * points[0][1].x;
	double condition1 = a * x1 + b * y1 - c, condition2 = a * x2 + b * y2 - c;
	if (condition1 * condition2 < 0 || (condition1 == 0 && condition2 < 0)) {
		if (line_type == "entry_line" && !(*it).second.walk_In_Zone) {
			pedestrian.walk_In_Zone = true;
			pedestrian.pos_Walk_In.x = pedestrian.radar_px;
			pedestrian.pos_Walk_In.y = pedestrian.radar_py;
			pedestrian.time_Walk_In = pedestrian.timestamp;
		}
		if (line_type == "stop_line" && !(*it).second.walk_Out_Zone) {
			pedestrian.walk_Out_Zone = true;
			pedestrian.pos_Walk_Out.x = pedestrian.radar_px;
			pedestrian.pos_Walk_Out.y = pedestrian.radar_py;
			pedestrian.time_Walk_Out = pedestrian.timestamp;
		}
	}
	if (it != map_Peds.end())
		map_Peds[pedestrian.id] = pedestrian;
	return;
}

map<int, Pedestrian_Extent> Location_Cross_Line::update_Map_Peds() {
	for (auto it = map_Peds.begin(); it != map_Peds.end(); ) {
		if (!(*it).second.ped_In_Zone)
			map_Peds.erase(it++);
		else
			it++;
	}
	return map_Peds;
}

Pedestrian_Extent Location_Cross_Line::update_Ped() {
	return pedestrian;
}

void Location_In_Zone::detect_Location() {
	//vector<double> test;
	double a = 0.0;
	int up = 0, down = 0;
	for (int j = 0 , length= points[0].size() - 1; j < length; j++){        //计算叉积
		a = (points[0][j + 1].x - points[0][j].x)*(pedestrian.radar_py - points[0][j].y) - (points[0][j + 1].y - points[0][j].y)*(pedestrian.radar_px - points[0][j].x);
		if (a >= 0) up++;
		if (a <= 0) down++;
	}

	if (up == points[0].size()-1 || down == points[0].size()-1)
		pedestrian.ped_In_Zone = true;
	return;
}

Pedestrian_Extent Location_In_Zone::update_Ped() {
	return pedestrian;
}

void Index_Caculation::get_Lanes_Info(map<string, vector<vector<Point>>> detect_Config) {
	stop_Distance = fabs((detect_Config["stop_line"][0][0].y + detect_Config["stop_line"][0][1].y) / 2);      //估计停止线到原点的距离
	lanes_Length = lanes_Num * (fabs((detect_Config["entry_line"][0][0].y + detect_Config["entry_line"][0][1].y) / 2) - stop_Distance); //区域内所有车道的总长度
	for (int i = 0; i < lanes_Num; i++) {
		lane_Code.emplace_back(i + 11);
	}
	return;
}

void Volume_Caculation::get_Pedestrians_Info(Pedestrian_Extent &ped) {   
	if (ped.time_Walk_In && ped.time_Walk_Out && ped.label == 5) {
		peds_Set[ped.lane_Num].emplace_back(ped);
	}
	return;
}

void Volume_Caculation::caculation_Index(){                      //计算车道流量和断面流量
	for (auto it=peds_Set.begin(); it != peds_Set.end(); it++) {
		for (int i = 0, length = (*it).second.size(); i < length; i++) {
			lanes_Volume[(*it).first] = lanes_Volume[(*it).first] + car_Type[(*it).second[i].cartype];
		}
		if (section_Flag == true)
			section_Volume = section_Volume + lanes_Volume[(*it).first];       //断面流量累加
	}
	return;
}

void Volume_Caculation::update_Pedestrians_Info() {
	peds_Set.clear();
	section_Volume = 0.0;
	lanes_Volume.clear();
}

void Space_Speed_Caculation::get_Pedestrians_Info(Pedestrian_Extent &ped) {
	if (ped.walk_In_Zone && ped.walk_Out_Zone && ped.label == 5) {                         //modified
		peds_Set.emplace(ped.id, ped);
	}
	return;
}

void Space_Speed_Caculation::caculation_Index() {
	double distance = 0.0, ave_Speed_Sum = 0.0, ave_Delay_Sum = 0.0, ave_Time_Sum = 0.0;
	int volume = 0;
	double time = 0.0;
	for (auto it = peds_Set.begin(); it != peds_Set.end(); it++) {                                     //modified
		distance = sqrt(pow((*it).second.pos_Walk_Out.x - (*it).second.pos_Walk_In.x, 2.0) +
				pow((*it).second.pos_Walk_Out.y - (*it).second.pos_Walk_In.y, 2.0));               //计算区间距离
		time = abs((*it).second.time_Walk_Out - (*it).second.time_Walk_In);            //计算区间时间
		if (time > 0) {
			ave_Speed_Sum = ave_Speed_Sum + distance / time;
			ave_Delay_Sum = ave_Delay_Sum + time - distance / speed_Max;
			ave_Time_Sum += time;
			volume++;
		}
	}
	if (volume > 0) {
		ave_Space_Speed = ave_Speed_Sum / volume;
		ave_Delay = ave_Delay_Sum / volume;
		ave_Travel_Time = ave_Time_Sum / volume;
	}
	return;
}

void Space_Speed_Caculation::update_Pedestrians_Info() {
	peds_Set.clear();
	return;
}

void ReadJsonFromFile(string filename, map<string, vector<vector<Point>>>& detect_Config_Points) {
	ifstream ifs;
	ifs.open(filename, ios::binary);
	if (!ifs.is_open()) {
		return ;
	}

	vector<string> data_Fields{"entry_line","stop_line","detect_zone","no_parking","intersection_entry_line","intersection_stop_line",
	"noparking_entry_line","noparking_stop_line","intersection_zone"};
	Json::Value root;
	Json::Reader reader;
	if (reader.parse(ifs, root)) {
		string code;
		if (!root["detect_config"].isNull()) {
			int nSize = data_Fields.size();
			Json::Value points;
			vector<vector<Point>> temp(1);
			for (int i = 0;i < nSize; i++){
				points = root["detect_config"][data_Fields[i]];

				for (int j = 0; j < (int)points.size(); j++) {
					double x = root["detect_config"][data_Fields[i]][j]["x"].asDouble();
					double y = root["detect_config"][data_Fields[i]][j]["y"].asDouble();
					temp[0].push_back(Point(x, y));
				}
				detect_Config_Points.emplace(data_Fields[i],temp);
				temp.clear();
				temp.resize(1);
			}
		}
	}
	ifs.close();
	return;
}


int main()
{
	//检测框类型划分为detect_zone,no_parking_zone,lane_canalization,lane_normal,entry_line,stop_line,no_change_line，对应着
    //                区域、禁停区、渠化车道、上游正常车道、驶入线、停止线、禁止换道起点线
	map<string, vector<vector<Point>>> detect_Config_Points;       //配置文件
	//ReadJsonFromFile("F:\\202011MEC交通开发\\code\\detect_Config.json", detect_Config_Points);

	vector<vector<Point>> entry_Points = { { Point(0,200),Point(-14,200) } };
	vector<vector<Point>> stop_Points = { { Point(0,40),Point(-14,40) } };
	vector<vector<Point>> detect_Zone_Points = { { Point(-1,30),Point(-14,30),Point(-14,210),Point(-1,210) } };

	detect_Config_Points.emplace("entry_line", entry_Points);
	detect_Config_Points.emplace("stop_line", stop_Points);
	detect_Config_Points.emplace("detect_zone", detect_Zone_Points);
	double entry_stop_Midpoint = (detect_Config_Points["entry_line"][0][0].y + detect_Config_Points["stop_line"][0][0].y) / 2.0;     //驶入线和停止线的y轴分割点
	
	bool flag = false;
	double time_sec = 0.0;         //当前时间
	int window_Interval = 15 * 60; //时间窗口
	double time_Interval = 5.0;       //时间间隔
	double speed_Start = 5.0 / 3.6, speed_End = 20.0 / 3.6;     //排队形成的速度阈值，排队消散的速度阈值
	int min_Vehs_Size = 3;                                      //车队的最小车辆数
	
	vector<vector<Pedestrian_Extent>> peds_test(2);
	for (int i = 0; i < 150; i++) {
		peds_test[0].emplace_back(Pedestrian_Extent(4, 1, i+1, rand() % 4 + 11, 5, "light", -1.7, 30 + rand() % 180, rand() % 10, 0, 4, 1.8));
	}
	for (int i = 0; i < 150; i++) {
		peds_test[1].emplace_back(Pedestrian_Extent(5, 0, i + 1, peds_test[0][i].lane_Num, 5, "light", -1.7, peds_test[0][i].radar_py- peds_test[0][i].speed, rand() % 10, 0, 4, 1.8));
	}

	map<int, Pedestrian_Extent> map_Peds;   //记录entry-stop对中，车辆上一时刻的信息
	map<int, bool> map_Lanes_Queue;   //记录车道排队的状态

	Volume_Caculation volume_test = Volume_Caculation(time_Interval,true, detect_Config_Points);                                                                //定义流量指标
	Space_Speed_Caculation speed_test = Space_Speed_Caculation(time_Interval, window_Interval, 60.0 / 3.6, detect_Config_Points);                       //定义平均空间速度指标 

	//开始测试
	for (int i = 0; i < peds_test.size(); i++) {
		for (int j = 0, length = peds_test[i].size(); j < length; j++) {
			
			time_sec = peds_test[i][j].timestamp;
			speed_test.current_Time = peds_test[i][j].timestamp;
			peds_test[i][j] = Location_In_Zone("detect_zone", peds_test[i][j], detect_Config_Points).update_Ped();                //区域检测
			
			if (peds_test[i][j].radar_py >= entry_stop_Midpoint)
			    peds_test[i][j] = Location_Cross_Line("entry_line", map_Peds, peds_test[i][j], detect_Config_Points, time_sec, window_Interval).update_Ped();  //驶入区域检测
			else
				peds_test[i][j] = Location_Cross_Line("stop_line", map_Peds, peds_test[i][j], detect_Config_Points, time_sec, window_Interval).update_Ped();   //驶出区域检测
			
			if (peds_test[i][j].ped_In_Zone) {
				volume_test.get_Pedestrians_Info(peds_test[i][j]);            //采集车辆，用于计算流量
				speed_test.get_Pedestrians_Info(peds_test[i][j]);             //采集车辆，用于计算空间平均速度
				
			}
		}
		
		if (fmod(time_sec, time_Interval) == 0.0 && !flag) {
			volume_test.caculation_Index();                                               //计算流量
			speed_test.caculation_Index();                                                //计算空间平均速度
			
			//printf("volume: %f \n",volume_test.section_Volume);
			volume_test.update_Pedestrians_Info();

			//printf("space_speed: %f \n",speed_test.ave_Space_Speed);
			speed_test.update_Pedestrians_Info();

			flag = true;
		}
		if (fmod(time_sec, time_Interval) != 0.0)
			flag = false;
		
	}
	return 0;
}

