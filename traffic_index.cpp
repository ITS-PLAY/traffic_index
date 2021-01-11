#include "traffic_index.h"
using namespace std;

void Vehicle::caculation_Value() {
	speed = sqrt(pow(radar_vx, 2.0) + pow(radar_vy, 2.0));
	timestamp = sec + nsec / pow(10.0, 9.0);
}

void Location_Detection::get_Boundary_Point() {                //读取配置文件
	points.swap(Location_Detection::detect_Config_Points[line_type]);
	return;
}

void Location_Cross_Line::detect_Location() {
	map<int, Vehicle_Extent>::iterator it = map_Vehs.find(vehicle.id);
	if (it == map_Vehs.end()) 
		map_Vehs.emplace(vehicle.id, vehicle);
	else {
		if ((*it).second.drive_In_Zone || (*it).second.drive_Out_Zone) {
			vehicle.drive_In_Zone = (*it).second.drive_In_Zone; vehicle.drive_Out_Zone = (*it).second.drive_Out_Zone;
			vehicle.pos_Drive_In = (*it).second.pos_Drive_In; vehicle.pos_Drive_Out = (*it).second.pos_Drive_Out;
			vehicle.time_Drive_In = (*it).second.time_Drive_In; vehicle.time_Drive_Out = (*it).second.time_Drive_Out;
			vehicle.veh_Is_Stop = (*it).second.veh_Is_Stop;
		}
		double y2 = vehicle.radar_py, y1 = (*it).second.radar_py,
			x2 = vehicle.radar_px, x1 = (*it).second.radar_px;
		double a = points[0][1].y - points[0][0].y, b = points[0][0].x - points[0][1].x;      //计算直线的参数：斜率和截距
		double c = points[0][0].x * points[0][1].y - points[0][0].y * points[0][1].x;
		double condition1 = a * x1 + b * y1 - c, condition2 = a * x2 + b * y2 - c;
		if (condition1 * condition2 < 0 || (condition1 == 0 && condition2 < 0)) {
			if (line_type == "entry_line" && !(*it).second.drive_In_Zone) {
				vehicle.drive_In_Zone = true;
				vehicle.pos_Drive_In.x = vehicle.radar_px;
				vehicle.pos_Drive_In.y = vehicle.radar_py;
				vehicle.time_Drive_In = vehicle.timestamp;
			}
			else if (line_type == "stop_line" && !(*it).second.drive_Out_Zone) {
				vehicle.drive_Out_Zone = true;
				vehicle.pos_Drive_Out.x = vehicle.radar_px;
				vehicle.pos_Drive_Out.y = vehicle.radar_py;
				vehicle.time_Drive_Out = vehicle.timestamp;
			}
		}
		map_Vehs[vehicle.id] = vehicle;	
	}
	return;
}

map<int, Vehicle_Extent> Location_Cross_Line::update_Map_Vehs() {
	for (auto it = map_Vehs.begin(); it != map_Vehs.end(); ) {
		if ((*it).second.drive_Out_Zone || ((*it).second.drive_In_Zone && (current_Time - (*it).second.time_Drive_In) > window_Interval))
			map_Vehs.erase(it++);
		else
			it++;
	}
	return map_Vehs;
}

Vehicle_Extent Location_Cross_Line::update_Veh() {
	return vehicle;
}

void Location_In_Zone::detect_Location() {
	vector<double> test;
	double a;
	for (unsigned int j = 0; j < points[0].size(); j++){        //计算叉积
		if (j == points[0].size() - 1){
			a = (points[0][0].x - points[0][j].x)*(vehicle.radar_py - points[0][j].y) - (points[0][0].y - points[0][j].y)*(vehicle.radar_px - points[0][j].x);
		}
		else{
			a = (points[0][j + 1].x - points[0][j].x)*(vehicle.radar_py - points[0][j].y) - (points[0][j + 1].y - points[0][j].y)*(vehicle.radar_px - points[0][j].x);
		}
		test.push_back(a);
	}
	int up = 0, down = 0;
	for (unsigned int i = 0; i < test.size(); i++){
		if (test[i] >= 0)
			up++;
		if (test[i] <= 0)
			down++;
	}
	if (up == test.size() || down == test.size())
		vehicle.veh_In_Zone = true;
	return;
}

Vehicle_Extent Location_In_Zone::update_Veh() {
	return vehicle;
}

void Location_In_Lane::detect_Location() {
	for (unsigned int i = 0; i < points.size(); i++){
		vector<double> test(0, 0);
		double a;
		int in = 0;
		for (unsigned int j = 0; j < points[i].size(); j++){               //计算叉积
			if (j == (points[i].size()) - 1)
				a = (points[i][0].x - points[i][j].x)*(vehicle.radar_py - points[i][j].y) - (points[i][0].y - points[i][j].y)*(vehicle.radar_px - points[i][j].x);
			else
				a = (points[i][j + 1].x - points[i][j].x)*(vehicle.radar_py - points[i][j].y) - (points[i][j + 1].y - points[i][j].y)*(vehicle.radar_px - points[i][j].x);
			test.push_back(a);
		}
		int up = 0,down = 0;
		for (unsigned int i = 0; i < test.size(); i++){
			if (test[i] >= 0)
				up++;
			if (test[i] <= 0)
				down++;
		}
		if ((up == test.size()) || (down == test.size())) {
			vehicle.lane_Num = i + 1;
			break;
		}
	}
	return;
}

Vehicle_Extent Location_In_Lane::update_Veh() {
	return vehicle;
}

void Index_Caculation::get_Lanes_Info(map<string, vector<vector<Point>>> detect_Config) {
	stop_Distance = fabs((detect_Config["stop_line"][0][0].y + detect_Config["stop_line"][0][1].y) / 2);      //估计停止线到原点的距离
	lanes_Length = lanes_Num * (fabs((detect_Config["entry_line"][0][0].y + detect_Config["entry_line"][0][1].y) / 2) - stop_Distance); //区域内所有车道的总长度
	for (int i = 0; i < lanes_Num; i++) {
		lane_Code.emplace_back(i + 11);
	}
	return;
}

/*
void Index_Caculation::get_Lanes_Info(map<string, vector<vector<Point>>> detect_Config) {
	lane_Canalization.swap(detect_Config["lane_canalization"]);
	lane_Normal.swap(detect_Config["lane_normal"]);

	stop_Distance = fabs((detect_Config["stop_line"][0][0].y + detect_Config["stop_line"][0][1].y) / 2);  //估计停止线到原点的距离
	double canalization_dist = 0.0, normal_dist = 0.0;
	if (lane_Canalization.size() != 0) {
		for (int i = 0; i < lane_Canalization.size(); i++) {
			lane_Code.emplace_back(i + 1);
			for (int j = 0; j < lane_Canalization[i].size(); j++) {
				canalization_dist = (canalization_dist < lane_Canalization[i][j].y) ? lane_Canalization[i][j].y : canalization_dist;
			}
		}
	}
	if (lane_Normal.size() != 0) {
		for (int i = 0; i < lane_Normal.size(); i++) {
			for (int j = 0; j < lane_Normal[i].size(); j++) {
				normal_dist = (normal_dist < lane_Normal[i][j].y) ? lane_Normal[i][j].y : normal_dist;
			}
		}
	}
	lanes_Length = canalization_dist * (int)lane_Canalization.size() + normal_dist * (int)lane_Normal.size();     //区域内所有车道的总长度
	return;
}
*/

void Volume_Caculation::get_Vehicles_Info(Vehicle_Extent &veh) {   
	if (veh.drive_Out_Zone == true && veh.label == 5) {
		vehs_Set[veh.lane_Num].emplace_back(veh);
	}
	return;
}

void Volume_Caculation::caculation_Index(){                      //计算车道流量和断面流量
	for (auto it=vehs_Set.begin(); it != vehs_Set.end(); it++) {
		for (int i = 0; i < (*it).second.size(); i++) {
			lanes_Volume[(*it).first] = lanes_Volume[(*it).first] + car_Type[(*it).second[i].cartype];
		}
		if (section_Flag == true)
			section_Volume = section_Volume + lanes_Volume[(*it).first];       //断面流量累加
	}
	return;
}

void Volume_Caculation::update_Vehicles_Info() {
	vehs_Set.clear();
	section_Volume = 0;
	lanes_Volume.clear();
}

void Time_Headway_Caculation::get_Vehicles_Info(Vehicle_Extent &veh) {          
	if (veh.drive_Out_Zone == true && veh.label == 5) {
		map<int, vector<Vehicle_Extent>>::iterator it = vehs_Set.find(veh.lane_Num);
		vehs_Set[veh.lane_Num].emplace_back(veh);
	}
	return;
}

void Time_Headway_Caculation::caculation_Index() {        //计算平均车头时距和饱和车头时距
	double ave_TT_Sum = 0.0, sat_TT_Sum = 0.0, ave_Volume = 0.0, sat_Volume = 0.0;
	double TT_temp = 0.0;
	for (auto it = vehs_Set.begin(); it != vehs_Set.end(); it++) {
		for (int i = 1; i < (*it).second.size(); i++) {
			TT_temp = ((*it).second[i].time_Drive_Out - (*it).second[i - 1].time_Drive_Out) * car_Type[(*it).second[i].cartype];  //加入车型换算系数
			if (((*it).second[i].time_Drive_Out - (*it).second[i - 1].time_Drive_Out) <= sat_Max_Headway) {
				sat_TT_Sum = sat_TT_Sum + TT_temp;
				sat_Volume = sat_Volume + car_Type[(*it).second[i].cartype];
			}
			ave_TT_Sum = ave_TT_Sum + TT_temp; 
			ave_Volume = ave_Volume + car_Type[(*it).second[i].cartype];
		}
		if (ave_Volume > 0.0)
			ave_Time_Headway[(*it).first] = ave_TT_Sum / ave_Volume;
		else
			ave_Time_Headway[(*it).first] = 0.0;
		if (sat_Volume > 0.0)
			sat_Time_Headway[(*it).first] = sat_TT_Sum / sat_Volume;
		else
			sat_Time_Headway[(*it).first] = 0.0;
	}
	return;
}

void Time_Headway_Caculation::update_Vehicles_Info() {
	vehs_Set.clear();
	ave_Time_Headway.clear();
	sat_Time_Headway.clear();
}

double Capacity_Caculation::time_Headway_Calibration(string cartype) {     //根据车型和限速，标定车头时距值
	return time_Reaction * (speed_Deceleration_Value[cartype] + 1) / speed_Deceleration_Value[cartype] +
		speed_Deceleration_Value[cartype] * pow(time_Reaction, 2) / (2.0F * speed_Max / 3.6F);
}

void Capacity_Caculation::caculation_Index(){
	double capacity_temp = 0.0;
	for (auto it = sat_Time_Headway.begin(); it != sat_Time_Headway.end(); it++) {
		if ((*it).second > 0.0)
			capacity_temp = 3600 / (*it).second;
		else
			capacity_temp = 3600 / time_Headway_Calibration("light");         //采用小轿车作为车头时距计算的标准
		lanes_Capactity.emplace((*it).first, capacity_temp);
	}
	return;
}

void Space_Speed_Caculation::get_Vehicles_Info(Vehicle_Extent &veh) {
	if (veh.drive_In_Zone == true && veh.label == 5) {
		map<int, Vehicle_Extent>::iterator it = vehs_Set.find(veh.id);
		if (it == vehs_Set.end()) {                                   //只保留车辆驶入时的状态（时间和位置）
			vehs_Set.emplace(veh.id, veh);
			vehs_ID_Set.emplace_front(veh.id);
		}
		else if (veh.drive_Out_Zone == true)                          //更新车辆驶出时的状态（时间和位置）
			vehs_Set[veh.id] = veh;
	}
	return;
}

void Space_Speed_Caculation::caculation_Index() {
	double distance=0.0,ave_Speed_Sum = 0.0, ave_Delay_Sum = 0.0;
	int volume = 0;
	double time = 0.0;
	for (int i = 0; i < vehs_ID_Set.size(); i++) {
		if (vehs_Set[vehs_ID_Set[i]].drive_Out_Zone && vehs_Set[vehs_ID_Set[i]].drive_In_Zone) {
			distance = sqrt(pow(vehs_Set[vehs_ID_Set[i]].pos_Drive_Out.x - vehs_Set[vehs_ID_Set[i]].pos_Drive_In.x, 2.0) +
				pow(vehs_Set[vehs_ID_Set[i]].pos_Drive_Out.y - vehs_Set[vehs_ID_Set[i]].pos_Drive_In.y, 2.0));               //计算区间距离
			time = vehs_Set[vehs_ID_Set[i]].time_Drive_Out - vehs_Set[vehs_ID_Set[i]].time_Drive_In;            //计算区间时间
			if (time > 0) {
				ave_Speed_Sum = ave_Speed_Sum + distance / time;
				ave_Delay_Sum = ave_Delay_Sum + time - distance / speed_Max;
				volume++;
			}
		}
	}
	if (volume > 0) {
		ave_Space_Speed = ave_Speed_Sum / volume;
		ave_Delay = ave_Delay_Sum / volume;
	}
	return;
}

void Space_Speed_Caculation::update_Vehicles_Info() {
	int n = vehs_ID_Set.size()-1;
	while ((n >= 0) && (current_Time - vehs_Set[vehs_ID_Set[n]].drive_In_Zone) >= window_Interval) {
		vehs_Set.erase(vehs_ID_Set[n]);
		vehs_ID_Set.pop_back();
		n--;
	}
	for (auto it = vehs_ID_Set.begin(); it != vehs_ID_Set.end();) {
		if (vehs_Set[*it].drive_Out_Zone && vehs_Set[*it].drive_In_Zone) {
			vehs_Set.erase(*it);
			it = vehs_ID_Set.erase(it);
		}
		else {
			it++;
		}
	}
	return;
}

void Headway_Density_Caculation::get_Vehicles_Info(Vehicle_Extent &veh) {
	if (veh.veh_In_Zone == true && veh.label == 5) {
		auto it = vehs_Set.find(veh.lane_Num);
		if (it == vehs_Set.end()) {
			vehs_Set[veh.lane_Num].emplace_back(veh);
		}
		else {
			//插入排序，按距离停止线从近到远，依次插入vector
			int j = vehs_Set[veh.lane_Num].size() - 1;
			while (j >= 0 && (fabs(veh.radar_px) + fabs(veh.radar_py) < fabs(vehs_Set[veh.lane_Num][j].radar_px) + fabs(vehs_Set[veh.lane_Num][j].radar_py))) {
				j--;
			}
			vehs_Set[veh.lane_Num].insert(vehs_Set[veh.lane_Num].begin() + 1 + j, veh);
		}
	}
	return;
}

void Headway_Density_Caculation::caculation_Index() {
	double volume = 0.0, headway = 0.0, speed_Sum = 0.0;
	zone_volume = 0;
	for (auto it = vehs_Set.begin(); it != vehs_Set.end(); it++) {
		int n = (*it).second.size() - 1, m = n+1;
		while (n > 0) {
			headway = headway + sqrt(pow((*it).second[n].radar_px- (*it).second[n-1].radar_px,2.0)+ pow((*it).second[n].radar_py - (*it).second[n - 1].radar_py, 2.0));
			volume = volume + car_Type[(*it).second[n].cartype];
			speed_Sum = speed_Sum + (*it).second[n].speed;
			n--;
		}
		speed_Sum = speed_Sum + (*it).second[n].speed;

		lanes_Space_Headway_Sum[(*it).first]  += headway;
		lanes_Headway_Volume_Sum[(*it).first] +=  volume;
		lanes_Speed_Volume_Sum[(*it).first] += m;
		lanes_Time_Speed_Sum[(*it).first] += speed_Sum;

		lanes_Space_Headway[(*it).first] = lanes_Space_Headway_Sum[(*it).first]/lanes_Headway_Volume_Sum[(*it).first];       //计算车道的平均车头间距
		lanes_Time_Speed[(*it).first] = lanes_Time_Speed_Sum[(*it).first]/ lanes_Speed_Volume_Sum[(*it).first];             //计算车道的平均速度

		zone_volume = zone_volume + m;                                                //计算区域总流量
		headway = 0.0; volume = 0.0; speed_Sum = 0.0;
	}
	if (section_Flag == true) {
		double volume_Headway_Sum = 0.0;
		int volume_Speed_Sum = 0;
		ave_Space_Headway = 0.0;
		ave_Time_Speed = 0.0;
		for (auto it = lanes_Space_Headway.begin(); it != lanes_Space_Headway.end(); it++) {
			ave_Space_Headway += lanes_Space_Headway_Sum[(*it).first];
			volume_Headway_Sum += lanes_Headway_Volume_Sum[(*it).first];
			
			ave_Time_Speed += lanes_Time_Speed_Sum[(*it).first];
			volume_Speed_Sum += lanes_Speed_Volume_Sum[(*it).first];
		}
		ave_Space_Headway = (volume_Headway_Sum > 0) ? ave_Space_Headway / volume_Headway_Sum : 0.0;                                 //计算平均车头间距
		ave_Time_Speed = (volume_Speed_Sum > 0) ? ave_Time_Speed / volume_Speed_Sum : 0.0;                                      //计算平均速度
	}
	section_density = zone_volume / lanes_Length;                   //区间密度，需要区间长度的信息
	return;
}

void Headway_Density_Caculation::update_Vehicles_Info() {
	lanes_Space_Headway_Sum.clear();
	lanes_Time_Speed_Sum.clear();
	lanes_Headway_Volume_Sum.clear();
	lanes_Speed_Volume_Sum.clear();
	lanes_Space_Headway.clear();
	lanes_Time_Speed.clear();
	vehs_Set.clear();
	return;
}

void Max_Queue_Caculation::create_Queue() {
	int lane_Num = 0;
	for (auto it = vehs_Set.begin(); it != vehs_Set.end(); it++) {
		lane_Num = (*it).first;
		double speed_Queue = (queue_Continue[lane_Num] == false) ? speed_Queue_Start : speed_Queue_End;
		vector<Vehicle_Extent> vehs = vehs_Set[lane_Num];
		vector<Vehicle_Extent> temp;
		double density_before = 0.0, volume = 0.0, dist = 0.0;
		bool veh_Status_Now = true, veh_Status_Before = true;
		for (int i = 0; i < vehs.size(); i++) {
			if (temp.size() == 0) {
				temp.emplace_back(vehs[i]);
				volume += car_Type[vehs[i].cartype];
				veh_Status_Before = (vehs[i].speed <= speed_Queue) ? true : false;
			}
			else {
				veh_Status_Now = (vehs[i].speed <= speed_Queue) ? true : false;
				while (veh_Status_Now == veh_Status_Before && veh_Status_Now && i < vehs.size()) {                   //以速度值划分车队，并且前后两车状态相同
					if (temp.size() < min_Queue_Size-1) 
						temp.emplace_back(vehs[i]);
					volume += car_Type[vehs[i].cartype];
					if (temp.size() >= min_Queue_Size - 1) {
						double x = temp[0].radar_px;
						double y = temp[0].radar_py;
						dist = sqrt(pow(vehs[i].radar_px, 2.0) + pow(vehs[i].radar_py, 2.0)) - sqrt(pow(x, 2.0) + pow(y, 2.0));
						double k = 1000 * (volume-1) / dist; 
						if (k > 1000.0 / max_Headway_Slow)                                           //车队的密度大于最小密度要求
							temp.emplace_back(vehs[i]);
						else
							break;
						density_before = k;
					}
					veh_Status_Before = veh_Status_Now;
					i++;
					if (i < vehs.size()) veh_Status_Now = (vehs[i].speed <= speed_Queue) ? true : false;
				}
				if (temp.size() >= min_Queue_Size) {
					queues_Set[lane_Num].push(temp);                                                    //车队集合
					queues_Density[lane_Num].emplace_back(density_before);                              //车队对应的密度                          
				}
				temp.clear();
				volume = 0;
				density_before = 0.0;
			}
		}
	}
	return;
}

void Max_Queue_Caculation::caculation_Index() {
	create_Queue();
	vector<Vehicle_Extent> temp;
	for (auto it = queues_Set.begin(); it != queues_Set.end(); it++) {
		temp = (*it).second.front();
		if ((*it).second.size() == 1) {
			queue_Continue[(*it).first] = true;
			lanes_Queue_Length[(*it).first] = sqrt(pow(temp[temp.size() - 1].radar_px, 2.0) + pow(temp[temp.size() - 1].radar_py, 2.0)) - stop_Distance;   //需要确定停止线的位置
			lanes_Queue_Num[(*it).first] = temp.size();
		}
		if ((*it).second.size() > 1) {
			queue_Continue[(*it).first] = true;
			if (queues_Density[(*it).first][0] >= 1000 / max_Headway_Queue && queues_Density[(*it).first][1] >= 1000 / max_Headway_Slow)      //排队聚集阶段
				caculate_Queue((*it).first, 1, speed_Queue_Start);  
			else if (queues_Density[(*it).first][0] >= 1000 / max_Headway_Slow && queues_Density[(*it).first][1] >= 1000 / max_Headway_Queue)  //排队消散阶段
				caculate_Queue((*it).first, (*it).second.size(), speed_Queue_End);
			else if (queues_Density[(*it).first][0] >= 1000 / max_Headway_Slow && queues_Density[(*it).first][1] >= 1000 / max_Headway_Slow) {                                                        //排队消失阶段    
				int i = 0;
				for (i = 0; i < vehs_Set[(*it).first].size(); i++) {
					if (vehs_Set[(*it).first][i].speed < speed_Queue_End) {
						caculate_Queue((*it).first, (*it).second.size(), speed_Queue_End);
						break;
					}
				}
				if (i == vehs_Set[(*it).first].size()) {                                                                                       //排队结束阶段
					lanes_Queue_Length[(*it).first] = 0.0;
					lanes_Queue_Num[(*it).first] = 0;
					queue_Continue[(*it).first] = false;
				}
			}
		}
		if (section_Flag == true) {                                                                        //计算断面的最大排队长度和排队车辆数
			if (lanes_Queue_Length[(*it).first] > section_Queue_Length)
				section_Queue_Length = lanes_Queue_Length[(*it).first];
			section_Queue_Num = section_Queue_Num + lanes_Queue_Num[(*it).first];
		}
	}
	for (auto it = queue_Continue.begin(); it != queue_Continue.end(); it++) {                             //某一车道没有车队时，表明无排队现象，重置为false
		if (queues_Set.find((*it).first) == queues_Set.end()) {
			queue_Continue[(*it).first] = false;
		}
	}
	return;
}

void Max_Queue_Caculation::caculate_Queue(int lane_Num, int queue_Index,double speed) {
	int i = 0,j = 0;
	vector<Vehicle_Extent> temp;
	int queue_Num = 0;
	while (i < queue_Index) {
		temp = queues_Set[lane_Num].front();
		for (int j = temp.size()-1; j > 0; j--) {
			if (temp[j].speed < speed) {
				lanes_Queue_Length[lane_Num] = sqrt(pow(temp[j].radar_px, 2.0) + pow(temp[j].radar_py, 2.0)) - stop_Distance;   //需要确定停止线的位置
				lanes_Queue_Num[lane_Num] = queue_Num + j + 1;
				break;
			}
		}
		i++;
		queue_Num = queue_Num + temp.size();
		queues_Set[lane_Num].pop();
	}
	return;
}

void Max_Queue_Caculation::update_Vehicles_Info() {
	queues_Set.clear();
	queues_Density.clear();
	lanes_Queue_Length.clear();
	lanes_Queue_Num.clear();

	lanes_Space_Headway.clear();
	vehs_Set.clear();
	section_Queue_Length = 0.0;
	section_Queue_Num = 0;
	return;
}

map<int, bool> Max_Queue_Caculation::update_Queue_Status() {
	return queue_Continue;
}

void Stops_Caculation::get_Vehicles_Info(Vehicle_Extent &veh) {
	if (veh.veh_In_Zone && veh.label == 5) {
		vehs_ID.emplace_back(veh.id);
		if (veh.speed <= speed_Stop_Start) {              //通过停车速度阈值，选择车辆
			if (vehs_Set.find(veh.id) == vehs_Set.end()) {
				vehs_Stop_Start.emplace(veh.id,veh.timestamp);
				vehs_Stops_Num.emplace(veh.id, 0.0);
			}
			if (current_Time - vehs_Stop_Start[veh.id] >= min_Stop_Duration && vehs_Set[veh.id].size()>=2)
				veh.veh_Is_Stop = true;
			vehs_Set[veh.id].emplace_back(veh);
		}
		else {
			veh.veh_Is_Stop = false;
			vehs_Set.erase(veh.id);
			vehs_Stop_Start.erase(veh.id);
			vehs_Stops_Num.erase(veh.id);
		}
	}
	return;
}

void Stops_Caculation::caculation_Index() {
	Vehicle_Extent veh_Temp,veh_Temp_Before;
	double nstops_Sum = 0.0;
	for (auto it = vehs_Set.begin(); it != vehs_Set.end(); it++) {
		if ((*it).second.size() > 1) {                                           //modified
			veh_Temp = (*it).second[(*it).second.size() - 1];
			veh_Temp_Before = (*it).second[(*it).second.size() - 2];
			double nstops = 0.0, speed_Sum = 0.0;
			bool index = false;
			for (auto s = (*it).second.begin(); s != (*it).second.end(); s++) {
				if ((*s).speed == 0) {                                          //车辆的速度存在0，停车次数为1
					nstops = 1.0;
					index = true;
					break;
				}
				speed_Sum += (*s).speed;
			}
			if (index == false)
				nstops = 1 - speed_Sum / (*it).second.size() / speed_Stop_Start;    //不完全停车状态下的，车辆停车次数

			if (veh_Temp.veh_Is_Stop == true && veh_Temp_Before.veh_Is_Stop == false) {
				vehs_Stops_Num[(*it).first] += nstops;
			}	
		}
	}
	for (auto it = vehs_ID.begin(); it != vehs_ID.end(); it++) {
		if (vehs_Stops_Num.find(*it) != vehs_Stops_Num.end()) {
			nstops_Sum += vehs_Stops_Num[*it];
		}
	}
	ave_Stops = (vehs_ID.size() > 0) ? nstops_Sum / vehs_ID.size() : 0.0;
	return;
}

void Stops_Caculation::update_Vehicles_Info() {
	Vehicle_Extent veh_Temp;
	for (auto it = vehs_Set.begin(); it != vehs_Set.end();) {
		veh_Temp = (*it).second[(*it).second.size()-1];
		if (veh_Temp.drive_Out_Zone == true) {
			vehs_Set.erase(it++);
			vehs_Stop_Start.erase((*it).first);
			vehs_Stops_Num.erase((*it).first);
			continue;
		}
		else if (current_Time - vehs_Stop_Start[(*it).first] > window_Interval) {
			vehs_Set.erase(it++);
			vehs_Stop_Start.erase((*it).first);
			vehs_Stops_Num.erase((*it).first);
		}
		else
			it++;
	}
	vehs_ID.clear();
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

	bool flag = false;
	double time_sec = 0.0;         //当前时间
	int window_Interval = 30 * 60; //时间窗口
	double time_Interval = 5.0;       //时间间隔
	double speed_Start = 5.0 / 3.6, speed_End = 20.0 / 3.6;     //排队形成的速度阈值，排队消散的速度阈值
	int min_Vehs_Size = 2;                                      //车队的最小车辆数
	

	vector<vector<Vehicle_Extent>> vehs_test(2);
	for (int i = 0; i < 150; i++) {
		vehs_test[0].emplace_back(Vehicle_Extent(4, 1, i+1, rand() % 4 + 11, 5, "light", -1.7, 30 + rand() % 180, rand() % 10, 0, 4, 1.8));

	}
	for (int i = 0; i < 150; i++) {
		vehs_test[1].emplace_back(Vehicle_Extent(5, 0, i + 1, vehs_test[0][i].lane_Num, 5, "light", -1.7, vehs_test[0][i].radar_py- vehs_test[0][i].speed, rand() % 10, 0, 4, 1.8));

	}

	/*
	                                             { {Vehicle_Extent(1,1,1,2, 5, "light", -1.7,205.0, 1.3, 0, 4, 1.8),
												  Vehicle_Extent(1,1,2,2, 5, "light", -1.7,203.5, 1.3, 0, 4, 1.8)},     //单帧数据
		                                         {Vehicle_Extent(2,1,1,2, 5, "light", -1.7,184.0, 1.3, 0, 4, 1.8),
												  Vehicle_Extent(2,1,2,2, 5, "light", -1.7,180.0, 1.3, 0, 4, 1.8)},    
	                                             {Vehicle_Extent(5,1,1,2, 5, "light", -1.7,75.0 , 1.3, 0, 4, 1.8),
												  Vehicle_Extent(5,1,2,2, 5, "light", -1.7,70.0 , 1.3, 0, 4, 1.8)},
	                                             //Vehicle_Extent(6,0,3,2,"5", "light", 1.7, 19.0, 6.5, 0, 4, 1.8),
		                                         //Vehicle_Extent(7,0,1,2,"5", "light", 1.7,4.0 , 20, 0, 4, 1.8),
		                                         //Vehicle_Extent(8,0,1,2,"5", "light", 1.7, 43.0, 1.3, 0, 4, 1.8),
												 {Vehicle_Extent(9,1,1,2,5, "light",-1.7,42.0 , 1.3, 0, 4, 1.8),
												  Vehicle_Extent(9,1,2,2,5, "light", -1.7,41.0 , 1.3, 0, 4, 1.8)},
												 {Vehicle_Extent(10,2,1,2,5, "light", -1.7,31.0 , 1.3, 0, 4, 1.8),
												  Vehicle_Extent(10,2,2,2,5, "light", -1.7, 30.5, 10, 0, 4, 1.8)}
										          //Vehicle_Extent(12,0,1,2,"5", "light", 1.7,2.0 , 10, 0, 4, 1.8)}
	                                    };
	*/
	map<int, Vehicle_Extent> map_Vehs_Entry, map_Vehs_Stop;   //记录entry-stop对中，车辆上一时刻的信息
	map<int, bool> map_Lanes_Queue;   //记录车道排队的状态

	Volume_Caculation volume_test = Volume_Caculation(time_Interval,true, detect_Config_Points);                                                                //定义流量指标
	Space_Speed_Caculation speed_test = Space_Speed_Caculation(time_Interval, window_Interval, 60.0 / 3.6, detect_Config_Points);                       //定义平均空间速度指标
	//Time_Headway_Caculation time_headway_test = Time_Headway_Caculation(time_Interval, 4.0, detect_Config_Points);                                               //定义平均车头时距指标
	//Headway_Density_Caculation headway_density_test = Headway_Density_Caculation(time_Interval,true,detect_Config_Points);                                       //定义平均车头间距和密度指标
	Capacity_Caculation capacity_test = Capacity_Caculation(time_Interval, 4.0, 60/3.6, detect_Config_Points);                                                    //定义通行能力指标

	Max_Queue_Caculation max_queue_test = Max_Queue_Caculation(time_Interval, true, speed_Start, speed_End, min_Vehs_Size, map_Lanes_Queue, detect_Config_Points);  //定义排队长度指标
	Stops_Caculation stops_test = Stops_Caculation(time_Interval, window_Interval, speed_Start, 3.0, detect_Config_Points);                             //定义停车次数指标

	//开始测试
	for (int i = 0; i < vehs_test.size(); i++) {
		for (int j = 0; j < vehs_test[i].size(); j++) {
			vehs_test[i][j] = Location_In_Zone("detect_zone", vehs_test[i][j], detect_Config_Points).update_Veh();                //区域检测
			time_sec = vehs_test[i][j].sec;
			speed_test.current_Time = vehs_test[i][j].timestamp;
			stops_test.current_Time = vehs_test[i][j].timestamp;
			if (vehs_test[i][j].veh_In_Zone) {
				//vehs_test[i] = Location_In_Lane("lane_canalization", vehs_test[i], detect_Config_Points).update_Veh();         //车道检测
				vehs_test[i][j] = Location_Cross_Line("entry_line", map_Vehs_Entry, vehs_test[i][j], detect_Config_Points, time_sec, window_Interval).update_Veh();  //驶入区域检测
				vehs_test[i][j] = Location_Cross_Line("stop_line", map_Vehs_Stop, vehs_test[i][j], detect_Config_Points, time_sec, window_Interval).update_Veh();   //驶出区域检测
				volume_test.get_Vehicles_Info(vehs_test[i][j]);            //采集车辆，用于计算流量
				speed_test.get_Vehicles_Info(vehs_test[i][j]);             //采集车辆，用于计算空间平均速度
				capacity_test.get_Vehicles_Info(vehs_test[i][j]);          //采集车辆，用于计算平均车头时距和车道通行能力

				max_queue_test.get_Vehicles_Info(vehs_test[i][j]);         //采集车辆，用于计算车头间距、密度和排队长度
				stops_test.get_Vehicles_Info(vehs_test[i][j]);             //采集车辆，用于计算停车次数
			}
		}
		max_queue_test.Headway_Density_Caculation::caculation_Index();                    //计算车头间距、密度
		max_queue_test.caculation_Index();                                                //计算排队长度
		stops_test.caculation_Index();                                                    //计算停车次数
		printf("zone_num: %d , section_density: %f ,section_queue: %f \n", max_queue_test.zone_volume, max_queue_test.section_density, max_queue_test.section_Queue_Length);
		printf("ave_Stops: %f \n", stops_test.ave_Stops);

		max_queue_test.Headway_Density_Caculation::update_Vehicles_Info();
		max_queue_test.update_Vehicles_Info();
		stops_test.update_Vehicles_Info();

		if (fmod(time_sec, time_Interval) == 0.0 && !flag) {
			volume_test.caculation_Index();                                               //计算流量
			speed_test.caculation_Index();                                                //计算空间平均速度
			capacity_test.Time_Headway_Caculation::caculation_Index();                   //计算平均车头时距
			capacity_test.caculation_Index();                                            //计算车道通行能力

			//printf("volume: %f \n",volume_test.section_Volume);
			volume_test.update_Vehicles_Info();

			//printf("space_speed: %f \n",speed_test.ave_Space_Speed);
			speed_test.update_Vehicles_Info();

			if (capacity_test.ave_Time_Headway.size() > 0) {
				printf("time_headway: %f, capacity: %f\n", capacity_test.ave_Time_Headway.begin()->second, capacity_test.lanes_Capactity.begin()->second);
			}

			capacity_test.update_Vehicles_Info();
			flag = true;
		}
		if (fmod(time_sec, time_Interval) != 0.0)
			flag = false;
		
	}
	return 0;
}

