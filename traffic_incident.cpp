#include "traffic_incident.h"
using namespace std;

void Vehicle::caculation_Value() {
	speed = sqrt(pow(radar_vx, 2.0) + pow(radar_vy, 2.0));
	timestamp = sec + nsec / pow(10.0, 9.0);
}

void Location_Detection::get_Boundary_Point() {                //读取配置文件
	points.swap(Location_Detection::detect_Config_Points[line_type]);
	return;
}

void Location_Cross_Line::detect_Location() {                              //跨线检测
	map<int, Vehicleincident_Detection>::iterator it = map_Vehs.find(vehicle.id);
	if (it == map_Vehs.end())
	{
		vehicle.time_Drive_In_NoParking = vehicle.timestamp;

		vehicle.time_Veh_Recognize = vehicle.timestamp;
		vehicle.pos_Veh_Recognize.x = vehicle.radar_px;
		vehicle.pos_Veh_Recognize.y = vehicle.radar_py;
		map_Vehs.emplace(vehicle.id, vehicle);
	}
	else {
		vehicle.time_Drive_In_NoParking = map_Vehs[vehicle.id].time_Drive_In_NoParking;
		vehicle.time_Veh_Recognize = (*it).second.time_Veh_Recognize;
		vehicle.pos_Veh_Recognize = (*it).second.pos_Veh_Recognize;

		if ((*it).second.drive_In_Zone || (*it).second.drive_Out_Zone) {
			vehicle.drive_In_Zone = (*it).second.drive_In_Zone; vehicle.drive_Out_Zone = (*it).second.drive_Out_Zone;
			vehicle.pos_Drive_In = (*it).second.pos_Drive_In; vehicle.pos_Drive_Out = (*it).second.pos_Drive_Out;
			vehicle.time_Drive_In = (*it).second.time_Drive_In; vehicle.time_Drive_Out = (*it).second.time_Drive_Out;
			vehicle.veh_Is_Stop = (*it).second.veh_Is_Stop;
		}
		if ((*it).second.drive_In_Intersection || (*it).second.drive_Out_Intersection)
		{
			vehicle.drive_In_Intersection = (*it).second.drive_In_Intersection; vehicle.drive_Out_Intersection = (*it).second.drive_Out_Intersection;
			vehicle.pos_Drive_In_Intersection = (*it).second.pos_Drive_In_Intersection; vehicle.pos_Drive_Out_Intersection = (*it).second.pos_Drive_Out_Intersection;
			vehicle.time_Drive_In_Intersection = (*it).second.time_Drive_In_Intersection; vehicle.time_Drive_Out_Intersection = (*it).second.time_Drive_Out_Intersection;
		}
		if ((*it).second.drive_In_NoParking || (*it).second.drive_Out_NoParking)
		{
			vehicle.drive_In_NoParking = (*it).second.drive_In_NoParking; vehicle.drive_Out_NoParking = (*it).second.drive_Out_NoParking;
			vehicle.pos_Drive_In_NoParking = (*it).second.pos_Drive_In_NoParking; vehicle.pos_Drive_Out_NoParking = (*it).second.pos_Drive_Out_NoParking;
			vehicle.time_Drive_In_NoParking = (*it).second.time_Drive_In_NoParking; vehicle.time_Drive_Out_NoParking = (*it).second.time_Drive_Out_NoParking;
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

			if (line_type == "intersection_entry_line" && !(*it).second.drive_In_Intersection) {
				vehicle.drive_In_Intersection = true;
				vehicle.pos_Drive_In_Intersection.x = vehicle.radar_px;
				vehicle.pos_Drive_In_Intersection.y = vehicle.radar_py;
				vehicle.time_Drive_In_Intersection = vehicle.timestamp;
			}
			else if (line_type == "intersection_stop_line" && !(*it).second.drive_Out_Intersection) {
				vehicle.drive_Out_Intersection = true;
				vehicle.pos_Drive_Out_Intersection.x = vehicle.radar_px;
				vehicle.pos_Drive_Out_Intersection.y = vehicle.radar_py;
				vehicle.time_Drive_Out_Intersection = vehicle.timestamp;
			}

			if (line_type == "noparking_entry_line" && !(*it).second.drive_In_NoParking) {
				vehicle.drive_In_NoParking = true;
				vehicle.pos_Drive_In_NoParking.x = vehicle.radar_px;
				vehicle.pos_Drive_In_NoParking.y = vehicle.radar_py;
				vehicle.time_Drive_In_NoParking = vehicle.timestamp;
			}
			else if (line_type == "noparking_stop_line" && !(*it).second.drive_Out_NoParking) {
				vehicle.drive_Out_NoParking = true;
				vehicle.pos_Drive_Out_NoParking.x = vehicle.radar_px;
				vehicle.pos_Drive_Out_NoParking.y = vehicle.radar_py;
				vehicle.time_Drive_Out_NoParking = vehicle.timestamp;
			}
		}
		map_Vehs[vehicle.id] = vehicle;
	}
	return;
}

map<int, Vehicleincident_Detection> Location_Cross_Line::update_Map_Vehs() {
	for (auto it = map_Vehs.begin(); it != map_Vehs.end(); ) {
		if ((*it).second.drive_Out_Zone || (current_Time - (*it).second.time_Veh_Recognize) > window_Interval)
			map_Vehs.erase(it++);
		else
			it++;
	}
	return map_Vehs;
}

Vehicleincident_Detection Location_Cross_Line::update_Veh() {
	return vehicle;
}

void Location_In_Zone::detect_Location() {                              //区域检测
	vector<double> test;
	double a;
	for (unsigned int j = 0; j < points[0].size(); j++) {               //计算叉积
		if (j == points[0].size() - 1) {
			a = (points[0][0].x - points[0][j].x) * (vehicle.radar_py - points[0][j].y) - (points[0][0].y - points[0][j].y) * (vehicle.radar_px - points[0][j].x);
		}
		else {
			a = (points[0][j + 1].x - points[0][j].x) * (vehicle.radar_py - points[0][j].y) - (points[0][j + 1].y - points[0][j].y) * (vehicle.radar_px - points[0][j].x);
		}
		test.push_back(a);
	}
	int up = 0, down = 0;
	for (unsigned int i = 0; i < test.size(); i++) {
		if (test[i] >= 0)
			up++;
		if (test[i] <= 0)
			down++;
	}
	if (up == test.size() || down == test.size())
	{
		if (line_type == "detect_zone")
		{
			vehicle.veh_In_Zone = true;                                        //返回是否位于区域的标志
		}
		if (line_type == "no_parking")
		{
			vehicle.veh_In_NoParking = true;
		}
		if (line_type == "intersection_zone")
		{
			vehicle.veh_In_Intersection = true;
		}
	}

	return;
}

Vehicleincident_Detection Location_In_Zone::update_Veh() {
	return vehicle;
}


void Location_In_Lane::detect_Location() {                               //车道检测
	for (unsigned int i = 0; i < points.size(); i++) {
		vector<double> test(0, 0);
		double a;
		int in = 0;
		for (unsigned int j = 0; j < points[i].size(); j++) {             //计算叉积
			if (j == (points[i].size()) - 1)
				a = (points[i][0].x - points[i][j].x) * (vehicle.radar_py - points[i][j].y) - (points[i][0].y - points[i][j].y) * (vehicle.radar_px - points[i][j].x);
			else
				a = (points[i][j + 1].x - points[i][j].x) * (vehicle.radar_py - points[i][j].y) - (points[i][j + 1].y - points[i][j].y) * (vehicle.radar_px - points[i][j].x);
			test.push_back(a);
		}
		int up = 0, down = 0;
		for (unsigned int i = 0; i < test.size(); i++) {
			if (test[i] >= 0)
				up++;
			if (test[i] <= 0)
				down++;
		}
		if ((up == test.size()) || (down == test.size())) {
			vehicle.lane_Num = i + 1;                                    //返回车道编号
			break;
		}
	}
	return;
}

Vehicleincident_Detection Location_In_Lane::update_Veh() {
	return vehicle;
}

void Index_Caculation::get_Lanes_Info(map<string, vector<vector<Point>>> detect_Config) {
	stop_Distance = fabs((detect_Config["stop_line"][0][0].y + detect_Config["stop_line"][0][1].y) / 2);  //估计停止线到原点的距离
	lanes_Length = lanes_Num * (fabs((detect_Config["entry_line"][0][0].y + detect_Config["entry_line"][0][1].y) / 2) - stop_Distance);     //区域内所有车道的总长度
	for (int i = 0; i < lanes_Num; i++) {
		lane_Code.emplace_back(i + 11);
	}
	return;
}

void Volume_Caculation::get_Vehicles_Info(Vehicleincident_Detection& veh) {
	if (veh.drive_Out_Zone == true && veh.label == 5) {
		vehs_Set[veh.lane_Num].emplace_back(veh);
	}
	return;
}

void Volume_Caculation::caculation_Index() {                      //计算车道流量和断面流量
	for (auto it = vehs_Set.begin(); it != vehs_Set.end(); it++) {
		for (int i = 0; i < (*it).second.size(); i++) {
			lanes_Volume[(*it).first] = lanes_Volume[(*it).first] + car_Type.at((*it).second[i].cartype);
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

void Time_Headway_Caculation::get_Vehicles_Info(Vehicleincident_Detection& veh) {
	if (veh.drive_Out_Zone == true && veh.label == 5) {
		map<int, vector<Vehicleincident_Detection>>::iterator it = vehs_Set.find(veh.lane_Num);
		vehs_Set[veh.lane_Num].emplace_back(veh);
	}
	return;
}

void Time_Headway_Caculation::caculation_Index() {        //计算平均车头时距和饱和车头时距	
	double TT_temp = 0.0;
	for (auto it = vehs_Set.begin(); it != vehs_Set.end(); it++) {
		double ave_TT_Sum = 0.0, sat_TT_Sum = 0.0, ave_Volume = 0.0, sat_Volume = 0.0;
		for (int i = 1; i < (*it).second.size(); i++) {
			TT_temp = ((*it).second[i].time_Drive_Out - (*it).second[i - 1].time_Drive_Out) * car_Type.at((*it).second[i].cartype);  //加入车型换算系数
			if (((*it).second[i].time_Drive_Out - (*it).second[i - 1].time_Drive_Out) <= sat_Max_Headway) {
				sat_TT_Sum = sat_TT_Sum + TT_temp;
				sat_Volume = sat_Volume + car_Type.at((*it).second[i].cartype);
			}
			ave_TT_Sum = ave_TT_Sum + TT_temp;
			ave_Volume = ave_Volume + car_Type.at((*it).second[i].cartype);
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
	return time_Reaction * (speed_Deceleration_Value.at(cartype) + 1) / speed_Deceleration_Value.at(cartype) +
		speed_Deceleration_Value.at(cartype) * pow(time_Reaction, 2) / (2.0F * speed_Max / 3.6F);
}

void Capacity_Caculation::caculation_Index() {
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

void Space_Speed_Caculation::get_Vehicles_Info(Vehicleincident_Detection& veh) {
	if (veh.label == 5) {                                             //if (veh.drive_In_Zone == true && veh.label == 5) {
		map<int, Vehicleincident_Detection>::iterator it = vehs_Set.find(veh.id);
		if (it == vehs_Set.end()) {                                   //只保留车辆驶入时的状态（时间和位置）
			vehs_Set.emplace(veh.id, veh);
			vehs_ID_Set.emplace_front(veh.id);
		}
		else if (veh.drive_Out_Zone == true) //更新车辆驶出时的状态（时间和位置）
			vehs_Set[veh.id] = veh;
	}
	return;
}

void Space_Speed_Caculation::caculation_Index() {
	double distance = 0.0, ave_Speed_Sum = 0.0, ave_Delay_Sum = 0.0;
	int volume = 0;
	double time = 0.0;
	for (int i = 0; i < vehs_ID_Set.size(); i++) {
		if (vehs_Set[vehs_ID_Set[i]].drive_Out_Zone) {
			distance = sqrt(pow(vehs_Set[vehs_ID_Set[i]].pos_Drive_Out.x - vehs_Set[vehs_ID_Set[i]].pos_Veh_Recognize.x, 2.0) +
				pow(vehs_Set[vehs_ID_Set[i]].pos_Drive_Out.y - vehs_Set[vehs_ID_Set[i]].pos_Veh_Recognize.y, 2.0));               //计算区间距离
			time = vehs_Set[vehs_ID_Set[i]].time_Drive_Out - vehs_Set[vehs_ID_Set[i]].time_Veh_Recognize;                         //计算区间时间

			if (time > 0) {
				double speed = distance / time;
				lanes_Space_Speed[vehs_Set[vehs_ID_Set[i]].lane_Num] += speed;
				lanes_Space_Speed_Volume[vehs_Set[vehs_ID_Set[i]].lane_Num]++;
				ave_Speed_Sum += speed;
				ave_Delay_Sum += (time - distance / speed_Max);
				volume++;
			}
		}
	}
	if (volume > 0) {
		ave_Space_Speed = ave_Speed_Sum / volume;
		ave_Delay = ave_Delay_Sum / volume;
	}
	for (auto it = lanes_Space_Speed_Volume.begin(); it != lanes_Space_Speed_Volume.end(); it++) {
		if (lanes_Space_Speed_Volume[it->first] > 0)
			lanes_Space_Speed[it->first] /= lanes_Space_Speed_Volume[it->first];
	}
	return;
}

void Space_Speed_Caculation::update_Vehicles_Info() {
	int n = vehs_ID_Set.size() - 1;
	while ((n >= 0) && (current_Time - vehs_Set[vehs_ID_Set[n]].time_Veh_Recognize) >= window_Interval) {
		vehs_Set.erase(vehs_ID_Set[n]);
		vehs_ID_Set.pop_back();
		n--;
	}
	for (auto it = vehs_ID_Set.begin(); it != vehs_ID_Set.end();) {
		if (vehs_Set[*it].drive_Out_Zone) {                                    //if (vehs_Set[*it].drive_Out_Zone && vehs_Set[*it].drive_In_Zone){
			vehs_Set.erase(*it);
			it = vehs_ID_Set.erase(it);
		}
		else {
			it++;
		}
	}
	ave_Delay = 0.0;
	ave_Space_Speed = 0.0;
	lanes_Space_Speed.clear();
	lanes_Space_Speed_Volume.clear();
	return;
}

void Headway_Density_Caculation::get_Vehicles_Info(Vehicleincident_Detection& veh) {
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
	double speed_Sum_all = 0.0;
	for (auto it = vehs_Set.begin(); it != vehs_Set.end(); it++) {
		double volume = 0.0, headway = 0.0, speed_Sum = 0.0;
		int n = (*it).second.size() - 1, m = n + 1;
		while (n > 0) {
			headway = headway + sqrt(pow((*it).second[n].radar_px - (*it).second[n - 1].radar_px, 2.0) + pow((*it).second[n].radar_py - (*it).second[n - 1].radar_py, 2.0));
			volume = volume + car_Type.at((*it).second[n].cartype);
			speed_Sum = speed_Sum + (*it).second[n].speed;
			n--;
		}
		speed_Sum = speed_Sum + (*it).second[n].speed;
		speed_Sum_all = speed_Sum_all + speed_Sum;
		lanes_Space_Headway_Sum[(*it).first] += headway;
		lanes_Headway_Volume_Sum[(*it).first] += volume;
		lanes_Speed_Volume_Sum[(*it).first] += m;
		lanes_Time_Speed_Sum[(*it).first] += speed_Sum;

		if (lanes_Headway_Volume_Sum[(*it).first] > 0.0)
			lanes_Space_Headway[(*it).first] = lanes_Space_Headway_Sum[(*it).first] / lanes_Headway_Volume_Sum[(*it).first];       //计算车道的平均车头间距
		if (lanes_Speed_Volume_Sum[(*it).first] > 0.0)
			lanes_Time_Speed[(*it).first] = lanes_Time_Speed_Sum[(*it).first] / lanes_Speed_Volume_Sum[(*it).first];             //计算车道的平均速度
		zone_volume = zone_volume + m;                                                                                       //计算区域总流量
	}
	if (section_Flag == true) {
		double volume_Headway_Sum = 0.0;
		int volume_Speed_Sum = 0;
		for (auto it = lanes_Space_Headway.begin(); it != lanes_Space_Headway.end(); it++) {
			ave_Space_Headway += lanes_Space_Headway_Sum[(*it).first];
			volume_Headway_Sum += lanes_Headway_Volume_Sum[(*it).first];

			ave_Time_Speed += lanes_Time_Speed_Sum[(*it).first];
			volume_Speed_Sum += lanes_Speed_Volume_Sum[(*it).first];
		}
		ave_Space_Headway = (volume_Headway_Sum > 0) ? ave_Space_Headway / volume_Headway_Sum : 0.0;                                 //计算平均车头间距
		// ave_Time_Speed  = (volume_Speed_Sum > 0)?  ave_Time_Speed/volume_Speed_Sum : 0.0;                                      //计算平均速度
		ave_Time_Speed = (zone_volume > 0) ? speed_Sum_all / zone_volume : 0.0;
	}
	section_density = zone_volume / lanes_Length;                                                                              //区间密度，需要区间长度的信息
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
	zone_volume = 0;
	section_density = 0.0;

	ave_Space_Headway = 0.0;
	ave_Time_Speed = 0.0;
	return;
}

void Max_Queue_Caculation::create_Queue() {
	int lane_Num = 0;
	for (auto it = vehs_Set.begin(); it != vehs_Set.end(); it++) {
		lane_Num = (*it).first;
		double speed_Queue = (queue_Continue[lane_Num] == false) ? speed_Queue_Start : speed_Queue_End;
		vector<Vehicleincident_Detection> vehs = vehs_Set[lane_Num];
		vector<Vehicleincident_Detection> temp;
		double density_before = 0.0, volume = 0.0, dist = 0.0;
		bool veh_Status_Now = true, veh_Status_Before = true;
		for (int i = 0; i < vehs.size(); i++) {
			if (temp.size() == 0) {
				temp.emplace_back(vehs[i]);
				volume += car_Type.at(vehs[i].cartype);
				veh_Status_Before = (vehs[i].speed <= speed_Queue) ? true : false;
			}
			else {
				veh_Status_Now = (vehs[i].speed <= speed_Queue) ? true : false;
				while (veh_Status_Now == veh_Status_Before && veh_Status_Now && i < vehs.size()) {                   //以速度值划分车队，并且前后两车状态相同
					if (temp.size() < min_Queue_Size - 1)
						temp.emplace_back(vehs[i]);
					volume += car_Type.at(vehs[i].cartype);
					if (temp.size() >= min_Queue_Size - 1) {
						double x = temp[0].radar_px;
						double y = temp[0].radar_py;
						dist = sqrt(pow(vehs[i].radar_px, 2.0) + pow(vehs[i].radar_py, 2.0)) - sqrt(pow(x, 2.0) + pow(y, 2.0));
						double k = 1000 * (volume - 1) / dist;
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
	vector<Vehicleincident_Detection> temp;
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

void Max_Queue_Caculation::caculate_Queue(int lane_Num, int queue_Index, double speed) {
	int i = 0, j = 0;
	vector<Vehicleincident_Detection> temp;
	int queue_Num = 0;
	while (i < queue_Index) {
		temp = queues_Set[lane_Num].front();
		for (int j = temp.size() - 1; j > 0; j--) {
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

void Stops_Caculation::get_Vehicles_Info(Vehicleincident_Detection& veh) {
	if (veh.label == 5) {
		vehs_ID.emplace_back(veh.id);
		if (veh.speed <= speed_Stop_Start) {              //通过停车速度阈值，选择车辆
			if (vehs_Set.find(veh.id) == vehs_Set.end()) {
				vehs_Stop_Start.emplace(veh.id, veh.timestamp);
				vehs_Stops_Num.emplace(veh.id, 0.0);
			}
			if (current_Time - vehs_Stop_Start[veh.id] >= min_Stop_Duration && vehs_Set[veh.id].size() >= 2)
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
	Vehicleincident_Detection veh_Temp, veh_Temp_Before;
	double nstops_Sum = 0.0;
	for (auto it = vehs_Set.begin(); it != vehs_Set.end(); it++) {
		if ((*it).second.size() > 1) {
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
	Vehicleincident_Detection veh_Temp;
	for (auto it = vehs_Set.begin(); it != vehs_Set.end();) {
		veh_Temp = (*it).second[(*it).second.size() - 1];
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

void Vehicleincident_Detection::Overspeed()
{
	//if (lane_Num>10&&lane_Num<=20&&veh_In_Zone== true)
	if (veh_In_Zone == true)
		//if(radar_px<5)
	{
		if (speed > MaxLimitedSpeed && speed < MaxSpeedUpper)
		{
			overspeed = true;
		}
	}
	// else if (lane_Num==255&&veh_In_Zone== true)
	// {
	// 	if (speed > MaxLimitedSpeed && speed < MaxSpeedUpper)
	// 	{
	// 		overspeed = true;
	// 	}
	// }
}
void Vehicleincident_Detection::Lowspeed()
{
	//if (lane_Num>10&&lane_Num<=20&&veh_In_Zone== true)
	if (veh_In_Zone == true)
		//if(radar_px<5)
	{
		if (speed < MinLimitedSpeed && speed > ParkingSpeed)
		{
			lowspeed = true;
		}
	}
	// else if(lane_Num==255&&veh_In_Zone== true)
	// {
	// 	if (speed < MinLimitedSpeed && speed > ParkingSpeed)
	// 	{
	// 		lowspeed = true;
	// 	}
	// }
}
void Vehicleincident_Detection::Retrograde()
{
	//if(radar_px<-3)
	if (lane_Num > 11 && lane_Num <= 20 && veh_In_Zone == true)
	{
		if (radar_vy > 3)       //默认为radar_vy > 0
		{
			retrograde = true;
		}
	}
	// else if(lane_Num>31&&lane_Num<=40&&veh_In_Zone== true)
	// {
	// 	if(radar_vy < -3)   
	// 	{
	// 		retrograde = true;
	// 	}
	// }	
	else if (lane_Num == 255 && veh_In_Zone == true)
	{
		if (radar_px < -3 && radar_vy>3)
		{
			retrograde = true;
		}
		else if (radar_px > 3 && radar_vy < -3)
		{
			retrograde = true;
		}
		// else if(radar_px>=0&&radar_vy<=0)
		// {
		// 	retrograde = true;
		// }
	}
}
void Vehicleincident_Detection::Illegalparking()
{
	if (lane_Num > 10 && lane_Num <= 20 && veh_In_NoParking == true)
	{
		if (speed <= ParkingSpeed && speed >= 0)
		{
			//默认为if ((timestamp - time_Drive_In_NoParking) > MaxLimitedPresenceTime)
			if ((timestamp - time_Drive_In_NoParking) > MaxLimitedPresenceTime && (timestamp - time_Drive_In_NoParking) <= MaxLimitedTime)
				//if ((timestamp - 1605682317.731000000) > MaxLimitedPresenceTime)
			{
				illegalparking = true;
			}
		}
	}
	else if (lane_Num == 255 && veh_In_NoParking == true)
	{
		if (speed <= ParkingSpeed && speed >= 0)
		{
			//默认为if ((timestamp - time_Drive_In_NoParking) > MaxLimitedPresenceTime)
			if ((timestamp - time_Drive_In_NoParking) > MaxLimitedPresenceTime && (timestamp - time_Drive_In_NoParking) <= MaxLimitedTime)
				//if ((timestamp - 1605682317.731000000) > MaxLimitedPresenceTime)
			{
				illegalparking = true;
			}
		}
	}
}

Vehicleincident_Detection Illegallanechange(Vehicleincident_Detection test, map<int, Vehicleincident_Detection>& illegal)
{
	if ((test.lane_Num > 40 || test.lane_Num < 30) && test.veh_In_NoParking == true)
	{
		if (test.radar_py < 150)
		{
			auto iter = illegal.find(test.id);
			if (iter != illegal.end())
			{
				if (test.lane_Num != (iter->second.lane_Num) && test.lane_Num != 255)
				{
					test.illegallanechange = true;
				}
			}
			illegal[test.id] = test;
			//illegal.insert(map<int, Vehicleincident_Detection>::value_type(test.id, test));

		}
	}
	return test;
}

bool Gateway(vector<Vehicleincident_Detection> vehs_test, int lanes_Num)
{
	bool gatewaywarning = false;
	bool gatewaywarning1 = false;
	bool gatewaywarning2 = false;
	if (vehs_test.size() >= 1)
	{
		for (int j = 0; j < vehs_test.size(); j++)
		{
			if (vehs_test[j].veh_In_Intersection == true) //==
			{
				//printf("符合条件1的sec: %f,nsec: %f, ID: %d, lane_No: %d, pos_x: %f, pos_y: %f\n",vehs_test[j].sec,vehs_test[j].nsec,vehs_test[j].id,vehs_test[j].lane_Num,vehs_test[j].radar_px,vehs_test[j].radar_py);
				gatewaywarning1 = true;
			}
			else if ((vehs_test[j].lane_Num == 10 + lanes_Num) && (vehs_test[j].radar_py < 140 && vehs_test[j].radar_py>50)) //==&&车道编号需提前定义
			{
				//printf("符合条件2的sec: %f,nsec: %f, ID: %d, lane_No: %d, pos_x: %f, pos_y: %f\n",vehs_test[j].sec,vehs_test[j].nsec,vehs_test[j].id,vehs_test[j].lane_Num,vehs_test[j].radar_px,vehs_test[j].radar_py);
				gatewaywarning2 = true;
			}
		}
	}
	if (gatewaywarning1 == true && gatewaywarning2 == true)
	{
		gatewaywarning = true;
	}
	return gatewaywarning;
}

bool Accident(Vehicleincident_Detection test)
{
	bool accident = false;
	if ((test.lane_Num < 30 || test.lane_Num>40) && test.veh_In_Intersection == true) // ==
	{
		if (test.speed <= ParkingSpeed)
		{
			if ((test.timestamp - test.time_Drive_In_Intersection) > MaxLimitedAccidentTime && (test.timestamp - test.time_Drive_In_Intersection) <= MaxLimitedTime)
			{
				accident = true;
			}
		}
	}
	return accident;
}

int LinkCongestion(Max_Queue_Caculation max_queue_test) //需要修改车辆数量阈值MaxLimitedVehicleNum_Link
{
	int link_congestionlevel = 0;
	//printf("检测区域流量：%d\n",max_queue_test.zone_volume);
	if (max_queue_test.zone_volume >= MaxLimitedVehicleNum_Link)
	{
		if (max_queue_test.ave_Time_Speed < CongestionSpeed_Slight && max_queue_test.ave_Time_Speed >= CongestionSpeed_Moderate)
		{
			link_congestionlevel = 1;
		}
		else if (max_queue_test.ave_Time_Speed < CongestionSpeed_Moderate && max_queue_test.ave_Time_Speed >= CongestionSpeed_Severe)
		{
			link_congestionlevel = 2;
		}
		else if (max_queue_test.ave_Time_Speed <= CongestionSpeed_Severe)
		{
			link_congestionlevel = 3;
		}
	}
	return link_congestionlevel;
}

int LaneCongestion(Max_Queue_Caculation max_queue_test, int item) //需要修改车辆数量阈值MaxLimitedVehicleNum_Lane
{
	int lane_congestionlevel = 0;
	//printf("车道编号：%d,车道流量：%d\n",item,max_queue_test.lanes_Speed_Volume_Sum[item]);
	if (max_queue_test.lanes_Speed_Volume_Sum[item] >= MaxLimitedVehicleNum_Lane)
	{
		if (max_queue_test.lanes_Time_Speed[item] < CongestionSpeed_Slight && max_queue_test.lanes_Time_Speed[item] >= CongestionSpeed_Moderate)
		{
			lane_congestionlevel = 1;
		}
		else if (max_queue_test.lanes_Time_Speed[item] < CongestionSpeed_Moderate && max_queue_test.lanes_Time_Speed[item] >= CongestionSpeed_Severe)
		{
			lane_congestionlevel = 2;
		}
		else if (max_queue_test.lanes_Time_Speed[item] <= CongestionSpeed_Severe)
		{
			lane_congestionlevel = 3;
		}
	}
	return lane_congestionlevel;
}

void Headway_Density_Caculation::linkcongestion()
{

}


void read_Detect_JsonFile(string filename, tuple<map<string, vector<vector<Point>>>, int>& detect_Config) {
	ifstream ifs;
	ifs.open(filename, ios::binary);
	if (!ifs.is_open()) {
		return;
	}

	vector<string> data_Fields{ "entry_line","stop_line","detect_zone","no_parking","intersection_entry_line","intersection_stop_line",
	"noparking_entry_line","noparking_stop_line","intersection_zone" };
	Json::Value root;
	Json::Reader reader;
	if (reader.parse(ifs, root)) {
		if (!root["detect_config"].isNull()) {
			int nSize = data_Fields.size();
			Json::Value points;
			vector<vector<Point>> temp(1);
			for (int i = 0; i < nSize; i++) {
				points = root["detect_config"][data_Fields[i]];

				for (int j = 0; j < (int)points.size(); j++) {
					double x = root["detect_config"][data_Fields[i]][j]["x"].asDouble();
					double y = root["detect_config"][data_Fields[i]][j]["y"].asDouble();
					temp[0].push_back(Point(x, y));
				}
				get<0>(detect_Config).emplace(data_Fields[i], temp);
				temp.clear();
				temp.resize(1);
			}
		}
		if (!root["lanes_num"].isNull()) {
			int lanes_num = root["lanes_num"].asInt();
			get<1>(detect_Config) = lanes_num;
		}
	}
	ifs.close();
	return;
}

void read_Intersection_JsonFile(string filename, tuple<int>& intersection_Config) {
	ifstream ifs;
	ifs.open(filename, ios::binary);
	if (!ifs.is_open()) {
		return;
	}
	Json::Value root;
	Json::Reader reader;
	if (reader.parse(ifs, root)) {
		if (!root["devices_num"].isNull()) {
			int devices_num = root["devices_num"].asInt();
			get<0>(intersection_Config) = devices_num;
		}
	}
	ifs.close();
	return;
}

int lanenum_detector(float object_px, float object_py, float Vy, int source, int lane_id)
{
	int laneno = 0;
	if (source == 0)
	{
		if (object_py > 75 && object_py <= 115)
		{
			if (object_px < 0.3 && object_px >= -5 && Vy < 0)
			{
				laneno = 11;
			}
			else if (object_px < -5 && object_px >= -8.1)
			{
				laneno = 12;
			}
			else
			{
				laneno = lane_id;
			}
		}
		else if (object_py > 115)
		{
			if (object_px < 0.3 && object_px >= -4.5 && Vy < 0)
			{
				laneno = 11;
			}
			else if (object_px < -4.5 && object_px >= -7.5)
			{
				laneno = 12;
			}
			else
			{
				laneno = lane_id;
			}
		}
		else
		{
			laneno = lane_id;
		}
	}
	else if (source == 1)
	{
		if (object_py > 58 && object_py <= 95)
		{
			if (object_px < -1.6 && object_px >= -4.5 && Vy < 0)
			{
				laneno = 11;
			}
			else if (object_px < -4.5 && object_px >= -7.7)
			{
				laneno = 12;
			}
			else if (object_px < -7.7 && object_px >= -10.6)
			{
				laneno = 13;
			}
			else if (object_px < -10.6 && object_px >= -13.8)
			{
				laneno = 14;
			}
			else
			{
				laneno = lane_id;
			}
		}
		else if (object_py > 95)
		{
			if (object_px < -3.5 && object_px >= -6.5 && Vy < 0)
			{
				laneno = 11;
			}
			else if (object_px < -6.5 && object_px >= -9.5)
			{
				laneno = 12;
			}
			else if (object_px < -9.5 && object_px >= -12.5)
			{
				laneno = 13;
			}
			else if (object_px < -12.5 && object_px >= -13.8)
			{
				laneno = 14;
			}
			else
			{
				laneno = lane_id;
			}
		}
		else
		{
			laneno = lane_id;
		}
	}
	else if (source == 2)
	{
		if (object_py > 70 && object_py <= 112)
		{
			if (object_px < 0.3 && object_px >= -4.2 && Vy < 0)
			{
				laneno = 11;
			}
			else if (object_px < -4.2 && object_px >= -8)
			{
				laneno = 12;
			}
			else
			{
				laneno = lane_id;
			}
		}
		else if (object_py > 112)
		{
			if (object_px < 0.3 && object_px >= -3.2 && Vy < 0)
			{
				laneno = 11;
			}
			else if (object_px<-3.3 && object_px>-6)
			{
				laneno = 12;
			}
			else
			{
				laneno = lane_id;
			}
		}
		else
		{
			laneno = lane_id;
		}
	}
	else if (source == 3)
	{
		if (object_py > 60 && object_py <= 100)
		{
			if (object_px < -4 && object_px >= -7.1 && Vy < 0)
			{
				laneno = 11;
			}
			else if (object_px < -7.1 && object_px >= -10.1)
			{
				laneno = 12;
			}
			else if (object_px < -10.1 && object_px >= -13.1)
			{
				laneno = 13;
			}
			else if (object_px < -13.1 && object_px >= -16.1)
			{
				laneno = 14;
			}
			else
			{
				laneno = lane_id;
			}
		}
		else if (object_py > 100)
		{
			if (object_px < -4 && object_px >= -7 && Vy < 0)
			{
				laneno = 11;
			}
			else if (object_px < -7 && object_px >= -10.1)
			{
				laneno = 12;
			}
			else if (object_px < -10.1 && object_px >= -13.1)
			{
				laneno = 13;
			}
			else if (object_px < -13.1 && object_px >= -16.1)
			{
				laneno = 14;
			}
			else
			{
				laneno = lane_id;
			}
		}
		else
		{
			laneno = lane_id;
		}
	}
	return laneno;
}