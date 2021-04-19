#include "event_detect_in_class.h"

Event_detect::Event_detect(){
    string json_File_Name = "F:\\202011MEC交通开发\\code\\traffic_index\\detect_config_file\\intersection_Config.json";
    read_Intersection_JsonFile(json_File_Name, intersection_Config);
    for (int i = 0; i < get<0>(intersection_Config); i++) {
		string temp = (i < 10) ? "0" + to_string(i) : to_string(i);
        json_File_Name="F:\\202011MEC交通开发\\code\\traffic_index\\detect_config_file\\detect_Config_" + temp + ".json";
		intersection_Devices_Detect.emplace(i, Device_Detect(json_File_Name));
    }
	recieve_data();
}

Event_detect::~Event_detect(){}

void Event_detect::recieve_data()
{
    int object_event_source = 4;
	vector<vector<Vehicleincident_Detection>> vehs_test = { {Vehicleincident_Detection(1,1,1,12, 5, "light", -1.7,205.0, 1.3, 0, 4, 1.8),
												  Vehicleincident_Detection(1,1,2,12, 5, "light", -1.7,203.5, 1.3, 0, 4, 1.8),
												 Vehicleincident_Detection(1,1,3,12, 5, "light", -1.7,184.0, 1.3, 0, 4, 1.8),
												  Vehicleincident_Detection(1,1,4,12, 5, "light", -1.7,180.0, 1.3, 0, 4, 1.8),
												 Vehicleincident_Detection(1,1,5,12, 5, "light", -1.7,75.0 , 1.3, 0, 4, 1.8) },
	                                             {Vehicleincident_Detection(2,1,1,12, 5, "light", -1.7,70.0 , 1.3, 0, 4, 1.8),
		                                         Vehicleincident_Detection(2,1,2,12,5, "light",-1.7,42.0 , 1.3, 0, 4, 1.8),
												  Vehicleincident_Detection(2,1,3,12,5, "light", -1.7,41.0 , 1.3, 0, 4, 1.8),
		                                         Vehicleincident_Detection(2,1,4,12,5, "light", -1.7,31.0 , 1.3, 0, 4, 1.8),
												  Vehicleincident_Detection(2,1,5,12,5, "light", -1.7, 30.5, 10, 0, 4, 1.8)}
	};

    if (vehs_test.size() == 0){
        printf("this frame has no object \n");
		return;
	}
	second_Clock_Type current_Time = time_point_cast<seconds>(system_clock::now());
	time_t now_Time = system_clock::to_time_t(current_Time);
	struct tm* now_Time_Norm;
	now_Time_Norm = localtime(&now_Time);
	string time_String = to_string(1900 + now_Time_Norm->tm_year) + "-" + to_string(1 + now_Time_Norm->tm_mon) + "-" + to_string(now_Time_Norm->tm_mday) + " " + to_string(now_Time_Norm->tm_hour) + ":" + to_string(now_Time_Norm->tm_min) + ":" + to_string(now_Time_Norm->tm_sec);

	double time_stamp = 0.0;
	double time_stamp_nsec = 0.0;
	for (int i = 0; i < vehs_test.size(); i++) {
		int source = 1;
		for (int j = 0; j < vehs_test[i].size(); j++) {
			vehs_test[i][j] = Location_In_Zone("detect_zone", vehs_test[i][j], intersection_Devices_Detect[source].detect_Config_Points).update_Veh();                //区域检测
			time_stamp = vehs_test[i][j].sec;
			time_stamp_nsec = vehs_test[i][j].timestamp;
			intersection_Devices_Detect[source].speed_test.current_Time = vehs_test[i][j].timestamp;
			//intersection_Devices_Detect[source].stops_test.current_Time = vehs_test[i][j].timestamp;

			/*
			vehs_test[i][j] = Location_In_Zone("intersection_zone", vehs_test[i][j], intersection_Devices_Detect[source].detect_Config_Points).update_Veh();  //区域检测
			vehs_test[i][j] = Location_In_Zone("no_parking", vehs_test[i][j], intersection_Devices_Detect[source].detect_Config_Points).update_Veh();  //区域检测
			vehs_test[i][j] = Location_Cross_Line("noparking_entry_line", intersection_Devices_Detect[source].map_Vehs_Entry_noparking, vehs_test[i][j], intersection_Devices_Detect[source].detect_Config_Points, time_stamp, window_Interval).update_Veh();
			vehs_test[i][j] = Location_Cross_Line("noparking_stop_line", intersection_Devices_Detect[source].map_Vehs_Stop_noparking, vehs_test[i][j], intersection_Devices_Detect[source].detect_Config_Points, time_stamp, window_Interval).update_Veh();
			vehs_test[i][j] = Location_Cross_Line("intersection_entry_line", intersection_Devices_Detect[source].map_Vehs_Entry_intersection, vehs_test[i][j], intersection_Devices_Detect[source].detect_Config_Points, time_stamp, window_Interval).update_Veh();
			vehs_test[i][j] = Location_Cross_Line("intersection_stop_line", intersection_Devices_Detect[source].map_Vehs_Stop_intersection, vehs_test[i][j], intersection_Devices_Detect[source].detect_Config_Points, time_stamp, window_Interval).update_Veh();
			vehs_test[i][j].Overspeed();                              //是否超速
			vehs_test[i][j].Lowspeed();                               //是否低速
			vehs_test[i][j].Retrograde();                             //是否逆行
			vehs_test[i][j].Illegalparking();                         //是否违规停车
			vehs_test[i][j] = Illegallanechange(vehs_test[i][j], intersection_Devices_Detect[source].illegal);   //是否违规车辆变道
			bool accidentlabel = false;
			accidentlabel = Accident(vehs_test[i][j]);                  //是否发生事故
			*/
			/*
			if (vehs_test[i][j].overspeed == true)
			{
				int eventType = 901;
				auto iter = intersection_Devices_Detect[source].map_trafficincident.find(vehs_test[i][j].id);
				if (iter == intersection_Devices_Detect[source].map_trafficincident.end())
				{
					eventID++;
					eventID = eventID % 65536;
					intersection_Devices_Detect[source].map_trafficincident[vehs_test[i][j].id] = eventID;
					//printf("eventID:%d, eventType:%d, description:{车辆ID:%d, 车道ID:%d, lat:%f, lon:%f, 速度:%f}, eventPos:{lat:%f, lon:%f, ele:%f}, eventSource:%d \n", eventID, eventType, id, vehs_test[i][j].lane_Num, lat, lon, vehs_test[i][j].speed*3.6, lat, lon, ele, object_event_source);
				}
				else
				{
					//printf("eventID:%d, eventType:%d, description:{车辆ID:%d, 车道ID:%d, lat:%f, lon:%f, 速度:%f}, eventPos:{lat:%f, lon:%f, ele:%f}, eventSource:%d \n", iter->second, eventType, id, vehs_test[i][j].lane_Num, lat, lon, vehs_test[i][j].speed*3.6, lat, lon, ele, object_event_source);
				}

			}
			else {
				printf("not overspeed,速度");
			}

			if (vehs_test[i][j].lowspeed == true && vehs_test[i][j].illegalparking == false && accidentlabel == false) {
				int eventType = 902;
				auto iter = intersection_Devices_Detect[source].map_trafficincident.find(vehs_test[i][j].id);
				if (iter == intersection_Devices_Detect[source].map_trafficincident.end()) {
					eventID++;
					eventID = eventID % 65536;
					intersection_Devices_Detect[source].map_trafficincident[vehs_test[i][j].id] = eventID;
					//printf("eventID:%d, eventType:%d, description:{车辆ID:%d, 车道ID:%d, lat:%f, lon:%f, 速度:%f}, eventPos:{lat:%f, lon:%f, ele:%f}, eventSource:%d \n", eventID, eventType, id, vehs_test[i][j].lane_Num, lat, lon, vehs_test[i][j].speed*3.6, lat, lon, ele, object_event_source);
				}
				else {
					//printf("eventID:%d, eventType:%d, description:{车辆ID:%d, 车道ID:%d, lat:%f, lon:%f, 速度:%f}, eventPos:{lat:%f, lon:%f, ele:%f}, eventSource:%d \n", iter->second, eventType, id, vehs_test[i][j].lane_Num, lat, lon, vehs_test[i][j].speed*3.6, lat, lon, ele, object_event_source);
				}

			}

			if (vehs_test[i][j].illegalparking == true && accidentlabel == false) {
				int eventType = 903;
				auto iter = intersection_Devices_Detect[source].map_trafficincident.find(vehs_test[i][j].id);
				if (iter == intersection_Devices_Detect[source].map_trafficincident.end()) {
					eventID++;
					eventID = eventID % 65536;
					intersection_Devices_Detect[source].map_trafficincident[vehs_test[i][j].id] = eventID;
					//printf("eventID:%d, eventType:%d, description:{车辆ID:%d, 车道ID:%d, lat:%f, lon:%f, 速度:%f}, eventPos:{lat:%f, lon:%f, ele:%f}, eventSource:%d \n", eventID, eventType, id, vehs_test[i][j].lane_Num, lat, lon, vehs_test[i][j].speed*3.6, lat, lon, ele, object_event_source);
				}
				else {
					//printf("eventID:%d, eventType:%d, description:{车辆ID:%d, 车道ID:%d, lat:%f, lon:%f, 速度:%f}, eventPos:{lat:%f, lon:%f, ele:%f}, eventSource:%d \n", iter->second, eventType, id, vehs_test[i][j].lane_Num, lat, lon, vehs_test[i][j].speed*3.6, lat, lon, ele, object_event_source);
				}
			}

			if (vehs_test[i][j].retrograde == true) {
				int eventType = 904;
				auto iter = intersection_Devices_Detect[source].map_trafficincident.find(vehs_test[i][j].id);
				if (iter == intersection_Devices_Detect[source].map_trafficincident.end()) {
					eventID++;
					eventID = eventID % 65536;
					intersection_Devices_Detect[source].map_trafficincident[vehs_test[i][j].id] = eventID;
					//printf("eventID:%d, eventType:%d, description:{车辆ID:%d, 车道ID:%d, lat:%f, lon:%f, 速度:%f}, eventPos:{lat:%f, lon:%f, ele:%f}, eventSource:%d \n", eventID, eventType, id, vehs_test[i][j].lane_Num, lat, lon, vehs_test[i][j].speed*3.6, lat, lon, ele, object_event_source);
				}
				else {
					//printf("eventID:%d, eventType:%d, description:{车辆ID:%d, 车道ID:%d, lat:%f, lon:%f, 速度:%f}, eventPos:{lat:%f, lon:%f, ele:%f}, eventSource:%d \n", iter->second, eventType, id, vehs_test[i][j].lane_Num, lat, lon, vehs_test[i][j].speed*3.6, lat, lon, ele, object_event_source);
				}
			}

			if (vehs_test[i][j].illegallanechange == true) {
				int eventType = 1026;
				auto iter = intersection_Devices_Detect[source].map_trafficincident.find(vehs_test[i][j].id);
				if (iter == intersection_Devices_Detect[source].map_trafficincident.end()) {
					eventID++;
					eventID = eventID % 65536;
					intersection_Devices_Detect[source].map_trafficincident[vehs_test[i][j].id] = eventID;
				}
				else {
				}
			}
			if (accidentlabel == true && vehs_test[i][j].illegalparking == false) {
				int eventType = 100;
				auto iter = intersection_Devices_Detect[source].map_trafficincident.find(vehs_test[i][j].id);
				if (iter == intersection_Devices_Detect[source].map_trafficincident.end()) {
					eventID++;
					eventID = eventID % 65536;
					intersection_Devices_Detect[source].map_trafficincident[vehs_test[i][j].id] = eventID;
				}
				else {

				}
			}
			*/
			//printf("sec: %f, nsec: %f, ID: %d,label: %d,lane_No: %d, pos_x: %f, pos_y: %f, Vx: %f,Vy: %f,existtime: %f,veh_In_Intersection: %d \n,",sec,nsec,id,vehs_test[i][j].label,vehs_test[i][j].lane_Num,object_px,object_py,object_vx,object_vy,(vehs_test[i][j].timestamp - vehs_test[i][j].time_Drive_In_NoParking),vehs_test[i][j].veh_In_Intersection);
			//printf("Overspeed: %d, Lowspeed: %d, Retrograde: %d, Illegalparking: %d, Illegallanechange: %d, Accident %d\n",vehs_test[i][j].overspeed,vehs_test[i][j].lowspeed,vehs_test[i][j].retrograde,vehs_test[i][j].illegalparking,vehs_test[i][j].illegallanechange,accidentlabel);

			if (vehs_test[i][j].lane_Num < intersection_Devices_Detect[source].lanes_Num) {
				//printf("sec: %f, nsec: %f, ID: %d, label: %d, lane_No: %d, pos_x: %f, pos_y: %f, Vx: %f,Vy: %f \n,",sec,nsec,id,vehs_test[i][j].label,vehs_test[i][j].lane_Num,object_px,object_py,object_vx,object_vy);
				vehs_test[i][j] = Location_Cross_Line("entry_line", intersection_Devices_Detect[source].map_Vehs_Entry, vehs_test[i][j], intersection_Devices_Detect[source].detect_Config_Points, time_stamp, window_Interval).update_Veh();  //驶入区域检测
				vehs_test[i][j] = Location_Cross_Line("stop_line", intersection_Devices_Detect[source].map_Vehs_Stop, vehs_test[i][j], intersection_Devices_Detect[source].detect_Config_Points, time_stamp, window_Interval).update_Veh();   //驶出区域检测

				intersection_Devices_Detect[source].volume_test.get_Vehicles_Info(vehs_test[i][j]);       //采集车辆，用于计算流量
				intersection_Devices_Detect[source].speed_test.get_Vehicles_Info(vehs_test[i][j]);        //采集车辆，用于计算空间平均速度
				intersection_Devices_Detect[source].capacity_test.get_Vehicles_Info(vehs_test[i][j]);     //采集车辆，用于计算平均车头时距和车道通行能力
				intersection_Devices_Detect[source].max_queue_test.get_Vehicles_Info(vehs_test[i][j]);    //采集车辆，用于计算速度、密度、车头间距和排队长度
			}
			//intersection_Devices_Detect[source].vehs_test.push_back(vehs_test[i][j]);
		}


	}


	for (auto it = intersection_Devices_Detect.begin(); it != intersection_Devices_Detect.end(); it++) {
		it->second.max_queue_test.Headway_Density_Caculation::caculation_Index();                           //计算车头间距、密度
		it->second.max_queue_test.caculation_Index();                                                       //计算排队长度

		//it->second.gatewaywarninglable = Gateway(it->second.vehs_test, it->second.lanes_Num);               //是否存在汇入预警
		//it->second.congestionlabel = Congestion(it->second.vehs_test, it->second.max_queue_test);          //是否发生拥堵
	}
	/*
	if (fmod(floor(time_stamp_nsec * 10), time_Interval) == 0.0) {
		for (auto it = intersection_Devices_Detect.begin(); it != intersection_Devices_Detect.end(); it++) {
			if (it->second.gatewaywarninglable == true) {
				eventID++;
				eventID = eventID % 65536;
				int eventType = 1004;
			}

			if (it->second.congestionlabel == 1 || it->second.congestionlabel == 2 || it->second.congestionlabel == 3) {
				eventID++;
				eventID = eventID % 65536;
				int eventType = 707;
			}
			else {

			}
		}
	}
	*/

	if (fmod(time_stamp, time_Interval) == 0.0 && !flag) {
		eventID = (eventID + 1) % 65536;
		int eventType = 1021;
		int effective_timeheadway_num = 0;
		string index_description = "{";
		for (auto it = intersection_Devices_Detect.begin(); it != intersection_Devices_Detect.end(); it++) {
			it->second.volume_test.caculation_Index();                                                               //计算流量
			it->second.capacity_test.Time_Headway_Caculation::caculation_Index();                                    //计算平均车头时距
			it->second.speed_test.caculation_Index();                                                                //计算空间平均速度

			double sum_Volume = 0.0, sum_timeheadway = 0.0, max_queue_length = 0.0;
			string lane_description = "{";
			for (int i = 0; i < it->second.volume_test.lane_Code.size(); i++) {
				sum_Volume += it->second.volume_test.lanes_Volume[it->second.volume_test.lane_Code[i]];
				sum_timeheadway += it->second.capacity_test.ave_Time_Headway[it->second.capacity_test.lane_Code[i]];
				if (it->second.capacity_test.ave_Time_Headway[it->second.capacity_test.lane_Code[i]] > 0.0)
					effective_timeheadway_num++;
				if (it->second.max_queue_test.lanes_Queue_Length[it->second.max_queue_test.lane_Code[i]] > max_queue_length)
					max_queue_length = it->second.max_queue_test.lanes_Queue_Length[it->second.max_queue_test.lane_Code[i]];

				lane_description += "{" + to_string(it->second.volume_test.lane_Code[i]) + "," + to_string(3.6 * it->second.speed_test.lanes_Space_Speed[it->second.volume_test.lane_Code[i]]) + "," + to_string(3600 / time_Interval * it->second.volume_test.lanes_Volume[it->second.volume_test.lane_Code[i]]) + "," + to_string(0.0) + "," + to_string(0.0) + "," + \
					to_string(it->second.max_queue_test.lanes_Space_Headway[it->second.volume_test.lane_Code[i]]) + "," + to_string(it->second.capacity_test.ave_Time_Headway[it->second.capacity_test.lane_Code[i]]) + "," + to_string(max(0.0, it->second.max_queue_test.lanes_Queue_Length[it->second.max_queue_test.lane_Code[i]])) + "}";
				if (i < (it->second.volume_test.lane_Code.size() - 1))
					lane_description += ",";

				printf("[%s] eventID:%d, eventType:%d, description:{车道ID:%d, 平均空间速度:%f}, eventSource:%d, deviceSource:%d \n", time_String.c_str(), eventID, eventType, it->second.speed_test.lane_Code[i], 3.6 * it->second.speed_test.lanes_Space_Speed[it->second.volume_test.lane_Code[i]], object_event_source, it->first);
				printf("[%s] eventID:%d, eventType:%d, description:{车道ID:%d, 车道流量:%f}, eventSource:%d, deviceSource:%d \n", time_String.c_str(), eventID, eventType, it->second.volume_test.lane_Code[i], it->second.volume_test.lanes_Volume[it->second.volume_test.lane_Code[i]], object_event_source, it->first);
				printf("[%s] eventID:%d, eventType:%d, description:{车道ID:%d, 平均车头间距:%f}, eventSource:%d, deviceSource:%d \n", time_String.c_str(), eventID, eventType, it->second.max_queue_test.lane_Code[i], it->second.max_queue_test.lanes_Space_Headway[it->second.volume_test.lane_Code[i]], object_event_source, it->first);
				printf("[%s] eventID:%d, eventType:%d, description:{车道ID:%d, 平均车头时距:%f}, eventSource:%d, deviceSource:%d \n", time_String.c_str(), eventID, eventType, it->second.capacity_test.lane_Code[i], it->second.capacity_test.ave_Time_Headway[it->second.capacity_test.lane_Code[i]], object_event_source, it->first);
				printf("[%s] eventID:%d, eventType:%d, description:{车道ID:%d, 车道排队长度:%f}, eventSource:%d, deviceSource:%d \n", time_String.c_str(), eventID, eventType, it->second.max_queue_test.lane_Code[i], max(0.0, it->second.max_queue_test.lanes_Queue_Length[it->second.max_queue_test.lane_Code[i]]), object_event_source, it->first);

			}
			lane_description += "}";
			index_description += "{" + to_string(it->first + 1) + "," + to_string(3.6 * it->second.speed_test.ave_Space_Speed) + "," + to_string(3600 / time_Interval * sum_Volume) + "," + to_string(0.0) + "," + to_string(0.0) + "," + to_string(it->second.max_queue_test.ave_Space_Headway) + "," \
				+ to_string((effective_timeheadway_num > 0) ? sum_timeheadway / effective_timeheadway_num : 0.0) + "," + to_string(max_queue_length) + "}," + lane_description;
			if (it->first < intersection_Devices_Detect.size() - 1)
				index_description += ",";

			printf("[%s] eventID:%d, eventType:%d, description:{平均空间速度:%f}, eventSource:%d, deviceSource:%d \n", time_String.c_str(), eventID, eventType, 3.6 * it->second.speed_test.ave_Space_Speed, object_event_source, it->first);
			printf("[%s] eventID:%d, eventType:%d, description:{流量:%f}, eventSource:%d, deviceSource:%d \n", time_String.c_str(), eventID, eventType, 3600 / time_Interval * sum_Volume, object_event_source, it->first);
			printf("[%s] eventID:%d, eventType:%d, description:{平均车头间距:%f}, eventSource:%d, deviceSource:%d \n", time_String.c_str(), eventID, eventType, it->second.max_queue_test.ave_Space_Headway, object_event_source, it->first);
			printf("[%s] eventID:%d, eventType:%d, description:{平均车头时距:%f}, eventSource:%d, deviceSource:%d \n", time_String.c_str(), eventID, eventType, (effective_timeheadway_num > 0) ? sum_timeheadway / effective_timeheadway_num : 0.0, object_event_source, it->first);
			printf("[%s] eventID:%d, eventType:%d, description:{排队长度:%f}, eventSource:%d, deviceSource:%d \n", time_String.c_str(), eventID, eventType, max_queue_length, object_event_source, it->first);

			it->second.volume_test.update_Vehicles_Info();
			it->second.speed_test.update_Vehicles_Info();
			it->second.capacity_test.update_Vehicles_Info();
		}
		index_description += "}";

		printf("description: %s \n", index_description.c_str());
		flag = true;
	}

	for (auto it = intersection_Devices_Detect.begin(); it != intersection_Devices_Detect.end(); it++) {
		it->second.max_queue_test.Headway_Density_Caculation::update_Vehicles_Info();
		it->second.max_queue_test.update_Vehicles_Info();
	}

	if (fmod(time_stamp, time_Interval) != 0.0)
		flag = false;
	printf("----traffic_incident for road succeed!!---- \n");

    return;
}

