#include "event_detect_in_class.h"

Event_detect::Event_detect(){
    string json_File_Name = "../detect_config_file/intersection_Config.json";
    read_Intersection_JsonFile(json_File_Name, intersection_Config);
	devices_num = get<0>(intersection_Config);
    for (int i = 0; i < devices_num; i++) {
		string temp = (i < 10) ? "0" + to_string(i) : to_string(i);
        json_File_Name="../detect_config_file/detect_Config_" + temp + ".json";
		intersection_Devices_Detect.emplace(i, Device_Detect(json_File_Name));
    }
	recieve_data();
}

void Event_detect::update_Config_Info() {
	MaxSpeedUpper = get<1>(intersection_Config);
	MaxLimitedSpeed = get<2>(intersection_Config);
	MinLimitedSpeed = get<3>(intersection_Config);
	ParkingSpeed = get<4>(intersection_Config);
	NegativeSpeed = get<5>(intersection_Config);
	MaxLimitedVehicleNum_Link = get<6>(intersection_Config);
	MaxLimitedVehicleNum_Lane = get<7>(intersection_Config);
	MaxLimitedPresenceTime = get<8>(intersection_Config);
	MaxLimitedAccidentTime = get<9>(intersection_Config);
	MaxLimitedTime = get<10>(intersection_Config);
	CongestionSpeed_Slight = get<11>(intersection_Config);
	CongestionSpeed_Moderate = get<12>(intersection_Config);
	CongestionSpeed_Severe = get<13>(intersection_Config);
	time_Interval = get<15>(intersection_Config);                                             //更新统计的时间间隔

	pz = get<21>(intersection_Config);
	for (auto it = intersection_Devices_Detect.begin(); it != intersection_Devices_Detect.end(); it++) {
		int i = it->first;
		std::string temp = (i < 10) ? "0" + to_string(i) : to_string(i);
		std::string json_File_Name = config_File_Name + "detect_Config_" + temp + ".json";
		read_Detect_JsonFile(json_File_Name, it->second.detect_Config);
		it->second.detect_Config_Points = get<0>(it->second.detect_Config);                     //更新进口道的配置信息，来源于detect_Config_

		it->second.speed_test.window_Interval = get<14>(intersection_Config);                   //更新时间窗口
		it->second.speed_test.speed_Max = get<2>(intersection_Config);                          //更新路段最大限速值

		it->second.capacity_test.sat_Max_Headway = get<16>(intersection_Config);                //更新饱和车头时距的最大阈值
		it->second.capacity_test.speed_Max = get<2>(intersection_Config);                       //更新路段最大限速值

		it->second.max_queue_test.get_Lanes_Info(it->second.detect_Config_Points);              //更新虚拟停车线的位置
		it->second.max_queue_test.speed_Queue_Start = get<17>(intersection_Config);             //更新排队形成的速度 m/s
		it->second.max_queue_test.speed_Queue_End = get<18>(intersection_Config);               //更新排队消散的速度 m/s
		it->second.max_queue_test.min_Queue_Size = get<19>(intersection_Config);                //更新车队的最小车辆数
	}
	return;
}

Event_detect::~Event_detect(){}

void Event_detect::recieve_data()
{
    int object_event_source = 4;
	vector<vector<Vehicleincident_Detection>> vehs_test = { {Vehicleincident_Detection(1621411818.0,973733361.0,2991,13,5,"light",-11.900000,57.299999,0.000000,-3.000000,4,1.8)},
		{Vehicleincident_Detection(1621411819.0,24762742.0,2991,13,5,"light",-11.900000,55.200001,0.000000,-1.400000,4,1.8)},
		{Vehicleincident_Detection(1621411820.0,41346170.0,2991,13,5,"light",-12.300000,55.200001,0.000000,0.000000,4,1.8)} };
	
	/*{ {Vehicleincident_Detection(1,1,1,12, 5, "light", -1.7,205.0, 1.3, 0, 4, 1.8),
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
	*/

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

	if (fmod(floor(now_Time_Norm->tm_sec), 5.0) == 0.0) {
		read_Intersection_JsonFile(config_File_Name + "intersection_Config.json", intersection_Config);               //更新配置参数
		update_Config_Info();
	}

	for (int i = 0; i < vehs_test.size(); i++) {
		int source = 0;
		for (int j = 0; j < vehs_test[i].size(); j++) {
			int direction = source + 1;
			int road_id = 0;
			if (direction > 2){
				if (vehs_test[i][j].radar_py > 15)
					road_id = direction - 2;
				else
					road_id = direction;
            }else{
				if (vehs_test[i][j].radar_py > 15)
					road_id = direction + 2;
				else
					road_id = direction;
			}

			if (pz >= 0 && source != pz)
				continue;
			if (vehs_test[i][j].speed > MaxSpeedUpper || vehs_test[i][j].id <= 0)
				continue;
			
			vehs_test[i][j] = Location_In_Zone("detect_zone", vehs_test[i][j], intersection_Devices_Detect[source].detect_Config_Points).update_Veh();                //区域检测
			time_stamp = vehs_test[i][j].sec;
			time_stamp_nsec = vehs_test[i][j].timestamp;
			intersection_Devices_Detect[source].speed_test.current_Time = vehs_test[i][j].timestamp;
			//intersection_Devices_Detect[source].stops_test.current_Time = vehs_test[i][j].timestamp;

			/*
			veh_test = Location_In_Zone("intersection_zone", veh_test, intersection_Devices_Detect[source].detect_Config_Points).update_Veh();  //区域检测
			veh_test = Location_In_Zone("no_parking", veh_test, intersection_Devices_Detect[source].detect_Config_Points).update_Veh();  //区域检测
            veh_test = Location_Cross_Line("noparking_entry_line", intersection_Devices_Detect[source].map_Vehs_Entry_noparking, veh_test, intersection_Devices_Detect[source].detect_Config_Points, time_stamp, window_Interval).update_Veh();
            veh_test = Location_Cross_Line("noparking_stop_line", intersection_Devices_Detect[source].map_Vehs_Stop_noparking, veh_test, intersection_Devices_Detect[source].detect_Config_Points, time_stamp, window_Interval).update_Veh();
            veh_test = Location_Cross_Line("intersection_entry_line", intersection_Devices_Detect[source].map_Vehs_Entry_intersection, veh_test, intersection_Devices_Detect[source].detect_Config_Points, time_stamp, window_Interval).update_Veh();
            veh_test = Location_Cross_Line("intersection_stop_line", intersection_Devices_Detect[source].map_Vehs_Stop_intersection, veh_test, intersection_Devices_Detect[source].detect_Config_Points, time_stamp, window_Interval).update_Veh();    
            veh_test.Overspeed(MaxLimitedSpeed,MaxSpeedUpper);                                                                  //是否超速
			veh_test.Lowspeed(MinLimitedSpeed,ParkingSpeed);                                                                   //是否低速
			veh_test.Retrograde(NegativeSpeed);                                                                 //是否逆行
			veh_test.Illegalparking(ParkingSpeed,MaxLimitedPresenceTime,MaxLimitedTime);                                                             //是否违规停车
            veh_test = Illegallanechange(veh_test, intersection_Devices_Detect[source].illegal);                //是否违规车辆变道
            bool accidentlabel=false;
            accidentlabel=Accident(veh_test,ParkingSpeed,MaxLimitedAccidentTime,MaxLimitedTime);                                                      //是否发生事故

            if(veh_test.overspeed==true){
				int eventType = 901;
				auto iter = intersection_Devices_Detect[source].map_trafficincident.find(id);
                std::string eventlog_msgs;
                if(iter==intersection_Devices_Detect[source].map_trafficincident.end()){
                    eventID++;
                    eventID=eventID%65536;
                    intersection_Devices_Detect[source].map_trafficincident[id]=eventID;
                    //printf("[%s] eventID:%d, eventType:%d, description:{路段ID:%d, 车道ID:%d, 车辆ID:%d, lat:%d, lon:%d, 速度:%f, 车牌:%s}, eventPos:{lat:%d, lon:%d, ele:%d}, eventSource:%d, deviceSource:%d \n",time_String.c_str(),eventID,eventType,road_id,lane_id,id,lat,lon,veh_test.speed*3.6,plate_num.c_str(),lat,lon,ele,object_event_source,source);
                    eventlog_msgs="["+time_String+"] eventID:"+to_string(eventID)+", eventType:"+to_string(eventType)+", description:{路段ID:"+to_string(road_id)+", 车道ID:"+to_string(lane_id)+", 车辆ID:"+to_string(id)+", lat:"+to_string(lat)+", lon:"+to_string(lon)+", 速度:"+to_string(veh_test.speed*3.6)+", 车牌:"+plate_num+"}, eventPos:{lat:"+to_string(lat)+", lon:"+to_string(lon)+", ele:"+to_string(ele)+"}, eventSource:"+to_string(object_event_source)+", deviceSource:"+to_string(source)+" \n";
                }else{
                    //printf("[%s] eventID:%d, eventType:%d, description:{路段ID:%d, 车道ID:%d, 车辆ID:%d, lat:%d, lon:%d, 速度:%f, 车牌:%s}, eventPos:{lat:%d, lon:%d, ele:%d}, eventSource:%d, deviceSource:%d \n",time_String.c_str(),iter->second,eventType,road_id,lane_id,id,lat,lon,veh_test.speed*3.6,plate_num.c_str(),lat,lon,ele,object_event_source,source);
                    eventlog_msgs="["+time_String+"] eventID:"+to_string(iter->second)+", eventType:"+to_string(eventType)+", description:{路段ID:"+to_string(road_id)+", 车道ID:"+to_string(lane_id)+", 车辆ID:"+to_string(id)+", lat:"+to_string(lat)+", lon:"+to_string(lon)+", 速度:"+to_string(veh_test.speed*3.6)+", 车牌:"+plate_num+"}, eventPos:{lat:"+to_string(lat)+", lon:"+to_string(lon)+", ele:"+to_string(ele)+"}, eventSource:"+to_string(object_event_source)+", deviceSource:"+to_string(source)+" \n";
                }
				std::string description = "{" + to_string(road_id) + "," + to_string(lane_id) + "," +  to_string(id) + "," + to_string(lat) + "," + to_string(lon) + "," + to_string(veh_test.speed*3.6) + "," + plate_num + "}";
			}
			else {
				printf("not overspeed,速度");
			}

			if (vehs_test[i][j].lowspeed == true && vehs_test[i][j].illegalparking == false && accidentlabel == false) {
				int eventType = 902;
				auto iter = intersection_Devices_Detect[source].map_trafficincident.find(vehs_test[i][j].id);
				std::string eventlog_msgs;
                if(iter==intersection_Devices_Detect[source].map_trafficincident.end()) {
                    eventID++;
                    eventID=eventID%65536;
                    intersection_Devices_Detect[source].map_trafficincident[id]=eventID;
                    //printf("[%s] eventID:%d, eventType:%d, description:{路段ID:%d, 车道ID:%d, 车辆ID:%d, lat:%d, lon:%d, 速度:%f, 车牌:%s}, eventPos:{lat:%d, lon:%d, ele:%d}, eventSource:%d, deviceSource:%d \n",time_String.c_str(),eventID,eventType,road_id,lane_id,id,lat,lon,veh_test.speed*3.6,plate_num.c_str(),lat,lon,ele,object_event_source,source);
                    eventlog_msgs="["+time_String+"] eventID:"+to_string(eventID)+", eventType:"+to_string(eventType)+", description:{路段ID:"+to_string(road_id)+", 车道ID:"+to_string(lane_id)+", 车辆ID:"+to_string(id)+", lat:"+to_string(lat)+", lon:"+to_string(lon)+", 速度:"+to_string(veh_test.speed*3.6)+", 车牌:"+plate_num+"}, eventPos:{lat:"+to_string(lat)+", lon:"+to_string(lon)+", ele:"+to_string(ele)+"}, eventSource:"+to_string(object_event_source)+", deviceSource:"+to_string(source)+" \n";

                }else{
                    //printf("[%s] eventID:%d, eventType:%d, description:{路段ID:%d, 车道ID:%d, 车辆ID:%d, lat:%d, lon:%d, 速度:%f, 车牌:%s}, eventPos:{lat:%d, lon:%d, ele:%d}, eventSource:%d, deviceSource:%d \n",time_String.c_str(),iter->second,eventType,road_id,lane_id,id,lat,lon,veh_test.speed*3.6,plate_num.c_str(),lat,lon,ele,object_event_source,source);
                    eventlog_msgs="["+time_String+"] eventID:"+to_string(iter->second)+", eventType:"+to_string(eventType)+", description:{路段ID:"+to_string(road_id)+", 车道ID:"+to_string(lane_id)+", 车辆ID:"+to_string(id)+", lat:"+to_string(lat)+", lon:"+to_string(lon)+", 速度:"+to_string(veh_test.speed*3.6)+", 车牌:"+plate_num+"}, eventPos:{lat:"+to_string(lat)+", lon:"+to_string(lon)+", ele:"+to_string(ele)+"}, eventSource:"+to_string(object_event_source)+", deviceSource:"+to_string(source)+" \n";

                }
				std::string description = "{" + to_string(road_id) + "," + to_string(lane_id) + "," +  to_string(id) + "," + to_string(lat) + "," + to_string(lon) + "," + to_string(veh_test.speed*3.6) + "," + plate_num + "}";
			}

			if (vehs_test[i][j].illegalparking == true && accidentlabel == false) {
				int eventType = 903;
				auto iter = intersection_Devices_Detect[source].map_trafficincident.find(vehs_test[i][j].id);
				if(iter==intersection_Devices_Detect[source].map_trafficincident.end()){
                    eventID++;
                    eventID=eventID%65536;
                    intersection_Devices_Detect[source].map_trafficincident[id]=eventID;
                    //printf("[%s] eventID:%d, eventType:%d, description:{路段ID:%d, 车道ID:%d, 车辆ID:%d, lat:%f, lon:%f, 速度:%f, 车牌:%s}, eventPos:{lat:%f, lon:%f, ele:%f}, eventSource:%d, deviceSource:%d \n",time_String.c_str(),eventID,eventType,road_id,lane_id,id,lat,lon,veh_test.speed*3.6,plate_num.c_str(),lat,lon,ele,object_event_source,source);
                    //printf("[%s],[停留时间：%f] eventID:%d, eventType:%d, description:{路段ID:%d, 车道ID:%d, 车辆ID:%d, lat:%d, lon:%d, 速度:%f, 车牌:%s}, eventPos:{lat:%d, lon:%d, ele:%d}, eventSource:%d, deviceSource:%d \n",time_String.c_str(),veh_test.timestamp-veh_test.time_Drive_In_NoParking,eventID,eventType,road_id,lane_id,id,lat,lon,veh_test.speed*3.6,plate_num.c_str(),lat,lon,ele,object_event_source,source);
                    eventlog_msgs="["+time_String+"],[停留时间:"+to_string(veh_test.timestamp-veh_test.time_Drive_In_NoParking)+"] eventID:"+to_string(eventID)+", eventType:"+to_string(eventType)+", description:{路段ID:"+to_string(road_id)+", 车道ID:"+to_string(lane_id)+", 车辆ID:"+to_string(id)+", lat:"+to_string(lat)+", lon:"+to_string(lon)+", 速度:"+to_string(veh_test.speed*3.6)+", 车牌:"+plate_num+"}, eventPos:{lat:"+to_string(lat)+", lon:"+to_string(lon)+", ele:"+to_string(ele)+"}, eventSource:"+to_string(object_event_source)+", deviceSource:"+to_string(source)+" \n";

                }else{
                    //printf("[%s] eventID:%d, eventType:%d, description:{路段ID:%d, 车道ID:%d, 车辆ID:%d, lat:%f, lon:%f, 速度:%f, 车牌:%s}, eventPos:{lat:%f, lon:%f, ele:%f}, eventSource:%d, deviceSource:%d \n",time_String.c_str(),iter->second,eventType,road_id,lane_id,id,lat,lon,veh_test.speed*3.6,plate_num.c_str(),lat,lon,ele,object_event_source,source);
                    printf("[%s],[停留时间：%f] eventID:%d, eventType:%d, description:{路段ID:%d, 车道ID:%d, 车辆ID:%d, lat:%d, lon:%d, 速度:%f, 车牌:%s}, eventPos:{lat:%d, lon:%d, ele:%d}, eventSource:%d, deviceSource:%d \n",time_String.c_str(),veh_test.timestamp-veh_test.time_Drive_In_NoParking,iter->second,eventType,road_id,lane_id,id,lat,lon,veh_test.speed*3.6,plate_num.c_str(),lat,lon,ele,object_event_source,source);
                    eventlog_msgs="["+time_String+"],[停留时间:"+to_string(veh_test.timestamp-veh_test.time_Drive_In_NoParking)+"] eventID:"+to_string(iter->second)+", eventType:"+to_string(eventType)+", description:{路段ID:"+to_string(road_id)+", 车道ID:"+to_string(lane_id)+", 车辆ID:"+to_string(id)+", lat:"+to_string(lat)+", lon:"+to_string(lon)+", 速度:"+to_string(veh_test.speed*3.6)+", 车牌:"+plate_num+"}, eventPos:{lat:"+to_string(lat)+", lon:"+to_string(lon)+", ele:"+to_string(ele)+"}, eventSource:"+to_string(object_event_source)+", deviceSource:"+to_string(source)+" \n";

                }
                //｛路段ID,车道ID,车辆ID,纬度lat,经度lon、速度speed,车牌｝
                std::string description = "{" + to_string(road_id) + "," + to_string(lane_id) + "," +  to_string(id) + "," + to_string(lat) + "," + to_string(lon) + "," + to_string(veh_test.speed*3.6) + "," + plate_num + "}";
			}

			if (vehs_test[i][j].retrograde == true) {
				int eventType = 904;
				auto iter = intersection_Devices_Detect[source].map_trafficincident.find(vehs_test[i][j].id);
				std::string eventlog_msgs;
                if(iter==intersection_Devices_Detect[source].map_trafficincident.end()){
                    eventID++;
                    eventID=eventID%65536;
                    intersection_Devices_Detect[source].map_trafficincident[id]=eventID;
                    //printf("[%s] eventID:%d, eventType:%d, description:{路段ID:%d, 车道ID:%d, 车辆ID:%d, lat:%d, lon:%d, 速度:%f, 车牌:%s}, eventPos:{lat:%d, lon:%d, ele:%d}, eventSource:%d, deviceSource:%d \n",time_String.c_str(),eventID,eventType,road_id,lane_id,id,lat,lon,veh_test.speed*3.6,plate_num.c_str(),lat,lon,ele,object_event_source,source);
                    eventlog_msgs="["+time_String+"] eventID:"+to_string(eventID)+", eventType:"+to_string(eventType)+", description:{路段ID:"+to_string(road_id)+", 车道ID:"+to_string(lane_id)+", 车辆ID:"+to_string(id)+", lat:"+to_string(lat)+", lon:"+to_string(lon)+", 速度:"+to_string(veh_test.speed*3.6)+", 车牌:"+plate_num+"}, eventPos:{lat:"+to_string(lat)+", lon:"+to_string(lon)+", ele:"+to_string(ele)+"}, eventSource:"+to_string(object_event_source)+", deviceSource:"+to_string(source)+" \n";
                }else{
                    //printf("[%s] eventID:%d, eventType:%d, description:{路段ID:%d, 车道ID:%d, 车辆ID:%d, lat:%d, lon:%d, 速度:%f, 车牌:%s}, eventPos:{lat:%d, lon:%d, ele:%d}, eventSource:%d, deviceSource:%d \n",time_String.c_str(),iter->second,eventType,road_id,lane_id,id,lat,lon,veh_test.speed*3.6,plate_num.c_str(),lat,lon,ele,object_event_source,source);
                    eventlog_msgs="["+time_String+"] eventID:"+to_string(iter->second)+", eventType:"+to_string(eventType)+", description:{路段ID:"+to_string(road_id)+", 车道ID:"+to_string(lane_id)+", 车辆ID:"+to_string(id)+", lat:"+to_string(lat)+", lon:"+to_string(lon)+", 速度:"+to_string(veh_test.speed*3.6)+", 车牌:"+plate_num+"}, eventPos:{lat:"+to_string(lat)+", lon:"+to_string(lon)+", ele:"+to_string(ele)+"}, eventSource:"+to_string(object_event_source)+", deviceSource:"+to_string(source)+" \n";
                    
                }
                //｛路段ID,车道ID,车辆ID,纬度lat,经度lon、速度speed,车牌｝

                std::string description = "{" + to_string(road_id) + "," + to_string(lane_id) + "," +  to_string(id) + "," + to_string(lat) + "," + to_string(lon) + "," + to_string(veh_test.speed*3.6) + "," + plate_num + "}";
			}

			if (vehs_test[i][j].illegallanechange == true) {
				int eventType = 1026;
				auto iter = intersection_Devices_Detect[source].map_trafficincident.find(vehs_test[i][j].id);
				std::string eventlog_msgs;
                if(iter==intersection_Devices_Detect[source].map_trafficincident.end()){
                    eventID++;
                    eventID=eventID%65536;
                    intersection_Devices_Detect[source].map_trafficincident[id]=eventID;
                    //printf("[%s] eventID:%d, eventType:%d, description:{路段ID:%d, 车道ID:%d, 车辆ID:%d, lat:%d, lon:%d, 速度:%f, 车牌:%s}, eventPos:{lat:%d, lon:%d, ele:%d}, eventSource:%d, deviceSource:%d \n",time_String.c_str(),eventID,eventType,road_id,lane_id,id,lat,lon,veh_test.speed*3.6,plate_num.c_str(),lat,lon,ele,object_event_source,source);
                    eventlog_msgs="["+time_String+"] eventID:"+to_string(eventID)+", eventType:"+to_string(eventType)+", description:{路段ID:"+to_string(road_id)+", 车道ID:"+to_string(lane_id)+", 车辆ID:"+to_string(id)+", lat:"+to_string(lat)+", lon:"+to_string(lon)+", 速度:"+to_string(veh_test.speed*3.6)+", 车牌:"+plate_num+"}, eventPos:{lat:"+to_string(lat)+", lon:"+to_string(lon)+", ele:"+to_string(ele)+"}, eventSource:"+to_string(object_event_source)+", deviceSource:"+to_string(source)+" \n";
                    
                }else{
                    //printf("[%s] eventID:%d, eventType:%d, description:{路段ID:%d, 车道ID:%d, 车辆ID:%d, lat:%d, lon:%d, 速度:%f, 车牌:%s}, eventPos:{lat:%d, lon:%d, ele:%d}, eventSource:%d, deviceSource:%d \n",time_String.c_str(),iter->second,eventType,road_id,lane_id,id,lat,lon,veh_test.speed*3.6,plate_num.c_str(),lat,lon,ele,object_event_source,source);
                    eventlog_msgs="["+time_String+"] eventID:"+to_string(iter->second)+", eventType:"+to_string(eventType)+", description:{路段ID:"+to_string(road_id)+", 车道ID:"+to_string(lane_id)+", 车辆ID:"+to_string(id)+", lat:"+to_string(lat)+", lon:"+to_string(lon)+", 速度:"+to_string(veh_test.speed*3.6)+", 车牌:"+plate_num+"}, eventPos:{lat:"+to_string(lat)+", lon:"+to_string(lon)+", ele:"+to_string(ele)+"}, eventSource:"+to_string(object_event_source)+", deviceSource:"+to_string(source)+" \n";
                    
                }
                //｛路段ID,车道ID,车辆ID,纬度lat,经度lon、速度speed,车牌｝
                std::string description = "{" + to_string(road_id) + "," + to_string(lane_id) + "," +  to_string(id) + "," + to_string(lat) + "," + to_string(lon) + "," + to_string(veh_test.speed*3.6) + "," + plate_num + "}";
			}
			if (accidentlabel == true) {
				int eventType = 100;
				auto iter = intersection_Devices_Detect[source].map_trafficincident.find(vehs_test[i][j].id);
				std::string eventlog_msgs;
                if(iter==intersection_Devices_Detect[source].map_trafficincident.end()){
                    eventID++;
                    eventID=eventID%65536;
                    intersection_Devices_Detect[source].map_trafficincident[id]=eventID;
                    //printf("[%s] eventID:%d, eventType:%d, description:{路段ID:%d, 车道ID:%d, 车辆ID:%d, lat:%f, lon:%f, 速度:%f, 车牌:%s}, eventPos:{lat:%f, lon:%f, ele:%f}, eventSource:%d, deviceSource:%d \n",time_String.c_str(),eventID,eventType,road_id,lane_id,id,lat,lon,veh_test.speed*3.6,plate_num.c_str(),lat,lon,ele,object_event_source,source);
                    //printf("[%s],[停留时间：%f] eventID:%d, eventType:%d, description:{路段ID:%d, 车道ID:%d, 车辆ID:%d, lat:%d, lon:%d, 速度:%f, 车牌:%s}, eventPos:{lat:%d, lon:%d, ele:%d}, eventSource:%d, deviceSource:%d \n",time_String.c_str(),veh_test.timestamp-veh_test.time_Drive_In_Intersection,eventID,eventType,road_id,lane_id,id,lat,lon,veh_test.speed*3.6,plate_num.c_str(),lat,lon,ele,object_event_source,source);
                    eventlog_msgs="["+time_String+"],[停留时间:"+to_string(veh_test.timestamp-veh_test.time_Drive_In_Intersection)+"] eventID:"+to_string(eventID)+", eventType:"+to_string(eventType)+", description:{路段ID:"+to_string(road_id)+", 车道ID:"+to_string(lane_id)+", 车辆ID:"+to_string(id)+", lat:"+to_string(lat)+", lon:"+to_string(lon)+", 速度:"+to_string(veh_test.speed*3.6)+", 车牌:"+plate_num+"}, eventPos:{lat:"+to_string(lat)+", lon:"+to_string(lon)+", ele:"+to_string(ele)+"}, eventSource:"+to_string(object_event_source)+", deviceSource:"+to_string(source)+" \n";
                }else{
                    //printf("[%s] eventID:%d, eventType:%d, description:{路段ID:%d, 车道ID:%d, 车辆ID:%d, lat:%f, lon:%f, 速度:%f, 车牌:%s}, eventPos:{lat:%f, lon:%f, ele:%f}, eventSource:%d, deviceSource:%d \n",time_String.c_str(),iter->second,eventType,road_id,lane_id,id,lat,lon,veh_test.speed*3.6,plate_num.c_str(),lat,lon,ele,object_event_source,source);
                    //printf("[%s],[停留时间：%f] eventID:%d, eventType:%d, description:{路段ID:%d, 车道ID:%d, 车辆ID:%d, lat:%d, lon:%d, 速度:%f, 车牌:%s}, eventPos:{lat:%d, lon:%d, ele:%d}, eventSource:%d, deviceSource:%d \n",time_String.c_str(),veh_test.timestamp-veh_test.time_Drive_In_Intersection,iter->second,eventType,road_id,lane_id,id,lat,lon,veh_test.speed*3.6,plate_num.c_str(),lat,lon,ele,object_event_source,source);
                    eventlog_msgs="["+time_String+"],[停留时间:"+to_string(veh_test.timestamp-veh_test.time_Drive_In_Intersection)+"] eventID:"+to_string(iter->second)+", eventType:"+to_string(eventType)+", description:{路段ID:"+to_string(road_id)+", 车道ID:"+to_string(lane_id)+", 车辆ID:"+to_string(id)+", lat:"+to_string(lat)+", lon:"+to_string(lon)+", 速度:"+to_string(veh_test.speed*3.6)+", 车牌:"+plate_num+"}, eventPos:{lat:"+to_string(lat)+", lon:"+to_string(lon)+", ele:"+to_string(ele)+"}, eventSource:"+to_string(object_event_source)+", deviceSource:"+to_string(source)+" \n";
                    
                }
                //｛路段ID,车道ID,车辆ID,纬度lat,经度lon、速度speed,车牌｝
                std::string description = "{" + to_string(road_id) + "," + to_string(lane_id) + "," +  to_string(id) + "," + to_string(lat) + "," + to_string(lon) + "," + to_string(veh_test.speed*3.6) + "," + plate_num + "}";
			}
			*/
			//printf("sec: %f, nsec: %f, ID: %d,label: %d,lane_No: %d, pos_x: %f, pos_y: %f, Vx: %f,Vy: %f,existtime: %f,veh_In_Intersection: %d \n,",sec,nsec,id,vehs_test[i][j].label,vehs_test[i][j].lane_Num,object_px,object_py,object_vx,object_vy,(vehs_test[i][j].timestamp - vehs_test[i][j].time_Drive_In_NoParking),vehs_test[i][j].veh_In_Intersection);
			//printf("Overspeed: %d, Lowspeed: %d, Retrograde: %d, Illegalparking: %d, Illegallanechange: %d, Accident %d\n",vehs_test[i][j].overspeed,vehs_test[i][j].lowspeed,vehs_test[i][j].retrograde,vehs_test[i][j].illegalparking,vehs_test[i][j].illegallanechange,accidentlabel);

			if (vehs_test[i][j].lane_Num <= (10 + intersection_Devices_Detect[source].lanes_Num)) {
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
		/*
		for (auto item = it->second.max_queue_test.lane_Code.begin(); item != it->second.max_queue_test.lane_Code.end(); item++){
			//printf("lanes_No: %d, lanes_Volume: %d\n",*item,it->second.max_queue_test.lanes_Speed_Volume_Sum[*item]);
			it->second.lane_congestionlabel[*item] = LaneCongestion(it->second.max_queue_test, *item, MaxLimitedVehicleNum_Lane, CongestionSpeed_Slight, CongestionSpeed_Moderate, CongestionSpeed_Severe);             //路段是否发生拥堵
		}
		*/
		//it->second.gatewaywarninglable = Gateway(it->second.vehs_test, it->second.lanes_Num);               //是否存在汇入预警
		//it->second.congestionlabel = Congestion(it->second.vehs_test, it->second.max_queue_test);          //是否发生拥堵
	}
	/*
	if ( fmod(floor(time_stamp_nsec * 10),time_Interval) == 0.0) {
            for (auto it = intersection_Devices_Detect.begin();it != intersection_Devices_Detect.end(); it++)
            {

                if( pz>=0 && it->first!=pz )
                {
                    continue;
                }
                if(it->second.gatewaywarninglable==true){
                    eventID++;
                    eventID=eventID%65536;
                    int eventType=1004;
                    //printf("[%s] eventID:%d, eventType:%d, eventSource:%d, deviceSource:%d \n",time_String.c_str(),eventID,eventType,object_event_source,it->first);
                    // printf("[%f] eventID:%d, eventType:%d, eventSource:%d, deviceSource:%d \n",time_stamp_nsec,eventID,eventType,object_event_source,it->first);
                    std::string eventlog_msgs="["+time_String+"] eventID:"+to_string(eventID)+", eventType:"+to_string(eventType)+", eventSource:"+to_string(object_event_source)+", deviceSource:"+to_string(it->first)+" \n";
                }else{
                    // printf("not gatewaywarning \n");
                }
              }

            for (auto it = intersection_Devices_Detect.begin();it != intersection_Devices_Detect.end(); it++)
			{

				int direction=it->first+1;
				int road_id = 0;
				if(direction>2)
				{
					road_id = direction - 2;
				}
				else
				{
					road_id = direction + 2;
				}
				if( pz>=0 && it->first!=pz )
				{
					continue;
				}

				if(it->second.link_congestionlabel==1 || it->second.link_congestionlabel==2 || it->second.link_congestionlabel==3)
				{
					eventID++;
					eventID=eventID%65536;
					int eventType=707;

					std::string description = "{{"+to_string(road_id)+","+to_string(it->second.link_congestionlabel)+"},{";   //实际用
					// std::string description = "{{"+to_string(road_id)+","+to_string(it->second.link_congestionlabel)+","+to_string(it->second.max_queue_test.ave_Time_Speed*3.6)+"},{";  //测试用
					for(auto item = it->second.max_queue_test.lane_Code.begin(); item !=it->second.max_queue_test.lane_Code.end(); item++)
					{
						if(item!=it->second.max_queue_test.lane_Code.end()-1)
						{
							description=description+"{"+to_string(*item)+","+to_string(it->second.lane_congestionlabel[*item])+"},";  //实际用
							// description=description+"{"+to_string(*item)+","+to_string(it->second.lane_congestionlabel[*item])+","+to_string(it->second.max_queue_test.lanes_Time_Speed[*item]*3.6)+"},";  //测试用
						}
						else
						{
							description=description+"{"+to_string(*item)+","+to_string(it->second.lane_congestionlabel[*item])+"}";   //实际用
							// description=description+"{"+to_string(*item)+","+to_string(it->second.lane_congestionlabel[*item])+","+to_string(it->second.max_queue_test.lanes_Time_Speed[*item]*3.6)+"}";  //测试用
						}
					}
					description=description+"}}";
					//std::string description = "{" + to_string(it->second.link_congestionlabel) + "," + to_string(it->second.max_queue_test.ave_Time_Speed*3.6) + "}";
					//printf("[%s] eventID:%d, eventType:%d, description:{拥堵状态:%d,平均车速:%f}, eventSource:%d, deviceSource:%d \n",time_String.c_str(),eventID,eventType,it->second.congestionlabel,it->second.max_queue_test.ave_Time_Speed*3.6,object_event_source,it->first);
					//printf("[%s] eventID:%d, eventType:%d, description:%s, eventSource:%d, deviceSource:%d \n",time_String.c_str(),eventID,eventType,description.c_str(),object_event_source,it->first);

					std::string eventlog_msgs="["+time_String+"] eventID:"+to_string(eventID)+", eventType:"+to_string(eventType)+", description:"+description+", eventSource:"+to_string(object_event_source)+", deviceSource:"+to_string(it->first)+" \n";
				}
				else{
					// printf("not congestion,拥堵状态:%d,平均车速:%f \n",it->second.congestionlabel,it->second.max_queue_test.ave_Time_Speed*3.6);
				}
			}
     }
	 for (auto it = intersection_Devices_Detect.begin();it != intersection_Devices_Detect.end(); it++){
		for(auto item=0;item<intersection_Devices_Detect.size();item++){
				intersection_Devices_Detect[item].vehs_test.clear();
		}
	}
	*/

	if (fmod(time_stamp, time_Interval) == 0.0 && !flag) {
		eventID = (eventID + 1) % 65536;
		int eventType = 1021;
		int effective_timeheadway_num = 0;
		string index_description = "{";
		for (auto it = intersection_Devices_Detect.begin(); it != intersection_Devices_Detect.end(); it++) {
			int direction = it->first + 1;
			int road_id = 0;
			if (direction > 2)
				road_id = direction - 2;
			else
				road_id = direction + 2;
			if (pz >= 0 && it->first != pz)
				continue;
			
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

			   /*
			    sprintf(eventlog_msgs," eventID:%d, eventType:%d, description:{车道ID:%d, 平均空间速度:%f}, eventSource:%d, deviceSource:%d \n",eventID, eventType, it->second.speed_test.lane_Code[i],3.6 * it->second.speed_test.lanes_Space_Speed[it->second.volume_test.lane_Code[i]], object_event_source,it->first);
                sprintf(eventlog_msgs," eventID:%d, eventType:%d, description:{车道ID:%d, 车道流量:%f}, eventSource:%d, deviceSource:%d \n",eventID,eventType,it->second.volume_test.lane_Code[i], it->second.volume_test.lanes_Volume[it->second.volume_test.lane_Code[i]],object_event_source,it->first); 
                sprintf(eventlog_msgs," eventID:%d, eventType:%d, description:{车道ID:%d, 平均车头间距:%f}, eventSource:%d, deviceSource:%d \n",eventID, eventType, it->second.max_queue_test.lane_Code[i],it->second.max_queue_test.lanes_Space_Headway[it->second.volume_test.lane_Code[i]],object_event_source,it->first); 
                sprintf(eventlog_msgs," eventID:%d, eventType:%d, description:{车道ID:%d, 平均车头时距:%f}, eventSource:%d, deviceSource:%d \n",eventID, eventType, it->second.capacity_test.lane_Code[i],it->second.capacity_test.ave_Time_Headway[it->second.capacity_test.lane_Code[i]],object_event_source,it->first); 
                sprintf(eventlog_msgs," eventID:%d, eventType:%d, description:{车道ID:%d, 车道排队长度:%f}, eventSource:%d, deviceSource:%d \n",eventID, eventType, it->second.max_queue_test.lane_Code[i],max(0.0,it->second.max_queue_test.lanes_Queue_Length[it->second.max_queue_test.lane_Code[i]]), object_event_source,it->first); 
			   */

			}
			lane_description += "}";
			index_description += "{" + to_string(it->first + 1) + "," + to_string(3.6 * it->second.speed_test.ave_Space_Speed) + "," + to_string(3600 / time_Interval * sum_Volume) + "," + to_string(0.0) + "," + to_string(0.0) + "," + to_string(it->second.max_queue_test.ave_Space_Headway) + "," \
				+ to_string((effective_timeheadway_num > 0) ? sum_timeheadway / effective_timeheadway_num : 0.0) + "," + to_string(max_queue_length) + "}," + lane_description;
			if (it->first < intersection_Devices_Detect.size() - 1)
				index_description += ",";

			/*
			sprintf(eventlog_msgs," eventID:%d, eventType:%d, description:{平均空间速度:%f}, eventSource:%d, deviceSource:%d \n",eventID, eventType, 3.6 * it->second.speed_test.ave_Space_Speed, object_event_source, it->first); 
            sprintf(eventlog_msgs," eventID:%d, eventType:%d, description:{流量:%f}, eventSource:%d, deviceSource:%d \n",eventID, eventType, 3600 / time_Interval * sum_Volume, object_event_source, it->first);
            sprintf(eventlog_msgs," eventID:%d, eventType:%d, description:{平均车头间距:%f}, eventSource:%d, deviceSource:%d \n",eventID, eventType, it->second.max_queue_test.ave_Space_Headway, object_event_source, it->first); 
            sprintf(eventlog_msgs," eventID:%d, eventType:%d, description:{平均车头时距:%f}, eventSource:%d, deviceSource:%d \n",eventID, eventType, (effective_timeheadway_num > 0) ? sum_timeheadway/effective_timeheadway_num:0.0, object_event_source, it->first);
            sprintf(eventlog_msgs," eventID:%d, eventType:%d, description:{排队长度:%f}, eventSource:%d, deviceSource:%d \n",eventID, eventType, it->second.max_queue_test.section_Queue_Length, object_event_source, it->first); 
			*/

			it->second.volume_test.update_Vehicles_Info();
			it->second.speed_test.update_Vehicles_Info();
			it->second.capacity_test.update_Vehicles_Info();
		}
		index_description += "}";

		/*
		std::string eventlog_msgs = "[" + time_String + "] eventID:" + to_string(eventID) + ", eventType:" + to_string(eventType) + ",description:" + index_description + ", eventSource:" + to_string(object_event_source) + ", deviceSource:0 \n";
		*/
		
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

