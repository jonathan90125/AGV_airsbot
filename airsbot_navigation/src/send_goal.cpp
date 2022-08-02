#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h> 
#include <math.h>
#include "std_msgs/Int8.h" 
 
using namespace std;
 
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
void callback(const geometry_msgs::Pose2D& pose);
void send_goal_pose(geometry_msgs::Pose2D init_pose,geometry_msgs::Pose2D goal_pose); 
void Dijkstra_cpp(vector<vector<double>>&vec, vector<double>& result, int v0,int v1);
void select_goal_station(geometry_msgs::Pose2D init_pose,int sta_num);
void p_callback(const geometry_msgs::Pose2D& pose);
void s_callback(const std_msgs::Int8& msg);


geometry_msgs::Pose2D pos_now;
geometry_msgs::Pose2D pos_goal;
//用来存储3个分解过后的pose
move_base_msgs::MoveBaseGoal tmp1_goal;
move_base_msgs::MoveBaseGoal tmp2_goal;
move_base_msgs::MoveBaseGoal tmp3_goal;
//用来存储站点导航途径的所有pose
geometry_msgs::Pose2D pos_path[10];
//站点导航的距离矩阵和结果存储
vector<vector<double>> vec(5, vector<double>(5, 0));
 
//模式标志位
int mode_flag=0;
int goal_station=-1;
 
double station[5][3];
double matrix[5][5] ;
//存储站点号和路径长度
int path[10];
int path_length=0;

 std_msgs::Int8 pos_state;
 std_msgs::Int8 sta_state;



int main(int argc, char** argv){
	ros::init(argc, argv, "simple_navigation_goals");
	ros::NodeHandle nh("~");
	ros::Subscriber odom_sub = nh.subscribe("tf_data", 1000, callback);
	ros::Publisher sta_pub = nh.advertise<std_msgs::Int8>("/sta_state", 10);            //0: free/finished 1:busy  2:failed
	ros::Publisher pose_pub = nh.advertise<std_msgs::Int8>("/pose_state", 10);    //0: free/finished 1:busy  2:failed
	ros::Subscriber sta_sub = nh.subscribe("/sta_num", 1000,s_callback);
	ros::Subscriber pose_sub = nh.subscribe("pose_data", 1000,p_callback);
	 ros::Rate loop_rate(2);	//设置发送数据的频率为50Hz

	 pos_state.data=0;
     sta_state.data=0;
	 //read  station pose from yaml
	  XmlRpc::XmlRpcValue sta_1;
      nh.getParam("station_1", sta_1);
    for (size_t i = 0; i < sta_1.size(); i++) 
    {
        XmlRpc::XmlRpcValue tmp_value = sta_1[i];
        if(tmp_value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
            station[0][i]=double(tmp_value);
            printf("sta_1: %f\r\n",  station[0][i]);
    }
	XmlRpc::XmlRpcValue sta_2;
      nh.getParam("station_2", sta_2);
    for (size_t i = 0; i < sta_2.size(); i++) 
    {
        XmlRpc::XmlRpcValue tmp_value = sta_2[i];
        if(tmp_value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
            station[1][i]=double(tmp_value);
            printf("sta_2: %f\r\n",  station[1][i]);
    }
	XmlRpc::XmlRpcValue sta_3;
      nh.getParam("station_3", sta_3);
    for (size_t i = 0; i < sta_3.size(); i++) 
    {
        XmlRpc::XmlRpcValue tmp_value = sta_3[i];
        if(tmp_value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
            station[2][i]=double(tmp_value);
            printf("sta_3: %f\r\n",  station[2][i]);
    }
	XmlRpc::XmlRpcValue sta_4;
      nh.getParam("station_4", sta_4);
    for (size_t i = 0; i < sta_4.size(); i++) 
    {
        XmlRpc::XmlRpcValue tmp_value = sta_4[i];
        if(tmp_value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
            station[3][i]=double(tmp_value);
            printf("sta_4: %f\r\n",  station[3][i]);
    }
	XmlRpc::XmlRpcValue sta_5;
      nh.getParam("station_5", sta_5);
    for (size_t i = 0; i < sta_5.size(); i++) 
    {
        XmlRpc::XmlRpcValue tmp_value = sta_5[i];
        if(tmp_value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
            station[4][i]=double(tmp_value);
            printf("sta_5: %f\r\n",  station[4][i]);
    }
   //read distnace between stations from yaml
	XmlRpc::XmlRpcValue dis_1;
      nh.getParam("dis_1",dis_1);
    for (size_t i = 0; i < dis_1.size(); i++) 
    {
        XmlRpc::XmlRpcValue tmp_value = dis_1[i];
        if(tmp_value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
           matrix[0][i]=double(tmp_value);
            printf("dis_1: %f\r\n",  matrix[0][i]);
    }
	 XmlRpc::XmlRpcValue dis_2;
      nh.getParam("dis_2",dis_2);
    for (size_t i = 0; i < dis_2.size(); i++) 
    {
        XmlRpc::XmlRpcValue tmp_value = dis_2[i];
        if(tmp_value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
           matrix[1][i]=double(tmp_value);
            printf("dis_2: %f\r\n",  matrix[1][i]);
    }
	XmlRpc::XmlRpcValue dis_3;
      nh.getParam("dis_3",dis_3);
    for (size_t i = 0; i < dis_1.size(); i++) 
    {
        XmlRpc::XmlRpcValue tmp_value = dis_3[i];
        if(tmp_value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
           matrix[2][i]=double(tmp_value);
            printf("dis_3: %f\r\n",  matrix[2][i]);
    }
	XmlRpc::XmlRpcValue dis_4;
      nh.getParam("dis_4",dis_4);
    for (size_t i = 0; i < dis_4.size(); i++) 
    {
        XmlRpc::XmlRpcValue tmp_value = dis_4[i];
        if(tmp_value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
           matrix[3][i]=double(tmp_value);
            printf("dis_4: %f\r\n",  matrix[3][i]);
    }
	XmlRpc::XmlRpcValue dis_5;
      nh.getParam("dis_5",dis_5);
    for (size_t i = 0; i < dis_5.size(); i++) 
    {
        XmlRpc::XmlRpcValue tmp_value = dis_5[i];
        if(tmp_value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
           matrix[4][i]=double(tmp_value);
            printf("dis_5: %f\r\n",  matrix[4][i]);
    }

	//配置站点导航的距离矩阵
	for (int i = 0; i < 5; ++i){
		for (int j = 0; j < 5; ++j){
			vec[i][j]=matrix[i][j];
		 
			printf("%f, ",matrix[i][j]);
		}
		printf("\n\r");
		
	} 
	for (int i = 0; i < 5; ++i){
		for (int j = 0; j < 5; ++j){
			vec[i][j]=matrix[i][j];
			printf("%f, ",vec[i][j]);
			 
		}
		printf("\n\r");
	}


	MoveBaseClient ac("move_base", true); 
	//tell the action client that we want to spin a thread by default
	int goals_index=0;

	//wait for the action server to come up
	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
	}


 

//start main loop
  while(ros::ok()){ 

	ROS_INFO("main loop ");
	tf::StampedTransform transform;
	tf::TransformListener listener;
	try
	{
	listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(3.0));
	listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
	}
	catch (tf::TransformException &ex)
	{
	ROS_ERROR("%s", ex.what());
	}
	//输出位置信息
	pos_now.x = static_cast<double>(transform.getOrigin().x());
	pos_now.y = static_cast<double>(transform.getOrigin().y());
	pos_now.theta = tf::getYaw(transform.getRotation());
	

	//坐标点导航
	if(mode_flag==1){
		pos_state.data=1;
		pose_pub.publish(pos_state);
		send_goal_pose(pos_now,pos_goal);
		
		//分别发送3次分解过后的子目标
		ROS_INFO("Sending goal 1 !!!!!   ");
		ROS_INFO("x,y,qz,qw: %f,%f,%f,%f \r\n",tmp1_goal.target_pose.pose.position.x,tmp1_goal.target_pose.pose.position.y,
		tmp1_goal.target_pose.pose.orientation.z,tmp1_goal.target_pose.pose.orientation.w);
		tmp1_goal.target_pose.header.frame_id = "map";
		tmp1_goal.target_pose.header.stamp = ros::Time::now();
		ac.sendGoal(tmp1_goal);
		ac.waitForResult();
		while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
		}

		ROS_INFO("Sending goal 2 !!!!!   ");
		ROS_INFO("x,y,qz,qw: %f,%f,%f,%f \r\n",tmp2_goal.target_pose.pose.position.x,tmp2_goal.target_pose.pose.position.y,
		tmp2_goal.target_pose.pose.orientation.z,tmp2_goal.target_pose.pose.orientation.w);
		tmp2_goal.target_pose.header.frame_id = "map";
		tmp2_goal.target_pose.header.stamp = ros::Time::now();
		ac.sendGoal(tmp2_goal);
		ac.waitForResult();
		while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
		}

		ROS_INFO("Sending goal 3 !!!!!   ");
		ROS_INFO("x,y,qz,qw: %f,%f,%f,%f \r\n",tmp3_goal.target_pose.pose.position.x,tmp3_goal.target_pose.pose.position.y,
		tmp3_goal.target_pose.pose.orientation.z,tmp3_goal.target_pose.pose.orientation.w);
		tmp3_goal.target_pose.header.frame_id = "map";
		tmp3_goal.target_pose.header.stamp = ros::Time::now();
		ac.sendGoal(tmp3_goal);
		ac.waitForResult();
		if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
			pos_state.data=0;
			pose_pub.publish(pos_state);
		}
		else{
			pos_state.data=2;
			pose_pub.publish(pos_state);
		}
		mode_flag=0;
	}



	//站点导航
        if(mode_flag==2){
			sta_state.data=1;
			sta_pub.publish(sta_state);
			ROS_INFO("mode_flag =2 !!!!!!!!!! ");
		select_goal_station(pos_now,goal_station);
		pos_path[0]=pos_now;
		 
		 
		for(int i=0;i<path_length;i++){
			ROS_INFO("path_sequence: %d\r\n",path[i]);
			pos_path[i+1].x=station[path[i]][0];
			pos_path[i+1].y=station[path[i]][1];
			pos_path[i+1].theta=station[path[i]][2];
		}
	 
		for(int i=0;i<path_length;i++){
			double offset_x=(pos_path[i].x-pos_path[i+1].x)*(pos_path[i].x-pos_path[i+1].x);
			double offset_y=(pos_path[i].y-pos_path[i+1].y)*(pos_path[i].y-pos_path[i+1].y);
			if (offset_x<0.04&&offset_y<0.04){
				continue;
			}
			else{
				send_goal_pose(pos_path[i],pos_path[i+1]);
					
				//分别发送3次分解过后的子目标 
				ROS_INFO("Sending goal 1 !!!!!   ");
				ROS_INFO("x,y,qz,qw: %f,%f,%f,%f \r\n",tmp1_goal.target_pose.pose.position.x,tmp1_goal.target_pose.pose.position.y,
				tmp1_goal.target_pose.pose.orientation.z,tmp1_goal.target_pose.pose.orientation.w);
				tmp1_goal.target_pose.header.frame_id = "map";
				tmp1_goal.target_pose.header.stamp = ros::Time::now();
				ac.sendGoal(tmp1_goal);
				ac.waitForResult();
				while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
				}

				ROS_INFO("Sending goal 2 !!!!!   ");
				ROS_INFO("x,y,qz,qw: %f,%f,%f,%f \r\n",tmp2_goal.target_pose.pose.position.x,tmp2_goal.target_pose.pose.position.y,
				tmp2_goal.target_pose.pose.orientation.z,tmp2_goal.target_pose.pose.orientation.w);
				tmp2_goal.target_pose.header.frame_id = "map";
				tmp2_goal.target_pose.header.stamp = ros::Time::now();
				ac.sendGoal(tmp2_goal);
				ac.waitForResult();
				while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
				}
			}
		 
		}
	 	ROS_INFO("Sending goal 3 !!!!!   ");
		ROS_INFO("x,y,qz,qw: %f,%f,%f,%f \r\n",tmp3_goal.target_pose.pose.position.x,tmp3_goal.target_pose.pose.position.y,
		tmp3_goal.target_pose.pose.orientation.z,tmp3_goal.target_pose.pose.orientation.w);
		tmp3_goal.target_pose.header.frame_id = "map";
		tmp3_goal.target_pose.header.stamp = ros::Time::now();
		ac.sendGoal(tmp3_goal);
		ac.waitForResult();
		if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
			sta_state.data=0;
			sta_pub.publish(sta_state);
		}
		else{
			sta_state.data=2;
			sta_pub.publish(sta_state);
		}

		mode_flag=0;
		goal_station=-1;
		path_length=0;
	}
	ros::spinOnce();
	loop_rate.sleep();
      }
 
  return 0;
}
 


//将一个导航目标分解为3个，解决 S_curve 的问题
void send_goal_pose(geometry_msgs::Pose2D init_pose,geometry_msgs::Pose2D goal_pose){
	double dx,dy;
	double absx,absy;
	double angle=0;
	dx=goal_pose.x-init_pose.x;
	dy=goal_pose.y-init_pose.y;
	absx=abs(dx);
	absy=abs(dy);
	if(absx==0){
		if(absy>0){
			angle=-1.57;
		}
		if(absy<0){
			angle=1.57;
		}
		if(absy==0){
			angle=0;
		}
	}
	else{
		if(dx>=0&&dy>=0){
			angle=atan(absy/absx);
		}
		if(dx>=0&&dy<=0){
			angle=-atan(absy/absx);
		}
		if(dx<=0&&dy>=0){
			angle=3.1416-atan(absy/absx);
		}
		if(dx<=0&&dy<=0){
			angle=atan(absy/absx)-3.1416;
		}
	}

	ROS_INFO("angle: %f\r\n",angle);
	
	tmp1_goal.target_pose.pose.position.x = init_pose.x;
	tmp1_goal.target_pose.pose.position.y = init_pose.y;
    tmp1_goal.target_pose.pose.orientation.z =sin(angle/2);
	tmp1_goal.target_pose.pose.orientation.w =cos(angle/2);

       
	tmp2_goal.target_pose.pose.position.x = goal_pose.x;
	tmp2_goal.target_pose.pose.position.y = goal_pose.y;
    tmp2_goal.target_pose.pose.orientation.z =sin(angle/2);
	tmp2_goal.target_pose.pose.orientation.w =cos(angle/2);

      
	tmp3_goal.target_pose.pose.position.x = goal_pose.x;
	tmp3_goal.target_pose.pose.position.y = goal_pose.y;
    tmp3_goal.target_pose.pose.orientation.z =sin(goal_pose.theta/2);
	tmp3_goal.target_pose.pose.orientation.w =cos(goal_pose.theta/2);
}

//找到距离当前位置最近的站点
void select_goal_station(geometry_msgs::Pose2D init_pose,int sta_num)
{
  double min_dis=99;
	int min_index;
	for(int i=0;i<5;i++){
	      if(min_dis>((init_pose.y-station[i][1])*(init_pose.y-station[i][1])+(init_pose.x-station[i][0])*(init_pose.x-station[i][0])))
		  {
			min_dis=(init_pose.y-station[i][1])*(init_pose.y-station[i][1])+(init_pose.x-station[i][0])*(init_pose.x-station[i][0]);
			min_index=i;
	     }}
	path[0]=min_index;//线路上的第一个点
	path_length=1;
	 vector<double> result(5, 9999); 
	//到达第一个站点之后寻找之后的路径
	if(sta_num!=min_index+1){
		Dijkstra_cpp(vec,result, min_index,sta_num-1);
	}
}

//寻找从站点A到站点B的最短距离上途径的所有站点编号
void Dijkstra_cpp(vector<vector<double>>&vec, vector<double>& result, int v0,int v1){
	ROS_INFO("using  Dijkstra planning path now!!!!!!!!!!!!");
	ROS_INFO("vec.size():    %d",vec.size());
	vector<int> visited(vec.size(), 0); // 表示顶点是否被选中，0：顶点未被选中；1：顶点已被选中
	double last_visited = 0;
	visited[v0] = 1; // 选中起始顶点
	// result[0] = 0;
	int v0_set[10];
	int v0_length=0;
	for (int i = 0; i < vec.size(); i++) { // N 个顶点需要做 N - 1 次循环
			ROS_INFO("v0:    %d", v0);
		// 查看顶点周围的所有点
		for (int j = 0; j < vec.size(); j++) { // 循环遍历所有顶点
			if (visited[j] == 0){ // 保证被查看的新顶点没有被访问到
				if (vec[v0][j] != 0){ // 保证当前顶点（V0）与新顶点（j）之间有路径
					ROS_INFO("v0, j,vec[v0][j]:    %d,%d,%f",v0, j,vec[v0][j]);
					double dist = vec[v0][j] + last_visited; // 计算 V0 到 J 的路径距离
					ROS_INFO(" last_visited:    %f",  last_visited);
					ROS_INFO("vec[v0][j]:    %f", vec[v0][j] );
					ROS_INFO("result[j]:    %f", result[j]);
					if (dist < result[j]){
						result[j] = dist; // 用新路径代替原来的路径
						ROS_INFO(" dist:    %f",  dist);
						ROS_INFO(" last_visited:    %f",  last_visited);
					    if(j==v1){
							 int k=0;
							 while(k<v0_length){
								path[k+1]=v0_set[k];
								k++;	
							}
							path[v0_length+1]=j;
							path_length=v0_length+2;
							ROS_INFO("path[v0_length+1], path_length :          [%d,%d]",   path[v0_length+1],path_length);
						}
				}}
			}
		}
		// 找出最小值
		int minIndex = 0;    
		while (visited[minIndex] == 1) minIndex++; // 找第一个没有被选中的节点
		for (int j = minIndex; j < vec.size(); ++j) {
			if (visited[j] == 0 && result[j] < result[minIndex]){
				minIndex = j;
			}
		}
 		for(int m=0;m<result.size();m++){
			 ROS_INFO(" result[all]:    %f",  result[m]);
		 }
		last_visited = result[minIndex]; // 更新最小值
		ROS_INFO(" last_visited after one step:    %f",  last_visited);
		ROS_INFO(" result[minIndex]:    %f",  result[minIndex]);
		v0_set[i]=minIndex;
		v0_length++;
	 
		visited[minIndex] = 1; // 将最小值顶点选中
		v0 = minIndex; // 下次查找从最限制顶点开始
	}
}
 

void callback(const geometry_msgs::Pose2D& pose)
{
	//ROS_INFO("Received a pose message!");
	double x = pose.x; 
	double y = pose.y;
	double a = pose.theta;
	//ROS_INFO("x,y,a:[%f,%f,%f]",x,y,a);
}

void s_callback(const std_msgs::Int8& msg){
	 
	goal_station=msg.data;
	ROS_INFO(" station num: %d",goal_station);
	mode_flag=2;
}



void p_callback(const geometry_msgs::Pose2D& pose)
{
	//ROS_INFO("Received a pose message!");
	pos_goal.x= pose.x; 
	pos_goal.y= pose.y;
	pos_goal.theta= pose.theta;
	mode_flag=1;
}









