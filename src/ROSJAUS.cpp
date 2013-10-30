/*! 
 ***********************************************************************
 * file      ROSJAUS.cpp
 *           Demo we can send JAUS message to JAUS Validation Tool http://www.usaric.org/JVT
 * author    Cheng-Lung Lee. University of Detroit Mercy 
 *            Advanced mobile Robotics Lab    
 * date      2009/04/09
 *
 ************************************************************************
 */
/*
 2012-06-11 Note
 change dist equation & change distance_epsilon from 0.5 to 1.2 meter , with 0342AM "bazinga_jaus_ou_6_11_2012_0342AM_25sec.cfg_.zip" we can do 25sec
 try change distance_epsilon from 1.2 to 2 meter , test ...
 add update_rate_Hz , 10 Hz
 
 */

#include <iostream>
#include <stdio.h>
#include <signal.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <cmath>
#include <string>
//#include <ctime>
#include <list>
//#include <sys/time.h>

// TODO put ROS stuff here

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h> // for  move_base_simple/goal
#include <geometry_msgs/PoseWithCovarianceStamped.h> // for  move_base /initialpose
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>  // for acceleration in our JAUS_move_base_simple
//from http://www.ros.org/wiki/navigation/Tutorials/SendingSimpleGoals
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <boost/thread.hpp>

//JAUS Stuff must in the end of the include section
#include "JuniorAPI.h"
#include "JAUSmessage.h"
#include "JAUS_MESSAGE_ID.h"
#include "JAUSservices.h"

using namespace std; 


// if MoveBase_ENABLED is False , no waypoint driver. robot will not move , but you still can joystick the robot to check the JAUS code
// if
#define MoveBase_ENABLED false


// **************JAUS JVT & IGVC COP Jaus ID
// change to ROS style parameter input
int COP_id; // 42
unsigned int COP_JausID; // 42-1-1
//***************Robot Subsystem ID
// change to ROS style parameter input
int Robot_id;
unsigned int Robot_JausID;

// Make sure you have smaller distance_epsilon in VFH otherwise vehicle will try to stop in waypoint
//const double distance_epsilon = 1.6; in high speed use bigger epsilon , count on the deceleration to let vehicle pass the center of the waypoint
// change to ROS style parameter input
double distance_epsilon ; // for simulator use 0.5
// change to ROS style parameter input
//double update_rate_Hz = 10.0;

// Define the message structure.  We have to tell
// the compiler to pack on 1-byte boundaries toStatus_INIT
// make sure it doesn't insert padding that would
// be non-compliant to AS6009.
#pragma pack(1)
double UInt32ToScale(double val, double low, double high);
double UInt16ToScale(double val, double low, double high);
void confirm_element_request(long handle, unsigned int client_id,unsigned char req_id);
unsigned int scaleToUInt32(double val, double low, double high);
unsigned short scaleToUInt16(double val, double low, double high);
unsigned int GetTimeStamp(void);
//static void handle_exit_signal( int signum );
void SEND_QUERY_IDENTIFICATION(long handle,unsigned int COP_id);
void BROADCAST_QUERY_IDENTIFICATION(long handle);
void SEND_REPORT_IDENTIFICATION(long handle,unsigned int COP_id);
void SEND_REPORT_STATUS(long handle,unsigned int COP_id);
void SEND_REPORT_CONTROL(long handle,unsigned int COP_id);
void SEND_REPORT_LOCAL_POSE (long handle,unsigned int COP_id);
void SEND_REPORT_VELOCITY_STATE (long handle,unsigned int COP_id);
void SEND_CONFIRM_CONTROL(long handle,unsigned int COP_id);
void SEND_REJECT_CONTROL(long handle,unsigned int COP_id);
void SEND_REPORT_ELEMENT_COUNT(long handle, unsigned int client_id, int list_ind);
void SEND_REPORT_ELEMENT_LIST(int list_ind, long handle, unsigned int client_id, ELEMENT* elm);
void ProcessJausMessage(long handle,unsigned int COP_JausID,unsigned int MessageID,unsigned int* source,unsigned int* bufsize, char* buffer);
unsigned int GenerateJausID(unsigned int subsystemID,unsigned int nodeID,unsigned int componentID);
void SEND_REPORT_ACTIVE_ELEMENT(long handle,unsigned int COP_JausID,ELEMENT* elm, int active_elm);
void Processelement(ELEMENT* elm, long handle, unsigned int client_id, char* buffer);
void SEND_REPORT_SERVICES(long handle,unsigned int client_id);
void SEND_REPORT_TRAVEL_SPEED(long handle, unsigned int client_id, double VX);
void SEND_REPOT_LOCAL_WAYPOINT(long handle, unsigned int client_id, ELEMENT* elm, int active_elm);
move_base_msgs::MoveBaseGoal ROSsetGoal(double x,double y,double w);
void Odom_Callback(const nav_msgs::Odometry::ConstPtr& odom);
void spinThread();
void translate_ROS_to_JAUS(double X_in, double Y_in, double W_in, double X_home_ROS, double Y_home_ROS, double theta_home_ROS, double * X_out, double * Y_out, double * W_out);
void translate_JAUS_to_ROS(double X_in, double Y_in, double X_home_ROS, double Y_home_ROS, double theta_home_ROS, double * X_out, double * Y_out);
double YawJAUS_to_YawROS(double W_in, double theta_home_ROS);

void Print_JAUS_Status(void);
void gotoxy(int x,int y);
void SaveCursor(void);
void RestoreCursor(void);


//struct timeval lastplayerupdate= {0, 0};
//struct timeval lastCOPupdate= {0, 0};
//struct timeval curr;
//double lastplayerupdate=0;
double lastCOPupdate= 0;
double current_time;

char Robot_Name[Max_IDENTIFICATION_StringLength];

double posX_JAUS,posY_JAUS;
double posZ_JAUS=0;
double posYaw_JAUS = 0;
double posSpeed_JAUS,posYawRate_JAUS;

double posx_ROS,posy_ROS,posz_ROS,posyaw_ROS;

double X_home_ROS = 0;//change these to the proper magic number
double Y_home_ROS = 0;
double Z_home_ROS = 0;
double Yaw_home_ROS = 0;



//This is the offset initialized to zero for the angle calculation
ELEMENT elm_list[8];//
double X_goal_ROS, Y_goal_ROS;

int list_ind=0;
double wrapTo2PI(double angle);
inline double normalize(double angle);
inline double angle_add(double pa, double pa_return);
int active_elm=0; //use for an offset to the base address of the element array

// Setup ROS Publisher and Subscriber
// Publish cmd_vel to control robot ( should change to waypoint driver )
//ros::Publisher cmd_vel_sub_;
// Subscribe to odom , this should be odometry from GPS+Digitalcompass   if run in IGVC for real JAUS
//ros::Subscriber odom_pub_ ;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
double ROS_loop_rate_Hz;
double move_base_max_acc;
double move_base_max_acc_last_waypoint;
double max_run_time_speed;


double drive_speed;
double drive_speed_scale;

unsigned int number_elements=0;
unsigned int debugi=1;
unsigned char VehicleStatus= Status_INIT ; // init
unsigned char VehicleControlStatus= 0 ; // init no control
unsigned int  Controller_JausID=0; // System controller id ( will be COP or OCU)
unsigned char COPConnectStatus= 0 ; // init no connection
// Define a signal handler, so we can clean-up properly
static int exit_flag = 0;
//static void handle_exit_signal( int signum )
//{
//    exit_flag = 1;
//}
bool drive = false;//variables for the drive section
bool elm_set = false;
    
int main(int argc, char* argv[])
{
    long handle;
// setup ROS parameter here
	ros::init(argc, argv, "ROS_JAUS");
	ros::NodeHandle n("~");
	// read parameters
	n.param("JAUS_COP_ID",      COP_id, 42);
	n.param("Robot_JAUS_ID",    Robot_id, 106);

  // Read Robot Name Here max 20 characters
  std::string Robot_name_str;
  n.param<std::string>("Robot_Name", Robot_name_str, "UDM AMRL");
  strcpy(Robot_Name,Robot_name_str.c_str());

	ROS_INFO("[Got parameters] Robot Name: [%s] , JAUS_ID: [%d] \n",Robot_Name, Robot_id);

	
// if you drive very fast like in IGVC2012 @1.9 m/s , set this value to 1.6m , We count on vehicle continue to drive and pass waypoint
// For IGVC2013 we use JAUS_move_base_simple to drive vehicle , need to play with this value.
	n.param("distance_epsilon", distance_epsilon, 0.5);
//	n.param("update_rate_Hz",    update_rate_Hz, 10.0);
	n.param("ROS_loop_rate_Hz",    ROS_loop_rate_Hz, 20.0);
	
	
	n.param("move_base_max_acc",    move_base_max_acc, 4.0);
	n.param("move_base_max_acc_last_waypoint",    move_base_max_acc_last_waypoint, 2.0);
	n.param("move_base_max_run_time_speed",    max_run_time_speed, 1.5);
	n.param("drive_speed_scale",    drive_speed_scale, 1.0);
	ros::Subscriber odom_sub = n.subscribe("/odom", 1, Odom_Callback ); // queue_size is 1 , we don't need old data

// this is simple one waypoint navigation
	ros::Publisher move_base_simple_goal_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
	ros::Publisher move_base_simple_goal_max_acc_pub = n.advertise<std_msgs::Float32>("/move_base_simple/max_acceleration", 1);
	ros::Publisher max_run_time_speed_pub            = n.advertise<std_msgs::Float32>("/move_base_simple/max_run_time_speed", 1);
	ros::Publisher move_base_initialpose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);

// setup publisher, should be waypoint 
	std_msgs::Float32 Max_acc_msgs;
	std_msgs::Float32 Max_run_time_speed_msgs;
// setup subscriber
    geometry_msgs::PoseStamped Goal_PoseStamped;
    Goal_PoseStamped.header.frame_id="/map";
    geometry_msgs::PoseWithCovarianceStamped Map_initialpose;
            Map_initialpose.header.stamp= ros::Time::now();
            Map_initialpose.header.frame_id="/map";
            Map_initialpose.pose.pose.position.x=0;
            Map_initialpose.pose.pose.position.y=0;
            Map_initialpose.pose.pose.orientation.x=0; 
            Map_initialpose.pose.pose.orientation.y=0; 
            Map_initialpose.pose.pose.orientation.z=0; 
            Map_initialpose.pose.pose.orientation.w=1; 


            move_base_initialpose_pub.publish(Map_initialpose); 
               
// for debug
#if(MoveBase_ENABLED)
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("/move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
#endif


	printf("================================================\r\n");
	printf("================================================\r\n");
	// **************JAUS JVT & IGVC COP Jaus ID : 42-1-1
    COP_JausID = GenerateJausID( COP_id,1,1) ;  // 90-1-1
	printf("My COP ID:%08X (%d-%d-%d)\r\n",COP_JausID,COP_JausID>>16,(COP_JausID&0xFF00)>>8,COP_JausID&0xFF); // print JAUS_ID
    // I'm using a subsystem ID of 331 decimal ( do not use 90 & 253 )
    // and node id 1, component id 1.  
	//***************Robot Subsystem ID : 106-1-1
	Robot_JausID =  GenerateJausID(Robot_id,1,1) ; ;  // 331-1-1
	printf("My JAUS ID:%08X (%d-%d-%d)\r\n",Robot_JausID,Robot_JausID>>16,(Robot_JausID&0xFF00)>>8,Robot_JausID&0xFF); // print JAUS_ID
	printf("================================================\r\n");
	printf("================================================\r\n");
    //
    // We have to call JrConnect before we send any messages.
    // JrConnect requires us to pass in the JAUS ID for our
    // component as well as the configuration file.  We 
    // get a handle back that's used for subsequent calls.
	
    // Pull the config file from the command line arguments
	std::string config_file = "../../bin/jr_config.xml"; // default configure file
    if (argc >= 2) config_file = std::string(argv[1]);  // if we have command line arguments use it
	
    if (JrConnect(Robot_JausID, config_file.c_str(), &handle) != Ok)
    {
        printf("Failed to connect to Junior.  Need more debug here...\n");
        return 0;
    }
    else printf("Successfully connected to Junior...\n");
					   
	// for receive data
	const unsigned short MaxMsgSize = 500;
    JrErrorCode ret;
    char buffer[MaxMsgSize]; 
    unsigned int buffersize;
    unsigned int sender;
    unsigned short msg_id;

	VehicleStatus=Status_STANDBY;// Standby

// Run spin() in different thread
   boost::thread spin_thread = boost::thread(boost::bind(&spinThread));


   ros::Rate loop_rate(ROS_loop_rate_Hz);
   ROS_INFO("ROS JAUS Loop Start!");
    while((ros::ok())&&(!exit_flag))
    {
		double diff;
        buffersize = MaxMsgSize;
		// Read Jaus message from Jr.
        ret = JrReceive(handle, &sender, &buffersize, buffer, NULL, NULL, &msg_id);
        if (ret == Ok)
        {
			printf("================================================\r\n");
			// printf("%X\n", std::string(buffer, buffersize).c_str());
			if (buffersize==2) 
			{
				printf("MessageID:%02X%02X  ",buffer[1]&0xFF,buffer[0]&0xFF); // print message ID
			} else if (buffersize<2) 
			{
                printf("Data(Hex): ");
                for (int i=0;i<buffersize;i++)  printf("%02X ",buffer[i]&0xFF); ;
			} else
			{
				printf("MessageID:%02X%02X , Data: ",buffer[1]&0xFF,buffer[0]&0xFF); // print message ID
			for (int i=2;i<buffersize;i++)  printf("%02X ",buffer[i]&0xFF); // Print DATA
			}
			printf(" ; %d bytes received. ",buffersize);
			printf("From :%8X ( %d-%d-%d )\n",sender,sender>>16,(sender&0xFF00)>>8,sender&0xFF);   
			
			// Do some message process here
			if ( buffersize>=2)
			{
				unsigned short MessageID=(unsigned short)(buffer[1]&0xFF)<<8 | (unsigned short)(buffer[0]&0xFF);
				ProcessJausMessage(handle,COP_JausID,MessageID, &sender, &buffersize, &buffer[0]);
			}
		
		}
		// use loop counter do some testing
		//loopcounter++;


		//drive the robot with this code if the execute list comand is received
	if (drive == true){
		unsigned short speed = ceil(drive_speed);//drive speed came during the execute list command.
		//printf("drive_speed = %f\n", speed);
		if (elm_set == false){ 	//get the first waypoint
			//X_goal = elm_list[active_elm].posx ; 
			//Y_goal = elm_list[active_elm].posy; //get goal from the element list
			
			translate_JAUS_to_ROS(elm_list[active_elm].posx, elm_list[active_elm].posy, X_home_ROS, Y_home_ROS,Yaw_home_ROS, &X_goal_ROS, &Y_goal_ROS);

            // publish ROS goal here 
#if(MoveBase_ENABLED)
			ac.sendGoal( ROSsetGoal(X_goal_ROS, Y_goal_ROS,Yaw_home_ROS) );
#else
            Goal_PoseStamped.header.stamp= ros::Time::now();
            Goal_PoseStamped.pose.position.x=X_goal_ROS;
            Goal_PoseStamped.pose.position.y=Y_goal_ROS;
            Goal_PoseStamped.pose.orientation.w=1;
            move_base_simple_goal_pub.publish(Goal_PoseStamped);
            
            // update max acceleration here
            Max_acc_msgs.data=move_base_max_acc;
            move_base_simple_goal_max_acc_pub.publish(Max_acc_msgs);
            // update max run time speed here
            Max_run_time_speed_msgs.data=max_run_time_speed*drive_speed_scale;
            max_run_time_speed_pub.publish(Max_run_time_speed_msgs);

#ifdef __APPLE__
  // in mac use say
	system("say set new Goal &");
#else
  // in linux use espeak
	system("espeak 'Set New Goal' &");
#endif			

#endif
			elm_set = true;
			printf("Active Element : [%i] Set goal: X_goal = %f Y_goal = %f \r\n", active_elm,X_goal_ROS, Y_goal_ROS);
		//printf("driving 2\n");
			}
		double dist= sqrt( pow((posx_ROS - X_goal_ROS),2)+pow((posy_ROS - Y_goal_ROS),2));
		//printf("Distance to goal = %f m \r\n", dist);
//		if (abs((posx_ROS - X_goal))< 0.5 && abs((posy_ROS - Y_goal))<0.5){  //these are taken w/respct to local coordinates
		if (dist < distance_epsilon ){  //these are taken w/respct to local coordinates
		active_elm++; //tell robot to drive to the next local waypoint
		//X_goal = elm_list[active_elm].posx ; Y_goal = elm_list[active_elm].posy; //get goal from the element list
		translate_JAUS_to_ROS(elm_list[active_elm].posx, elm_list[active_elm].posy, X_home_ROS, Y_home_ROS,Yaw_home_ROS, &X_goal_ROS, &Y_goal_ROS);

		// publish ROS goal here 
#if(MoveBase_ENABLED)
			ac.sendGoal( ROSsetGoal(X_goal_ROS, Y_goal_ROS,Yaw_home_ROS) );
#else
            Goal_PoseStamped.header.stamp= ros::Time::now();
            Goal_PoseStamped.pose.position.x=X_goal_ROS;
            Goal_PoseStamped.pose.position.y=Y_goal_ROS;
            Goal_PoseStamped.pose.orientation.w=1;
            move_base_simple_goal_pub.publish(Goal_PoseStamped);
            
            // update max acceleration here
            if(active_elm < (list_ind-1)){
            	Max_acc_msgs.data=move_base_max_acc;
            	}
            	// if it is last waypoint use different acceleration inorder to stop
            else if (active_elm == (list_ind-1)){
            	Max_acc_msgs.data=move_base_max_acc_last_waypoint;
            	}
            	
            move_base_simple_goal_max_acc_pub.publish(Max_acc_msgs);
            Max_run_time_speed_msgs.data=max_run_time_speed*drive_speed_scale;
            max_run_time_speed_pub.publish(Max_run_time_speed_msgs);


#ifdef __APPLE__
  // in mac use say
	system("say set new Goal &");
#else
  // in linux use espeak
	system("espeak 'Set New Goal' &");
#endif			

#endif

			printf("Active Element : [%i] Set goal: X_goal = %f Y_goal = %f \r\n", active_elm,X_goal_ROS, Y_goal_ROS);
		} //drive if not	
	if (active_elm > list_ind) { // if we are there increment to drive to the next one the next time throught the loop
		active_elm =0; drive =false; elm_set = false;
		
		
#ifdef __APPLE__
  // in mac use say
	system("say End of Waypoint List Vehicle Stop &");
#else
  // in linux use espeak
	system("espeak 'End of Waypoint List Vehicle Stop' &");
#endif			
					}//if we reach the end of the list stop driving
	}
		
		// Query COP id every 5 secs, if cop is not found
		if(COPConnectStatus == 0){ // send query ID every 5 secs if no COP found
		double current_time =ros::Time::now().toSec();
		//diff = (curr.tv_sec + curr.tv_usec/1e6) - (lastCOPupdate.tv_sec + lastCOPupdate.tv_usec/1e6);
		diff = current_time - lastCOPupdate;
        		 if (diff>=5){
			SEND_QUERY_IDENTIFICATION(handle,COP_JausID);
					 lastCOPupdate=current_time;
		 }
		}
		
		// Sleep a bit before looping again
        //ROS_INFO("ROS Loop sleep Once !");
		//ros::spinOnce();
		//usleep(100); // 100 micro-seconds
		// %Tag(RATE_SLEEP)%
		
		Print_JAUS_Status();
		
		    loop_rate.sleep();
		// %EndTag(RATE_SLEEP)%

    }	
	spin_thread.join();

    // Clean-up
    JrDisconnect(handle);
	printf("\nExit.... Disconnect Jr....\r\n");
	
    return 0;
}

// ///////////////////////////////////////////////////////////////////////////////
unsigned int GenerateJausID(unsigned int subsystemID,unsigned int nodeID,unsigned int componentID)
{
	return ( subsystemID<<16 | nodeID <<8 | componentID );
}


void SEND_QUERY_IDENTIFICATION(long handle,unsigned int COP_ID)  // TODO should add QueryType here
{
	// Query Identification
	QUERY_IDENTIFICATION_MSG qimsg;
	qimsg.msg_id=JAUS_ID_QueryIdentification;
	qimsg.QueryType = 0x02; 
    if (JrSend(handle, COP_ID,sizeof(qimsg), (char*)&qimsg) != Ok)
        printf("Unable to send QueryID message.  \n");
		else printf("Sent message QueryID to the COP\n");
			
}
 
void BROADCAST_QUERY_IDENTIFICATION(long handle)  // TODO should add QueryType here
	{
	// BROADCAST Query Identification ( to everyone in the network )
	QUERY_IDENTIFICATION_MSG qimsg;
	qimsg.msg_id= JAUS_ID_QueryIdentification; 
	qimsg.QueryType = ID_QueryType_Subsystem; //subsystem
	if (JrBroadcast(handle,sizeof(qimsg), (char*)&qimsg) != Ok)
		printf("Unable to send QueryID message to everyone.  \n");
		else printf("Sent message QueryID to everyone \n");
}
			
void SEND_REPORT_IDENTIFICATION(long handle,unsigned int COP_ID)
{
	REPORT_IDENTIFICATION_MSG idmsg;
	idmsg.msg_id= JAUS_ID_ReportIdentification;
	idmsg.QueryType=ID_QueryType_Subsystem; // subsystem
	idmsg.Type= ID_Type_Vehicle; // Vehicle
	
	//idmsg.Name = {'C', 'e', 'r','b','e','r','u','s'};
	//char buff[9] = "Cerberus";
  // Max_IDENTIFICATION_StringLength is 8 , change it in JAUSmessage.h
  //	  char buff[Max_IDENTIFICATION_StringLength+1] = "Bazinga!";
  //
  /* 
  	  char buff[] = "Bazinga!";
	//strcpy(buff,"Cerberus"); // System ID
	int i =0;
	while(buff[i] != 0){
	idmsg.Name[i] = buff[i];	
		i++;}
	idmsg.StringLength=sizeof(idmsg) - 6;
	*/
	
	strcpy(idmsg.Name,Robot_Name);
  idmsg.StringLength=strlen(idmsg.Name);
	long int msg_size=sizeof(idmsg)-Max_IDENTIFICATION_StringLength+strlen(idmsg.Name);
		
    // that the COP subsystem id is decimal 90 (0x005A hex)
    //
	printf("Message: Data Send [%s] Size:%li\n",((char*)&idmsg)+6,msg_size);
	
	
//    if (JrSend(handle, COP_ID, sizeof(idmsg), (char*)&idmsg) != Ok)
    if (JrSend(handle, COP_ID, msg_size, (char*)&idmsg) != Ok)
        printf("Unable to send System ID message.  Need more debug here...\n");
		else printf("Sent message System ID to the COP\n");
    // Now we send the message to the COP using Junior. 
}
void SEND_REPORT_SERVICES(long handle,unsigned int client_id)
{	

	REPORT_SERVICES rcmsg;
	rcmsg.msg_id=ReportServices;
	rcmsg.count_field_node = 1;
	rcmsg.count_field_component = 1;
	rcmsg.NodeID = 1; // Subsystem
	rcmsg.ComponentID = 1; // Vehicle
	rcmsg.InstanceID = 1;
	rcmsg.ListSize = 6;
	rcmsg.size1 = sizeof(Transport)-1;
	//rcmsg.size1 = sizeof(Transport)-1;
	rcmsg.MajorVersionNumber1=1;
	rcmsg.MinorVersionNumber1=0;
	rcmsg.size2 = sizeof(VelocityStateSensor)-1;
	//rcmsg.size2 = sizeof(VelocityStateSensor)-1;
	rcmsg.MajorVersionNumber2=1;
	rcmsg.MinorVersionNumber2=0;
	rcmsg.size3 = sizeof(LocalPoseSensor)-1;
	//rcmsg.size3 = sizeof(LocalPoseSensor)-1;
	rcmsg.MajorVersionNumber3=1;
	rcmsg.MinorVersionNumber3=0;
	rcmsg.size4 = sizeof(LocalWaypointDriver)-1;
	//rcmsg.size4 = sizeof(LocalWaypointDriver)-1;
	rcmsg.MajorVersionNumber4=1;
	rcmsg.MinorVersionNumber4=0;
	rcmsg.size5 = sizeof(AccessControl)-1;
	//rcmsg.size5 = sizeof(AccessControl)-1;
	rcmsg.MajorVersionNumber5=1;
	rcmsg.MinorVersionNumber5=0;
	rcmsg.size6 = sizeof(AccessControl)-1;
	//rcmsg.size6 = sizeof(AccessControl)-1;
	rcmsg.MajorVersionNumber6=1;
	rcmsg.MinorVersionNumber6=0;
	printf("size1 = %i, size2 = %i, size3 = %i, size4 = %i, size5 = %i, size6 = %i\n", rcmsg.size1, rcmsg.size2, rcmsg.size3, rcmsg.size4, rcmsg.size5, rcmsg.size6);
	for(int i = 0; i< (sizeof(Transport)-1); i++){  //subtract one to ignore the null terminator in the loop
	rcmsg.transport[i] = Transport[i];
	}

	for(int i = 0; i< (sizeof(VelocityStateSensor)-1); i++){
	rcmsg.velocity[i] = VelocityStateSensor[i];
	}
	printf("Velocitystatesensor size =  %li\n", (sizeof(VelocityStateSensor)-1));	
	
	for(int i = 0; i< (sizeof(LocalPoseSensor)-1); i++){
	rcmsg.localpose[i] = LocalPoseSensor[i];
	}
	
	for(int i = 0; i< (sizeof(LocalWaypointDriver)-1); i++){
	rcmsg.localwaypoint[i] = LocalWaypointDriver[i];
	}

	for(int i = 0; i< (sizeof(AccessControl)-1); i++){
	rcmsg.accesscontrol[i] = AccessControl[i];
	}

	for(int i = 0; i< (sizeof(Events)-1); i++){
	rcmsg.events[i] = Events[i];
	}


	printf("Message: Size:%li Data Sent \n",sizeof(rcmsg));
	for(int i = 0; i<sizeof(rcmsg);i++){ printf("%.*s",1, ((char*)&rcmsg + i));}
	printf("\n");
    if (JrSend(handle, client_id, sizeof(rcmsg), (char*)&rcmsg) != Ok)
        printf("Unable to send System ID message.  Need more debug here...\n");
		else printf("Sent message System ID to the COP\n");
 
}

void SEND_REPORT_STATUS(long handle,unsigned int COP_ID)
{
	// ID 4002 REPORT_STATUS
	//STATUS
    // 0 INIT 
	// 1 READY 
	// 2 STANDBY 
	// 3 SHUTDOWN 
	// 4 FAILURE 
	// 5 EMERGENCY 
	
	REPORT_STATUS_MSG rsmsg;
	rsmsg.msg_id=JAUS_ID_ReportStatus;
	rsmsg.Status=VehicleStatus;
	rsmsg.UNUSED=0;    
	if (JrSend(handle, COP_ID, sizeof(rsmsg), (char*)&rsmsg) != Ok)
        printf("Unable to send REPORT_STATUS Standby message.  Need more debug here...\n");
		else printf("Sent message REPORT_STATUS Standby to the COP\n");
			
}

void SEND_REPORT_CONTROL(long handle,unsigned int COP_ID)
{
	// ID 400D REPORT_CONTROL_MSG
	REPORT_CONTROL_MSG rcmsg;
	rcmsg.msg_id= JAUS_ID_ReportControl;
	// report who control this component, of course , here we always set to COP for testing
	if (VehicleControlStatus== 0)  // not conteoled
	{
		rcmsg.SubsystemID=0;
		rcmsg.NodeID=0;
		rcmsg.ComponentID=0;
		rcmsg.AuthorityCode=0;
	} else // for now report COP controled
	{ // TODO change it to who been confirm control
//		rcmsg.SubsystemID=90;
//		rcmsg.NodeID=1;
//		rcmsg.ComponentID=1;
//		rcmsg.AuthorityCode=255;
		
		rcmsg.SubsystemID=(unsigned short)(Controller_JausID>>16);
		rcmsg.NodeID=(unsigned char)((Controller_JausID&0xFF00)>>8);
		rcmsg.ComponentID=(unsigned char)(Controller_JausID&0xFF);
		rcmsg.AuthorityCode=255;

	}
	if (JrSend(handle, COP_ID, sizeof(rcmsg), (char*)&rcmsg) != Ok)
        printf("Unable to send REPORT_CONTROL message.  Need more debug here...\n");
		else printf("Sent message REPORT_CONTROL  to the COP\n");
			
}

void SEND_REPORT_LOCAL_POSE (long handle,unsigned int COP_ID)
{
	//only report local pose when vehicle in ready mode
    if ( VehicleStatus== Status_READY )
	{
		REPORT_LOCAL_POSE_MSG lpmsg;
		//    if (sizeof(msg) != 12) printf("Packing error!\n");
		
		//
		// Populate the message.  The message id is fixed, but the
		// X and Y data are bogus.  The PV is set to indicate that
		// the first 2 optional fields are present.
		//
		lpmsg.msg_id = JAUS_ID_ReportLocalPose;
		//recalculate the posistion
		//translate_ROS_to_JAUS(posx_ROS, posy_ROS, X_home_ROS, Y_home_ROS ,Yaw_home_ROS, &posX_JAUS, &posY_JAUS);	
		
		lpmsg.pv = LOCAL_POSE_PVBit_X+LOCAL_POSE_PVBit_Y+LOCAL_POSE_PVBit_Z+LOCAL_POSE_PVBit_Yaw+LOCAL_POSE_PVBit_TimeStamp ;
		lpmsg.X = scaleToUInt32(posX_JAUS, -100000, 100000);
		lpmsg.Y = scaleToUInt32(posY_JAUS, -100000, 100000);
		lpmsg.Z = scaleToUInt32(posZ_JAUS, -100000, 100000);
		
		lpmsg.Yaw = scaleToUInt16(posYaw_JAUS, -1*PI, PI);
		
		//	tm lpTime
		lpmsg.TimeStamp=GetTimeStamp();
		//printf("TimeStamp Check: %u",lpmsg.TimeStamp);
		//
		// Now we send the message to the COP using Junior.  Recall
		// that the COP subsystem id is decimal 90 (0x005A hex)
		//
ROS_INFO("ROS:[%5.2f,%5.2f,%5.2f,%5.2f],JAUS:[%5.2f,%5.2f,%5.2f,%5.2f],Speed:[%5.2f,%5.2f]", posx_ROS,posy_ROS,posz_ROS,posyaw_ROS,posX_JAUS,posY_JAUS,posZ_JAUS,posYaw_JAUS,posSpeed_JAUS,posYawRate_JAUS);
		if (JrSend(handle, COP_ID , sizeof(lpmsg), (char*)&lpmsg) != Ok)
			printf("Unable to send Local Pose message.  Need more debug here...\n");
		else printf("Sent message Local Pose to the COP\n");
	}
	else
	{
		printf("*** Vehicle is not in Ready mode , no local pose report \r\n"); 
	}
	
}


void SEND_REPORT_VELOCITY_STATE (long handle,unsigned int COP_ID)
{
	//only velocity state when vehicle in ready mode
    if ( VehicleStatus== Status_READY )
	{
		
		REPORT_VELOCITY_STATE_MSG rvmsg;
		rvmsg.msg_id=JAUS_ID_ReportVelocityState;
		rvmsg.pv=VELOCITY_STATE_PVBit_VX+VELOCITY_STATE_PVBit_YawRate+VELOCITY_STATE_PVBit_TimeStamp;  //REPORT_VELOCITY_STATE_MSG_pv;
		rvmsg.VX=scaleToUInt32( posSpeed_JAUS, -327.68, 327.67);
		rvmsg.YawRate=scaleToUInt16(posYawRate_JAUS, -32.768 , 32.767 );
		rvmsg.TimeStamp=GetTimeStamp();
		//printf("Debug--->posYawRate=%f,rvmsg.YawRate=%i\n",posYawRate,rvmsg.YawRate);
		
		if (JrSend(handle, COP_ID , sizeof(rvmsg), (char*)&rvmsg) != Ok)
			printf("Unable to SEND_REPORT_VELOCITY_STATE message.  Need more debug here...\n");
		else printf("Sent message SEND_REPORT_VELOCITY_STATE to the COP\n");
	}
	else
	{
		printf("*** Vehicle is not in Ready mode , no velocity state report \r\n"); 
	}
	
	
}
void SEND_CONFIRM_CONTROL(long handle,unsigned int COP_ID)
{
	CONFIRM_CONTROL_MSG ccmsg;
	ccmsg.msg_id=JAUS_ID_ConfirmControl;
	ccmsg.ResponseCode=0;
	VehicleStatus=Status_STANDBY; // set to STANDBY , need double check here . check AS5710. P29
	VehicleControlStatus= 1; // controled
	/* ResponseCode
	 0 CONTROL ACCEPTED 
	 1 NOT AVAILABLE 
	 2 INSUFFICIENT AUTHORITY 
	 */
    if (JrSend(handle, COP_ID , sizeof(ccmsg), (char*)&ccmsg) != Ok)
        printf("Unable to SEND_CONFIRM_CONTROL message.  Need more debug here...\n");
    else printf("Sent message SEND_CONFIRM_CONTROL to the COP\n");
	
}
void SEND_REJECT_CONTROL(long handle,unsigned int COP_ID)
{
	// TODO add a correct response here
}
//================================================
void ProcessJausMessage(long handle,unsigned int COP_JausID,unsigned int MessageID,unsigned int* sender,unsigned int* bufsize, char* buffer)

{
	switch (MessageID)
	{
		case JAUS_ID_SetAuthority             :
			printf("Message: SetAuthority              received\r\n"); 
			break; 
		case JAUS_ID_Shutdown                 :
			printf("Message: Shutdown                  received\r\n"); 
			if (*sender==Controller_JausID) // check who want set local pose, make sure is controler
			{

			printf("Set Vehicle Status to Shutdown ! \r\n"); 
			VehicleStatus=Status_SHUTDOWN; // Shutdown
			//SEND_REPORT_STATUS(handle,Controller_JausID); // tell COP what happen before we shutdown
			
#ifdef __APPLE__
		  // in mac use say
			system("say shutdown ROS jaws &");
#else
		  // in linux use espeak
			system("espeak 'shutdown ROS jaws' &");
#endif			
			
			
			
			exit_flag=1; // Leave program , stop JAUS
			}
			else
			{
				printf("Sensder is not controlling, can not Set Vehicle Status to Shutdown!!\r\n");				
			}
			
			break; 
		case JAUS_ID_Standby                  :
			printf("Message: Standby                   received\r\n"); 
			if (*sender==Controller_JausID) // check who want set local pose, make sure is controler
			{
				
				
#ifdef __APPLE__
		  // in mac use say
			system("say system state standby &");
#else
		  // in linux use espeak
			system("espeak 'system state standby' &");
#endif			
			

				VehicleStatus=Status_STANDBY; // Standby , in standby mode we don't report local pose
				//SEND_REPORT_STATUS(handle,Controller_JausID); // tell COP what status we are
				printf("Set Vehicle Status to Standby ! \r\n"); 
			}
			else
			{
				printf("Sensder is not controlling, can not Set Vehicle Status to Standby!!\r\n");				
				
			}
			break; 
		case JAUS_ID_Resume                   :
			printf("Message: Resume                    received\r\n");
			if (*sender==Controller_JausID) // check who want set local pose, make sure is controler
			{
			VehicleStatus=Status_READY; // Ready
			//SEND_REPORT_STATUS(handle,Controller_JausID); // tell COP what status we are
			printf("Set Vehicle Status to Ready\r\n"); 
			
#ifdef __APPLE__
  		// in mac use say
	    system("say system state ready &");
#else
  		// in linux use espeak
	    system("espeak 'say system state ready' &");
#endif			
			
			}
			else
			{
				printf("Sensder is not controlling, can not Set Vehicle Status to Ready!!\r\n");				
				
			}
			break; 
		case JAUS_ID_Reset                    :
			printf("Message: Reset                     received\r\n"); 
			VehicleStatus=Status_STANDBY; // Standby , need check here ???
			printf("Set Vehicle Status to Standby ! \r\n"); 			
			break; 
		case JAUS_ID_SetEmergency             :
			printf("Message: SetEmergency              received\r\n");
			VehicleStatus=Status_EMERGENCY; // Emergency
			printf("Set Vehicle Status to Emergency ! \r\n"); 
			break; 
		case JAUS_ID_ClearEmergency           :
			printf("Message: ClearEmergency            received\r\n"); 
			VehicleStatus=Status_STANDBY; // Standby , ???
			printf("Set Vehicle Status to Standby ! \r\n"); 
			break; 
		case JAUS_ID_RequestControl           :
			printf("Message: RequestControl            received\r\n");
			if (VehicleControlStatus==0)
	        {
				SEND_CONFIRM_CONTROL(handle,COP_JausID); // Confirm Control}
				VehicleControlStatus=1; // controled
				Controller_JausID=*sender;
				printf("Now Vehicle Control by: %d-%d-%d \r\n",*sender>>16,(*sender&0xFF00)>>8,*sender&0xFF); // Print JAUS ID
				
#ifdef __APPLE__
  		// in mac use say
	    system("say system control by C O P &");
#else
  		// in linux use espeak
	    system("espeak 'system control by C O P' &");
#endif			
				
			}
			else
			{
				SEND_REJECT_CONTROL(handle,COP_JausID); //  Reject Control
			}
			
			break; 
		case JAUS_ID_ReleaseControl           :
			printf("Message: ReleaseControl            received\r\n"); 
			break; 
		case JAUS_ID_ConfirmControl           :
			printf("Message: ConfirmControl            received\r\n"); 
			break; 
		case JAUS_ID_RejectControl            :
			printf("Message: RejectControl             received\r\n"); 
			break; 
		case JAUS_ID_SetTime                  :
			printf("Message: SetTime                   received\r\n"); 
			break; 
		case JAUS_ID_CreateEvent              :
			printf("Message: CreateEvent               received\r\n"); 
			break; 
		case JAUS_ID_UpdateEvent              :
			printf("Message: UpdateEvent               received\r\n"); 
			break; 
		case JAUS_ID_CancelEvent              :
			printf("Message: CancelEvent               received\r\n"); 
			break; 
		case JAUS_ID_ConfirmEventRequest      :
			printf("Message: ConfirmEventRequest       received\r\n"); 
			break; 
		case JAUS_ID_RejectEventRequest       :
			printf("Message: RejectEventRequest        received\r\n"); 
			break; 
		case JAUS_ID_RegisterServices         :
			printf("Message: RegisterServices          received\r\n"); 
			break; 
		case JAUS_ID_QueryAuthority           :
			printf("Message: QueryAuthority            received\r\n"); 
			break; 
		case JAUS_ID_QueryStatus              :
			printf("Message: QueryStatus               received\r\n"); 
				SEND_REPORT_STATUS(handle,COP_JausID);
			break; 
		case JAUS_ID_QueryTimeout             :
			printf("Message: QueryTimeout              received\r\n"); 
			break; 
		case JAUS_ID_QueryTime                :
			printf("Message: QueryTime                 received\r\n"); 
			break; 
		case JAUS_ID_QueryControl             :
			printf("Message: QueryControl              received\r\n"); 
			SEND_REPORT_CONTROL( handle, COP_JausID);
			break; 
		case JAUS_ID_QueryEvents              :
			printf("Message: QueryEvents               received\r\n"); 
			break; 
		case JAUS_ID_QueryHeartbeatPulse      :
			printf("Message: QueryHeartbeatPulse       received\r\n"); 
			break; 
		case JAUS_ID_QueryIdentification      :
			printf("Message: QueryIdentification       received\r\n");
			SEND_REPORT_IDENTIFICATION(handle,COP_JausID);
			break; 
		case JAUS_ID_QueryConfiguration       :
			printf("Message: QueryConfiguration        received\r\n"); 
			break; 
		case JAUS_ID_QuerySubsystemList       :
			printf("Message: QuerySubsystemList        received\r\n"); 
			break; 
		case JAUS_ID_QueryServices            :
			printf("Message: QueryServices             received\r\n"); 
			SEND_REPORT_SERVICES(handle,COP_JausID);
			break; 
		case JAUS_ID_ReportAuthority          :
			printf("Message: ReportAuthority           received\r\n"); 
			break; 
		case JAUS_ID_ReportStatus             :
			printf("Message: ReportStatus              received\r\n"); 
			break; 
		case JAUS_ID_ReportTimeout            :
			printf("Message: ReportTimeout             received\r\n"); 
			break; 
		case JAUS_ID_ReportTime               :
			printf("Message: ReportTime                received\r\n"); 
			break; 
		case JAUS_ID_ReportControl            :
			printf("Message: ReportControl             received\r\n"); 
			break; 
		case JAUS_ID_ReportEvents             :
			printf("Message: ReportEvents              received\r\n"); 
			break; 
		case JAUS_ID_Event                    :
			printf("Message: Event                     received\r\n"); 
			break; 
		case JAUS_ID_ReportHeartbeatPulse    :
			printf("Message: ReportHeartbeatPulse     received\r\n"); 
			break; 
		case JAUS_ID_ReportIdentification     :
			printf("Message: ReportIdentification      received\r\n");
			printf("From: %d-%d-%d \r\n",*sender>>16,(*sender&0xFF00)>>8,*sender&0xFF); // Print JAUS ID

			if (*sender==COP_JausID)
			{ // Get ID from COP , start doing something else
				COPConnectStatus= 1; // connected
				// TODO : Print JAUS COP ID here
				printf("================================================\r\n");
				printf("==============JAUS COP FOUND!!!!================\r\n");
				printf("================================================\r\n");
			}
			
			
			break; 
		case JAUS_ID_ReportConfiguration      :
			printf("Message: ReportConfiguration       received\r\n"); 
			break; 
		case JAUS_ID_ReportSubsystemList      :
			printf("Message: ReportSubsystemList       received\r\n"); 
			break; 
		case JAUS_ID_ReportServices           :
			printf("Message: ReportServices            received\r\n"); 
			break; 
		case JAUS_ID_SetGlobalPose            :
			printf("Message: SetGlobalPose             received\r\n"); 
			break; 
		case JAUS_ID_SetLocalPose             :
			{			
			printf("Message: SetLocalPose              received\r\n");
			// TODO should check the package see what value the COP want then set odom,
			// now just set all to zero
			if (*sender==Controller_JausID) // check who want set local pose, make sure is controler
			{ // 
			// TODO get XYZA data to set loca pose
			
			// for IGVC2013 this is for fix PV 0x47 ,x,y,z,yaw
			unsigned int X_hold = (unsigned short)(buffer[7]&0xFF)<<24 | (unsigned short)(buffer[6]&0xFF)<<16 |  (unsigned short)(buffer[5]&0xFF)<<8 |  (unsigned short)(buffer[4]&0xFF);
			unsigned int Y_hold = (unsigned short)(buffer[11]&0xFF)<<24 | (unsigned short)(buffer[10]&0xFF)<<16 |  (unsigned short)(buffer[9]&0xFF)<<8 |  (unsigned short)(buffer[8]&0xFF);
			unsigned int Z_hold = (unsigned short)(buffer[15]&0xFF)<<24 | (unsigned short)(buffer[14]&0xFF)<<16 |  (unsigned short)(buffer[13]&0xFF)<<8 |  (unsigned short)(buffer[12]&0xFF);

			unsigned short Yaw_hold = (unsigned short)(buffer[17]&0xFF)<<8 | (unsigned short)(buffer[16]&0xFF);
			//playerc_position2d_set_odom(position2d,0,0,0);
			double X_hold_jaus = ceil(UInt32ToScale(X_hold, -100000, 100000));
			double Y_hold_jaus = ceil(UInt32ToScale(Y_hold, -100000, 100000));
			double Z_hold_jaus = ceil(UInt32ToScale(Z_hold, -100000, 100000));
			float Yaw_hold_jaus= ceil(UInt16ToScale(Yaw_hold, -1*PI, PI));
		  
			//Yaw_home_ROS = posyaw_ROS;//angle_add(posyaw_ROS, (-Yaw));
			// TODO Fix here ... the logic is not correct when X_hold_jaus,Y_hold_jaus,Yaw_hold_jaus is not zero
			// if X_hold_jaus==0,y_hold_jaus==0,Yaw_hold_jaus==0
			Yaw_home_ROS = posyaw_ROS ;
			X_home_ROS   = posx_ROS;
			Y_home_ROS   = posy_ROS;
			Z_home_ROS   = posz_ROS;
			// else
			//convert ros_x,y,yaw to JAUS_X,Y,YAW , with home=0,0,0
			//offset jaus_home but X,Y,Yaw_hold_jaus
			// convert jaus_home back to ROS_xy with home=0,0
			//
			
			
			//
			printf("SetLocalPose to X = %f, Y = %f, Z=%f, Yaw = %f\n", X_hold_jaus , Y_hold_jaus, Z_hold_jaus, Yaw_hold_jaus);
			//playerc_position2d_set_odom(position2d,X,Y,posyaw_ROS); // now just set to zero
			
#ifdef __APPLE__
  		// in mac use say
	    system("say set local pose &");
#else
  		// in linux use espeak
	    system("espeak 'set local pose' &");
#endif
			
			printf("Reset set local pose!!\r\n");				
			} else
			{
				printf("Sensder is not controlling, can not set local pose!!\r\n");				
				
			}
			}			
			break; 
		case JAUS_ID_SetWrenchEffort          :
			printf("Message: SetWrenchEffort           received\r\n"); 
			break; 
		case JAUS_ID_SetGlobalVector          :
			printf("Message: SetGlobalVector           received\r\n"); 
			break; 
		case JAUS_ID_SetLocalVector           :
			printf("Message: SetLocalVector            received\r\n"); 
			break; 
		case JAUS_ID_SetTravelSpeed           :
			printf("Message: SetTravelSpeed            received\r\n"); 
			break; 
		case JAUS_ID_SetGlobalWaypoint        :
			printf("Message: SetGlobalWaypoint         received\r\n"); 
			break; 
		case JAUS_ID_SetLocalWaypoint         :
			printf("Message: SetLocalWaypoint          received\r\n"); 
			break; 
		case JAUS_ID_SetGlobalPathSegment     :
			printf("Message: SetGlobalPathSegment      received\r\n"); 
			break; 
		case JAUS_ID_SetLocalPathSegment      :
			printf("Message: SetLocalPathSegment       received\r\n"); 
			break; 
		case JAUS_ID_SetGeomagneticProperty   :
			printf("Message: SetGeomagneticProperty    received\r\n"); 
			break; 
		case JAUS_ID_SetVelocityCommand       :
			printf("Message: SetVelocityCommand        received\r\n"); 
			break; 
		case JAUS_ID_SetAccelerationLimit     :
			printf("Message: SetAccelerationLimit      received\r\n"); 
			break; 
		case JAUS_ID_SetElement               :{
			printf("Message: SetElement                received\r\n");
			list_ind = 0; //set to sero if the confirm element message gets lost so they can retransmit.
			unsigned int elm_place = 4; //place the element data starts in the list
			unsigned int list_size = ((unsigned short)(buffer[3]));
			for(int i = 0; i < list_size; i++){
			Processelement(&elm_list[list_ind],handle, COP_JausID, &buffer[elm_place]);
			list_ind++; printf("linst index = %i \r\n", list_ind);
			elm_place = elm_place + 22;
			}
			confirm_element_request(handle, COP_JausID, buffer[2]);
			if(list_ind>4) printf("error, more elements received than expected\n");
			}	
			break; 
		case JAUS_ID_DeleteElement            :
			printf("Message: DeleteElement             received\r\n"); 
			break; 
		case JAUS_ID_ConfirmElementRequest    :
			printf("Message: ConfirmElementRequest     received\r\n"); 
			break; 
		case JAUS_ID_RejectElementRequest     :
			printf("Message: RejectElementRequest      received\r\n"); 
			break; 
		case JAUS_ID_ExecuteList              :
			{
			printf("Message: ExecuteList               received\r\n"); 
			drive = true; //this is a global variable because I am lazy
			unsigned short speed_hold = ((unsigned short)(buffer[4]&0xFF)<<8 | (unsigned short)(buffer[3]&0xFF)); // add 1byte PV offset IGVC2013	
			drive_speed = UInt16ToScale(speed_hold,0,327.67);
			max_run_time_speed=drive_speed;
			printf("drive speed = %f m/s\r\n",drive_speed);
	        
#ifdef __APPLE__
  		// in mac use say
	    system("say Execute List Vehicle Driving &");
#else
  		// in linux use espeak
	    system("espeak 'Execute List Vehicle Driving' &");
#endif	        
	
			}
			break; 

		case JAUS_ID_QueryGlobalPose          :
			printf("Message: QueryGlobalPose           received\r\n"); 
			break; 
		case JAUS_ID_QueryLocalPose           :
			printf("Message: QueryLocalPose            received\r\n");
			SEND_REPORT_LOCAL_POSE (handle,COP_JausID);
			break; 
		case JAUS_ID_QueryVelocityState       :
			printf("Message: QueryVelocityState        received\r\n");
			SEND_REPORT_VELOCITY_STATE(handle,COP_JausID);
			break; 
		case JAUS_ID_QueryWrenchEffort        :
			printf("Message: QueryWrenchEffort         received\r\n"); 
			break; 
		case JAUS_ID_QueryGlobalVector        :
			printf("Message: QueryGlobalVector         received\r\n"); 
			break; 
		case JAUS_ID_QueryLocalVector         :
			printf("Message: QueryLocalVector          received\r\n"); 
			break; 
		case JAUS_ID_QueryTravelSpeed         :
			printf("Message: QueryTravelSpeed          received\r\n"); 
			SEND_REPORT_TRAVEL_SPEED(handle, COP_JausID,posSpeed_JAUS);
			break; 
		case JAUS_ID_QueryGlobalWaypoint      :
			printf("Message: QueryGlobalWaypoint       received\r\n"); 
			break; 
		case JAUS_ID_QueryLocalWaypoint       :
			printf("Message: QueryLocalWaypoint        received\r\n"); 
			SEND_REPOT_LOCAL_WAYPOINT(handle,COP_JausID,elm_list, active_elm);
			break; 
		case JAUS_ID_QueryGlobalPathSegment   :
			printf("Message: QueryGlobalPathSegment    received\r\n"); 
			break; 
		case JAUS_ID_QueryLocalPathSegment    :
			printf("Message: QueryLocalPathSegment     received\r\n"); 
			break; 
		case JAUS_ID_QueryGeomagneticProperty :
			printf("Message: QueryGeomagneticProperty  received\r\n"); 
			break; 
		case JAUS_ID_QueryVelocityCommand     :
			printf("Message: QueryVelocityCommand      received\r\n"); 
			break; 
		case JAUS_ID_QueryAccelerationLimit   :
			printf("Message: QueryAccelerationLimit    received\r\n"); 
			break; 
		case JAUS_ID_QueryAccelerationState   :
			printf("Message: QueryAccelerationState    received\r\n"); 
			break; 
		case JAUS_ID_QueryElement             :
			printf("Message: QueryElement              received\r\n"); 
			break; 
		case JAUS_ID_QueryElementList         :
			printf("Message: QueryElementList          received\r\n");
			SEND_REPORT_ELEMENT_LIST(list_ind, handle, COP_JausID, elm_list);
			break; 
		case JAUS_ID_QueryElementCount      :
			printf("Message: QueryElementCount         received\r\n"); 
			SEND_REPORT_ELEMENT_COUNT(handle, COP_JausID, list_ind);
			break; 
		case JAUS_ID_QueryActiveElement       :
			printf("Message: QueryActiveElement        received\r\n"); 
			SEND_REPORT_ACTIVE_ELEMENT(handle,COP_JausID,elm_list,active_elm);
			break; 
		case JAUS_ID_ReportGlobalPose         :
			printf("Message: ReportGlobalPose          received\r\n"); 
			break; 
		case JAUS_ID_ReportLocalPose          :
			printf("Message: ReportLocalPose           received\r\n"); 
			break; 
		case JAUS_ID_ReportVelocityState      :
			printf("Message: ReportVelocityState       received\r\n"); 
			break; 
		case JAUS_ID_ReportWrenchEffort       :
			printf("Message: ReportWrenchEffort        received\r\n"); 
			break; 
		case JAUS_ID_ReportGlobalVector       :
			printf("Message: ReportGlobalVector        received\r\n"); 
			break; 
		case JAUS_ID_ReportLocalVector        :
			printf("Message: ReportLocalVector         received\r\n"); 
			break; 
		case JAUS_ID_ReportTravelSpeed        :
			printf("Message: ReportTravelSpeed         received\r\n"); 
			break; 
		case JAUS_ID_ReportGlobalWaypoint     :
			printf("Message: ReportGlobalWaypoint      received\r\n"); 
			break; 
		case JAUS_ID_ReportLocalWaypoint      :
			printf("Message: ReportLocalWaypoint       received\r\n"); 
			break; 
		case JAUS_ID_ReportGlobalPathSegment  :
			printf("Message: ReportGlobalPathSegment   received\r\n"); 
			break; 
		case JAUS_ID_ReportLocalPathSegment   :
			printf("Message: ReportLocalPathSegment    received\r\n"); 
			break; 
		case JAUS_ID_ReportGeomagneticProperty:
			printf("Message: ReportGeomagneticProperty received\r\n"); 
			break; 
		case JAUS_ID_ReportVelocityCommand    :
			printf("Message: ReportVelocityCommand     received\r\n"); 
			break; 
		case JAUS_ID_ReportAccelerationLimit  :
			printf("Message: ReportAccelerationLimit   received\r\n"); 
			break; 
		case JAUS_ID_ReportAccelerationState  :
			printf("Message: ReportAccelerationState   received\r\n"); 
			break; 
		case JAUS_ID_ReportElement            :
			printf("Message: ReportElement             received\r\n"); 
			break; 
		case JAUS_ID_ReportElementList        :
			printf("Message: ReportElementList         received\r\n"); 
			break; 
		case JAUS_ID_ReportElementCount       :
			printf("Message: ReportElementCount        received\r\n"); 
			break; 
		case JAUS_ID_ReportActiveElement      :
			printf("Message: ReportActiveElement       received\r\n"); 
			break; 
	default:
		printf(" Unknow Message: %4X received. ", MessageID);
		break;
	}
}







// Define some helper functions to convert to/from
// scaled integers.
unsigned int scaleToUInt32(double val, double low, double high)
{
	return (unsigned int)((val - low) / ((high - low) / UInt32Range));
}
unsigned short scaleToUInt16(double val, double low, double high)
{

	/*float conv = (((val*(high - low))/(UInt16Range)) + low);
	printf("val %f converted to %f\n",val ,conv);
	if(conv = PI){  return 0;}
	else return conv;*/
	return (unsigned short)((val - low) / ((high - low) / UInt16Range));
}


/*
 Time integer Bits 0 - 9:  milliseconds, range 0...999 
 Bits 10-15:  Seconds, range 0...59 
 Bits 16-21:  Minutes, range 0...59 
 Bits 22-26:  Hour (24 hour clock), range 0..23 
 Bits 27-31:  Day, range 1...31 
 
 */
unsigned int GetTimeStamp(void)
{
	
	struct timeval tv;
	struct tm *timeinfo;
	time_t rawtime;
	unsigned int tstamp = 0;
	
	gettimeofday(&tv, NULL);
	time(&rawtime);
	timeinfo = gmtime ( &rawtime );
	
	unsigned int mMilliseconds = (unsigned int)(tv.tv_usec/1000.0);
	int mDay = timeinfo->tm_mday;
	int mHour = timeinfo->tm_hour;
	int mMinute = timeinfo->tm_min;
	int mSecond = timeinfo->tm_sec;
	
	tstamp |= (unsigned int)(mDay)    << 27;
	tstamp |= (unsigned int)(mHour)   << 22;
	tstamp |= (unsigned int)(mMinute) << 16;
	tstamp |= (unsigned int)(mSecond) << 10;
	tstamp |= mMilliseconds;
	cout << "Time Stamp: " << mDay << ":" << mHour << ":" << mMinute;
    cout << ":" << mSecond << ":" << mMilliseconds << endl;
	return tstamp ;
	//if (debugi==0x80000000) debugi=1;
	//else debugi=debugi<<1;
	//cout << "Debug i=" << debugi << endl;
	//return debugi;
}
void Processelement(ELEMENT* elm, long handle, unsigned int client_id, char* buffer)
{	
	/*typedef struct {
	double posx_ROS;
	double posy_ROS;
	float  theta;
	unsigned short UID;
	unsigned short next_id;
	unsigned short previous_id;
}ELEMENT;*/
	//get element id(unsigned short)(buffer[4]<<8) | (unsigned short)(buffer[3]);
	//unsigned int UID_hold = (unsigned short)(buffer[4]) | (unsigned short)(buffer[3])
	//double UID = UID_hold;

	(*elm).UID = (unsigned short)(buffer[1]<<8) | (unsigned short)(buffer[0]); //scaleToUInt16(UID_hold, -100000, 100000);
	//get position X
	(*elm).next_id =  (unsigned short)(buffer[3]<<8) | (unsigned short)(buffer[2]);
	(*elm).previous_id =  (unsigned short)(buffer[5]<<8) | (unsigned short)(buffer[4]);
	//unsigned int id_check = (unsigned short)(buffer[12]<<8) | (unsigned short)(buffer[11]);
	//if( id_check == JAUS_ID_SetLocalWaypoint){

	unsigned int X_hold = (unsigned short)(buffer[15]&0xFF)<<24 | (unsigned short)(buffer[14]&0xFF)<<16 |  (unsigned short)(buffer[13]&0xFF)<<8 |  (unsigned short)(buffer[12]&0xFF);
	double X = X_hold;	
	(*elm).posx = UInt32ToScale(X, -100000, 100000);
	unsigned int Y_hold = (unsigned short)(buffer[19]&0xFF)<<24 | (unsigned short)(buffer[18]&0xFF)<<16 |  (unsigned short)(buffer[17]&0xFF)<<8 |  (unsigned short)(buffer[16]&0xFF);
	double Y = Y_hold;	
	(*elm).posy = UInt32ToScale(Y, -100000, 100000);
			//
	printf("elm.x = %f elm.y = %f\n", (*elm).posx,(*elm).posy );

	/*unsigned int Z_hold = (unsigned short)(buffer[18]&0xFF)<<24 | (unsigned short)(buffer[17]&0xFF)<<16 |  (unsigned short)(buffer[16]&0xFF)<<8 |  (unsigned short)(buffer[15]&0xFF);
	double Z = Z_hold;	
	*elm.posz= scaleToUInt32(Z, -100000, 100000);*/
	//unsigned short yaw_hold = (unsigned short)(buffer[22]<<8) | (unsigned short)(buffer[21]);
	//(*elm).theta = UInt16ToScale(yaw_hold, -1*PI, PI);
		
}

void confirm_element_request(long handle, unsigned int client_id, unsigned char req_id)
{	
	CONFIRM_SET_ELEMENT_MESSAGE msg;
	msg.msg_id = ConfirmElementRequest;
	msg.request_id_ret = req_id;
	 if (JrSend(handle, client_id, sizeof(msg), (char*)&msg) != Ok)
        printf("Unable to send confirm_element_request message.  Need more debug here...\n");
		else printf("Sent message confirm element request\n");
}
double UInt32ToScale(double val, double low, double high)
{
	return (((val*(high - low))/(UInt32Range)) + low);
}
//============================
double UInt16ToScale(double val, double low, double high)
{
	return (((val*(high - low))/(UInt16Range)) + low);
}

void SEND_REPORT_ELEMENT_LIST(int list_ind, long handle, unsigned int client_id, ELEMENT* elm){
	REPORT_ELEMENT_LIST msg;
	msg.msg_id = ReportElementList;
	msg.count_field = 4; //we know there will be four waypoints.
	//char* point = (char*)&msg;
	//msg.UID[50]; //estimate for upper bound on the number of elements
	for(int i = 0; i<list_ind;i++){
		msg.UID[i] = 0;
		}
	for(int i = 0; i<list_ind;i++){
		msg.UID[i] = (*elm).UID;
		elm++;
	printf("element %i put in list\n", msg.UID[i]);
		}
	if (JrSend(handle, client_id, (sizeof(msg) + list_ind*2), (char*)&msg) != Ok)
        printf("Unable to send System ID message.  Need more debug here...\n");
		else printf("Sent message report element list\n");
	
}
void SEND_REPORT_ELEMENT_COUNT(long handle, unsigned int client_id, int list_ind){
	REPORT_ELEMENT_COUNT msg;
	msg.msg_id = JAUS_ID_ReportElementCount;
	msg.count = 4;//(list_ind-1);//hard code a value of four cause that's what we will get.
	if (JrSend(handle, client_id, sizeof(msg), (char*)&msg) != Ok)
        printf("Unable to send System ID message.  Need more debug here...\n");
		else printf("Sent message report element count\n");
}
void SEND_REPORT_ACTIVE_ELEMENT(long handle,unsigned int COP_JausID, ELEMENT* elm, int active_elm){
	elm = elm + active_elm; //add to the base of the array to get the current element	
	REPORT_ACTIVE_ELEMENT msg;
	msg.msg_id= JAUS_ID_ReportActiveElement;
	msg.active_id= (*elm).UID;
	if (JrSend(handle, COP_JausID, sizeof(msg), (char*)&msg) != Ok)
        printf("Unable to send System ID message.  Need more debug here...\n");
		else printf("Sent message Report active element\n");
}

void SEND_REPORT_TRAVEL_SPEED(long handle, unsigned int client_id, double VX){
	REPORT_TRAVEL_SPEED msg;
	msg.msg_id = JAUS_ID_ReportTravelSpeed;
	msg.speed = scaleToUInt16(VX,0, 327.67 );
	printf("REPORT_TRAVEL_SPEED=%f m/s, [msg.speed = %x]\n",VX, msg.speed);
	if (JrSend(handle, client_id, sizeof(msg), (char*)&msg) != Ok)
        printf("Unable to send SREPORT_TRAVEL_SPEED message.  Need more debug here...\n");
		else printf("Sent message REPORT_TRAVEL_SPEED\n");
}

void SEND_REPOT_LOCAL_WAYPOINT(long handle, unsigned int client_id, ELEMENT* elm, int active_elm){
	elm = elm + active_elm;
	REPORT_LOCAL_WAYPOINT msg;
	msg.msg_id = JAUS_ID_ReportLocalWaypoint;
	msg.pv =3; //dictated by the rules, wrong though
	msg.X = scaleToUInt32((*elm).posx, -100000, 100000);
	msg.Y = scaleToUInt32((*elm).posy, -100000, 100000);
	if (JrSend(handle, client_id, sizeof(msg), (char*)&msg) != Ok)
        printf("Unable to send SEND_REPOT_LOCAL_WAYPOINT message.  Need more debug here...\n");
		else printf("Sent message SEND_REPOT_LOCAL_WAYPOINT \n");
}

double wrapTo2PI(double angle) // convert abgle to range [0~2*PI]
{
double a=fmod(angle , 2*PI); // a is [-2*PI~2*PI]
if (a<0) return (a+2*PI);
else return a;
}
inline double normalize(double angle) // convert abgle to range [-PI ~PI]
{
    return (wrapTo2PI(angle+M_PI) - M_PI); 
}


inline double angle_add(double pa,double pa_return)
{

	return normalize(pa+pa_return) ;
}



void translate_ROS_to_JAUS(double X_in, double Y_in, double W_in, double X_home_ROS, double Y_home_ROS, double theta_home_ROS, double * X_out, double * Y_out, double * W_out)
{
	double theta_to_local = atan2(-Y_home_ROS, -X_home_ROS);
	//theta_to_local = anle_add(Yaw_home_ROS, -theta_to_local);
	theta_to_local = theta_to_local - Yaw_home_ROS;
	//theta_to_local = no
	double d = sqrt(pow(X_home_ROS,2) + pow(Y_home_ROS,2));
	double X_back = d*cos(theta_to_local);
	double Y_back = d*sin(theta_to_local);
	
	
	*X_out = cos(theta_home_ROS)*X_in + sin(theta_home_ROS)*Y_in + X_back;
	*Y_out = -1*((-sin(theta_home_ROS)*X_in +cos(theta_home_ROS)*Y_in) + Y_back); // -1 is for rotate 180 degree in x axis , Y=-Y
	*W_out = normalize(-1*( W_in-theta_home_ROS));
	
/*
ROS=[2,2,0] Home_ROS=[0,0,0],    JAUS=[2    ,-2,0]
ROS=[2,2,0] Home_ROS=[0,0,PI/4], JAUS=[2.428, 0,0]
ROS=[2,2,0] Home_ROS=[1,1,PI/4], JAUS=[1.416, 0,0]



*/
}
void translate_JAUS_to_ROS(double X_in, double Y_in, double X_home_ROS, double Y_home_ROS, double theta_home_ROS, double * X_out, double * Y_out)
{
	Y_in=-Y_in; // have to rotate 180 degree on x axis, Y=-Y
	*X_out = cos(theta_home_ROS)*X_in - sin(theta_home_ROS)*Y_in + X_home_ROS;
	*Y_out = ((sin(theta_home_ROS)*X_in + cos(theta_home_ROS)*Y_in) + Y_home_ROS);

	//W_out= normalize(theta_home_ROS-W_in) ;

}

double YawJAUS_to_YawROS(double W_in, double theta_home_ROS)
{
	//W_out= normalize(theta_home_ROS-W_in) ;
	return normalize(theta_home_ROS-W_in);
}

move_base_msgs::MoveBaseGoal ROSsetGoal(double x,double y,double w)
{
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.orientation.w = w;
  ROS_INFO("Sending goal");
  return goal;
// ac.sendGoal( ROSsetGoal(double x,double y,double w) );
  //ac.sendGoal(goal);
  /*ac.waitForResult();
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved 1 meter forward");
  else
    ROS_INFO("The base failed to move forward 1 meter for some reason"); */
}

void Odom_Callback(const nav_msgs::Odometry::ConstPtr& odom)
{
//  ROS_INFO("I received odom: [%f,%f]", odom->pose.pose.position.x, odom->pose.pose.position.y);
  tf::Pose pose;
  tf::poseMsgToTF(odom->pose.pose, pose);
  posx_ROS=odom->pose.pose.position.x;
  posy_ROS=odom->pose.pose.position.y;
  posz_ROS=odom->pose.pose.position.z;
  posyaw_ROS = tf::getYaw(pose.getRotation());

  translate_ROS_to_JAUS(posx_ROS, posy_ROS, posyaw_ROS, X_home_ROS, Y_home_ROS ,Yaw_home_ROS, &posX_JAUS, &posY_JAUS, &posYaw_JAUS);
  posz_ROS=-(posz_ROS-Z_home_ROS);			

  posSpeed_JAUS  =   odom->twist.twist.linear.x;
  posYawRate_JAUS= -(odom->twist.twist.angular.z); // need to double check this, the rotation direction

//  ROS_INFO("ROS:[%5.2f,%5.2f,%5.2f],JAUS:[%5.2f,%5.2f,%5.2f],Speed:[%5.2f,%5.2f]", posx_ROS,posy_ROS,posyaw_ROS,posX_JAUS,posY_JAUS,posYaw_JAUS,posSpeed,posYawRate);

}


/*
ROS=[2,2,0] Home_ROS=[0,0,0],    JAUS=[2    ,-2,0]
ROS=[2,2,0] Home_ROS=[0,0,PI/4], JAUS=[2.828, 0,0]
ROS=[2,2,0] Home_ROS=[1,1,PI/4], JAUS=[1.416, 0,0]
ROS=[-2,-2,0] Home_ROS=[0,0,-PI/4], JAUS=[0, 2.828,0]
ROS=[-2,-2,0] Home_ROS=[1,1,-PI/4], JAUS=[0, 4.24,0]

// just for debug test
posx_ROS  =2; posy_ROS  =2; posyaw_ROS  =0.1; 
X_home_ROS=0; Y_home_ROS=0; Yaw_home_ROS=0;
  translate_ROS_to_JAUS(posx_ROS, posy_ROS, posyaw_ROS, X_home_ROS, Y_home_ROS ,Yaw_home_ROS, &posX_JAUS, &posY_JAUS, &posYaw_JAUS);			

ROS_INFO("Debug    ROS:[%5.2f,%5.2f,%5.2f],JAUS:[%5.2f,%5.2f,%5.2f]", posx_ROS,posy_ROS,posyaw_ROS,posX_JAUS,posY_JAUS,posYaw_JAUS);


translate_JAUS_to_ROS(posX_JAUS, posY_JAUS,X_home_ROS, Y_home_ROS ,Yaw_home_ROS, &posx_ROS, &posy_ROS);
ROS_INFO("DebugJ2R ROS:[%5.2f,%5.2f,%5.2f],JAUS:[%5.2f,%5.2f,%5.2f]", posx_ROS,posy_ROS,posyaw_ROS,posX_JAUS,posY_JAUS,posYaw_JAUS);
[ INFO] [1357450809.586152135, 4197.575000000]: Debug    ROS:[ 2.00, 2.00, 0.10],JAUS:[ 2.00,-2.00,-0.10]
[ INFO] [1357450809.586215760, 4197.575000000]: DebugJ2R ROS:[ 2.00, 2.00, 0.10],JAUS:[ 2.00,-2.00,-0.10]
[ INFO] [1357450809.586278480, 4197.575000000]: Debug    ROS:[ 2.00, 2.00, 0.20],JAUS:[ 2.83,-0.00, 0.59]
[ INFO] [1357450809.586340910, 4197.575000000]: DebugJ2R ROS:[ 2.00, 2.00, 0.20],JAUS:[ 2.83,-0.00, 0.59]
[ INFO] [1357450809.586422723, 4197.575000000]: Debug    ROS:[ 2.00, 2.00,-0.10],JAUS:[ 1.41,-0.00, 0.89]
[ INFO] [1357450809.586485497, 4197.575000000]: DebugJ2R ROS:[ 2.00, 2.00,-0.10],JAUS:[ 1.41,-0.00, 0.89]
[ INFO] [1357450809.586547739, 4197.575000000]: Debug    ROS:[-2.00,-2.00,-0.20],JAUS:[-0.00, 2.83,-0.59]
[ INFO] [1357450809.586609049, 4197.575000000]: DebugJ2R ROS:[-2.00,-2.00,-0.20],JAUS:[-0.00, 2.83,-0.59]
[ INFO] [1357450809.586670730, 4197.575000000]: Debug    ROS:[-2.00,-2.00,-0.79],JAUS:[-0.00, 4.24,-0.00]
[ INFO] [1357450809.586733013, 4197.575000000]: DebugJ2R ROS:[-2.00,-2.00,-0.79],JAUS:[-0.00, 4.24,-0.00]


// end of debug

*/


// Use different Thread to spin(); (get message) forever
void spinThread()
{
    ros::spin();
}


void Print_JAUS_Status(void)
{
SaveCursor();
gotoxy(1,6);
	printf("                                                                                \r\n");
	printf("                                                                                \r\n");
	printf("                                                                                \r\n");
	printf("                                                                                \r\n");
	printf("                                                                                \r\n");
	printf("                                                                                \r\n");
	printf("                                                                                \r\n");
	printf("                                                                                \r\n");
//printf("                                                                                \r\n");
//printf("                                                                                \r\n");
//printf("                                                                                \r\n");
//printf("                                                                                \r\n");
//printf("                                                                                \r\n");
gotoxy(1,1);



	printf("COP   JAUS ID:%08X (%d-%d-%d)              \r\n",COP_JausID,COP_JausID>>16,(COP_JausID&0xFF00)>>8,COP_JausID&0xFF); // print JAUS_ID
	printf("Robot JAUS ID:%08X (%d-%d-%d)              \r\n",Robot_JausID,Robot_JausID>>16,(Robot_JausID&0xFF00)>>8,Robot_JausID&0xFF); // print JAUS_ID

	printf("ROS goal[%i]: [%+6.2f,%+6.2f ]                \r\n",active_elm, X_goal_ROS, Y_goal_ROS);
	printf("ROS     pose: [%+6.2f,%+6.2f,%+6.2f,%+6.1f degree];  \r\n", posx_ROS,posy_ROS,posz_ROS,posyaw_ROS*180/PI);
	printf("ROS_HOMEpose: [%+6.2f,%+6.2f,%+6.1f degree];  \r\n", X_home_ROS, Y_home_ROS ,Yaw_home_ROS*180/PI);
	printf("JAUS    pose: [%+6.2f,%+6.2f,%+6.2f,%+6.1f degree];  \r\n", posX_JAUS,posY_JAUS,posZ_JAUS,posYaw_JAUS*180/PI);
	printf("JAUS   Speed: [%+6.2f m/s, %+6.2f rad/s]      \r\n", posSpeed_JAUS,posYawRate_JAUS);
	printf("Max Run Time Speed : [%+6.2f m/s]      \r\n", max_run_time_speed);
//	gotoxy(42,1);
//	gotoxy(42,2);	
	printf("Waypoint[0]: JAUS(X,Y)= (%6.2f, %6.2f)\r\n",elm_list[0].posx, elm_list[0].posy);
//	gotoxy(42,3);	
	printf("Waypoint[1]: JAUS(X,Y)= (%6.2f, %6.2f)\r\n",elm_list[1].posx, elm_list[1].posy);
//	gotoxy(42,4);	
	printf("Waypoint[2]: JAUS(X,Y)= (%6.2f, %6.2f)\r\n",elm_list[2].posx, elm_list[2].posy);
//	gotoxy(42,5);	
	printf("Waypoint[3]: JAUS(X,Y)= (%6.2f, %6.2f)\r\n",elm_list[3].posx, elm_list[3].posy);
	
	
//	printf("COP State:%2i\n", state);
//	printf("Robot (X,Y):(%8.4f,%8.4f); Yaw=%8.2f degree\n", robot_x,robot_y,robot_yaw*180/PI);
//	printf("       Vx=%8.4f m/s, Va=%8.4f rad/s\n", robot_v,robot_w);

	
	
	
RestoreCursor();
}

//============================================
// Terminal control code
void gotoxy(int x,int y)
{
 printf("%c[%d;%df",0x1B,y,x);
}
void SaveCursor(void)
{
 printf("%c[s",0x1B);
}
void RestoreCursor(void)
{
 printf("%c[u",0x1B);
}


