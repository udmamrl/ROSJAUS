/*! 
 ***********************************************************************
 * file      WaypointCOP.cpp
 * author    Jason Osborne. University of Detroit Mercy 
 *            Advanced mobile Robotics Lab    
 * date      May 2010
 *
 ************************************************************************
 */
 #define DWT (-4)
#include <string>
#include "JuniorAPI.h"
#include <list>
//#include <stream.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <iostream>
#include "JAUSwaypoint.h" 

// Must put JAUS stuff in the end
#include "JAUSmessage.h"
#include "JAUS_MESSAGE_ID.h"

using namespace std;
 
 
// Define the maximum message size we can support
const unsigned short MaxMsgSize = 500;
 
// Define a signal handler, so we can clean-up properly
static int exit_flag = 0;
static void handle_exit_signal( int signum )
{
    exit_flag = 1;
}
//
// The Juas Common Operation Picture (COP) program listens for ID 
// requests, and prints incoming messages.
//
// char  8bit
// short 16bit
// int   32bit
// long  64bit
void QUERY_VELOCITY_STATE(long handle,unsigned int client_id);
void SEND_QUERY_SERVICES(long handle,unsigned int client_id);
void SEND_QUERY_IDENTIFICATION(long handle,unsigned int client_id);
void SEND_REPORT_IDENTIFICATION(long handle,unsigned int client_id);
void SEND_QUERY_CONTROL(long handle,unsigned int client_id);
void SEND_REQUEST_CONTROL(long handle,unsigned int client_id);
//void SEND_RESUME(long handle,unsigned int client_id);
void SET_LOCAL_POSE(long handle,unsigned int client_id);
void QUERY_VELOCITY_STATE(long handle,unsigned int client_id);
void SEND_QUERY_STATUS(long handle,unsigned int client_id);
void SEND_RESUME(long handle,unsigned int client_id);
void SHUTDOWN(long handle,unsigned int client_id);
double UInt32ToScale(double val, double low, double high);
double UInt16ToScale(double val, double low, double high);
void QUERY_LCAOL_POSE(long handle,unsigned int client_id);
unsigned long int GetTimeStamp(int x);
unsigned int GenerateJausID(unsigned int subsystemID,unsigned int nodeID,unsigned int componentID);
void ProcessJausMessage(long handle,unsigned int COP_JausID,unsigned int MessageID,unsigned int* source,unsigned int* bufsize, char* buffer);
void send_set_element(long handle, unsigned int client_id, double X, double Y, unsigned short element_id, unsigned short previous_id, unsigned short next_id, unsigned char req_id);
void send_query_element_list(long handle,unsigned int client_id);
void send_query_element_count(long handle,unsigned int client_id);
void send_execute_list(long handle,unsigned int client_id);
void send_query_active_element(long handle,unsigned int client_id);
void send_query_travel_speed(long handle,unsigned int client_id);
void send_query_waypoints(long handle,unsigned int client_id);
unsigned int scaleToUInt32(double val, double low, double high);
unsigned short scaleToUInt16(double val, double low, double high);
void Print_JAUS_Status(void);
void gotoxy(int x,int y);
void SaveCursor(void);
void RestoreCursor(void);
 
unsigned char VehicleStatus= 0 ; // init
bool VehicleControled = 1;
unsigned int client_id;
bool QCloop = 0;
bool Vloop = 0;
bool Ploop = 0;
bool done = 0;
unsigned int t1;
int state = 0;
bool discovered = false;
unsigned short Active_element;
unsigned int COP_id ;
double robot_x,robot_y,robot_yaw,robot_v,robot_w;


int main(int argc, char* argv[]){
		short prev_state = 0;

		short watch_dog = 0;
    JrErrorCode ret;
    int loop = 0; 
    char buffer[MaxMsgSize]; 
    unsigned int buffersize;
    unsigned int sender;
    unsigned short msg_id;
    srand((unsigned long)(time(0)));
 
    unsigned long int x = GetTimeStamp(1);
    printf("time is %lu\n",x);
 
    // Initiate a connection to the Junior Run-Time Engine.
    // We need to use the returned handle in all subsequent calls.
    long handle;
// COP ID 42-1-1 = 0x2A 01 01
    COP_id = GenerateJausID( 42,1,1) ;  // 42-1-1
  	printf("JAUS COP ID:%08X (%d-%d-%d)\r\n",COP_id,COP_id>>16,(COP_id&0xFF00)>>8,COP_id&0xFF); // print message ID
 
    // Pull the config file from the command line arguments
	  std::string config_file = "/usr/bin/jr_config.xml"; // default configure file
    if (argc >= 2) config_file = std::string(argv[1]);
 
    if (JrConnect(COP_id, config_file.c_str() , &handle) != Ok)
    {
        printf("Init failed.  Terminating execution\n");
        return -1;
    }
 
    // Catch the termination signals
    signal( SIGINT, handle_exit_signal );
    signal( SIGTERM, handle_exit_signal );
    signal( SIGABRT, handle_exit_signal );
 
    // Every loop, handle incoming messages.
    while(!exit_flag)
    { 
    		prev_state = state;
    		//printf("prev_state = %i\n",prev_state);
        buffersize = MaxMsgSize;
        ret = JrReceive(handle, &sender, &buffersize, buffer, NULL, NULL, &msg_id);
            //                        NULL, NULL, &msg_id);
        if (ret == Ok)
        {
		      //print the recieved message type and the size of the message
		      printf("Message type:%4X , %d bytes received. ",msg_id,buffersize);
		      //print the subsystem ID, Node ID and component ID
		      printf("From :%8X ( %d-%d-%d )\n",sender,sender>>16,(sender&0xFF00)>>8,sender&0xFF);
		      //get the id of the sender
		      unsigned int subsystem_id = (int)sender>>16;
		      client_id = GenerateJausID(subsystem_id,1,1) ; 
		      //printf(" The subsystem ID of the sender %d\n",client_id);
		 
		      // printf("%X\n", std::string(buffer, buffersize).c_str());
		      if (buffersize==2) 
		      {
		        printf("MessageID:%02X%02X \n ",buffer[1]&0xFF,buffer[0]&0xFF); // print message ID
		      } 
		      else if (buffersize<2) 
		      {
		      	printf("Data(Hex): ");
		      	for (int i=0;i<buffersize;i++)  printf("%02X ",buffer[i]&0xFF); printf("\n");
		      } 
		      else
		      {
		        printf("MessageID:%02X%02X , Data: ",buffer[1]&0xFF,buffer[0]&0xFF); // print message ID
		      	for (int i=2;i<buffersize;i++)  printf("%02X ",buffer[i]&0xFF); printf("\n"); // Print DATA
		        unsigned short MessageID=(unsigned short)(buffer[1]&0xFF)<<8 | (unsigned short)(buffer[0]&0xFF);
		        ProcessJausMessage(handle,client_id,MessageID, &sender, &buffersize, &buffer[0]);
		      }
       	}
				int req_id =1;
				    
				switch (state)   // IGVC JAUS state machine
				  {
				  
				  case 0  : //wait for query identification
				  printf("waiting for discovery\n");
				  //state++;
				  usleep(1000000); // 1 second delay
				  break;
				  
				  case 1 : //wait for report id
				  printf("wait for report id\n");
				  usleep(1000000); // 1 second delay
				  break;
				  
				  case 2  : //Query services
				  printf("Asked for services\n");
				  SEND_QUERY_SERVICES(handle, client_id);
				  state++;
				  usleep(1000000); // 1 second delay  
				  case 3 : //waiting for services
				  printf("Waiting for services\n");
				  usleep(1000000); // 1 second delay
				  break;
				
				  case 4 : //Query control
				  SEND_QUERY_CONTROL(handle,client_id);
				  printf("sent query control\n");
				  state++;
				  usleep(1000000); // 1 second delay
				  break;
				 
				  case 5 : //have not yet received the report control response
				  printf("waiting for control\n");
				  usleep(1000000); // 1 second delay
				  break;
				
				  case 6 : //send resume
				  SEND_RESUME(handle,client_id);
				  SEND_QUERY_STATUS(handle,client_id);
				  state++;
				  printf("send resume message\n");
				  usleep(1000000); // 1 second delay
				  break;
				  
				  case 7: //get the status message
				  printf("waiting to hear confirm of ready\n");
				  usleep(1000000); // 1 second delay
				  break;
				
				  case 8://
				  printf("send set local pose message\n");
				  SET_LOCAL_POSE(handle,client_id);
				  state++;
				  usleep(1000000); // 1 second delay
				  break;
				//void send_set_element(long handle, unsigned int client_id, double X, double Y, unsigned char pv,unsigned short element_id, unsigned short previous_id, unsigned short next_id, unsigned char req_id)
				  case 9 : //we have control, so send the first element 
				  send_set_element(handle, client_id,X1, Y1 , 1, 0, 2, 1);
				  //req_id++;
				  state++;
				  usleep(1000000); // 1 second delay
				  break;
				 
				  case 10 ://wait to get the confirm element message
				  printf("waiting to hear confirm element\n");
				  usleep(1000000); // 1 second delay
				  break;
				  
				  case 11 ://send the next element
				  send_set_element(handle, client_id,X2, Y2 , 2, 1, 3, 2);
				  req_id++;
				  state++;//printf("waiting to hear confirm element);
				  usleep(1000000); // 1 second delay
				  break;
				
				  case 12 ://wait to get the confirm element message
				  printf("waiting to hear confirm element\n");
				  break;  
				
				  case 13 ://send the next element
				  send_set_element(handle, client_id,X3,Y3, 3, 2, 4, 3);
				  req_id++;
				  usleep(1000000); // 1 second delay
				  //printf("waiting to hear confirm element);
				  state++;
				  break;
				
				  case 14 ://wait to get the confirm element message
				  printf("waiting to hear confirm element\n");
				  break;
				  
				  case 15 ://send the next element
				  send_set_element(handle, client_id,X4,Y4, 4, 3, 5, 4);
				  req_id++;
				  usleep(1000000); // 1 second delay
				  //printf("waiting to hear confirm element);
				  state++;
				  break;
				
				  case 16 ://wait to get the confirm element message
				  printf("waiting to hear confirm element\n");
				  break;
				
				  case 17  : //all elements have been sent so test the other messages
				  send_query_element_list(handle,client_id);
				  printf("sent query element list\n");
				  state++;
				  usleep(1000000); // 1 second delay
				  break;
				  
				  case 18 ://wait to get the element list
				  printf("waiting to hear element list\n");
				  //state++;
				  break;  
				
				  case 19  : //all elements have been sent so test the other messages
				  send_query_element_count(handle,client_id);
				  state++;
				  break;  
				  
				  case 20 ://wait to get the element "list
				  printf("waiting to hear element count\n");
				  break;  
				
				  case 21 : 
				  send_execute_list(handle,client_id);
				  state++;
				  break;
				  
				  case 22 : //all elements have been sent so test the other messages
				  send_query_active_element(handle,client_id);
				  state++;
				  break;
				
				  case 23  : //wait to hear active element
				  printf("waiting to hear active element\n");
				  break;
				  
				  case 24  : //wait to hear speed
				  send_query_travel_speed(handle,client_id);
				  state++;
				  break;
				
				  case 25  : //wait to hear travel speed
				  printf("waiting to hear travel speed\n");
				  break;
				
				  case 26  :// query waypoints
				  send_query_waypoints(handle,client_id);
				  state++;
				  break;    
				
				  case 27  : //waiting to hear waypoints send back
				  printf("waiting to hear local waypoint\n");
				  break;
				
				  case 28  : //Query Velocity State
				  QUERY_VELOCITY_STATE(handle, client_id);
				  state++; 
				  printf("Query Velocity State sent\n");
				  break;
				
				  case 29  : //Waiting for Velocity message
				  printf("Waiting for Velocity message\n");
				  break;
				  
				
				  case 30  : // 
				  
				  state++; loop++;  //increment loop to use to break out.
				  //printf("loop = %i\n", loop);
				  break;
				
				  case 31  : //query_local pose
				  QUERY_LCAOL_POSE(handle, client_id);
				  state++;
				  break;
				
				  case 32 : //waiting for pose message
				  printf("waiting for local pose message\n");
				  break;
				
				  case 33 :
				  state = 22;
				  break;
				
				  default  : printf("oops, state = %i\n", state);
				  loop++;
				  break;  
				  }
				printf("============[Waypoint-COP]==State:%2i;Loop=%2i=====================\n", state, loop);
				  if(loop >= 600) {
				    printf("Max loop = %i reached, Shutdown JAUS-COP \n", loop); 
				    SHUTDOWN(handle, client_id); 
				    JrDisconnect(handle);
				    printf("Exit.... Disconnect Jr....\r\n");
				    return 0;}

				  if(prev_state == state){
				  watch_dog++; printf("watch dog = %i\n",watch_dog);
				    if(watch_dog >= 100 ){ 
				    	watch_dog = 0;
				      if (state >0)  state--;
				    }
				  }
				  else{watch_dog = 0;}
				  
				 Print_JAUS_Status(); // Test Terminal print function
				 
				 //usleep(1000000); // 1 second delay
				 usleep(100000); // .1 second delay
  	}
  SHUTDOWN(handle, client_id); 
  JrDisconnect(handle);
  printf("Exit.... Disconnect Jr....\r\n");
      return 0;

return 0;
}
 
 
void SEND_REPORT_IDENTIFICATION(long handle,unsigned int client_id){
  REPORT_IDENTIFICATION_MSG idmsg;
  idmsg.msg_id=0x4B00;
  idmsg.QueryType=2; // subsystem
  idmsg.Type= 10001; // Vehicle

  strcpy(idmsg.Name,"JAUS-COP");
  idmsg.StringLength=strlen(idmsg.Name);

  //printf("Message: Data Send [%s] Size:%li\n",((char*)&idmsg)+6,sizeof(idmsg));
long int msg_size=sizeof(idmsg)-Max_IDENTIFICATION_StringLength+strlen(idmsg.Name);
  
  
  
    if (JrSend(handle, client_id, msg_size, (char*)&idmsg) != Ok)
        printf("Unable to send System ID message. Need more debug here...\n");
    else printf("Sent Report Identification to the client ID\n");
 
}
 
unsigned int GenerateJausID(unsigned int subsystemID,unsigned int nodeID,unsigned int componentID){
  return ( subsystemID<<16 | nodeID <<8 | componentID );
}
//================================================
 
void SEND_QUERY_CONTROL(long handle,unsigned int client_id){
  // Query Identification
  QUERY_CONTROL_MSG qcmsg;
  qcmsg.msg_id=0x200D;
    if (JrSend(handle, client_id,sizeof(qcmsg), (char*)&qcmsg) != Ok)
        printf("Unable to send SEND_QUERY_CONTROL message.  Need more debug here...\n");
    else printf("Sent Quert control to the entry\n");
 
}
 
//================================================
 
void SHUTDOWN(long handle,unsigned int client_id){
  SHUTDOWN_MSG smsg;
  smsg.msg_id = 0x0002;
  if (JrSend(handle, client_id,sizeof(smsg), (char*)&smsg) != Ok)
        printf("Unable to send SHUTDOWN_MSG message.  Need more debug here...\n");
    else printf("Sent message SHUTDOWN_MSG to Robot\n");
 
}
 
//================================================
void SEND_QUERY_IDENTIFICATION(long handle,unsigned int client_id){
  // Query Identification
  QUERY_IDENTIFICATION_MSG qimsg;
  qimsg.msg_id=0x2B00;
  qimsg.QueryType = 0x02; 
    if (JrSend(handle, client_id,sizeof(qimsg), (char*)&qimsg) != Ok)
        printf("Unable to send Query IDentification message.  Need more debug here...\n");
    else printf("Sent message Query IDentification to Robot\n");
 
}
 
//================================================
void SEND_REQUEST_CONTROL(long handle,unsigned int client_id){
  REQUEST_CONTROL_MSG rcmsg;
  rcmsg.msg_id=0x000D;
  rcmsg.AuthorityCode=255; // subsystem
  /*
  printf("Message: Data Send [%s] Size:%li\n",((char*)&rcmsg),sizeof(rcmsg)); 
  printf("DataSend(Hex): ");
  for (int i=0;i<sizeof(rcmsg);i++)  printf("%02X ",(*((char*)&rcmsg+i))&0xFF); printf("\n");
  */
  
  
    if (JrSend(handle, client_id, sizeof(rcmsg), (char*)&rcmsg) != Ok)
        printf("Unable to send request for control\n");
    else printf("Sent message request control to Robot\n");
 
}
//================================================
void SET_LOCAL_POSE(long handle,unsigned int client_id){
  SET_LOCAL_POSE_MSG rlpmsg;
  rlpmsg.msg_id = JAUS_ID_SetLocalPose;
  // X+Y+X+Yaw bits=0x47
  rlpmsg.pv = LOCAL_POSE_PVBit_X+LOCAL_POSE_PVBit_Y+LOCAL_POSE_PVBit_Z+LOCAL_POSE_PVBit_Yaw;
  
  rlpmsg.X = scaleToUInt32(0,-100000,100000);
  rlpmsg.Y = scaleToUInt32(0,-100000,100000);
  rlpmsg.Z = scaleToUInt32(0,-100000,100000);
  rlpmsg.Yaw = scaleToUInt16(0,-1*PI,PI);
  //rlpmsg.TimeStamp = GetTimeStamp(1);
  /*
  printf("Message: Data Send [%s] Size:%li\n",((char*)&rlpmsg)+6,sizeof(rlpmsg));
  */
    if (JrSend(handle, client_id, sizeof(rlpmsg), (char*)&rlpmsg) != Ok)
        printf("Unable to send SET_LOCAL_POSE message.  Need more debug here...\n");
    else printf("Sent SET_LOCAL_POSE\n");
}
//===============================================
void SEND_QUERY_STATUS(long handle,unsigned int client_id){
  QUERY_STATUS_MSG qsmsg;
  qsmsg.msg_id = 0x2002;
  /*
  printf("Message: Data Send [%s] Size:%li\n",((char*)&qsmsg)+6,sizeof(qsmsg));
  */
    if (JrSend(handle, client_id, sizeof(qsmsg), (char*)&qsmsg) != Ok)
        printf("Unable to send SEND_QUERY_STATUS message.  Need more debug here...\n");
    else printf("Sent message SEND_QUERY_STATUS to Robot\n");
 
}
//================================================
 
void SEND_RESUME(long handle,unsigned int client_id){
  RESUME_MSG  rmsg;
  rmsg.msg_id = 0x0004;
  /*
  printf("Message: Data Send [%s] Size:%li\n",((char*)&rmsg),sizeof(rmsg));
  */
    if (JrSend(handle, client_id, sizeof(rmsg), (char*)&rmsg) != Ok)
        printf("Unable to send SEND_RESUME message.  Need more debug here...\n");
    else printf("Sent message SEND_RESUME to Robot\n");
 
}
 
//===============================================
void QUERY_LCAOL_POSE(long handle,unsigned int client_id){
  QUERY_LCAOL_POSE_MSG qlpmsg;
  qlpmsg.msg_id = 0x2403;
  //qlpmsg.pv = 0x015c;
  qlpmsg.pv = 0x0147; // IGVC2013 change
  //printf("Message: Data Send [%s] Size:%i\n",((char*)&qlpmsg),sizeof(qlpmsg));
    if (JrSend(handle, client_id, sizeof(qlpmsg), (char*)&qlpmsg) != Ok)
        printf("Unable to send System ID message.  Need more debug here...\n");
    else printf("Sent query local pose\n");
 
}
//==============================================
 
//===============================================
/*
void Query_LOCAL_CONTROL(long handle,unsigned int client_id)
{
  QUERY_LCAOL_POSE_MSG qlpmsg;
  qlpmsg.msg_id = 0x2403;
  qlpmsg.pv = 0x015c;
  printf("Message: Data Send [%s] Size:%i\n",((char*)&rmsg)+6,sizeof(rmsg));
    if (JrSend(handle, client_id, sizeof(rmsg), (char*)&rmsg) != Ok)
        printf("Unable to send System ID message.  Need more debug here...\n");
    else printf("Sent message System ID to Robot\n");
 
}
*/
///////////////////////////////////////////////////
void QUERY_VELOCITY_STATE(long handle,unsigned int client_id){
  QUERY_VELOCITY_STATE_MSG qvmsg;
  qvmsg.msg_id = 0x2404;
  qvmsg.pv = 0x01; // IGVC2013
  //printf("Message: Data Send [%s] Size:%i\n",((char*)&qvmsg),(sizeof(qvmsg)));
    if (JrSend(handle, client_id, sizeof(qvmsg), (char*)&qvmsg) != Ok)
        printf("Unable to send QUERY_VELOCITY_STATE_MSG message.  Need more debug here...\n");
    else printf("Sent message QUERY_VELOCITY_STATE_MSG to Robot\n");
}
//==============================================
void ProcessJausMessage(long handle, unsigned int client_id,unsigned int MessageID,unsigned int* sender,unsigned int* bufsize, char* buffer){
  switch (MessageID)
  {
    case SetAuthority             :
      printf("Message: SetAuthority              received\r\n"); 
      break; 
    case Shutdown                 :
      printf("Message: Shutdown                  received\r\n"); 
      VehicleStatus=3; // Shutdown
      //exit_flag=1; // Leave program
      printf("Set Vehicle Status to Shutdown ! \r\n"); 
      break; 
    case Standby                  :
      printf("Message: Standby                   received\r\n"); 
      VehicleStatus=2; // Standby
      printf("Set Vehicle Status to Standby ! \r\n"); 
      break; 
    case Resume                   :
      printf("Message: Resume                    received\r\n");
        VehicleStatus=1; // Ready
      printf("Set Vehicle Status to Ready\r\n"); 
      break; 
    case Reset                    :
      printf("Message: Reset                     received\r\n"); 
      VehicleStatus=2; // Standby
      printf("Set Vehicle Status to Standby ! \r\n");       
      break; 
    case SetEmergency             :
      printf("Message: SetEmergency              received\r\n");
      VehicleStatus=5; // Emergency
      printf("Set Vehicle Status to Emergency ! \r\n"); 
      break; 
    case ReportLocalPose             :
    	{
    		unsigned int ReportLocalPose_PV;
	      printf("Message: Report local pose              received\r\n");
	 			ReportLocalPose_PV = (unsigned short)(  (unsigned short)(buffer[3]&0xFF)<<8 |  (unsigned short)(buffer[2]&0xFF));
	 			if(ReportLocalPose_PV==259){
	 			// PV=259 , only X,Y and Time Stamp
	 	      unsigned int X_hold = (unsigned short)(buffer[7]&0xFF)<<24 | (unsigned short)(buffer[6]&0xFF)<<16 |  (unsigned short)(buffer[5]&0xFF)<<8 |  (unsigned short)(buffer[4]&0xFF);
		      unsigned int Y_hold = (unsigned short)(buffer[11]&0xFF)<<24 | (unsigned short)(buffer[10]&0xFF)<<16 |  (unsigned short)(buffer[9]&0xFF)<<8 |  (unsigned short)(buffer[8]&0xFF);
		      //unsigned int Z_hold = (unsigned short)(buffer[15]&0xFF)<<24 | (unsigned short)(buffer[14]&0xFF)<<16 |  (unsigned short)(buffer[13]&0xFF)<<8 |  (unsigned short)(buffer[12]&0xFF);
		      //unsigned int  Yaw_hold = (unsigned short)(buffer[17]&0xFF)<<8 |  (unsigned short)(buffer[16]&0xFF);
		      unsigned int times = (unsigned short)(buffer[15]&0xFF)<<24 | (unsigned short)(buffer[14]&0xFF)<<16 |  (unsigned short)(buffer[13]&0xFF)<<8 |  (unsigned short)(buffer[12]&0xFF);
	          double X = X_hold;
	          double Y = Y_hold;
	          X = UInt32ToScale(X,-100000,100000);
	          Y = UInt32ToScale(Y,-100000,100000);
	          //Z = UInt32ToScale(Z,-100000,100000);
	          //Yaw = UInt16ToScale(Yaw,-1*PI,PI);
	          //printf("yaw = %f\n",Yaw);
	            int msec = (times << 22)>>22;
	            int sec = (times << 16)>>26;
	            int min = (times << 10)>>26;
	            int hour = (times << 5)>>27;
	            int day = (times >>27);
	     printf("Robot(X,Y)=(%8.3f,%8.3f) Day=%2d UTC Time=%2d:%02d:%02d'%03d\n",X,Y,day,hour,min,sec,msec);
        robot_x=X;
        robot_y=Y;
	 			} else {
		      unsigned int X_hold = (unsigned short)(buffer[7]&0xFF)<<24 | (unsigned short)(buffer[6]&0xFF)<<16 |  (unsigned short)(buffer[5]&0xFF)<<8 |  (unsigned short)(buffer[4]&0xFF);
		      unsigned int Y_hold = (unsigned short)(buffer[11]&0xFF)<<24 | (unsigned short)(buffer[10]&0xFF)<<16 |  (unsigned short)(buffer[9]&0xFF)<<8 |  (unsigned short)(buffer[8]&0xFF);
		      unsigned int Z_hold = (unsigned short)(buffer[15]&0xFF)<<24 | (unsigned short)(buffer[14]&0xFF)<<16 |  (unsigned short)(buffer[13]&0xFF)<<8 |  (unsigned short)(buffer[12]&0xFF);
		      unsigned int  Yaw_hold = (unsigned short)(buffer[17]&0xFF)<<8 |  (unsigned short)(buffer[16]&0xFF);
		      unsigned int times = (unsigned short)(buffer[21]&0xFF)<<24 | (unsigned short)(buffer[20]&0xFF)<<16 |  (unsigned short)(buffer[19]&0xFF)<<8 |  (unsigned short)(buffer[18]&0xFF);
	          double X = X_hold;
	          double Y = Y_hold;
	          double Z = Z_hold;
	          double Yaw = Yaw_hold;  
	          X = UInt32ToScale(X,-100000,100000);
	          Y = UInt32ToScale(Y,-100000,100000);
	          Z = UInt32ToScale(Z,-100000,100000);
	          Yaw = UInt16ToScale(Yaw,-1*PI,PI);
	          printf("yaw = %f\n",Yaw);
	            int msec = (times << 22)>>22;
	            int sec = (times << 16)>>26;
	            int min = (times << 10)>>26;
	            int hour = (times << 5)>>27;
	            int day = (times >>27);
	        printf("The local pose info is X = %f, Y = %f, Z = %f, Yaw = %f msec = %d, sec = %d, min = %d, hour = %d, day = %d \n",                 X,Y,Z,Yaw,msec,sec,min,hour,day);
            robot_x=X;
            robot_y=Y;
            robot_yaw=Yaw;

	        }
	      if(state == 32){state++;}
 
			}
      break;
 
    case ReportIdentification             :
      printf("Message: Report Identification              received\r\n");
      //done = 1;
	  //usleep(10000000); // 1 second delay
	  // TODO decode ID here
      if(state ==1) state++;
      break;
    case ReportVelocityState             :
{
      printf("Message: Report Velocity State              received\r\n");
      unsigned int Velocity_hold=(unsigned short)(buffer[7]&0xFF)<<24 | (unsigned short)(buffer[6]&0xFF)<<16 |  (unsigned short)(buffer[5]&0xFF)<<8 |  (unsigned short)(buffer[4]&0xFF);
      double Velocity = Velocity_hold;
      unsigned int yaw_hold=((unsigned short)(buffer[9]&0xFF)<<8 |  (unsigned short)(buffer[8]&0xFF));
      //printf("yaw_hold = %x\n", yaw_hold);
      double V_yaw = yaw_hold;
      Velocity = UInt32ToScale(Velocity,-327.68 , 327.67);
      V_yaw    = UInt16ToScale(V_yaw,   -32.768 , 32.767);
      //  printf(" yaw_hold = %i (0x%X), V_yaw = %f\n", yaw_hold,yaw_hold, V_yaw);
        printf("Velocity X = %f, Velocity Yaw = %f\n", Velocity, V_yaw);
        robot_v=Velocity;
        robot_w=V_yaw;
 
        
        
      state++;
    //done = 1;
}
      break;  
 
 
    case ReportStatus             :
      printf("Message: Report Status              received\r\n");
      if(state == 7) state++;
      break;
    case ReportControl            :
{
      printf("Message: ReportControl             received\r\n"); 
      // Do not forget to check the rest IPs
      unsigned short int SubsystemID=(unsigned short)(buffer[2]&0xFF)<<24 | (unsigned short)(buffer[3]&0xFF)<<16 |  (unsigned short)(buffer[4]&0xFF)<<8 |  (unsigned short)(buffer[5]&0xFF);
      printf("the Subsystem ID is %u", SubsystemID);
      if (SubsystemID == 0)
      {
      SEND_REQUEST_CONTROL(handle, client_id);
      printf("The vehicle was not controlled\n");
      VehicleControled = 0;
      }
      else
      {
      printf("The vehicle is controleed\n");
      VehicleControled = 1;
      //state++;
      }
 }
      break; 
      case ConfirmControl           :
      printf("Message: ConfirmControl            received\r\n"); 
      //done = 1;
      state++;
      break; 
    case QueryIdentification      :
      printf("Message: QueryIdentification       received\r\n");
      if(discovered == false){
      SEND_REPORT_IDENTIFICATION(handle,client_id);
      SEND_QUERY_IDENTIFICATION(handle,client_id);
      discovered = true;}
      if (state ==0) state++;
      break; 
    default:
    printf(" Unknow Message: %4X received. ", MessageID);
    break;

    case JAUS_ID_ConfirmElementRequest    :
      printf("Message: ConfirmElementRequest     received\r\n"); 
      state = 17; //bypass states in the middle of the state machine, JDO.
      break;

    case JAUS_ID_ReportTravelSpeed        :
      printf("Message: ReportTravelSpeed         received\r\n"); 
      state++;
      break;

    case JAUS_ID_ReportElement            :
      printf("Message: ReportElement             received\r\n"); 
      state++;
      break; 
    case JAUS_ID_ReportElementList        :
      printf("Message: ReportElementList         received\r\n"); 
      // TODO Decode message here 
      state++;
      break; 
    case JAUS_ID_ReportElementCount       :
      printf("Message: ReportElementCount        received\r\n"); 
      // TODO Decode message here 
      state++;
      break; 
    case JAUS_ID_ReportActiveElement      :
      printf("Message: ReportActiveElement       received\r\n"); 
      // TODO Decode message here 
      Active_element=(unsigned short)(buffer[3]&0xFF)<<8 |  (unsigned short)(buffer[2]&0xFF);
      state++;
      break; 
    case JAUS_ID_ReportLocalWaypoint        :
      printf("Message: ReportLocalWaypoint       received\r\n"); 
      state++;
      break;
    case JAUS_ID_ReportServices : 
      printf("Report Services Message Received\n");
      state++;
      break;
    
  }
}
//============================
double UInt32ToScale(double val, double low, double high){
  return ((val*(high - low))/(UInt32Range)) + low;
}
//============================
double UInt16ToScale(double val, double low, double high){
  
  /*float conv = (((val*(high - low))/(UInt16Range)) + low);
  //printf("val %f converted to %f\n",val ,conv);
  if(conv == PI){  return 0;}
  else return conv;
  	*/
  	return ((val*(high - low))/(UInt16Range)) + low;
}
//=============================
unsigned long int GetTimeStamp(int x){
 
  struct timeval tv;
  struct tm *timeinfo;
  time_t rawtime;
  unsigned long int tstamp = 0;
  unsigned long int test = 0;
  gettimeofday(&tv, NULL);
  time(&rawtime);
  timeinfo = gmtime ( &rawtime );
 
  unsigned int mMilliseconds = (unsigned int)(tv.tv_usec/1000.0);
  int mDay = timeinfo->tm_mday;
  int mHour = (timeinfo->tm_hour + DWT + 24)%24;    
  int mMinute = timeinfo->tm_min;
  int mSecond = timeinfo->tm_sec;
  //printf("hour %d minute %d second %d",mHour, mMinute, mSecond);
  unsigned int timestamp = mHour*60 + mMinute*60 + mSecond;
  tstamp |= (unsigned int)(mDay)    << 27;
  //  printf("ggg day = %u\n",tstamp);
  tstamp |= (unsigned int)(mHour)   << 22;
  tstamp |= (unsigned int)(mMinute) << 16;
  tstamp |= (unsigned int)(mSecond) << 10;
  tstamp |= mMilliseconds;
  unsigned int times = tstamp;
        /*    int msec = (times << 22)>>22;
            int sec = (times << 16)>>26;
            int min = (times << 10)>>26;
            int hour = (times << 5)>>27;
            int day = (times >>27); 
            printf("msec = %d,sec = %d,min = %d, hour = %d, day = %d",msec, sec, min, hour, day);
  */
  if(!x)
  return timestamp ;
  return tstamp;
  //if (debugi==0x80000000) debugi=1;
  //else debugi=debugi<<1;
  //cout << "Debug i=" << debugi << endl;
  //return debugi;
}
void send_set_element(long handle, unsigned int client_id, double X, double Y, unsigned short element_id, unsigned short previous_id, unsigned short next_id, unsigned char req_id){

  /*  how the datatpye is defined
typedef struct {
  //unsigned short msg_id;  //0&1
  //unsigned char request_id;//
  //unsigned char count_field; //Jaus makes you explicitly say how long the list is
  unsigned short element_id; //3&4 
  unsigned short previous_id;//5&6
  unsigned short next_id;//7&8
  char enum_bit;
  unsigned short count_field;//bits9%10 //value of 9 for our purposes
  //unsigned short waypoint_msg_id;//bit 11&12
  unsigned char pv;//bit 13 //3
  unsigned int X; //14-17
  unsigned int Y;//18-21
} LOCAL_WAPOINT;

typedef struct {
  unsigned short msg_id;  //0&1
  unsigned char request_id;//
  unsigned char count_field; //Jaus makes you explicitly say how long the list is
  waypoints[4] waypoints; 
} SET_ELEMENT_MESSAGE;
  */

  SET_ELEMENT_MESSAGE rcmsg;
  rcmsg.msg_id = SetElement;
  rcmsg.request_id = 1;
  rcmsg.count_field = 4; //we will send four elements
  (rcmsg.waypoints[0]).element_id = 1 ;
  (rcmsg.waypoints[0]).next_id = 2;
  (rcmsg.waypoints[0]).previous_id = 0;
  (rcmsg.waypoints[0]).pv = 3;
  //rcmsg.waypoint_msg_id = JAUS_ID_SetLocalWaypoint;
  (rcmsg.waypoints[0]).count_field = 13;  
  (rcmsg.waypoints[0]).X = scaleToUInt32(X1,-100000, 100000);
  (rcmsg.waypoints[0]).Y = scaleToUInt32(Y1,-100000, 100000);

  (rcmsg.waypoints[1]).element_id = 2 ;
  (rcmsg.waypoints[1]).next_id = 3;
  (rcmsg.waypoints[1]).previous_id = 1;
  (rcmsg.waypoints[1]).pv = 3;
  //rcmsg.waypoint_msg_id = JAUS_ID_SetLocalWaypoint;
  (rcmsg.waypoints[1]).count_field = 13;  
  (rcmsg.waypoints[1]).X = scaleToUInt32(X2,-100000, 100000);
  (rcmsg.waypoints[1]).Y = scaleToUInt32(Y2,-100000, 100000);

  (rcmsg.waypoints[2]).element_id = 3 ;
  (rcmsg.waypoints[2]).next_id = 4;
  (rcmsg.waypoints[2]).previous_id = 2;
  (rcmsg.waypoints[2]).pv = 3;
  //rcmsg.waypoint_msg_id = JAUS_ID_SetLocalWaypoint;
  (rcmsg.waypoints[2]).count_field = 13;  
  (rcmsg.waypoints[2]).X = scaleToUInt32(X3,-100000, 100000);
  (rcmsg.waypoints[2]).Y = scaleToUInt32(Y3,-100000, 100000);

  (rcmsg.waypoints[3]).element_id = 4;
  (rcmsg.waypoints[3]).next_id = 5;
  (rcmsg.waypoints[3]).previous_id = 3;
  (rcmsg.waypoints[3]).pv = 3;
  //rcmsg.waypoint_sg_id = JAUS_ID_SetLocalWaypoint;
  (rcmsg.waypoints[3]).count_field = 13;  
  (rcmsg.waypoints[3]).X = scaleToUInt32(X4,-100000, 100000);
  (rcmsg.waypoints[3]).Y = scaleToUInt32(Y4,-100000, 100000);
  //rcmsg.Z = scaleToUInt32(Z,-100000, 100000);
  //rcmsg.yaw = scaleToUInt16(yaw, -1*PI, PI);
  
  //printf("Message: Data Send [%s] Size:%li\n",((char*)&rcmsg)+6,sizeof(rcmsg));
    if (JrSend(handle, client_id, sizeof(rcmsg), (char*)&rcmsg) != Ok)
        printf("Unable to send SET_ELEMENT_MESSAGE message.  Need more debug here...\n");
    else printf("Sent message SET_ELEMENT_MESSAGE \n");
}

void send_query_element_list(long handle, unsigned client_id){
  SEND_QUERY_ELEMENT_LIST msg;
  msg.msg_id = JAUS_ID_QueryElementList;
  if (JrSend(handle, client_id,sizeof(msg), (char*)&msg) != Ok)
        printf("Unable to send query element list.  Need more debug here...\n");
    else printf("Sent message Query element list to Robot\n");
}


void send_query_element_count(long handle, unsigned client_id){
  QUERY_ELEMENT_COUNT msg;
  msg.msg_id = JAUS_ID_QueryElementCount;
  if (JrSend(handle, client_id,sizeof(msg), (char*)&msg) != Ok)
        printf("Unable to send query element count.  Need more debug here...\n");
    else printf("Sent message Query element count to Robot\n");

}


void send_execute_list(long handle, unsigned client_id){
  EXECUTE_LIST msg;
  msg.msg_id = JAUS_ID_ExecuteList;
  msg.pv=0x01;
  msg.speed = scaleToUInt16(1,0,327.67);  //this is from the competition rule book that a value of 1 will be sent
  msg.UID_start = 0x0001;  // first element
  if (JrSend(handle, client_id,sizeof(msg), (char*)&msg) != Ok)
        printf("Unable to send execute list.  Need more debug here...\n");
    else printf("Sent message execute list to Robot\n");

}


void send_query_active_element(long handle, unsigned client_id){

  QUERY_ACTIVE_ELEMENT msg;
  msg.msg_id = JAUS_ID_QueryActiveElement;
  if (JrSend(handle, client_id,sizeof(msg), (char*)&msg) != Ok)
        printf("Unable to send query active element.  Need more debug here...\n");
    else printf("Sent message Query active element to Robot\n");

}

void send_query_travel_speed(long handle, unsigned client_id){
  QUERY_TRAVEL_SPEED msg;
  msg.msg_id = JAUS_ID_QueryTravelSpeed;
  if (JrSend(handle, client_id,sizeof(msg), (char*)&msg) != Ok)
        printf("Unable to send query travel speed.  Need more debug here...\n");
    else printf("Sent message Query travel speed to Robot\n");

}

void send_query_waypoints(long handle, unsigned client_id){

  QUERY_LOCAL_WAYPOINT msg;
  msg.msg_id = JAUS_ID_QueryLocalWaypoint;
  msg.pv = 3;//they use this in the compitition, but its wrong
  if (JrSend(handle, client_id,sizeof(msg), (char*)&msg) != Ok)
        printf("Unable to send QUERY_LOCAL_WAYPOINT.  Need more debug here...\n");
    else printf("Sent message QUERY_LOCAL_WAYPOINT to Robot\n");
}
unsigned int scaleToUInt32(double val, double low, double high)
{ 
  unsigned int conv = ((val - low) / ((high - low) / UInt32Range));
  //printf("The val %f was converted to %x\n", val ,conv);
  return conv;
}
unsigned short scaleToUInt16(double val, double low, double high)
{
  unsigned short conv = ((val - low) / ((high - low) / UInt16Range));
  //printf("The val %f was converted to %x\n", val ,conv);
  return conv;
}

void SEND_QUERY_SERVICES(long handle,unsigned int client_id)
{
  Query_Services rcmsg;
  rcmsg.msg_id=QueryServices;
  rcmsg.Node_ID=1;
  rcmsg.Component_ID=00;
  //printf("Message: Data Send [%s] Size:%li\n",((char*)&rcmsg)+7,sizeof(rcmsg));
    if (JrSend(handle, client_id, sizeof(rcmsg), (char*)&rcmsg) != Ok)
        printf("Unable to send SEND_QUERY_SERVICES message.  Need more debug here...\n");
    else printf("Sent message SEND_QUERY_SERVICES to Robot\n"); 
}


void Print_JAUS_Status(void)
{
SaveCursor();
gotoxy(1,1);
printf("                                                                                \r\n");
printf("                                                                                \r\n");
printf("                                                                                \r\n");
printf("                                                                                \r\n");
printf("                                                                                \r\n");
printf("                                                                                \r\n");
gotoxy(1,1);



  	printf("COP   ID:%08X (%d-%d-%d)\r\n",COP_id,COP_id>>16,(COP_id&0xFF00)>>8,COP_id&0xFF); // print COP ID
  	printf("Robot ID:%08X (%d-%d-%d)\r\n",client_id,client_id>>16,(client_id&0xFF00)>>8,client_id&0xFF); // print ROBOT ID
	printf("COP State:%2i  ; Active element: %2i \r\n", state, Active_element);
	printf("Robot (X,Y):(%8.4f,%8.4f); Yaw=%8.2f degree\n", robot_x,robot_y,robot_yaw*180/PI);
	printf("       Vx=%8.4f m/s, Va=%8.4f rad/s\n", robot_v,robot_w);

	
	
	
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

