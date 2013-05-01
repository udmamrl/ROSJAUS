/*!
 ***********************************************************************
 * file      ROSJAUS_simple.cpp
 *           Demo we can send JAUS message in ROS
 * author    Cheng-Lung Lee. University of Detroit Mercy
 *            Advanced Mobile Robotics Lab
 * date      2013/01/07
 *
 ************************************************************************
 */

#include <iostream>
#include <stdio.h>
#include <signal.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <cmath>
#include <string>
#include <ctime>
#include <list>
#include <sys/time.h>
#include <boost/thread.hpp>

// Put ROS stuff here

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
//from http://www.ros.org/wiki/navigation/Tutorials/SendingSimpleGoals
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>

// JrMiddleware
#include "JuniorAPI.h"

//JAUS message stuff Must putin the end of include , because we use #pragma pack(1) in the header file
#include "JAUSmessage.h"
#include "JAUS_MESSAGE_ID.h"
#include "JAUSservices.h"

using namespace std;

void spinThread();
unsigned int GenerateJausID(unsigned int subsystemID,unsigned int nodeID,unsigned int componentID);
void SEND_QUERY_IDENTIFICATION(long handle,unsigned int COP_id);



int JAUS_COP_ID, Robot_JAUS_ID;
std::string config_file; 
double ROS_loop_rate_Hz;




// Define the message structure.  We have to tell
// the compiler to pack on 1-byte boundaries toStatus_INIT
// make sure it doesn't insert padding that would
// be non-compliant to AS6009.
//#pragma pack(1)  already define in header file


int main(int argc, char* argv[])
{
  long handle;

  // Setup ROS here
  ros::init(argc, argv, "ROS_JAUS");
  ros::NodeHandle n("~");
  // read parameters
  n.param("JAUS_COP_ID",            JAUS_COP_ID, 42);
  n.param("Robot_JAUS_ID",          Robot_JAUS_ID, 106);
  n.param("ROS_loop_rate_Hz",       ROS_loop_rate_Hz, 100.0);
  n.param<std::string>("jr_config", config_file, "jr_config.xml");

  ROS_INFO("================================================\r\n");
  ROS_INFO("================================================\r\n");
  unsigned int COP_JausID = GenerateJausID( JAUS_COP_ID,1,1) ;  // 90-1-1
  ROS_INFO("My COP ID:%08X (%d-%d-%d)\r\n",COP_JausID,COP_JausID>>16,(COP_JausID&0xFF00)>>8,COP_JausID&0xFF); // print JAUS_ID
  unsigned int My_JausID =  GenerateJausID(Robot_JAUS_ID,1,1) ; ;  // 331-1-1
  ROS_INFO("My JAUS ID:%08X (%d-%d-%d)\r\n",My_JausID,My_JausID>>16,(My_JausID&0xFF00)>>8,My_JausID&0xFF); // print JAUS_ID
  ROS_INFO("================================================\r\n");
  ROS_INFO("================================================\r\n");

  // We have to call JrConnect before we send any messages.
  // JrConnect requires us to pass in the JAUS ID for our
  // component as well as the configuration file.  We
  // get a handle back that's used for subsequent calls.
  // Note:JuniorRTE must installed in searchable path like /bin or /usr/bin
  // If you want to use autoconnect function, Usually we manually run JuniorRTE
  if (JrConnect(My_JausID, config_file.c_str(), &handle) != Ok)
  {
      ROS_INFO("Failed to connect to Junior.  Need more debug here...\n");
      return 0;
  }
  else ROS_INFO("Successfully connected to Junior...\n");

  // Send some message
  SEND_QUERY_IDENTIFICATION(handle,COP_JausID); // Query COP ID
  //SEND_REPORT_IDENTIFICATION(handle,COP_JausID);

  // for receive data
  const unsigned short MaxMsgSize = 500;
  JrErrorCode ret;
  char buffer[MaxMsgSize];
  unsigned int buffersize;
  unsigned int sender;
  unsigned short msg_id;


  // Run spin() in different thread
  boost::thread spin_thread = boost::thread(boost::bind(&spinThread));

  ros::Rate loop_rate(ROS_loop_rate_Hz);
  ROS_INFO("ROS JAUS Loop Start!");
    while(ros::ok())
    {
    // Read Jaus message from Jr.
        ret = JrReceive(handle, &sender, &buffersize, buffer, NULL, NULL, &msg_id);
        if (ret == Ok)
        {
          ROS_INFO("================================================\r\n");
          if (buffersize==2)
          {
            ROS_INFO("MessageID:%02X%02X  ",buffer[1]&0xFF,buffer[0]&0xFF); // print message ID
          }
          else if (buffersize<2)
          {
            ROS_INFO("Data(Hex): ");
            for (int i=0;i<buffersize;i++)  ROS_INFO("%02X ",buffer[i]&0xFF); ;
          }
          else
          {
            ROS_INFO("MessageID:%02X%02X , Data: ",buffer[1]&0xFF,buffer[0]&0xFF); // print message ID
          for (int i=2;i<buffersize;i++)  ROS_INFO("%02X ",buffer[i]&0xFF); // Print DATA
          }
            ROS_INFO(" ; %d bytes received. ",buffersize);
            ROS_INFO("From :%8X ( %d-%d-%d )\n",sender,sender>>16,(sender&0xFF00)>>8,sender&0xFF);

          // Do some message process here
          if ( buffersize>=2)
          {
            unsigned short MessageID=(unsigned short)(buffer[1]&0xFF)<<8 | (unsigned short)(buffer[0]&0xFF);
            //ProcessJausMessage(handle,COP_JausID,MessageID, &sender, &buffersize, &buffer[0]);
          }

        }
     // Sleep a bit before looping again
     ROS_INFO("ROS Loop sleep Once !");
     //ros::spinOnce();
     loop_rate.sleep();

    }

  spin_thread.join();
  // Clean-up
  JrDisconnect(handle);
  ROS_INFO("\nExit.... Disconnect Jr....\r\n");
  printf("\nExit.... Disconnect Jr....\r\n");

  return 0;
}

// ///////////////////////////////////////////////////////////////////////////////
// Use different Thread to spin(); (get message) forever
void spinThread()
{
    ros::spin();
}
// ///////////////////////////////////////////////////////////////////////////////
// TODO we should put all send function to different C file , make it easy to reuse.

unsigned int GenerateJausID(unsigned int subsystemID,unsigned int nodeID,unsigned int componentID)
{
  return ( subsystemID<<16 | nodeID <<8 | componentID );
}

void SEND_QUERY_IDENTIFICATION(long handle,unsigned int COP_id)  // TODO should add QueryType here
{
  // Query Identification
  QUERY_IDENTIFICATION_MSG qimsg;
  qimsg.msg_id=JAUS_ID_QueryIdentification;
  qimsg.QueryType = 0x02;
    if (JrSend(handle, COP_id,sizeof(qimsg), (char*)&qimsg) != Ok)
         ROS_INFO("Unable to send QueryID message.\n");
    else ROS_INFO("Sent message QueryID to the COP.\n");

}

