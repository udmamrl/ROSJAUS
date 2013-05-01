//JAUSmessage.h
#ifndef __JAUS_MESSAGES_H
#define __JAUS_MESSAGES_H
// can't use undefined size array so define the max size here
// 4 waypoing in IGVC JAUS
#define Max_ELEMENT_Numbers 4
// Only use 8 characters in Vehicle Name string
#define Max_IDENTIFICATION_StringLength 8
// Define the message structure.  We have to tell
// the compiler to pack on 1-byte boundaries to
// make sure it doesn't insert padding that would
// be non-compliant to AS6009.
#pragma pack(1)
const double PI=3.14159265358979323846;
const double ByteRange       = 255.0;
const double UInt64HalfRange = 1.8446744073709552E+19;
const double Int64HalfRange  = 9.2233720368547758E+18;  
const double UInt32Range     = 4294967295.0;  
const double Int32Range      = 4294967294.0;
const double UInt16Range     = 65535.0;             
const double Int16Range      = 65534.0;                         
const double Epsilon = .00000000000000000000001;
const unsigned int state1  = 0x0001;
const unsigned int state2  = 0x002;
const unsigned int state3  = 0x004;
const unsigned int state4  = 0x008;
const unsigned int state5  = 0x10;
const unsigned int state6  = 0x20;
const unsigned int state7  = 0x40;
const unsigned int state8  = 0x80;
const unsigned int state9  = 0x100;
const unsigned int state10  = 0x200;
const unsigned int state11 = 0x400;
const unsigned int state12  = 0x800;
const unsigned int state13  = 0x1000;
const unsigned int state14  = 0x2000;
const unsigned int state15  = 0x4000;
const unsigned int state16  = 0x8000;
//=======================================
// ID 000Dh: RequestControl 
//========================================
//ID 4002h: ReportStatus 
//STATUS
// 0 INIT 
// 1 READY 
// 2 STANDBY 
// 3 SHUTDOWN 
// 4 FAILURE 
// 5 EMERGENCY
#define Status_INIT 0
#define Status_READY 1
#define Status_STANDBY 2
#define Status_SHUTDOWN 3
#define Status_FAILURE 4
#define Status_EMERGENCY 5
typedef struct {
	unsigned short msg_id;
	unsigned char AuthorityCode;
} REQUEST_CONTROL_MSG;

// ID 000Fh: ConfirmControl 
/* ResponseCode
0 CONTROL ACCEPTED 
1 NOT AVAILABLE 
2 INSUFFICIENT AUTHORITY 
*/
typedef struct {
	unsigned short msg_id;
	unsigned char ResponseCode;
} CONFIRM_CONTROL_MSG;

// ID 0010h: RejectControl 
/* ResponseCode
 0 CONTROL RELEASED 
 1 NOT AVAILABLE
 */
typedef struct {
	unsigned short msg_id;
	unsigned char ResponseCode;
} REJECT_CONTROL_MSG;
//========================================
//ID 200Dh: QueryControl
typedef struct {
	unsigned short msg_id;
} QUERY_CONTROL_MSG;
//======================================
//ID 0004h: Resume
typedef struct {
	unsigned short msg_id;
} RESUME_MSG;
typedef struct {
	unsigned short msg_id;
	unsigned char Node_ID;
	unsigned char Component_ID;	
} Query_Services;
//========================================
//ID 2002h: QueryStatus
typedef struct {
	unsigned short msg_id;
} QUERY_STATUS_MSG;
//======================================
//ID 2002h: QueryStatus
typedef struct {
	unsigned short msg_id;
	unsigned short pv;
} QUERY_LCAOL_POSE_MSG;

//=====================================

//ID 2B00h: : QueryIdentification
/*QueryType unsigned byte 
0: Reserved 
1: System Identification   
2: Subsystem Identification 
3: Node Identification 
4: Component Identification 
5 – 255: Reserved */
typedef struct {
	unsigned short msg_id;
	unsigned char QueryType;
} QUERY_IDENTIFICATION_MSG;

//========================================
//ID 4002h: ReportStatus 
typedef struct {
	unsigned short msg_id;
	unsigned char Status;
	unsigned int  UNUSED;
} REPORT_STATUS_MSG;
//ID 400Dh: ReportControl 
typedef struct {
	unsigned short msg_id;
	unsigned short SubsystemID;
	unsigned char NodeID;
	unsigned char ComponentID;
	unsigned char AuthorityCode;
} REPORT_CONTROL_MSG;

// Message ID 4403 REPORT_LOCAL_POSE
typedef struct {
  unsigned short msg_id; //0,1
  unsigned short pv;//2,3
  unsigned int   X;//4-7
  unsigned int   Y;//8-11
  //unsigned int   Z;//12-15
  //unsigned short Yaw;//16,17
  //put in position rms roll pitch attitude
  unsigned int   TimeStamp;
} REPORT_LOCAL_POSE_MSG;
#define LOCAL_POSE_PVBit_X			  0x0001
#define LOCAL_POSE_PVBit_Y            0x0002
#define LOCAL_POSE_PVBit_Z            0x0004
#define LOCAL_POSE_PVBit_PositionRms  0x0008
#define LOCAL_POSE_PVBit_Roll         0x0010
#define LOCAL_POSE_PVBit_Pitch        0x0020
#define LOCAL_POSE_PVBit_Yaw          0x0040
#define LOCAL_POSE_PVBit_AttitudeRms  0x0080
#define LOCAL_POSE_PVBit_TimeStamp    0x0100
#define REPORT_LOCAL_POSE_MSG_pv LOCAL_POSE_PVBit_X+LOCAL_POSE_PVBit_Y+LOCAL_POSE_PVBit_Z+LOCAL_POSE_PVBit_Yaw+LOCAL_POSE_PVBit_TimeStamp

typedef struct {
  unsigned short msg_id; //0,1
  unsigned short pv;//2,3
  unsigned int   X;//4-7
  unsigned int   Y;//8-11
  //unsigned int   Z;//12-15
  unsigned short Yaw;//16,17
  //put in position rms roll pitch attitude
  //unsigned int   TimeStamp;
} SET_LOCAL_POSE_MSG;

//======================================
typedef struct {
	unsigned short msg_id;
	unsigned short pv;
} QUERY_VELOCITY_STATE_MSG;
//======================================
typedef struct {
	unsigned short msg_id;
}SHUTDOWN_MSG;
//=====================================
// Message ID 4404 REPORT_VELOCITY_STATE
typedef struct {
	unsigned short msg_id;
	unsigned short pv;
	unsigned int   VX;
	unsigned short YawRate;
	unsigned int TimeStamp;
	//unsigned int   Vyaw;
} REPORT_VELOCITY_STATE_MSG;
#define VELOCITY_STATE_PVBit_VX			0x0001
#define VELOCITY_STATE_PVBit_VY         0x0002
#define VELOCITY_STATE_PVBit_VZ         0x0004
#define VELOCITY_STATE_PVBit_VRms		0x0008
#define VELOCITY_STATE_PVBit_RollRate   0x0010
#define VELOCITY_STATE_PVBit_PitchRate  0x0020
#define VELOCITY_STATE_PVBit_YawRate    0x0040
#define VELOCITY_STATE_PVBit_RateRms	0x0080
#define VELOCITY_STATE_PVBit_TimeStamp  0x0100
#define REPORT_VELOCITY_STATE_MSG_pv VELOCITY_STATE_PVBit_VY
typedef struct {
	unsigned short msg_id;
	unsigned char count_field_node;
	unsigned char NodeID; // Subsystem	
	unsigned char count_field_component;
	
	unsigned char ComponentID; // Vehicle
	unsigned char InstanceID;
	unsigned char ListSize;
	char size1;
	char transport[27];
	unsigned char MajorVersionNumber1;
	unsigned char MinorVersionNumber1;
	char size2;	
	char velocity[41];
	unsigned char MajorVersionNumber2;
	unsigned char MinorVersionNumber2;
	char size3;
	char localpose[37];
	unsigned char MajorVersionNumber3;
	unsigned char MinorVersionNumber3;
	char localwaypoint[41];
	char size4;
	unsigned char MajorVersionNumber4;
	unsigned char MinorVersionNumber4;
	char size5;	
	char accesscontrol[31];
	unsigned char MajorVersionNumber5;
	unsigned char MinorVersionNumber5;
	char size6;
	char events[24];
	unsigned char MajorVersionNumber6;
	unsigned char MinorVersionNumber6;
} REPORT_SERVICES;

// Message ID 4B00h
typedef struct {
	unsigned short msg_id;
	unsigned char QueryType; // Subsystem
	unsigned short Type; // Vehicle
	char StringLength;

	// TODO change to pointer instead of fix length string array
	char Name[Max_IDENTIFICATION_StringLength]; // currently use fixed length
	
} REPORT_IDENTIFICATION_MSG;

#define ID_QueryType_Subsystem 2
#define ID_Type_Vehicle 10001
typedef struct {
	//unsigned short msg_id;	//0&1
	//unsigned char request_id;//
	//unsigned char count_field; //Jaus makes you explicitly say how long the list is
	unsigned short element_id; //3&4 
	unsigned short previous_id;//5&6
	unsigned short next_id;//7&8
	char enum_bit;
	unsigned short count_field;//bits9%10	//value of 13 for our purposes
	//unsigned short waypoint_msg_id;//bit 11&12
	unsigned short msg_id;	
	unsigned char pv;//bit 13	//3
	unsigned int X; //14-17
	unsigned int Y;//18-21
	unsigned short tolerance;
} LOCAL_WAYPOINT;

typedef struct {
	unsigned short msg_id;	//0&1
	unsigned char request_id;//
	unsigned char count_field; //Jaus makes you explicitly say how long the list is
	
	// TODO change to pointer instead of fix length array	
	LOCAL_WAYPOINT waypoints[Max_ELEMENT_Numbers]; // Currently use fixed length
	
} SET_ELEMENT_MESSAGE;
typedef struct {
	double posx;
	double posy;
	//float  theta;
	unsigned short UID;
	unsigned short next_id;
	unsigned short previous_id;
}ELEMENT;
typedef struct {
	unsigned short msg_id;
	unsigned char request_id_ret;
}CONFIRM_SET_ELEMENT_MESSAGE;
typedef struct {
	unsigned short msg_id;
}SEND_QUERY_ELEMENT_LIST;

typedef struct {
	unsigned short msg_id;
	unsigned short count_field;
	
	//unsigned short UID[4];  //this is for stupid macbook pro.  3000 dollars of suck.
	//unsigned short UID[];//linux is better and can handle this, Mac sucks and cannot
	// TODO change to pointer instead of fix length array	
	unsigned short UID[Max_ELEMENT_Numbers]; // Currently use fixed length
	
}REPORT_ELEMENT_LIST;

typedef struct {
	unsigned short msg_id;
}QUERY_ELEMENT_COUNT;
typedef struct {
	unsigned short msg_id;
	unsigned short count;
}REPORT_ELEMENT_COUNT;
typedef struct {
	unsigned short msg_id;
	unsigned short speed;
	unsigned short UID_start;
}EXECUTE_LIST;
typedef struct {
	unsigned short msg_id;
}QUERY_ACTIVE_ELEMENT;
typedef struct {
	unsigned short msg_id;
	unsigned short active_id;
}REPORT_ACTIVE_ELEMENT;
typedef struct {
	unsigned short msg_id;
}QUERY_TRAVEL_SPEED;
typedef struct {
	unsigned short msg_id;
	unsigned short speed;
}REPORT_TRAVEL_SPEED;
typedef struct {
	unsigned short msg_id;
	unsigned char pv;
}QUERY_LOCAL_WAYPOINT;
typedef struct {
	unsigned short msg_id;
	unsigned char pv;
	unsigned int X;
	unsigned int Y;	
}REPORT_LOCAL_WAYPOINT;

#endif





