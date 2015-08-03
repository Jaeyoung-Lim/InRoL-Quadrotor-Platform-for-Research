# pragma once

#include <Windows.h>
#include <string>
#include <assert.h> 
#include "Client.h"
#include "Vector3.h"
#pragma pack(1)
typedef struct SCIENFITIC_COMMANDDATA
{
	//always 0x17
	//#define CommandDataDescriptor 0x17
	//unsigned char packetdescriptor;
	//pitch, roll, thrust, yaw commands. 0..4095 2048=middle
	unsigned short pitch;
	unsigned short roll;
	unsigned short thrust;
	unsigned short yaw;
	//flags
	//Bit 0(0x01): Pitch control through serial interface enabled
	//Bit 1(0x02): Roll control through serial interface enabled
	//Bit 2(0x04): Thrust control through serial interface enabled
	//Bit 3(0x08): Yaw control through serial interface enabled
	//Bit 4(0x10): ACC-Mode on/off
	//Bit 5(0x20): Height control - on/off (only with ACC)
	//Bit 6(0x40): overwrite ACC/Height mode control
	//(0=mode selected by RC 1=mode selected by Bit 4 and 5)
	//Bit 7(0x80): Trigger Scientific status packet (triggers a response
	//with the actual scientific state)
	//Bit 8..15: send rate of the scientific packet in 5Hz
	//(0=off;1=5Hz, 20=100Hz, 200=1kHz)
	//Scientific packet is send for three seconds max.
	//after the last command_data packet
	unsigned short flags;
	//0x0000

	//long long time_to_go;
} XCommand;
typedef struct XCOMMAND_DEQUE_PACK
{
	XCommand xCommand;
	long long time_to_go;
	unsigned long long seqNo;//changes
}XCommand_dq_pack;

typedef struct SCIENTIFIC_STATUSDATA
{
	//alway = PD_SCIENTIFICSTATUS
	//#define StatusDataDescriptor 0x18
	unsigned char packetdescriptor;
	//flags
	//Bit 0-5 represent the actual status of the ResearchPilot!
	//if the interface is disabled by the remote, all bits are 0!
	//Bit 0(0x01): Pitch control through serial interfacae enabled
	//Bit 1(0x02): Roll control through serial interface enabled
	//Bit 2(0x04): Thrust control through serial interface enabled
	//Bit 3(0x08): Yaw control through serial interface enabled
	//Bit 4(0x10): ACC-Mode on/off
	//Bit 5(0x20): Height control - on/off (only with ACC)
	//Bit 6(0x40): unused
	//Bit 7(0x80): Scientific interface enabled by Remote?
	//1=Interface enabled (control through serial link)
	//0=interface disabled (control through RC)
	//Bit 8..15: sendrate of the scientific packet in 5Hz
	//(0=off;1=5Hz, 20=100Hz, 200=1kHz). Scientific packet
	//is send for three seconds max. after the last command_data packet
	unsigned short flags;
} XStatus;

/*
typedef struct SCIENTIFIC_DATA
{
	int calcData_angle_pitch;
	int calcData_angle_roll;
	int calcData_angle_yaw;
	int calcData_angvel_pitch;
	int calcData_angvel_roll;
	int calcData_angvel_yaw;
	short calcData_acc_x;
	short calcData_acc_y;
	short calcData_acc_z;
	int calcData_height;
	short rcdata_channel0;//pitch
	short rcdata_channel1;
	short rcdata_channel2;
	short rcdata_channel3;

	long long time_to_go;
} XState;
*/

typedef struct SCIENTIFIC_CONFIG
{
	//always PD_SCIENTIFICCONFIG
	//#define PD_SCIENTIFICCONFIG 0x20
	unsigned char packetdescriptor;
	unsigned char data_select[128];
} XConfig;

struct Selected_DATA
{
	// Gyro Raw Data
	short raw_gyro_pitch;
	short raw_gyro_roll;
	short raw_gyro_yaw;
	// Accelerator Raw Data
	short raw_acc_x;
	short raw_acc_y;
	short raw_acc_z;
	// Mag Raw Data
	short raw_mag_x;
	short raw_mag_y;
	short raw_mag_z;
	int raw_pressure;
	int raw_temperature;
	//Calculated Data
	int calcData_angle_pitch;
	int calcData_angle_roll;
	int calcData_angle_yaw;
	int calcData_angvel_pitch;
	int calcData_angvel_roll;
	int calcData_angvel_yaw;
	short calcData_acc_x;
	short calcData_acc_y;
	short calcData_acc_z;
	short calcData_trans_acc_x;
	short calcData_trans_acc_y;
	short calcData_trans_acc_z;
	//Magnetic field sensor
	int calcData_mag_x;
	int calcData_mag_y;
	int calcData_mag_z;

	int calcData_acc_length;
	// Reference angles
	int calcData_acc_angle_pitch;
	int calcData_acc_angle_roll;
	int calcData_mag_heading;
	// Height
	int calcData_height;
	int calcData_dheight;
	int calcData_height_reference;
	int calcData_dheight_reference;
	int calcData_speed_z;
	// Control
	int ctrl_out_pitch;
	int ctrl_out_roll;
	short ctrl_out_yaw;
	short ctrl_out_thrust;
	// RC Data
	short rcdata_channel0;
	short rcdata_channel1;
	short rcdata_channel2;
	short rcdata_channel3;
	int uptime;
	short cpu_load;
	int voltage;
	short current;
};

struct Normal_DATA
{
	int calcData_angle_pitch;
	int calcData_angle_roll;
	int calcData_angle_yaw;
	int calcData_angvel_pitch;
	int calcData_angvel_roll;
	int calcData_angvel_yaw;
	short calcData_acc_x;
	short calcData_acc_y;
	short calcData_acc_z;
	int calcData_height;
	short rcdata_channel0;
	short rcdata_channel1;
	short rcdata_channel2;
	short rcdata_channel3;
};


struct SXPacket
{
	char startMark[3];
	BYTE lengthH;
	BYTE lengthL;
	char command[sizeof(XCommand)];
	BYTE crcH;
	BYTE crcL;
};


// Pelican

struct IMU_RAWDATA //28Byte
{
	//pressure sensor 24-bit value, not scaled but bias free
	int pressure;

	//16-bit gyro readings; 32768 = 2.5V
	short gyro_x;
	short gyro_y;
	short gyro_z;

	//10-bit magnetic field sensor readings
	short mag_x;
	short mag_y;
	short mag_z;

	//16-bit accelerometer readings
	short acc_x;
	short acc_y;
	short acc_z;

	//16-bit temperature measurement using yaw-gyro internal sensor
	unsigned short temp_gyro;

	//16-bit temperature measurement using ADC internal sensor
	unsigned int temp_ADC;
};

typedef struct IMU_CALCDATA //92Byte
{
	//angles derived by integration of gyro_outputs, drift compensated by data fusion
	//-90000..+90000 pitch(nick) and roll, 0..360000 yaw; 1000 = 1 degree

	int angle_nick;
	int angle_roll;
	int angle_yaw;

	//angular velocities, raw values [16 bit] but bias free
	int angvel_nick;
	int angvel_roll;
	int angvel_yaw;

	//acc-sensor outputs, calibrated: -10000..+10000 = -1g..+1g
	short acc_x_calib;
	short acc_y_calib;
	short acc_z_calib;

	//horizontal / vertical accelerations: -10000..+10000 = -1g..+1g
	short acc_x;
	short acc_y;
	short acc_z;

	//reference angles derived by accelerations only: -90000..+90000; 1000 = 1 degree
	int acc_angle_nick;
	int acc_angle_roll;

	//total acceleration measured (10000 = 1g)
	int acc_absolute_value;

	//magnetic field sensors output, offset free and scaled; units not determined, 
	//as only the direction of the field vector is taken into account
	int Hx;
	int Hy;
	int Hz;

	//compass reading: angle reference for angle_yaw: 0..360000; 1000 = 1 degree
	int mag_heading;

	//pseudo speed measurements: integrated accelerations, pulled towards zero 
	//units unknown; used for short-term position stabilization
	int speed_x;
	int speed_y;
	int speed_z;

	//height in mm (after data fusion)
	int height;

	//diff. height in mm/s (after data fusion)
	int dheight;

	//diff. height measured by the pressure sensor [mm/s]
	int dheight_reference;

	//height measured by the pressure sensor [mm]
	int height_reference;
} XPState; //92Byte

typedef struct CTRL_INPUT { 
	//serial commands(= Scientific Interface)
	short pitch; //Pitch input: 2047..+2047 (0=neutral)
	short roll; //Roll input: -2047..+2047 (0=neutral)
	short yaw; //(=R/C Stick input) -2047..+2047 (0=neutral)
	short thrust; //Collective: 0..4095 = 0..100%
	/*
	short ctrl; /*control byte:
					bit 0: pitch control enabled
					bit 1: roll control enabled
					bit 2: yaw control enabled
					bit 3: thrust control enabled
					These bits can be used to only enable one axis at a time and&
					thus to control the other axes manually. This usually &
					helps a lot to set up and finetune controllers for each axis seperately. */
	//short chksum;
} CTRL_Input;


struct PelicanPollPacket
{
	char startMark[4];
	unsigned short packets;
};

struct PelicanComPacket
{
	char startMark[5];
	char command[sizeof(CTRL_Input)];
};
#pragma pack()

/*
typedef struct HAPTIC_STATE
{
	double x;
	double y;
	double z;
	double xv;
	double yv;
	double zv;

	Vector3 xd;
	Vector3 dxd;
	Vector3 ddxd;
	Vector3 dddxd;

	Vector3 F; 
	double trans[16];
	int buttons;
}HState;
typedef struct HAPTIC_COMMAND
{
	double Fx;
	double Fy;
	double Fz;
	double xset;
	double yset;
	double zset;
	double seqNo;//changes
	char check;
	double time_to_go;
}HCommand;
*/
class IStringFactory
{
public:
  virtual char * AllocAndCopyString( const char * i_pSource ) = 0;
  virtual void FreeString( char * i_pString ) = 0;
protected:
  virtual ~IStringFactory() {}
};
class String
{
public:
  // A string which we are not responsible for deallocating
  inline
  String( const char * i_pString = 0 )
  : m_pString( 0 )
  , m_pConstString( i_pString )
  , m_pStringFactory( 0 )
  {
  }

  // A string which we are not responsible for deallocating
  String( const std::string & i_rString )
  : m_pString( 0 )
  , m_pConstString( i_rString.c_str() )
  , m_pStringFactory( 0 )
  {
  }

  // Copy constructor
  inline
  String( const String & i_rString )
  {
    m_pConstString = i_rString.m_pConstString;
    m_pStringFactory = i_rString.m_pStringFactory;
    if( m_pStringFactory )
    {
      m_pString = m_pStringFactory->AllocAndCopyString( i_rString.m_pString );
    }
    else
    {
      m_pString = 0;
    }
  }

  inline
  ~String()
  {
    if( m_pStringFactory )
    {
      m_pStringFactory->FreeString( m_pString );
    }
  }

  // A string which we are responsible for deallocating
  inline
  void Set( const char * i_pString, IStringFactory & i_rStringFactory )
  {
    m_pString = i_rStringFactory.AllocAndCopyString( i_pString );
    m_pStringFactory = &i_rStringFactory;
    m_pConstString = 0;
  }

  inline
  operator std::string() const
  {
    if( m_pStringFactory )
    {
      return std::string( m_pString );
    }
    else
    {
      return std::string( m_pConstString );
    }
  }

private:
        char     * m_pString;
  const char     * m_pConstString;
  IStringFactory * m_pStringFactory;
};

typedef struct TRACKER_STATE{
// Frame #
    unsigned int FrameNumber;

// Latency = processed time + divery time
	double TotalLatency;

// No. of total Subjects
    unsigned int NoOfSubjects;
// Subject Name
	String SubjectName;

// Root Segment
//    String       RootSegmentName;

// No. of total Segments
    unsigned int NoOfSegments;
//	Segment Name
    String       SegmentName;

// Parent// Children
//    String       ParentName;
//    String       ChildName;

	double		Translation[ 3 ];
	double		Velocity[3];
	double      RotationMatrix[ 9 ];
	double      RotationEuler[ 3 ];

	long long time_to_go;
	unsigned long long  seqNo;
}TState;

/*
typedef struct PTAM_STATE{

	double		Translation[ 3 ];
	double		Velocity[3];
	double      RotationMatrix[ 9 ];
	unsigned long long  seqNo;
}PState;
*/
typedef struct Simple_DATA //52Byte
{
	//check for state of ptam
	int signal;

	//position directly from ptam in mm
	//1000 = 1(m)
	int x;
	int y;
	int z;

	//rotation matrix with 1,000,000 times scale
	// r11=1,000,000 then R(11)=1

	int r11;
	int r12;
	int r13;
	int r21;
	int r22;
	int r23;
	int r31;
	int r32;
	int r33;

}UState;//52Byte

typedef struct PTAM_Signal{

	int signal;
}Sig;


typedef struct Desired_Motion{
	double x;
	double y;
	double z;
	double vx;
	double vy;
	double vz;
	double ax;
	double ay;
	double az;
}DMotion;

/*
typedef struct Haptic_Data
{
	double i[3];
	double acquisition_time;
	double time_to_go;
	int Loss;
}Data;
*/
typedef struct SEND_PACKET
{
	Data data;
}Packet_send;