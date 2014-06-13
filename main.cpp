#include "communication.h"
#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <time.h>

#define USE_SERIAL_OUT_BUFFER

/************************************** MultiWii Serial Protocol *******************************************************/
// Multiwii Serial Protocol 0 
#define MSP_VERSION              0

 #define PIDITEMS 10
 #define RC_CHANS 12
 int16_t rcData[RC_CHANS];    // interval [1000;2000]

 uint8_t confP[PIDITEMS];
 uint8_t confI[PIDITEMS];
 uint8_t confD[PIDITEMS];

//to multiwii developpers/committers : do not add new MSP messages without a proper argumentation/agreement on the forum
#define MSP_IDENT                100   //out message         multitype + multiwii version + protocol version + capability variable
#define MSP_STATUS               101   //out message         cycletime & errors_count & sensor present & box activation & current setting number
#define MSP_RAW_IMU              102   //out message         9 DOF
#define MSP_SERVO                103   //out message         8 servos
#define MSP_MOTOR                104   //out message         8 motors
#define MSP_RC                   105   //out message         8 rc chan and more
#define MSP_RAW_GPS              106   //out message         fix, numsat, lat, lon, alt, speed, ground course
#define MSP_COMP_GPS             107   //out message         distance home, direction home
#define MSP_ATTITUDE             108   //out message         2 angles 1 heading
#define MSP_ALTITUDE             109   //out message         altitude, variometer
#define MSP_ANALOG               110   //out message         vbat, powermetersum, rssi if available on RX
#define MSP_RC_TUNING            111   //out message         rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_PID                  112   //out message         P I D coeff (9 are used currently)
#define MSP_BOX                  113   //out message         BOX setup (number is dependant of your setup)
#define MSP_MISC                 114   //out message         powermeter trig
#define MSP_MOTOR_PINS           115   //out message         which pins are in use for motors & servos, for GUI 
#define MSP_BOXNAMES             116   //out message         the aux switch names
#define MSP_PIDNAMES             117   //out message         the PID names
#define MSP_WP                   118   //out message         get a WP, WP# is in the payload, returns (WP#, lat, lon, alt, flags) WP#0-home, WP#16-poshold
#define MSP_BOXIDS               119   //out message         get the permanent IDs associated to BOXes
#define MSP_SERVO_CONF           120   //out message         Servo settings

#define MSP_SET_RAW_RC           200   //in message          8 rc chan
#define MSP_SET_RAW_GPS          201   //in message          fix, numsat, lat, lon, alt, speed
#define MSP_SET_PID              202   //in message          P I D coeff (9 are used currently)
#define MSP_SET_BOX              203   //in message          BOX setup (number is dependant of your setup)
#define MSP_SET_RC_TUNING        204   //in message          rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_ACC_CALIBRATION      205   //in message          no param
#define MSP_MAG_CALIBRATION      206   //in message          no param
#define MSP_SET_MISC             207   //in message          powermeter trig + 8 free for future use
#define MSP_RESET_CONF           208   //in message          no param
#define MSP_SET_WP               209   //in message          sets a given WP (WP#,lat, lon, alt, flags)
#define MSP_SELECT_SETTING       210   //in message          Select Setting Number (0-2)
#define MSP_SET_HEAD             211   //in message          define a new heading hold direction
#define MSP_SET_SERVO_CONF       212   //in message          Servo settings
#define MSP_SET_MOTOR            214   //in message          PropBalance function

#define MSP_BIND                 240   //in message          no param

#define MSP_EEPROM_WRITE         250   //in message          no param

#define MSP_DEBUGMSG             253   //out message         debug string buffer
#define MSP_DEBUG                254   //out message         debug1,debug2,debug3,debug4



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/************************************************* Serial functions *****************************************************************/
static uint8_t cmdMSP;
static uint8_t rcvChecksum;
static uint8_t readIndex;

int los = 0; // distnacia
int relativedir = 0;

float MwAngle[2] = {0, 0};        // Those will hold Accelerator Angle

int16_t GPS_distanceToHome = 0;
uint8_t GPS_fix = 0;


int32_t GPS_altitude = 0;
uint16_t GPS_speed = 0;
int16_t GPS_directionToHome = 0;
uint8_t GPS_numSat = 0;
int16_t heading = 0 ;
int latitude = 0;
int longitude = 0;
long altitude_num2 = 0;
/////////////////////////////////////////////////////
int received_messages = 0;


int GPSfix = 0;
int MwHeading = 0;
int vbat =0;
int powermeter = 0;
int mwcurrent = 0;
int vario = 0;


#define SERIALBUFFERSIZE 256
static uint8_t serialBuffer[SERIALBUFFERSIZE]; // this hold the imcoming string from serial O string
static uint8_t receiverIndex;
static uint8_t dataSize;


#define ARMEDMODE    1 //0b00000001
#define STABLEMODE   2//0b00000010
#define HORIZONMODE  4//0b00000100
#define BAROMODE     8
#define MAGMODE      16
#define HEADFREEMODE 32
#define HEADFREEADJ  64
#define GPSHOMEMODE  128//0b00001000
#define GPSHOLDMODE  256//0b00010000


uint8_t mode_armed = 1;
uint8_t mode_stable = 1 ;
uint8_t mode_horizon = 1;
uint8_t mode_baro = 1;
uint8_t mode_mag = 1;
uint8_t mode_gpshome = 1;
uint8_t mode_gpshold = 1;
uint8_t mode_osd_switch = 1;












/////////////////////////////////////qq
int request = 0;
communication* comm;
int fd = 0;
uint8_t read8();
uint16_t read16();
uint32_t read32();
//uint16_t msgindex = 0;
uint8_t checksum = 0;
int should_process_now = 0;
int temp1 = 0;
uint32_t read32()
{
    uint32_t t = read16();
    t |= (uint32_t)read16() << 16;
    return t;
}

uint16_t read16()
{
    uint16_t t = read8();
    t |= (uint16_t)read8() << 8;
    return t;
}

uint8_t read8()
{
//printf("%d",readIndex);
if (readIndex>254)
{
readIndex = 0;


}
    return serialBuffer[readIndex++];
    //return comm->serialport_read(fd);
}

void blankserialRequest(uint8_t requestMSP)
{
    comm->serialport_writebyte(fd,(uint8_t)'$');
    comm->serialport_writebyte(fd,(uint8_t)'M');
    comm->serialport_writebyte(fd,(uint8_t)'<');
    comm->serialport_writebyte(fd,(uint8_t)0x00);
    comm->serialport_writebyte(fd,(uint8_t)requestMSP);
    comm->serialport_writebyte(fd,(uint8_t)requestMSP);
}

void serialMSPCheck()
{
//    printf("\nbuffer: %d %d %d %d %d %d", serialBuffer[0],serialBuffer[1],serialBuffer[2],serialBuffer[3],serialBuffer[4],serialBuffer[5]);
    readIndex = 0;
    received_messages++;
    if (cmdMSP == MSP_RAW_GPS)
    {
        GPS_fix = read8();
        GPS_numSat = read8();
        latitude = read32();
        longitude = read32();
        //read32();
        //read32();
        read16();
        GPS_speed = read16() / 10;
        printf("received gps data! lat: %d, lon: %d, fix: %d, sats: %d\n", latitude,longitude,GPS_fix,GPS_numSat); 
    }
     else if (cmdMSP == MSP_COMP_GPS)
    {
        los = GPS_distanceToHome = read16();
        int homedir = read16();
        relativedir = homedir - heading - 20 ;
        if (relativedir < -180)
        {
            relativedir += 360;
        }
        else if (relativedir > 180)
        {
            relativedir -= 360;
        }
    }
    else if (cmdMSP == MSP_ATTITUDE)
    {
        for (uint8_t i = 0; i < 2; i++)
            MwAngle[i] = float((int16_t)(read16())) / 10;
        MwHeading = read16();
        heading = MwHeading;
        printf("received attitude! roll: %.2f pitch: %.2f \n", MwAngle[0], MwAngle[1]);
        
    }
    else if (cmdMSP == MSP_ANALOG)
    {
        vbat = read8();
        powermeter = read16();
        //rssi = read16();
        read16();
        mwcurrent = read16();
        
    }
    else if (cmdMSP == MSP_ALTITUDE)
    {
        GPS_altitude =int(read32()) / 10;
        vario = read16();
        //powermeter = read16();
        //rssi = read16();
        //mwcurrent = read16();
        
        
    }
    else if (cmdMSP == MSP_STATUS)
    {
        read16();
        read16();
        read16();
        int32_t mode = read32();
        mode_stable = mode & STABLEMODE;
        mode_horizon = mode & HORIZONMODE;
        mode_gpshold = mode & GPSHOLDMODE;
        mode_gpshome = mode & GPSHOMEMODE;
        mode_mag = mode & MAGMODE;
        mode_baro = mode & BAROMODE;
        mode_armed = mode & ARMEDMODE;

    }
    else if (cmdMSP == MSP_RC)
    {
        for (int j = 0; j < RC_CHANS; j++)
        {
            rcData[j] = read16();
        }
        //rc_updated_flag = 1;


    }
    else if (cmdMSP == MSP_PID)
    {

        for (char i = 0; i < PIDITEMS; i++)
        {
            confP[i] = read8();
            confI[i] = read8();
            confD[i] = read8();

        }
        //pid_reloaded_flag = 1;
    }
 should_process_now = 0;
}

// End of decoded received commands from MultiWii
// --------------------------------------------------------------------------------------


static enum _serial_state
{
    IDLE,
    HEADER_START,
    HEADER_M,
    HEADER_ARROW,
    HEADER_SIZE,
    HEADER_CMD,
}
c_state = IDLE;

void receive_loop()
{
 while(1)
{
 char c = 12;
char ok= -1;
 blankserialRequest(MSP_ATTITUDE);
 while(comm->serialport_available(fd)){
   c = comm->serialport_read(fd);
 
   printf("%d ok: %d\n",c, ok);
 }
}
}

// receives whole commands from multiwii
void serialMSPreceive()
{
    char c;
    //int available = 0;
    //while (UCSR0A & (1 << RXC0))
    while (comm->serialport_available(fd))
    {
	//comm->available(fd,&c);
        c = comm->serialport_read(fd);
        //printf("%c",c);
    
        //#c = comm->serialport_read(fd);

        if (c_state == IDLE)
        {
            c_state = (c == '$') ? HEADER_START : IDLE;
        }
        else if (c_state == HEADER_START)
        {
            c_state = (c == 'M') ? HEADER_M : IDLE;
        }
        else if (c_state == HEADER_M)
        {
            c_state = (c == '>') ? HEADER_ARROW : IDLE;
        }
        else if (c_state == HEADER_ARROW)
        {
            if (c > SERIALBUFFERSIZE)
            {
                // now we are expecting the payload size
                c_state = IDLE;
            }
            else
            {
                dataSize = c;
                c_state = HEADER_SIZE;
                rcvChecksum = c;
            }
        }
        else if (c_state == HEADER_SIZE)
        {
            c_state = HEADER_CMD;
            cmdMSP = c;
            rcvChecksum ^= c;
            receiverIndex = 0;
        }
        else if (c_state == HEADER_CMD)
        {
            rcvChecksum ^= c;
            if (receiverIndex == dataSize) // received checksum byte
            {
                if (rcvChecksum == 0)
                {
                    should_process_now = 1;
                }
                else
                {
                    temp1 = rcvChecksum;
                }
                c_state = IDLE;
            }
            else
                serialBuffer[receiverIndex++] = c;
        }
    }
 //printf("\n");
}

void requestData()
{
 switch(request){
 case 0:
 	blankserialRequest(MSP_ATTITUDE);
        break;
 case 1:
        blankserialRequest(MSP_RAW_GPS);
}

request +=1;
request = request % 3;
}

int main()
{
 communication* comm = new communication();
// fd = comm->serialport_init("/dev/ttyUSB0",115200);
 fd = comm->serialport_init("/dev/ttyUSB0",115200);
 //SerialBegin();

  struct timespec last_time, current_time;
  clock_gettime(CLOCK_MONOTONIC ,&last_time);
  /* start after one second */
 usleep(5000000);
 char c = 0;/*
 while(1)
 {
  clock_gettime(CLOCK_MONOTONIC ,&current_time);
  if ( current_time.tv_sec > last_time.tv_sec || current_time.tv_nsec > last_time.tv_nsec+500000)
  {
    clock_gettime(CLOCK_MONOTONIC ,&last_time); 
    printf("requesting data\n");
    requestData();
  }
  //printf("%d chars avaialble\n",comm->serialport_available(fd)); 
  while(comm->serialport_available(fd))
  {
   //printf("[%d]",comm->serialport_available(fd));
    c= comm->serialport_read(fd);
    //printf("%c",c);}
  }*/
 //receive_loop();
 while(1)
 {
    clock_gettime(CLOCK_MONOTONIC ,&current_time);
  if ( current_time.tv_sec > last_time.tv_sec || current_time.tv_nsec > last_time.tv_nsec+50000000)
  {
    clock_gettime(CLOCK_MONOTONIC ,&last_time); 
    printf("requesting data\n");
    requestData();
  }

  requestData();
  char rec; // comm->serialport_read(fd);
  //comm->available(fd,&rec);
  serialMSPreceive();
  //printf("done\n");
  if(should_process_now)
  {
    serialMSPCheck();
  }
/*
  if (received_messages >=100)
  {
    return 0;
  }*/
 }
}
