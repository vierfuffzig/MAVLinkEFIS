/* MAVLInk_DroneLights
 *  by Juan Pedro López
 *  
 * This program was developed to connect an Arduino board with a Pixhawk via MAVLink 
 *   with the objective of controlling a group of WS2812B LED lights on board of a quad
 * 
 * The current version of the program is working properly.
 * 
 * TO DO:
 *  - Move STREAMS request to RC_CHANNELS to use values in logic
 *  - Add RC_CHANNLES_RAW messages monitoring: move #30 to RC_CHANNELS_RAW (#35)
 *      http://mavlink.org/messages/common#RC_CHANNELS_RAW
 *  - Look for message on low battery:
 *      To be tested: http://mavlink.org/messages/common#PARAM_REQUEST_READ
 *      To be checked: http://mavlink.org/messages/common#SYS_STATUS
 *  - Potential implementation of other alarms, like high intensity
 *      
 * You can restrict the maximum package size with this parameter in mavlink_types.h:

    #ifndef MAVLINK_MAX_PAYLOAD_LEN_
    // it is possible to override this, but be careful! Defa_
    #define **MAVLINK_MAX_PAYLOAD_LEN 255 ///< Maximum payload length_
    #endif_
 */


// In case we need a second serial port for debugging
// #define SOFT_SERIAL_DEBUGGING   // Comment this line if no serial debugging is needed
#ifdef SOFT_SERIAL_DEBUGGING
// Library to use serial debugging with a second board
#include <SoftwareSerial.h>
SoftwareSerial mySerial(11, 12); // RX, TX
SoftwareSerial pxSerial(9,10);   // RX, TX
#endif

#include "mavlink.h"
//#include "common/mavlink_msg_request_data_stream.h"






////////////////////////////////////////// TFT Setup ///////////////////////////////////////////////////////////

#include <SPI.h>

// Use ONE of these three highly optimised libraries, comment out other two!

// For S6D02A1 based TFT displays
//#include <TFT_S6D02A1.h>         // Bodmer's graphics and font library for S6D02A1 driver chip
//TFT_S6D02A1 tft = TFT_S6D02A1(); // Invoke library, pins defined in User_Setup.h
//                                    https://github.com/Bodmer/TFT_S6D02A1

// For ST7735 based TFT displays
#include <TFT_ST7735.h>          // Bodmer's graphics and font library for ST7735 driver chip
TFT_ST7735 tft = TFT_ST7735();   // Invoke library, pins defined in User_Setup.h
//                                    https://github.com/Bodmer/TFT_ST7735

// For ILI9341 based TFT displays (note sketch is currently setup for a 160 x 128 display)
//#include <TFT_ILI9341.h>         // Bodmer's graphics and font library for ILI9341 driver chip
//TFT_ILI9341 tft = TFT_ILI9341(); // Invoke library, pins defined in User_Setup.h
//                                    https://github.com/Bodmer/TFT_ILI9341

#define REDRAW_DELAY 16 // minimum delay in milliseconds between display updates

#define HOR 102    // Horizon circle outside radius (205 is corner to corner

#define BROWN      0x5140 //0x5960
#define SKY_BLUE   0x02B5 //0x0318 //0x039B //0x34BF
#define DARK_RED   0x8000
#define DARK_GREY  0x39C7

#define XC 64 // x coord of centre of horizon
#define YC 80 // y coord of centre of horizon

#define ANGLE_INC 1 // Angle increment for arc segments, 1 will give finer resolution, 2 or more gives faster rotation

#define DEG2RAD 0.0174532925

//int roll = 3;
//int pitch;

int roll_angle = 180; // These must be initialed to 180 so updateHorizon(0); in setup() draws

int last_roll = 0; // the whole horizon graphic
int last_pitch = 0;

int roll_delta = 90;  // This is used to set arc drawing direction, must be set to 90 here


// Variables for test only
//int test_angle = 0;
//int delta = ANGLE_INC;

unsigned long redrawTime = 0;



////////////////////////////////////////////// end of tft setup ////////////////////////////////////////////////////////



///////////////////////////////////////////// Mavlink variables ///////////////////////////////////////////////////////////

unsigned long previousMillisMAVLink = 0;     // will store last time MAVLink was transmitted and listened
unsigned long next_interval_MAVLink = 1000;  // next interval to count
const int num_hbs = 60;                      // # of heartbeats to wait before activating STREAMS from Pixhawk. 60 = one minute.
int num_hbs_pasados = num_hbs;

int16_t roll = 0;
int16_t pitch = 0;
int16_t heading = 0;
float alt = 0;
float volt = 0;
int16_t rssi = 0;
float climb = 0;
float aspd = 0;
int16_t mode = 0;

//////////////////////////////////////////// end of mav variables //////////////////////////////////////////////////////////////



void setup() {
  // MAVLink interface start
  Serial.begin(57600);


#ifdef SOFT_SERIAL_DEBUGGING
  // [DEB] Soft serial port start
  Serial.begin(57600);
  
  mySerial.begin(57600);
  mySerial.println("MAVLink starting.");
#endif


///////////////// TFT setup ///////////////////////////////////////////////////

  tft.begin();
  tft.setRotation(0);

  tft.fillRect(0,  0, 128, 80, SKY_BLUE);
  tft.fillRect(0, 80, 128, 80, BROWN);

  // Draw the horizon graphic
  drawHorizon(0, 0);
  drawInfo();
  delay(2000); // Wait to permit visual check

  // Test roll and pitch
  // testRoll();
  // testPitch();

  tft.setTextColor(TFT_YELLOW, SKY_BLUE);
  tft.setTextDatum(TC_DATUM);            // Centre middle justified
  tft.drawString("ArduPilot", 64, 10, 1);
}

void loop() {
  
  unsigned long currentMillis = millis();
  int i=0;

  if (currentMillis - previousMillisMAVLink >= next_interval_MAVLink) {
    // Keep time last mode changed
    previousMillisMAVLink = currentMillis;
  }
  


        
  // MAVLink
  /* The default UART header for your MCU */ 
  int sysid = 1;                   ///< ID 20 for this airplane. 1 PX, 255 ground station
  int compid = 158;                ///< The component sending the message
  int type = MAV_TYPE_QUADROTOR;   ///< This system is an airplane / fixed wing
 
  // Define the system type, in this case an airplane -> on-board controller
  // uint8_t system_type = MAV_TYPE_FIXED_WING;
  uint8_t system_type = MAV_TYPE_GENERIC;
  uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;
 
  uint8_t system_mode = MAV_MODE_PREFLIGHT; ///< Booting up
  uint32_t custom_mode = 0;                 ///< Custom mode, can be defined by user/adopter
  uint8_t system_state = MAV_STATE_STANDBY; ///< System ready for flight
  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
 
  // Pack the message
  //mavlink_msg_heartbeat_pack(sysid,compid, &msg, type, autopilot_type, system_mode, custom_mode, system_state);
  mavlink_msg_heartbeat_pack(1,0, &msg, type, autopilot_type, system_mode, custom_mode, system_state);
 
  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
 
  // Send the message with the standard UART send function
  // uart0_send might be named differently depending on
  // the individual microcontroller / library in use.
  unsigned long currentMillisMAVLink = millis();
  if (currentMillisMAVLink - previousMillisMAVLink >= next_interval_MAVLink) {
    // Guardamos la última vez que se cambió el modo
    previousMillisMAVLink = currentMillisMAVLink;

    Serial.write(buf, len);

    //Mav_Request_Data();
    num_hbs_pasados++;
    if(num_hbs_pasados>=num_hbs) {
      // Request streams from Pixhawk
#ifdef SOFT_SERIAL_DEBUGGING
      mySerial.println("Streams requested!");
#endif
      Mav_Request_Data();
      num_hbs_pasados=0;
    }

  }

  // Check reception buffer
  comm_receive();

////////////////////////////////////// TFT display //////////////////////////

  if (millis() > redrawTime) {
    redrawTime = millis() + REDRAW_DELAY;
    updateHorizon(roll, pitch);

  }
   
}




////////////////////////////////////// telemetry functions /////////////////////////////////////////////////////////////////////////


void Mav_Request_Data()
{
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];


  /*
   * Definitions are in common.h: enum MAV_DATA_STREAM
   *   
   * MAV_DATA_STREAM_ALL=0,                   // Enable all data streams
   * MAV_DATA_STREAM_RAW_SENSORS=1,           /* Enable IMU_RAW, GPS_RAW, GPS_STATUS packets.
   * MAV_DATA_STREAM_EXTENDED_STATUS=2,       /* Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS
   * MAV_DATA_STREAM_RC_CHANNELS=3,           /* Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW
   * MAV_DATA_STREAM_RAW_CONTROLLER=4,        /* Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT.
   * MAV_DATA_STREAM_POSITION=6,              /* Enable LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT messages.
   * MAV_DATA_STREAM_EXTRA1=10,               /* Dependent on the autopilot
   * MAV_DATA_STREAM_EXTRA2=11,               /* Dependent on the autopilot
   * MAV_DATA_STREAM_EXTRA3=12,               /* Dependent on the autopilot
   * MAV_DATA_STREAM_ENUM_END=13,
   * 
   */

  // To be setup according to the needed information to be requested from the Autopilot
  const int  maxStreams = 2;
  const uint8_t MAVStreams[maxStreams] = {
//        MAV_DATA_STREAM_RAW_SENSORS,
//        MAV_DATA_STREAM_EXTENDED_STATUS,
//        MAV_DATA_STREAM_RC_CHANNELS,
//        MAV_DATA_STREAM_POSITION,
        MAV_DATA_STREAM_EXTRA1, 
        MAV_DATA_STREAM_EXTRA2};
  const uint16_t MAVRates[maxStreams] = {0x05,0x05};

    
  for (int i=0; i < maxStreams; i++) {
    mavlink_msg_request_data_stream_pack(2, 200, &msg, 1, 0, MAVStreams[i], MAVRates[i], 1);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial.write(buf, len);
    }
}



void comm_receive() {
 
  mavlink_message_t msg;
  mavlink_status_t status;
 
  while(Serial.available()>0) {
    uint8_t c = Serial.read();

    // Try to get a new message
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {

      // Handle message
      switch(msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:  // #0: Heartbeat
          {
            mavlink_heartbeat_t heartbeat;
            mavlink_msg_heartbeat_decode(&msg, &heartbeat);

            mode = heartbeat.custom_mode;
          }
          break;

        case MAVLINK_MSG_ID_SYS_STATUS:  // #1: SYS_STATUS
          {
            mavlink_sys_status_t sys_status;
            mavlink_msg_sys_status_decode(&msg, &sys_status);
            
            volt = sys_status.voltage_battery * 0.001;
          }
          break;

        case MAVLINK_MSG_ID_VFR_HUD:
          {
            mavlink_vfr_hud_t vfr_hud;
            mavlink_msg_vfr_hud_decode(&msg, &vfr_hud);
            
            alt = (float)round(vfr_hud.alt * 0.01);
            climb = (float)round(vfr_hud.climb);
            aspd = (float)round(vfr_hud.airspeed);
            heading = (int16_t)round(vfr_hud.heading);
            // if (uav_heading >= 180 ) uav_heading = -360+uav_heading; //convert from 0-360 to -180/180°
          }
          break;

        case MAVLINK_MSG_ID_PARAM_VALUE:  // #22: PARAM_VALUE
          {
            mavlink_param_value_t param_value;
            mavlink_msg_param_value_decode(&msg, &param_value);
          }
          break;

        case MAVLINK_MSG_ID_ATTITUDE:  // #30
          {
            /* Message decoding: PRIMITIVE
             *    mavlink_msg_attitude_decode(const mavlink_message_t* msg, mavlink_attitude_t* attitude)
             */
            mavlink_attitude_t attitude;
            mavlink_msg_attitude_decode(&msg, &attitude);
            
            roll = attitude.roll * 57.2958;
            pitch = attitude.pitch * 57.2958;
          }
          break;

        
       default:
#ifdef SOFT_SERIAL_DEBUGGING

            mySerial.print("roll = ");
            mySerial.print(roll);
            mySerial.print("  pitch = ");
            mySerial.print(pitch);
            mySerial.print("  hdg = ");
            mySerial.print(heading);
            mySerial.print("  alt = ");
            mySerial.print(alt);            
            mySerial.print("  volt = ");
            mySerial.print(volt);
            mySerial.print("  rssi = ");
            mySerial.print(rssi);
            mySerial.print("  climb = ");
            mySerial.print(climb);
            mySerial.print("  aspd = ");
            mySerial.print(aspd);
            mySerial.print("  mode = ");
            mySerial.println(mode);

//int8_t roll = 0;
//int8_t pitch = 0;
//int8_t heading = 0;
//float alt = 0;
//float volt = 0;
//int8_t rssi = 0;
//float climb = 0;
//int8_t mode = 0;

            
#endif
          break;
      }
    }
  }
}

// #########################################################################
// Update the horizon with a new angle (angle in range -180 to +180)
// #########################################################################
void updateHorizon(int roll, int pitch)
{
  bool draw = 1;
  int delta_pitch = 0;
  int pitch_error = 0;
  int delta_roll  = 0;
  while ((last_pitch != pitch) || (last_roll != roll))
  {
    delta_pitch = 0;
    delta_roll  = 0;

    if (last_pitch < pitch) {
      delta_pitch = 1;
      pitch_error = pitch - last_pitch;
    }
    
    if (last_pitch > pitch) {
      delta_pitch = -1;
      pitch_error = last_pitch - pitch;
    }
    
    if (last_roll < roll) delta_roll  = 1;
    if (last_roll > roll) delta_roll  = -1;
    
    if (delta_roll == 0) {
      if (pitch_error > 1) delta_pitch *= 2;
    }
    
    drawHorizon(last_roll + delta_roll, last_pitch + delta_pitch);
    drawInfo();
  }
}

void drawHorizon(int roll, int pitch)
{
  //roll = uav_roll;
  // Fudge factor adjustment for this sketch (so horizon is horizontal when start angle is 0)
  // This is needed as we draw extra pixels to avoid leaving plotting artifacts

  // Calculate coordinates for line start
  float sx = cos(roll * DEG2RAD);
  float sy = sin(roll * DEG2RAD);

  int16_t x0 = sx * HOR;
  int16_t y0 = sy * HOR;

  if ((roll != last_roll) && ((abs(roll) > 35)  || (pitch != last_pitch)))
  {
    tft.drawLine(XC - x0, YC - y0 - 3 + pitch, XC + x0, YC + y0 - 3 + pitch, SKY_BLUE);
    tft.drawLine(XC - x0, YC - y0 + 3 + pitch, XC + x0, YC + y0 + 3 + pitch, BROWN);
    tft.drawLine(XC - x0, YC - y0 - 4 + pitch, XC + x0, YC + y0 - 4 + pitch, SKY_BLUE);
    tft.drawLine(XC - x0, YC - y0 + 4 + pitch, XC + x0, YC + y0 + 4 + pitch, BROWN);
  }

  tft.drawLine(XC - x0, YC - y0 - 2 + pitch, XC + x0, YC + y0 - 2 + pitch, SKY_BLUE);
  tft.drawLine(XC - x0, YC - y0 + 2 + pitch, XC + x0, YC + y0 + 2 + pitch, BROWN);

  tft.drawLine(XC - x0, YC - y0 - 1 + pitch, XC + x0, YC + y0 - 1 + pitch, SKY_BLUE);
  tft.drawLine(XC - x0, YC - y0 + 1 + pitch, XC + x0, YC + y0 + 1 + pitch, BROWN);

  tft.drawLine(XC - x0, YC - y0 + pitch,   XC + x0, YC + y0 + pitch,   TFT_WHITE);

  last_roll = roll;
  last_pitch = pitch;

}


// #########################################################################
// Draw the information
// #########################################################################

void drawInfo(void)
{
  // Update things near middle of screen first (most likely to get obscured)

  // Level wings graphic
  tft.fillRect(64 - 1, 80 - 1, 3, 3, TFT_RED);
  tft.drawFastHLine(64 - 30,   80, 24, TFT_RED);
  tft.drawFastHLine(64 + 30 - 24, 80, 24, TFT_RED);
  tft.drawFastVLine(64 - 30 + 24, 80, 3, TFT_RED);
  tft.drawFastVLine(64 + 30 - 24, 80, 3, TFT_RED);

  tft.drawFastHLine(64 - 12,   80 - 40, 24, TFT_WHITE);
  tft.drawFastHLine(64 -  6,   80 - 30, 12, TFT_WHITE);
  tft.drawFastHLine(64 - 12,   80 - 20, 24, TFT_WHITE);
  tft.drawFastHLine(64 -  6,   80 - 10, 12, TFT_WHITE);

  tft.drawFastHLine(64 -  6,   80 + 10, 12, TFT_WHITE);
  tft.drawFastHLine(64 - 12,   80 + 20, 24, TFT_WHITE);
  tft.drawFastHLine(64 -  6,   80 + 30, 12, TFT_WHITE);
  tft.drawFastHLine(64 - 12,   80 + 40, 24, TFT_WHITE);


  tft.setTextColor(TFT_WHITE);
  tft.setCursor(64 - 12 - 13, 80 - 20 - 3);
  tft.print("10");
  tft.setCursor(64 + 12 + 1, 80 - 20 - 3);
  tft.print("10");
  tft.setCursor(64 - 12 - 13, 80 + 20 - 3);
  tft.print("10");
  tft.setCursor(64 + 12 + 1, 80 + 20 - 3);
  tft.print("10");

  tft.setCursor(64 - 12 - 13, 80 - 40 - 3);
  tft.print("20");
  tft.setCursor(64 + 12 + 1, 80 - 40 - 3);
  tft.print("20");
  tft.setCursor(64 - 12 - 13, 80 + 40 - 3);
  tft.print("20");
  tft.setCursor(64 + 12 + 1, 80 + 40 - 3);
  tft.print("20");

  // Display justified angle value near bottom of screen
  tft.setTextColor(TFT_YELLOW, BROWN); // Text with background
  tft.setTextDatum(MC_DATUM);            // Centre middle justified
  tft.setTextPadding(24);                // Padding width to wipe previous number
  tft.drawNumber(roll, 64, 142, 1);

  // Draw fixed text
  tft.setTextColor(TFT_YELLOW);
  tft.setTextDatum(TC_DATUM);            // Centre middle justified
  tft.drawString("MAV Data", 64, 1, 1);
  tft.drawString("vierfuffzig", 64, 151, 1);


}
