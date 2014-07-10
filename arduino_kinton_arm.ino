
//  Serial Data protocol
//  +--------------+----------+---------------------+---------------+------------+
//  | Sync         | Command  | Data length         | Data          | Checksum   |
//  | (2 bytes)    | (1 byte) | (2 bytes)           | (0-38 bytes)  | (1 byte)   |
//  +--------------+----------+---------------------+---------------+------------+
//

//Library 
#include <Servo.h>
#include <MsTimer2.h>
#include <avr/wdt.h>

// num servos
#define N_SERVOS 6

typedef enum {SET_INI=0x69,
              ATTACH=0x61,
              DETACH=0x64,
              SET_JOINTS=0x73,
              GET_JOINTS=0x67,
              IS_FINISHED=0x66,
              RUNNING=0x11,
              ACK=0x06,
              NAK=0x15} ArmCmd;
              
typedef enum {read_sync,
              read_cmd,
              read_length,
              read_data,
              read_checksum} read_state;

union ShortByteUnion{
    signed short asShort;
    unsigned char asBytes[2];
};

union FloatByteUnion{
    float asFloat;
    unsigned char asBytes[4];
};

union SUnsCharUnion{
    signed char asSig;
    unsigned char asUnsig;
};


// Serial COMM
double serial_speed = 115200;

// led
int led = 13;

// Servo definitions
Servo armServo[N_SERVOS]; 

// Pin Definitions 
//const int servo_pin[N_SERVOS] = {3, 5, 6, 9, 10, 11};
const int servo_pin[N_SERVOS] = {2, 3, 4, 5, 6, 7};

// Joint initial positions
float joint_ini[N_SERVOS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
int joint_ini_us[N_SERVOS] = {1474, 780, 1972, 1580, 1627, 1627};

// Saturation values
float servo_min[N_SERVOS] = {-67.5,  0.0,   0.0,   -85.0, -30.0, -110.0};
float servo_max[N_SERVOS] = {67.5, 135.0, 115.0,    85.0,  30.0, 110.0};
// Global vars initialization
int pos_us[N_SERVOS] = {0, 0, 0, 0, 0, 0};
// Positions during movement
float pos_deg[N_SERVOS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
// Initial positions before movement
float pos_deg_ini[N_SERVOS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
// Final required positions
float pos_deg_end[N_SERVOS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
// Servo velocities
float servo_vel[N_SERVOS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
// Servo movement angle range 
float angle_range[N_SERVOS] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//Empty array of data
const unsigned char empty_data[38] = {};
// Max angle range
float range_max = 0.0;
// Number of iterations done to reach the desired angles
int steps_done = 0;
// Total number of iterations to reach the desired angles
int num_steps = 0;
// Minimum iteration time (20ms -> 50Hz)
float t_step = 0.02;
// Initial joint velocity set to 180 degrees/sec  
int deg_x_sec = 50;
// Time since last communication using serial
unsigned long time_last_comm = 0;

// Activate motion
boolean start_moving = false;
// Enable servo writtings in the separate timer
boolean enable_write = false;
// Send serial end if the movement has finished
boolean finished = true;

//************ Send DATA *******************
boolean send_data(ArmCmd cmd_id,const unsigned char* out_data, boolean ack_check=true)
{
  ShortByteUnion length;
  switch(cmd_id)
  {
    case SET_JOINTS: 
      length.asShort = 38;
      break;
    default: 
      length.asShort = 0;
      break;
  }
  
  // 2B sync + 1B cmd + 2B length + data + 1B CRC
  unsigned char data[2+1+2+length.asShort+1];
  memset(data, 0, sizeof(data));
  
  // Sync Bytes
  data[0] = 0xFF;
  data[1] = 0xFF;
  // Cmd Byte
  data[2] = cmd_id;
  // Length Byte
  data[3] = length.asBytes[1];
  data[4] = length.asBytes[0];
  // CRC
  uint8_t checksum = (uint8_t)data[2] + (uint8_t)(length.asBytes[0]) + (uint8_t)(length.asBytes[1]);
    
  // Data Bytes
  for (int ii = 0; ii < length.asShort; ++ii)
  {
    data[ii+5] = out_data[ii];
    checksum = checksum + (uint8_t)out_data[ii];
  }
  // CRC
  data[length.asShort+5] = (unsigned char)(checksum%0xFF);

  for(int ii=0;ii<length.asShort+6;++ii){
    Serial.write(data[ii]);
  }

  if(cmd_id!=ACK && cmd_id!=NAK){
    if(ack_check && !check_ack())
      return false;
  }
    
  return true;
}

//************ Receive DATA *******************
void receive_data(ArmCmd& cmd_id, unsigned char* data, boolean ack_check=true)
{
  boolean success = false;

  ShortByteUnion length;
  int num=0,byte_read=0,num_sync_bytes=2,num_length_bytes=2;
  uint8_t checksum=0;
  read_state state = read_sync; 
 
  ArmCmd cmdByte=NAK;
  unsigned char in_data[44];
  memset(in_data, 0, sizeof(in_data));
  unsigned char read_byte[38];
  memset(read_byte, 0, sizeof(read_byte));
  int read_=0;
  SUnsCharUnion inByte;
  
  boolean new_input = false;
  
  while (Serial.available() > 0) {
    inByte.asSig = Serial.read();
    in_data[read_] = inByte.asUnsig;
    read_++;
    delayMicroseconds(90);
    new_input = true;
  }
  //Serial.flush();
  
  int databyte = 0;
  
  if(read_>0){
    for(int ii=0;ii<read_;ii++){
      switch(state){
        case read_sync:
          if(in_data[ii]==0xFF){
            num_sync_bytes--;
            if(num_sync_bytes==0){
              num_sync_bytes=2;
              checksum=0;
              state=read_cmd;
            }
          }
          else num_sync_bytes=2;
          break; 
        case read_cmd:
          if (in_data[ii] ==0xFF){
            num_sync_bytes--;
            if(num_sync_bytes==0){
              num_sync_bytes=2;
              checksum=0;
              state=read_cmd;
            }
          }
          else{
            num_sync_bytes=2;
            if(in_data[ii]==SET_INI) cmdByte=SET_INI;
            else if(in_data[ii]==ATTACH) cmdByte=ATTACH;
            else if(in_data[ii]==DETACH) cmdByte=DETACH;            
            else if(in_data[ii]==SET_JOINTS) cmdByte=SET_JOINTS;
            else if(in_data[ii]==GET_JOINTS) cmdByte=GET_JOINTS;
            else if(in_data[ii]==IS_FINISHED) cmdByte=IS_FINISHED; 
            else if(in_data[ii]==ACK) cmdByte=ACK;
            else cmdByte=NAK;
            checksum = checksum+(uint8_t)in_data[ii];
            state = read_length;
            num_length_bytes = 2;
          }
          break;        
        case read_length: 
            if (in_data[ii] == 0xFF){
              num_sync_bytes--;
              if(num_sync_bytes==0){
                num_sync_bytes=2;
                checksum=0;
                state=read_cmd;
              }
            }
            else{
              num_length_bytes--;
              num_sync_bytes=2;
              if(num_length_bytes==1) length.asBytes[1] = in_data[ii];
              if(num_length_bytes==0){
               num_length_bytes = 2;
               length.asBytes[0] = in_data[ii];
               checksum = checksum+(uint8_t)(length.asBytes[0])+(uint8_t)(length.asBytes[1]);
               if(length.asShort>0 && length.asShort==38){
                state=read_data;
                databyte=0;
               }
               else
                state=read_checksum;  
              }
            }           
          break;
        case read_data: 
            if (in_data[ii] == 0xFF){
              num_sync_bytes--;
              if(num_sync_bytes==0){
                num_sync_bytes=2;
                checksum=0;
                state=read_cmd;
              }
            } 
            else{   
              if(databyte < length.asShort){
                read_byte[databyte] = in_data[ii]; 
                checksum = checksum+(uint8_t)in_data[ii];
                databyte++;
                if(databyte==length.asShort) state = read_checksum;
              }
            }
          break;
        case read_checksum:
          if (in_data[ii] == 0xFF){
            num_sync_bytes--;
            if(num_sync_bytes==0){
              num_sync_bytes=2;
              checksum=0;
              state=read_cmd;
            }
          }
          else{
            num_sync_bytes=2;
            if (checksum%0xFF == (uint8_t)in_data[ii]){
              success = true;
            }
            state=read_sync;              
          }
          break;  
      }
    }
    if(success) {
      if (ack_check && cmdByte!=ACK && cmdByte!=NAK)
          send_data(ACK,empty_data);
      for(int jj=0;jj<38;jj++)
        data[jj] = read_byte[jj];
      cmd_id=cmdByte;     
    }
    else
      send_data(NAK,empty_data);
  }
}

//************* Check ACK *******************
boolean check_ack()
{
  ArmCmd cmd_id=NAK;
  unsigned char data[38];
  memset(data, 0, sizeof(data));
  receive_data(cmd_id,data);
  
  if(cmd_id!=ACK)
    return false;
  return true;  
}

void send_positions(float* pos, int vel, boolean ack_check = true)
{
  unsigned char out_data[38]; // 2 Bvel + 36 Bpos
  memset(out_data, 0, sizeof(out_data));
  
  //velocity
  ShortByteUnion vel_u;
  vel_u.asShort = (short int)vel;
  out_data[0] = vel_u.asBytes[0];
  out_data[1] = vel_u.asBytes[1];
 
  //Joints and positions
  for (unsigned int ii = 0; ii < N_SERVOS; ++ii)
  {
    // Joint number
    ShortByteUnion joint_u;
    joint_u.asShort = (short int)ii;
    out_data[2+ii*6] = joint_u.asBytes[0];
    out_data[2+ii*6+1] = joint_u.asBytes[1];

    // Position
    FloatByteUnion pos_u;
    pos_u.asFloat = pos[ii];
    out_data[2+ii*6+2] = pos_u.asBytes[0];
    out_data[2+ii*6+3] = pos_u.asBytes[1];
    out_data[2+ii*6+4] = pos_u.asBytes[2];
    out_data[2+ii*6+5] = pos_u.asBytes[3];
  }
  
  send_data(SET_JOINTS,out_data,ack_check);
}

void get_new_positions(unsigned char* data)
{
  // get velocity
  ShortByteUnion vel_u;
  vel_u.asBytes[0] = data[0];  
  vel_u.asBytes[1] = data[1];
  deg_x_sec = vel_u.asShort;

  //Joints and positions
  for (int ii = 0; ii < N_SERVOS; ++ii)
  {
    // Joint number
    ShortByteUnion joint_u;
    joint_u.asBytes[0] = data[2+ii*6];
    joint_u.asBytes[1] = data[2+ii*6+1];
    int joint = (int)joint_u.asShort;

    // Position
    FloatByteUnion pos_u;
    pos_u.asBytes[0] = data[2+ii*6+2]; 
    pos_u.asBytes[1] = data[2+ii*6+3]; 
    pos_u.asBytes[2] = data[2+ii*6+4]; 
    pos_u.asBytes[3] = data[2+ii*6+5]; 
    pos_deg_end[joint] = pos_u.asFloat;
  }
}

//****************** Get servos velocities **********
void get_motion_values()
{
  range_max=0.0;
  for (int ii = 0; ii < N_SERVOS; ++ii){
    angle_range[ii] =  pos_deg_end[ii] - pos_deg[ii];   
    if (range_max < abs(angle_range[ii])){
     range_max = abs(angle_range[ii]);
    }
  }

  float t_total = range_max/deg_x_sec;

  num_steps = round(t_total/t_step);
  
  if(num_steps>0){
    for (int ii = 0; ii < N_SERVOS; ++ii){
      float angle_step = angle_range[ii]/(float)num_steps;
      servo_vel[ii] = angle_step/t_step;   
    }
    start_moving = true;
  }
  else {
    for (int ii = 0; ii < N_SERVOS; ++ii){
      servo_vel[ii] = 0.0;   
    }
    start_moving = false;
  } 
}

//******************* Move to default values *************
void move_to_default()
{
  for (int ii = 0; ii < N_SERVOS; ++ii){
    pos_us[ii]=joint_ini_us[ii];
  }
  write_to_servos(); 
}

// **************** S-curve to smooth movement ************
void moveSmooth()
{  
  float pi = 3.14159;
  for (int joint=0;joint<N_SERVOS;++joint){
    //pos_deg[joint] = pos_deg_end[joint];
    //pos_deg[joint] =  pos_deg[joint]+servo_vel[joint]*t_step; 
    float num = (pi*steps_done)/num_steps;
    pos_deg[joint] =  pos_deg_ini[joint]+((angle_range[joint]/pi)*(num-cos(num)*sin(num)));
  }  
}

// *********** Deg to us ******************
void deg_to_us()
{  
  float middle = 90.0;
  // Relationship between real and desired rotation (differential joint)
  float alpha6 = 0.7;
  pos_us[0] = 1474 + int(pos_deg[0]*10.37);
  pos_us[1] = 780 + int(pos_deg[1]*10.37);
  pos_us[2] = 1972 - int(pos_deg[2]*10.37);
  pos_us[3] = 1580 - int(pos_deg[3]*9.412);
  pos_us[4] = 780 + int((middle+pos_deg[4]-(pos_deg[5]*alpha6)) * 9.412);
  pos_us[5] = 780 + int((middle-pos_deg[4]-(pos_deg[5]*alpha6)) * 9.412);
}

//******************** Write to servos ************
void write_to_servos()
{
  // Servos 0 to 2 -> 780 - 1400 us
  // Servos 3 to 6 -> 780 - 1600 us
  for (int ii = 0; ii < N_SERVOS; ++ii){
    armServo[ii].writeMicroseconds(pos_us[ii]);
    delay(1);
  }
}

//***************** Function to move the joints ****************
void moveServos()
{
  if(enable_write){
    // smooth movements
    moveSmooth();
  
    // Compute miliseconds
    deg_to_us(); 
    
    // Send servo commands
    write_to_servos();

    // check if end moving
    steps_done = steps_done + 1;
    if(steps_done == num_steps){
      enable_write = false;
      finished = true;
    }
  }
}
  

// *************** Attach servos **************
void attachServos()
{
//  for (int ii = 0; ii < N_SERVOS; ++ii)
//    armServo[ii].attach(servo_pin[ii]);
  
  for (int ii = 0; ii < 3; ++ii)  
    armServo[ii].attach(servo_pin[ii],774,2174);  
    
  for (int ii = 3; ii < N_SERVOS; ++ii)  
    armServo[ii].attach(servo_pin[ii],780,2380);
}

// *************** Detach servos **************
void detachServos()
{
  for (int ii = 0; ii < N_SERVOS; ++ii) 
    armServo[ii].detach();
}


//******************* Initial Setup **************************
void setup() {   
  
  // initialize the digital pin as an output.
  pinMode(led, OUTPUT);   
  
  // Initialize serial communication
  Serial.begin(serial_speed);

  // Attach servos
  attachServos();
  
  // Move to default position
  move_to_default();
    
  MsTimer2::set(20, moveServos);
  MsTimer2::start();
}

//********************* Main loop *****************************
void loop() {
  ArmCmd cmd_id=NAK;
  unsigned char data[38];
  memset(data, 0, sizeof(data));
  
  receive_data(cmd_id,data);

//  if(cmd_id!=NAK) time_last_comm=millis();
//  unsigned long total_time = millis();
//  if ((total_time - time_last_comm) > 1000) {
//    move_to_default();
//    time_last_comm = millis();
//  }
  
  switch(cmd_id){
   case ATTACH:
     attachServos();
     break;    
   case DETACH:
     detachServos();
     break;
   case SET_INI:
     for (int ii = 0; ii < N_SERVOS; ++ii)
       pos_deg_end[ii] = joint_ini[ii];
     get_motion_values();
     break;
   case SET_JOINTS:
     get_new_positions(data);
     for (int ii = 0; ii < N_SERVOS; ++ii) {
       if (pos_deg_end[ii] < servo_min[ii]) pos_deg_end[ii]=servo_min[ii];
       if (pos_deg_end[ii] > servo_max[ii]) pos_deg_end[ii]=servo_max[ii];
     } 
     get_motion_values();
     break;
   case GET_JOINTS:
     send_positions(pos_deg,deg_x_sec);
     break;
   case IS_FINISHED:
     if(finished)       
       send_data(IS_FINISHED,empty_data);
     else
       send_data(RUNNING,empty_data);
    break;
   default: 
    break; 
  }
  
  if(start_moving){
   start_moving = false;
   finished = false;
   enable_write = true;
   steps_done = 0;
   for (int ii = 0; ii < N_SERVOS; ++ii) 
     pos_deg_ini[ii] = pos_deg[ii];
  } 
  
}




