/* Dynamixel Position Mode Example
 
 This example shows how to use dynamixel as position mode
 All dynamixels are set as joint mode in factory,

               Compatibility
 CM900                  O
 OpenCM9.04             O
 
                Dynamixel Compatibility
               AX    MX      RX    XL-320    Pro
 CM900          O      O      O        O      X
 OpenCM9.04     O      O      O        O      X
 **** OpenCM 485 EXP board is needed to use 4 pin Dynamixel and Pro Series ****
 
 created 22 May 2014
 by ROBOTIS CO,.LTD.
 */
#define DXL_BUS_SERIAL1 1  //Dynamixel on Serial1(USART1)  <-OpenCM9.04

typedef enum {RETURN_DELAY_TIME=5,
              CW_ANGLE_LIM=6,
              CCW_ANGLE_LIM=8,
              CONTROL_MODE=11,
              TORQUE_ENABLE=24,
              LED_ON=25,
              GOAL_POSITION=30,
              PRESENT_POSITION=37,
              MOVING=49} registers;

typedef enum {SET_INI=0x69,
              ATTACH=0x61,
              DETACH=0x64,
              SET_JOINTS=0x73,
              GET_JOINTS=0x67,
              IS_FINISHED=0x66,
              RUNNING=0x11,
              ACK=0x06,
              NAK=0x15} ArmCmd;

typedef enum {DETACHED,
              STOPPED,
              GOING} ArmState;

typedef enum {read_sync,
              read_cmd,
              read_length,
              read_data,
              read_checksum} read_state;
              
#define EEPROM_INSTR_DELAY 10
#define SERVO_INSTR_DELAY 1

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

// Servos Management *********
//****************************
#define N_SERVOS 6
Dynamixel Dxl(DXL_BUS_SERIAL1);
int serial_speed_ = 3; // Serial COMM. Dynamixel 2.0 Baudrate -> 0: 9600, 1: 57600, 2: 115200, 3: 1Mbp
HardwareTimer DYNXTimer(1);// Instanciate HardwareTimer class on timer device 1
#define DYNX_RATE 20000 // Microseconds
boolean DYNX_read_ok_ = false;
//boolean FLAG_dynx_ = false;

// USB Serial Management *******
//****************************
int usb_serial_speed_ = 921600;
//HardwareTimer USBTimer(2);// Instanciate HardwareTimer class on timer device 2
//#define USB_RATE 1000000 // Microseconds
//boolean FLAG_usb_ = false;

// Servo Variables ***********
//****************************
// Initial positions before movement
float pos_deg_ini_[] = {0.0, 0.0, -20.0, 135.0, 60.0, 0.0};
// Final required positions
float pos_deg_end_[] = {0.0, 0.0, -20.0, 135.0, 60.0, 0.0};
// Positions during movement
float pos_deg_[] = {0.0, 0.0, -20.0, 135.0, 60.0, 0.0};
// Range between current and desired positions
float angle_range_[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; 
// Desired velocity set for each servo
int servo_vel_[] = {100, 100, 100, 100, 100, 100};
// Saturation values
float servo_min_[] = {-20.0, -30.0, -20.0, -135.0, -135.0, -90.0};
float servo_max_[] = { 20.0,  30.0,  110.0, 135.0,  135.0,  90.0};

// S-Curve variables *********
//****************************
float pos_deg_singlemov_des_[] = {0.0, 0.0, 80.0, 100.0, 0.0, 0.0}; // Desired positions at the end of the S-curve
float pos_deg_singlemov_ini_[] = {0.0, 0.0, 80.0, 100.0, 0.0, 0.0}; // Positions at the beginning of the S-curve
int servo_vel_step_[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // Desired velocities during steps of the the S-curve
// Number of iterations done to reach the desired angles
int steps_done_ = 0;
// Total number of iterations to reach the desired angles
int num_steps_ = 0;
// Minimum iteration time (20ms -> 50Hz)
float t_step_ = 0.02;

// Arm variables *************
//****************************
// Arm state
ArmState arm_state_ = DETACHED;
// Moving
boolean arm_moving_ = false;

//**************************************************************************************************************************************************************//
//************************************************************************ COMUNICATIONS ***********************************************************************//
//**************************************************************************************************************************************************************//

//************ Send DATA *******************
boolean send_data(ArmCmd cmd_id, unsigned char* out_data=NULL)
{
  ShortByteUnion length;
  switch(cmd_id)
  {
    case SET_JOINTS: 
      length.asShort = 48;
      break;
    default: 
      length.asShort=0;
      break;
  }
  
  // 2B sync + 1B cmd + 2B length + data + 1B CRC
  unsigned char data[2+1+2+length.asShort+1];
  
  // Sync Bytes
  data[0] = 0xFF;
  data[1] = 0xFF;
  // Cmd Byte
  data[2] = cmd_id;
  // Length Byte
  data[3] = length.asBytes[1];
  data[4] = length.asBytes[0];
  // CRC
  uint8 checksum = (uint8)(cmd_id) + (uint8)(length.asBytes[0]) + (uint8)(length.asBytes[1]);
    
  // Data Bytes
  for (int ii = 0; ii < length.asShort; ++ii)
  {
    data[ii+5] = out_data[ii];
    checksum = checksum + (uint8)out_data[ii];
  }
  // CRC
  data[length.asShort+5] = (unsigned char)(checksum%0xFF);

  for(int ii=0;ii<length.asShort+6;++ii)
    SerialUSB.write(data[ii]);

  if(cmd_id!=ACK && cmd_id!=NAK){
    if(!check_ack())
      return false;
  }
    
  return true;
}

//************ Receive DATA *******************
void receive_data(ArmCmd& cmd_id, unsigned char* data=NULL)
{ 
  boolean success = false;

  ShortByteUnion length;
  int num=0,byte_read=0,num_sync_bytes=2,num_length_bytes=2;
  uint8 checksum=0;
  read_state state = read_sync; 
 
  ArmCmd cmdByte=NAK;
  unsigned char in_data[54];
  unsigned char read_byte[48];
  int read_=0;
  SUnsCharUnion inByte;
  
  boolean new_input = false;
  
  while (SerialUSB.available() > 0) {
    inByte.asSig = SerialUSB.read();
    in_data[read_] = inByte.asUnsig;
    read_++;
    delayMicroseconds(90);
    new_input = true;
  }
 
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
            if(in_data[ii]==ATTACH) cmdByte=ATTACH; 
            if(in_data[ii]==DETACH) cmdByte=DETACH;             
            if(in_data[ii]==SET_JOINTS) cmdByte=SET_JOINTS;
            if(in_data[ii]==GET_JOINTS) cmdByte=GET_JOINTS;
            if(in_data[ii]==IS_FINISHED) cmdByte=IS_FINISHED;
            if(in_data[ii]==ACK) cmdByte=ACK;
            if(in_data[ii]==NAK) cmdByte=NAK;
            checksum = checksum+(uint8)in_data[ii];
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
               checksum = checksum+(uint8)(length.asBytes[0])+(uint8)(length.asBytes[1]);
               if(length.asShort>0 && length.asShort==48){
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
                checksum = checksum+(uint8)in_data[ii];
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
            if (checksum%0xFF == (uint8)in_data[ii]){
              success = true;
            }
            num_sync_bytes=2;
            state=read_sync;   
            checksum = 0;           
          }
          break;  
      }
    }
    
    if(success) {
      if (cmdByte!=ACK && cmdByte!=NAK)
        send_data(ACK);
      for(int jj=0;jj<48;jj++)
        data[jj] = read_byte[jj];
      cmd_id=cmdByte;     
    }
    else
      send_data(NAK);
  }
}

//************* Check ACK *******************
boolean check_ack()
{
  ArmCmd cmd_id=NAK;
  unsigned char data[48];
  receive_data(cmd_id,data);
  
  if(cmd_id!=ACK)
    return false;
  return true;  
}

//************* Data to command *************
void data2cmd(unsigned char *data)
{
  for (unsigned int ii = 0; ii < N_SERVOS; ++ii)
  {
    // Joint number
    ShortByteUnion joint_u;
    joint_u.asBytes[0] = data[ii*8];
    joint_u.asBytes[1] = data[ii*8+1];
    int joint = (int)joint_u.asShort;

    // Position
    FloatByteUnion pos_u;
    pos_u.asBytes[0] = data[ii*8+2]; 
    pos_u.asBytes[1] = data[ii*8+3]; 
    pos_u.asBytes[2] = data[ii*8+4]; 
    pos_u.asBytes[3] = data[ii*8+5]; 
    pos_deg_end_[joint] = pos_u.asFloat;
    
    // Velocity
    ShortByteUnion vel_u;
    vel_u.asBytes[0] = data[ii*8+6];  
    vel_u.asBytes[1] = data[ii*8+7];
    servo_vel_[joint] = vel_u.asShort;
  }
}

//*************** Command to data ************
void cmd2data(unsigned char *out_data)
{
  for (unsigned int ii = 0; ii < N_SERVOS; ++ii)
  {
    // Joint number
    ShortByteUnion joint_u;
    joint_u.asShort = (short int)ii;
    out_data[ii*8] = joint_u.asBytes[0];
    out_data[ii*8+1] = joint_u.asBytes[1];

    // Position
    FloatByteUnion pos_u;
    pos_u.asFloat = pos_deg_[ii];
    out_data[ii*8+2] = pos_u.asBytes[0]; 
    out_data[ii*8+3] = pos_u.asBytes[1]; 
    out_data[ii*8+4] = pos_u.asBytes[2]; 
    out_data[ii*8+5] = pos_u.asBytes[3]; 
    
    // Velocity
    ShortByteUnion vel_u;
    vel_u.asShort = (short int)servo_vel_[ii];
    out_data[ii*8+6] = vel_u.asBytes[0];  
    out_data[ii*8+7] = vel_u.asBytes[1];  
  }
}

//**************************************************************************************************************************************************************//
//***************************************************************************** MISC ***************************************************************************//
//**************************************************************************************************************************************************************//


//********** Degrees to Servo steps ****
int deg2pos(float de)
{
  int ret = floor((de+150.0)/0.293);
  return ret; 
}

//********** Servo steps to Degrees ****
float pos2deg(int pos)
{
  float ret = (pos*0.293)-150.0;
  return ret; 
}

//**************************************************************************************************************************************************************//
//***************************************************************************** SERVOS *************************************************************************//
//**************************************************************************************************************************************************************//

// *************** Attach servos **************
void attachServos(void)
{
  Dxl.writeByte(BROADCAST_ID, TORQUE_ENABLE, 1); 
  Dxl.writeByte(BROADCAST_ID, LED_ON, 2); 
}

// *************** Detach servos ***************
void detachServos(void)
{
  Dxl.writeByte(BROADCAST_ID, TORQUE_ENABLE, 0); 
  Dxl.writeByte(BROADCAST_ID, LED_ON, 1); 
}

//****************** Get motion values *********
void get_motion_values(void)
{
  // Match time of the slowest servo
  float maxt= 0;
  
  if (DYNX_read_ok_)
  {
    for (unsigned int ii = 0; ii < N_SERVOS; ++ii){
      pos_deg_singlemov_ini_[ii] = pos_deg_[ii];
      angle_range_[ii] = pos_deg_end_[ii] - pos_deg_[ii];
      float tim = abs(angle_range_[ii]/servo_vel_[ii]);
      if (tim > maxt)
        maxt = tim; 
    }
 
    //move to calculated position 
    for (int ii = 0; ii < N_SERVOS; ++ii)
    {
      pos_deg_singlemov_ini_[ii] = pos_deg_[ii];
      servo_vel_step_[ii] = floor(abs(angle_range_[ii])/maxt); 
    }
  }
  
  // S-curve steps
  num_steps_ = round(maxt/t_step_);
  steps_done_ = 0;

  if(num_steps_>0){
    arm_moving_ = true;
  }
  else 
    arm_moving_ = false;
}

//********************* Move servos *****************
void move_servos(void)
{
  // S-curve: smooth movements
  if (arm_moving_)
  {
    float pi = 3.14159;
    for (int ii=0;ii<N_SERVOS;++ii){
      float num = (pi*steps_done_)/num_steps_;
      pos_deg_singlemov_des_[ii] =  pos_deg_singlemov_ini_[ii]+((angle_range_[ii]/pi)*(num-cos(num)*sin(num)));
    }
    steps_done_ = steps_done_ + 1;
    if(steps_done_ == num_steps_)
      arm_moving_ = false;
  }
  else
    Dxl.writeByte(BROADCAST_ID, LED_ON, 3); 
  
  // Send servo commands
  word SyncPage[3*N_SERVOS];
  for (int ii = 0; ii < N_SERVOS; ++ii)
  {
    SyncPage[3*ii] = ii+1;
    SyncPage[3*ii+1] = deg2pos(pos_deg_singlemov_des_[ii]);
    SyncPage[3*ii+2] = servo_vel_step_[ii];    
  }
  Dxl.syncWrite(GOAL_POSITION,2,SyncPage,3*N_SERVOS);
  delay(SERVO_INSTR_DELAY);
}

//*********** Write servo values ********
void motion_update(void)
{
  // Match time of the slowest servo
  float maxt= 0;
  for (unsigned int ii = 0; ii < N_SERVOS; ++ii){
    angle_range_[ii] = abs(pos_deg_end_[ii] - pos_deg_[ii]);
    float tim = angle_range_[ii]/servo_vel_[ii];
    if (tim > maxt)
      maxt = tim; 
  }
 
  //move to calculated position 
  int out_vel[N_SERVOS];
  for (int ii = 0; ii < N_SERVOS; ++ii)
  {
    pos_deg_singlemov_des_[ii] = pos_deg_end_[ii];
    out_vel[ii] = floor(angle_range_[ii]/maxt); 
  }
 
  //move to calculated position 
  word SyncPage[3*N_SERVOS];
  for (int ii = 0; ii < N_SERVOS; ++ii)
  {
    SyncPage[3*ii] = ii+1;
    SyncPage[3*ii+1] = deg2pos(pos_deg_singlemov_des_[ii]);
    SyncPage[3*ii+2] = out_vel[ii];    
  }
  Dxl.syncWrite(GOAL_POSITION,2,SyncPage,3*N_SERVOS);
  delay(SERVO_INSTR_DELAY);
}
 

//******************* Move to default values *************
void move_to_default(void)
{
  for (int ii = 0; ii < N_SERVOS; ++ii)
    pos_deg_end_[ii]=pos_deg_ini_[ii];
}

//******************* Check if finished ******************
boolean is_stopped()
{
  boolean stopped = true;

  if (arm_moving_)
    stopped = false;
  else
  {  
    for (int ii = 0; ii < N_SERVOS; ++ii)
    {
      byte isMoving = Dxl.readByte(ii+1, MOVING);
      delay(SERVO_INSTR_DELAY);
      if (isMoving == 1 || Dxl.getResult()!=(1<<COMM_RXSUCCESS))
        stopped = false;
    }
  }
  return stopped;
}

//**************** Read currnt position *************
void read_position(void)
{
  boolean value_ok = true;
  for (int ii = 0; ii < N_SERVOS; ++ii)
  {
    pos_deg_[ii] = pos2deg(Dxl.readWord(ii+1, PRESENT_POSITION)); 
    delay(SERVO_INSTR_DELAY);
    if (360 < abs(pos_deg_[ii])) // avoid wrong servo readings
      value_ok = false; 
  }
  if (Dxl.getResult()==(1<<COMM_RXSUCCESS) && value_ok)
    DYNX_read_ok_ = true;
  else
    DYNX_read_ok_ = false;
}

//**************************************************************************************************************************************************************//
//***************************************************************************** TIMER FLAGS ********************************************************************//
//**************************************************************************************************************************************************************//

//void toggle_usb_flag(void){ 
//  Dxl.writeByte(BROADCAST_ID, LED_ON, 1);
//  FLAG_usb_ = true; 
//}
void toggle_dynx_flag(void){
  digitalWrite(BOARD_LED_PIN, HIGH); 
  read_position();
  if (arm_moving_)
    move_servos(); 
  digitalWrite(BOARD_LED_PIN, LOW);   
//  FLAG_dynx_ = true; 
}


//**************************************************************************************************************************************************************//
//***************************************************************************** SETUP **************************************************************************//
//**************************************************************************************************************************************************************//

void setup() {
  
  // Set up the LED to blink
  pinMode(BOARD_LED_PIN, OUTPUT);
  
  // Initialize the dynamixel bus:
  Dxl.begin(serial_speed_); 

  //Set Low Joint limits
  for(int ii=0; ii<N_SERVOS; ii++ ){
    Dxl.writeWord(ii+1, CW_ANGLE_LIM, deg2pos(servo_min_[ii]));
    delay(EEPROM_INSTR_DELAY);
    Dxl.writeWord(ii+1, CCW_ANGLE_LIM, deg2pos(servo_max_[ii]));  
    delay(EEPROM_INSTR_DELAY);  
  }

  //Set delay time to zero   
  if(!Dxl.writeByte(BROADCAST_ID, RETURN_DELAY_TIME, 0))
  delay(EEPROM_INSTR_DELAY);

  DYNXTimer.pause();
  DYNXTimer.setPeriod(DYNX_RATE);
  DYNXTimer.setChannel1Mode(TIMER_OUTPUT_COMPARE);
  DYNXTimer.setCompare(TIMER_CH1, 1);
  DYNXTimer.attachCompare1Interrupt(toggle_dynx_flag);
  DYNXTimer.refresh(); // Refresh the timer's count, prescale, and overflow
  DYNXTimer.resume(); // Start the timer counting

//  DYNXTimer.setMode(TIMER_CH1, TIMER_OUTPUT_COMPARE); // Set up an interrupt on channel 1
//  USBTimer.setMode(TIMER_CH1, TIMER_OUTPUT_COMPARE); // Set up an interrupt on channel 1
//  DYNXTimer.pause(); // Pause the timer while we're configuring it
//  USBTimer.pause(); // Pause the timer while we're configuring it
//  DYNXTimer.setPeriod(DYNX_RATE); // Set up period in microseconds
//  USBTimer.setPeriod(USB_RATE); // Set up period in microseconds
//  DYNXTimer.setCompare(TIMER_CH1, 2);  // Interrupt 1 count after each update
//  USBTimer.setCompare(TIMER_CH1, 1);  // Interrupt 1 count after each update
//  DYNXTimer.attachCompare1Interrupt(toggle_dynx_flag);
//  USBTimer.attachCompare1Interrupt(toggle_usb_flag);
//  DYNXTimer.refresh(); // Refresh the timer's count, prescale, and overflow
//  USBTimer.refresh(); // Refresh the timer's count, prescale, and overflow
//  DYNXTimer.resume(); // Start the timer counting
//  USBTimer.resume(); // Start the timer counting

  // Attach servos
  attachServos();
  arm_state_ = STOPPED;
 
  // Move to default position
  while (!DYNX_read_ok_) 
    read_position();
  
  move_to_default();

  get_motion_values();
  while (arm_moving_)
    move_servos();
}

//**************************************************************************************************************************************************************//
//***************************************************************************** MAIN LOOP **********************************************************************//
//**************************************************************************************************************************************************************//

void loop() {
 
  
//  if (FLAG_dynx_)
//  {
//    FLAG_dynx_ = false;
//    read_position();
//    if (arm_moving_)
//      move_servos();
//  }
//  
//  else if (FLAG_usb_ && DYNX_read_ok_)
//  {
//    FLAG_usb_ = false;
  if (DYNX_read_ok_)
  {
    unsigned char data[48];
    ArmCmd cmd_id=NAK;
    receive_data(cmd_id,data);    

    switch(cmd_id){  
      case ATTACH:
        if (arm_state_ == DETACHED)
        {
          attachServos();
          arm_state_ = STOPPED;   
          move_to_default();
          get_motion_values();
        }
        break;
      case DETACH:
        if (arm_state_ != DETACHED)
        {
          detachServos();
          arm_state_ = DETACHED;
        }
        break;
      case SET_INI:
        if (arm_state_ == DETACHED)
          send_data(NAK);
        else
        {
          move_to_default();
          get_motion_values();
          arm_state_ = GOING;
        }
        break;  
      case SET_JOINTS:
        if (arm_state_ == DETACHED)
          send_data(NAK);
        else
        {
          data2cmd(data);
          get_motion_values();
          arm_state_ = GOING;
        }
        break;
      case GET_JOINTS:
        unsigned char out_data[48];
        cmd2data(out_data);
        send_data(SET_JOINTS,out_data);
        break;
      case IS_FINISHED:
        if (arm_state_ == STOPPED)
          send_data(IS_FINISHED);
        else if (arm_state_ == GOING)
          send_data(RUNNING);
        break; 
    }  

  }

  if (arm_state_ == GOING)
    if (is_stopped())
      arm_state_ = STOPPED;

}

