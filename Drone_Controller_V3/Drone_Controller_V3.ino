/*

  Drone Controller Sketch
  This code was written by Julio Nyakunga on 7th of July 2020 

  http://www.mdudegiulio.org/en/Tutorial/esc_arming
    
*/

#include <EnableInterrupt.h>
#include <Servo.h>
#include <Wire.h>   //Include I2C library
#include <EEPROM.h>

#define SERIAL_PORT_SPEED 9600
#define RC_NUM_CHANNELS  5

#define RC_CH1  0 // Roll Channel
#define RC_CH2  1 // Pitch Channel
#define RC_CH3  2 // Throttle Channel
#define RC_CH4  3 // Yaw/Rudder Channel
#define RC_CH5  4

#define MIN_PULSE_WIDTH 1000
#define MAX_PULSE_WIDTH 2000

#define RC_CH1_INPUT  1   // Yaw/Rudder Channel 
#define RC_CH2_INPUT  4   // Pitch Channel
#define RC_CH3_INPUT  11  // Throttle Channel
#define RC_CH4_INPUT  3   // Roll Channel
#define RC_CH5_INPUT  13  // Ball Channel

#define THROTTLE 1000
#define PITCH 1500
#define YAW 1500
#define ROLL 1500

int addr = 0;

int changeInThrottle = 0;
int changeInPitch = 0;
int changeInRoll = 0;
int changeInYaw = 0;

uint16_t rc_values[RC_NUM_CHANNELS];
uint32_t rc_start[RC_NUM_CHANNELS];
volatile uint16_t rc_shared[RC_NUM_CHANNELS];

Servo esc1; // Motor 1
Servo esc2; // Motor 2
Servo esc3; // Motor 3
Servo esc4; // Motor 4

int motor_1 = 5;
int motor_2 = 6;
int motor_3 = 10;
int motor_4 = 9;

int throttle1 = 0; 
int throttle2 = 0;
int throttle3 = 0;
int throttle4 = 0;

boolean isArmed = false;

/*
Gyro - Arduino pro mini
VCC  -  5V
GND  -  GND
SDA  -  A4
SCL  -  A5

LCD  - Arduino pro mini
VCC  -  5V
GND  -  GND
SDA  -  A4
SCL  -  A5
*/

long loop_timer;
int loop_counter;

int pitch = 0;
int roll = 0;
int yaw = 0;

int pin_7 = 7;
int pin_8 = 8;
int isLEDOn = false;
int lEDOnTime = 0;
int lEDOnTime2 = 0;

/*MPU-6050 gives you 16 bits data so you have to create some 16int constants
 * to store the data for accelerations and gyro*/

int16_t Acc_rawX, Acc_rawY, Acc_rawZ,Gyr_rawX, Gyr_rawY, Gyr_rawZ;
 
float Acceleration_angle[2];
float Gyro_angle[2];
float Total_angle[2];
float angle_x_error, angle_y_error, angle_z_error;

float elapsedTime, time, timePrev;
int i;
float rad_to_deg = 180/3.141592654;

float pid_roll, pid_pitch, pid_yaw, error_roll, error_pitch, error_yaw, previous_error_roll, previous_error_pitch, previous_error_yaw;
float pid_p_roll=0;
float pid_i_roll=0;
float pid_d_roll=0;

float pid_p_pitch=0;
float pid_i_pitch=0;
float pid_d_pitch=0;

float pid_p_yaw=0;
float pid_i_yaw=0;
float pid_d_yaw=0;

/////////////////PID CONSTANTS/////////////////
//double kp=3.55;//3.55
//double ki=0.005;//0.003
//double kd=2.05;//2.05
double kp = 2;
double ki=0.005;
double kd = 1;

float desired_angle = 0; //This is the angle in which we whant the drone to stay steady

boolean isDroneLevel = false;

void setup() {
  Serial.begin(SERIAL_PORT_SPEED);
  
  pinMode(pin_7, OUTPUT);
  pinMode(pin_8, OUTPUT);
  digitalWrite(pin_7, LOW);
  digitalWrite(pin_8, LOW);
  delay(50);
  digitalWrite(pin_7, HIGH);
  digitalWrite(pin_8, HIGH);
  delay(1000);
  digitalWrite(pin_7, LOW);
  digitalWrite(pin_8, LOW);
  delay(500);
  
  pinMode(RC_CH1_INPUT, INPUT);
  pinMode(RC_CH2_INPUT, INPUT);
  pinMode(RC_CH3_INPUT, INPUT);
  pinMode(RC_CH4_INPUT, INPUT);
  pinMode(RC_CH5_INPUT, INPUT);

  enableInterrupt(RC_CH1_INPUT, calc_ch1, CHANGE);
  enableInterrupt(RC_CH2_INPUT, calc_ch2, CHANGE);
  enableInterrupt(RC_CH3_INPUT, calc_ch3, CHANGE);
  enableInterrupt(RC_CH4_INPUT, calc_ch4, CHANGE);
  enableInterrupt(RC_CH5_INPUT, calc_ch5, CHANGE);

  esc1.attach(motor_1,MIN_PULSE_WIDTH,MAX_PULSE_WIDTH);
  esc2.attach(motor_2,MIN_PULSE_WIDTH,MAX_PULSE_WIDTH);
  esc3.attach(motor_3,MIN_PULSE_WIDTH,MAX_PULSE_WIDTH);
  esc4.attach(motor_4,MIN_PULSE_WIDTH,MAX_PULSE_WIDTH);

//  Serial.println("Started");
  setup_mpu_6050_registers();
//  Serial.println("Started2");

  for (int cal_int = 0; cal_int < 2000 ; cal_int ++){                  //Run this code 2000 times
    get_mpu_6050_data();                                                //Read the roll and pitch data from the MPU-6050
    angle_x_error += Total_angle[0];                                              //Add the gyro x-axis offset to the gyro_x_cal variable
    angle_y_error += Total_angle[1];                                              //Add the gyro y-axis offset to the gyro_y_cal variable
    delay(3);                                                          //Delay 3us to simulate the 250Hz program loop
  }
  angle_x_error /= 2000;                                                  //Divide the gyro_x_cal variable by 2000 to get the avarage offset
  angle_y_error /= 2000;                                                  //Divide the gyro_y_cal variable by 2000 to get the avarage offset
  
  time = millis(); //Start counting time in milliseconds
  
  loop_timer = micros();                                               //Reset the loop timer
//  Serial.println("Started 5");
}

void rc_read_values() {
  noInterrupts();
  memcpy(rc_values, (const void *)rc_shared, sizeof(rc_shared));
  interrupts();
}

void calc_input(uint8_t channel, uint8_t input_pin) {
  if (digitalRead(input_pin) == HIGH) {
    rc_start[channel] = micros();
  } else {
    uint16_t rc_compare = (uint16_t)(micros() - rc_start[channel]);
    rc_shared[channel] = rc_compare;
  }
}

void calc_ch1() { calc_input(RC_CH1, RC_CH1_INPUT); }
void calc_ch2() { calc_input(RC_CH2, RC_CH2_INPUT); }
void calc_ch3() { calc_input(RC_CH3, RC_CH3_INPUT); }
void calc_ch4() { calc_input(RC_CH4, RC_CH4_INPUT); }
void calc_ch5() { calc_input(RC_CH5, RC_CH5_INPUT); }

void calculate_elapsed_time(){
  /////////////////////////////I M U/////////////////////////////////////
  timePrev = time;  // the previous time is stored before the actual time read
  time = millis();  // actual time read
  elapsedTime = (time - timePrev) / 1000; 
  /*
   * The tiemStep is the time that elapsed since the previous loop. 
    * This is the value that we will use in the formulas as "elapsedTime" 
    * in seconds. We work in ms so we haveto divide the value by 1000 
    to obtain seconds
  */
}

void setup_mpu_6050_registers(){
  Wire.begin(); //begin the wire comunication
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}

void store_mpu_6050_data_to_EEPROM(float val){
  EEPROM.put(addr, val);
  addr++;
}

void get_mpu_6050_data(){                      
  /*Reed the values that the accelerometre gives.
    * We know that the slave adress for this IMU is 0x68 in
    * hexadecimal. For that in the RequestFrom and the 
    * begin functions we have to put this value.
  */
  Wire.beginTransmission(0x68);
  Wire.write(0x3B); //Ask for the 0x3B register- correspond to AcX
  Wire.endTransmission(false);
  Wire.requestFrom(0x68,6,true); 
   
  /*We have asked for the 0x3B register. The IMU will send a brust of register.
    * The amount of register to read is specify in the requestFrom function.
    * In this case we request 6 registers. Each value of acceleration is made out of
    * two 8bits registers, low values and high values. For that we request the 6 of them  
    * and just make then sum of each pair. For that we shift to the left the high values 
    * register (<<) and make an or (|) operation to add the low values.
  */
    
  Acc_rawX=Wire.read()<<8|Wire.read(); //each value needs two registres
  Acc_rawY=Wire.read()<<8|Wire.read();
  Acc_rawZ=Wire.read()<<8|Wire.read();
 
  /*///This is the part where you need to calculate the angles using Euler equations///*/
    
  /* - Now, to obtain the values of acceleration in "g" units we first have to divide the raw   
    * values that we have just read by 16384.0 because that is the value that the MPU6050 
    * datasheet gives us.
  */
  /* - Next we have to calculate the radian to degree value by dividing 180º by the PI number
    * which is 3.141592654 and store this value in the rad_to_deg variable. In order to not have
    * to calculate this value in each loop we have done that just once before the setup void.
  */

  /* Now we can apply the Euler formula. The atan will calculate the arctangent. The
    *  pow(a,b) will elevate the a value to the b power. And finnaly sqrt function
    *  will calculate the rooth square.
  */
  /*---X---*/
  Acceleration_angle[0] = atan((Acc_rawY/16384.0)/sqrt(pow((Acc_rawX/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
  /*---Y---*/
  Acceleration_angle[1] = atan(-1*(Acc_rawX/16384.0)/sqrt(pow((Acc_rawY/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
 
  /*Now we read the Gyro data in the same way as the Acc data. The adress for the
    * gyro data starts at 0x43. We can see this adresses if we look at the register map
    * of the MPU6050. In this case we request just 4 values. W don¡t want the gyro for 
    * the Z axis (YAW).
  */
    
  Wire.beginTransmission(0x68);
  Wire.write(0x43); //Gyro data first adress
  Wire.endTransmission(false);
  Wire.requestFrom(0x68,4,true); //Just 4 registers
   
  Gyr_rawX=Wire.read()<<8|Wire.read(); //Once again we shif and sum
  Gyr_rawY=Wire.read()<<8|Wire.read();
 
  /*Now in order to obtain the gyro data in degrees/seconda we have to divide first
    the raw value by 131 because that's the value that the datasheet gives us
  */

  /*---X---*/
  Gyro_angle[0] = Gyr_rawX/131.0; 
  /*---Y---*/
  Gyro_angle[1] = Gyr_rawY/131.0;

  /*Now in order to obtain degrees we have to multiply the degree/seconds
    *value by the elapsedTime.
  */
  /*Finnaly we can apply the final filter where we add the acceleration
    *part that afects the angles and ofcourse multiply by 0.98 
  */

  /*---X axis angle---*/
  Total_angle[0] = 0.98 *(Total_angle[0] + Gyro_angle[0]*elapsedTime) + 0.02*Acceleration_angle[0]; 
  Total_angle[0] = Total_angle[0] - angle_x_error;          //Subtracting the error
  
  /*---Y axis angle---*/
  Total_angle[1] = 0.98 *(Total_angle[1] + Gyro_angle[1]*elapsedTime) + 0.02*Acceleration_angle[1];
  Total_angle[1] = Total_angle[1] - angle_y_error;          //Subtracting the error

   /*Now we have our angles in degree and values from -100º to 100º aprox*/   
}

void pid_controller(){
  /*///////////////////////////P I D///////////////////////////////////*/
  /*Remember that for the balance we will use just one axis. I've choose the x angle
    *to implement the PID with. That means that the x axis of the IMU has to be paralel to
    *the balance
  */

  //PID Calculation for Roll
  /*First calculate the error between the desired angle and 
    *the real measured angle
  */
  error_roll = Total_angle[1] - desired_angle;
    
  /*Next the proportional value of the PID is just a proportional constant
    *multiplied by the error
  */

  pid_p_roll = kp*error_roll;

  /*The integral part should only act if we are close to the
    desired position but we want to fine tune the error. That's
    why I've made a if operation for an error between -2 and 2 degree.
    To integrate we just sum the previous integral value with the
    error multiplied by  the integral constant. This will integrate (increase)
    the value each loop till we reach the 0 point
  */
  if(-3 <error_roll <3)
  {
    pid_i_roll = pid_i_roll+(ki*error_roll);  
  }

  /*The last part is the derivate. The derivate acts upon the speed of the error.
    As we know the speed is the amount of error that produced in a certain amount of
    time divided by that time. For taht we will use a variable called previous_error.
    We substract that value from the actual error and divide all by the elapsed time. 
    Finnaly we multiply the result by the derivate constant
  */

  pid_d_roll = kd*((error_roll - previous_error_roll)/elapsedTime);
  previous_error_roll = error_roll;

  /*The final PID values is the sum of each of this 3 parts*/
  pid_roll = pid_p_roll + pid_i_roll + pid_d_roll;
  
  //PID Calculation for Pitch
  /*First calculate the error between the desired angle and 
    *the real measured angle
  */
  error_pitch = Total_angle[0] - desired_angle;
    
  /*Next the proportional value of the PID is just a proportional constant
    *multiplied by the error
  */

  pid_p_pitch = kp*error_pitch;

  /*The integral part should only act if we are close to the
    desired position but we want to fine tune the error. That's
    why I've made a if operation for an error between -2 and 2 degree.
    To integrate we just sum the previous integral value with the
    error multiplied by  the integral constant. This will integrate (increase)
    the value each loop till we reach the 0 point
  */
  if(-3 <error_pitch <3)
  {
    pid_i_pitch = pid_i_pitch+(ki*error_pitch);
  }

  /*The last part is the derivate. The derivate acts upon the speed of the error.
    As we know the speed is the amount of error that produced in a certain amount of
    time divided by that time. For taht we will use a variable called previous_error.
    We substract that value from the actual error and divide all by the elapsed time. 
    Finnaly we multiply the result by the derivate constant
  */

  pid_d_pitch = kd*((error_pitch - previous_error_pitch)/elapsedTime);
  previous_error_pitch = error_pitch;

  /*The final PID values is the sum of each of this 3 parts*/
  pid_pitch = pid_p_pitch + pid_i_pitch + pid_d_pitch;

//  Serial.print("Total_angle0 = ");
//  Serial.println(Total_angle[0]);
//  Serial.print("\tTotal_angle1 = ");
//  Serial.println(Total_angle[1]);
//  Serial.print("\terror_roll : ");
//  Serial.print(error_roll);
//  Serial.print("\terror_pitch : ");
//  Serial.print(error_pitch);
//  Serial.print("\tpid_roll : ");
//  Serial.print(pid_roll);
//  Serial.print("\t  ");
//  Serial.println(throttle1);
//  Serial.print("\tpid_pitch : ");
//  Serial.println(pid_pitch);
}

void loop() {
  if(((angle_x_error <-3 ) && (angle_x_error > 3)) || ((angle_y_error <-3 ) && (angle_y_error > 3))){
    angle_x_error = 0;
    angle_y_error = 0;
    
    for (int cal_int = 0; cal_int < 20 ; cal_int ++){                  //Run this code 2000 times
      get_mpu_6050_data();                                                //Read the roll and pitch data from the MPU-6050
      angle_x_error += Total_angle[0];                                              //Add the gyro x-axis offset to the gyro_x_cal variable
      angle_y_error += Total_angle[1];                                              //Add the gyro y-axis offset to the gyro_y_cal variable
      delay(3);                                                          //Delay 3us to simulate the 250Hz program loop
    }
    angle_x_error /= 20;                                                  //Divide the gyro_x_cal variable by 2000 to get the avarage offset
    angle_y_error /= 20;
    if(((angle_x_error >-3 ) && (angle_x_error < 3)) && ((angle_y_error >-3 ) && (angle_y_error < 3))){
      isDroneLevel = true;
    }
  }
  
  calculate_elapsed_time();
  
  get_mpu_6050_data();
  
  // get angular corrections from PID Controller
  pid_controller();

  /*We know taht the min value of PWM signal is 1000us and the max is 2000. So that
    *tells us that the PID value can/s oscilate more than -1000 and 1000 because when we
    *have a value of 2000us the maximum value taht we could sybstract is 1000 and when
    *we have a value of 1000us for the PWM sihnal, the maximum value that we could add is 1000
    *to reach the maximum 2000us
  */
  if(pid_roll < -1000){
    pid_roll = -1000;
  }else if(pid_roll > 1000){
    pid_roll = 1000;
  }
  
  if(pid_pitch < -1000){
    pid_pitch = -1000;
  }else if(pid_pitch > 1000){
    pid_pitch = 1000;
  }
  
  rc_read_values();

  if(rc_values[RC_CH5] >= 1500){
//    Serial.println("Dropping Ball");
  }
  
  if((rc_values[RC_CH3] <= 1000) && (rc_values[RC_CH4] <= 1000) && !isArmed && isDroneLevel){
    isArmed = true;
//    Serial.println("ARMED");
    digitalWrite(pin_7, HIGH);
    digitalWrite(pin_8, HIGH);
  }
  if((rc_values[RC_CH3] <= 1000) && (rc_values[RC_CH4] >= 2000) && isArmed){
    isArmed = false;
//    Serial.println("DISARMED");
    digitalWrite(pin_7, LOW);
    digitalWrite(pin_8, LOW);
    while(throttle1 > 1000){
      throttle1--;
      throttle2--;
      throttle3--;
      throttle4--;
      delay(100);
    }
    if(throttle1 <= 1000){
      throttle1 = 0;
      throttle2 = 0;
      throttle3 = 0;
      throttle4 = 0;
    }
    esc1.write(throttle1);
    esc2.write(throttle2);
    esc3.write(throttle3);
    esc4.write(throttle4);
  }
  
  if(isArmed){
    
//    Serial.print("CH1:"); Serial.print(rc_values[RC_CH1]); Serial.print("\t"); Serial.print("\t");
//    Serial.print("CH2:"); Serial.print(rc_values[RC_CH2]); Serial.print("\t"); Serial.print("\t");
//    Serial.print("CH3:"); Serial.print(rc_values[RC_CH3]); Serial.print("\t"); Serial.print("\t");
//    Serial.print("CH4:"); Serial.println(rc_values[RC_CH4]);
  
    throttle1 = rc_values[RC_CH3];
    throttle2 = rc_values[RC_CH3];
    throttle3 = rc_values[RC_CH3];
    throttle4 = rc_values[RC_CH3];

    changeInRoll = ROLL - rc_values[RC_CH1];
    changeInPitch = PITCH - rc_values[RC_CH2];
    changeInYaw = YAW - rc_values[RC_CH4];
    
    if((changeInPitch > 0) && (rc_values[RC_CH3] > 1020)){
      throttle3 = throttle3 + changeInPitch;
      throttle4 = throttle4 + changeInPitch;
    }else if((changeInPitch < 0) && (rc_values[RC_CH3] > 1020)){
      throttle1 = throttle1 - changeInPitch;
      throttle2 = throttle2 - changeInPitch;
    }

    if((changeInRoll > 0) && (rc_values[RC_CH3] > 1020)){
      throttle2 = throttle2 + changeInRoll;
      throttle3 = throttle3 + changeInRoll;
    }else if((changeInRoll < 0) && (rc_values[RC_CH3] > 1020)){
      throttle1 = throttle1 - changeInRoll;
      throttle4 = throttle4 - changeInRoll;
    }

    if((changeInYaw > 0) && (rc_values[RC_CH3] > 1020)){
      throttle1 = throttle1 + changeInYaw;
      throttle3 = throttle3 + changeInYaw;
      throttle2 = throttle2 - changeInYaw;
      throttle4 = throttle4 - changeInYaw;
    }else if((changeInYaw < 0) && (rc_values[RC_CH3] > 1020)){
      throttle1 = throttle1 + changeInYaw;
      throttle3 = throttle3 + changeInYaw;
      throttle2 = throttle2 - changeInYaw;
      throttle4 = throttle4 - changeInYaw;
    }

    //Adding PID corrections
    throttle1 = throttle1 - (int)pid_roll;
    throttle4 = throttle4 - (int)pid_roll;
    throttle2 = throttle2 + (int)pid_roll;
    throttle3 = throttle3 + (int)pid_roll;
    

    throttle1 = throttle1 + (int)pid_pitch;
    throttle2 = throttle2 + (int)pid_pitch;
    throttle3 = throttle3 - (int)pid_pitch;
    throttle4 = throttle4 - (int)pid_pitch;

    if(throttle1 < 1000) throttle1= 1000;
    if(throttle1 > 1700) throttle1=1700;
    if(throttle2 < 1000) throttle2= 1000;
    if(throttle2 > 1700) throttle2=1700;
    if(throttle3 < 1000) throttle3= 1000;
    if(throttle3 > 1700) throttle3=1700;
    if(throttle4 < 1000) throttle4= 1000;
    if(throttle4 > 1700) throttle4=1700;
    

//    Serial.print("CH3:"); Serial.print(rc_values[RC_CH3]); Serial.print("\t"); Serial.print("\t");
//    Serial.print("CH5:"); Serial.print(rc_values[RC_CH5]); Serial.print("\t"); Serial.print("\t");
//    Serial.print("changeInThrottle:"); Serial.println(changeInThrottle);
    
    esc1.write(throttle1);
    esc2.write(throttle2);
    esc3.write(throttle3);
    esc4.write(throttle4);
  }else{
    lEDOnTime = millis() - lEDOnTime2;
    if(isLEDOn && (lEDOnTime >= 500)){
      if(Total_angle[1] < -3){
        digitalWrite(pin_7, HIGH);
        Serial.println("OOONNNNNNN");
        delay(300);
      }
//      if(angle_y_error >-3){
//      }
      digitalWrite(pin_8, LOW);
      isLEDOn = false;
      lEDOnTime2 = millis();
      lEDOnTime = 0;
    }else if(!isLEDOn && (lEDOnTime >= 500)){
      if(Total_angle[1] < -3){
        digitalWrite(pin_7, LOW);
        Serial.println("OOOFFFFFFF");
        delay(300);
      }
      digitalWrite(pin_8, HIGH);
      isLEDOn = true;
      lEDOnTime2 = millis();
      lEDOnTime = 0;
    }
  }
  Serial.print("Total_angle[1] = ");
  Serial.println(Total_angle[1]);
  while(micros() - loop_timer < 4000);                                 //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop
  loop_timer = micros();                                               //Reset the loop timer
}
