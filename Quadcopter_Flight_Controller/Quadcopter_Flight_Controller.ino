/*

  Quadcopter Flight Controller Sketch
  This code was written by Julio Nyakunga on 7th of July 2020 
  http://www.mdudegiulio.org/en/Tutorial/Quadcopter_Flight_Controller
  
*/

#include <EnableInterrupt.h>
#include <Servo.h>
#include <Wire.h>   //Include I2C library
//#include <SoftwareSerial.h>

//SoftwareSerial mySoftwareSerial(4,13);


#define SERIAL_PORT_SPEED 9600
#define RC_NUM_CHANNELS  5

#define RC_CH1  0 // Roll Channel
#define RC_CH2  1 // Pitch Channel
#define RC_CH3  2 // Throttle Channel
#define RC_CH4  3 // Yaw/Rudder Channel
#define RC_CH5  4

#define MIN_PULSE_WIDTH 1000
#define MAX_PULSE_WIDTH 2000

#define RC_CH1_INPUT  13    // Roll Channel 
#define RC_CH2_INPUT  4     // Pitch Channel
#define RC_CH3_INPUT  11    // Throttle Channel
#define RC_CH4_INPUT  3     // Yaw/Rudder Channel
#define RC_CH5_INPUT  0     // Ball Channel

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

uint16_t throttle = 0;
uint16_t throttle0 = 1000;
int throttle1 = 1000; 
int throttle2 = 1000;
int throttle3 = 1000;
int throttle4 = 1000;

boolean isArmed = false;

/*
Gyro - Arduino pro mini
VCC  -  5V
GND  -  GND
SDA  -  A4
SCL  -  A5
*/

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
float Total_angle_tmp[2];
float pitch, pitch_tmp, roll, roll_tmp, yaw, yaw_tmp;

float elapsedTime, total_elapsedTime, time, timePrev;
int loop_counter = 0;
float rad_to_deg = 180/3.141592654;
int avg_count = 0;
float pid_roll, pid_pitch, pid_yaw, error_roll, rate_error_roll, error_pitch, error_pitch_avg, error_yaw, previous_error_roll, previous_error_pitch, previous_error_yaw;
float pid_p_roll=0;
float pid_i_roll=0;
float pid_d_roll=0;

float pid_p_pitch=0;
float pid_i_pitch=0;
float pid_d_pitch=0;

float pid_p_yaw=0;
float pid_i_yaw=0;
float pid_d_yaw=0;

float pid_max = 400;

/////////////////PID CONSTANTS/////////////////
//double kp_roll = 3.44;    //Proportionality Constant for Roll
//double ki_roll = 0.048;   //Integral Constant for Roll
//double kd_roll = 1.92;    //Derivative Constant for Roll

double kp_roll = 15.0;    // 15.0
double ki_roll = 0.05;    // 0.05  
double kd_roll = 0.0;     // 10.0

double kp_pitch = 15.0;   // 15.0
double ki_pitch = 0.05;   // 0.5
double kd_pitch = 0.0;

double kp_yaw = 0.0;
double ki_yaw = 0.0;
double kd_yaw = 0.0;
 
float desired_roll_angle = 0.0;        //This is the angle in which we whant the drone to stay steady
float desired_pitch_angle = 0.0;        //This is the angle in which we whant the drone to stay steady
float desired_yaw_angle = 0.0;        //This is the angle in which we whant the drone to stay steady
boolean isDroneLevel = false;   //This boolean variable is used to check if the drone is level at startup

void setup() {
//  Serial.begin(SERIAL_PORT_SPEED);
//  mySoftwareSerial.begin(9600);
//  mySoftwareSerial.println("Starting");

  DDRB = B11010111;   //Setting PORTB, digital pin 8,9 and 10 to OUTPUT and digital pin 11 and 13 to INPUT (0=INPUT, 1=OUTPUT)
  DDRD = B11100101;   //Setting PORTD, digital pin 7,6 and 5 to OUTPUT and digital pin 1,3 and 4 to INPUT (0=INPUT, 1=OUTPUT)
  
  PORTD |= B10000000;   // Setting digital pin 7 HIGH
  PORTB |= B00000001;   // Setting digital pin 8 HIGH
  delay(1000);
  PORTD &= B01111111;   // Setting digital pin 7 LOW
  PORTB &= B11111110;   // Setting digital pin 8 LOW

  enableInterrupt(RC_CH1_INPUT, calc_ch1, CHANGE);
  enableInterrupt(RC_CH2_INPUT, calc_ch2, CHANGE);
  enableInterrupt(RC_CH3_INPUT, calc_ch3, CHANGE);
  enableInterrupt(RC_CH4_INPUT, calc_ch4, CHANGE);
  enableInterrupt(RC_CH5_INPUT, calc_ch5, CHANGE);

  Wire.begin(); //begin the wire comunication
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  esc1.attach(motor_1);       //attatch the right motor to pin 5
  esc2.attach(motor_2);
  esc3.attach(motor_3);
  esc4.attach(motor_4);
  delay(50);
  esc1.writeMicroseconds(throttle1);
  esc2.writeMicroseconds(throttle2);
  esc3.writeMicroseconds(throttle3);
  esc4.writeMicroseconds(throttle4);

  time = millis(); //Start counting time in milliseconds
  /*In order to start up the ESCs we have to send a min value
   * of PWM to them before connecting the battery. Otherwise
   * the ESCs won't start up or enter in the configure mode.
   * The min value is 1000us and max is 2000us, REMEMBER!*/
  esc1.writeMicroseconds(1000); 
  esc2.writeMicroseconds(1000);
  esc3.writeMicroseconds(1000); 
  esc4.writeMicroseconds(1000);
  delay(1000); /*Give some delay, 7s, to have time to connect
                *the propellers and let everything start up*/ 

//  mySoftwareSerial.println("Started");
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
   * The elapsedTime is the time that elapsed since the previous loop. 
    * This is the value that we will use in the formulas as "elapsedTime" 
    * in seconds. We work in ms so we haveto divide the value by 1000 
    to obtain seconds
  */
}

void get_mpu_6050_data(){
  /////////////////////////////I M U/////////////////////////////////////                   
  /*Reed the values that the accelerometre gives.
   * We know that the slave adress for this IMU is 0x68 in
   * hexadecimal. For that in the RequestFrom and the 
   * begin functions we have to put this value.
  */

//    setting mpu_6050 registers
     Wire.beginTransmission(0x68);
     Wire.write(0x3B); //Ask for the 0x3B register- correspond to AcX
     Wire.endTransmission(false);
     Wire.requestFrom(0x68,6,true);
   
   /*We have asked for the 0x3B register. The IMU will send a brust of register.
    * The amount of register to read is specify in the requestFrom function.
    * In this case we request 6 registers. Each value of acceleration is made out of
    * two 8bits registers, low values and high values. For that we request the 6 of them  
    * and just make thenif(loop_counter < 10){
    Total_angle_tmp
      loop_counter++;
   }else{
      loop_counter = 0;
   } sum of each pair. For that we shift to the left the high values 
    * register (<<) and make an or (|) operation to add the low values.*/
    
     Acc_rawX=Wire.read()<<8|Wire.read(); //each value needs two registres
     Acc_rawY=Wire.read()<<8|Wire.read();
     Acc_rawZ=Wire.read()<<8|Wire.read();

 
    /*///This is the part where you need to calculate the angles using Euler equations///*/
    
    /* - Now, to obtain the values of acceleration in "g" units we first have to divide the raw   
     * values that we have just read by 16384.0 because that is the value that the MPU6050 
     * datasheet gives us.*/
    /* - Next we have to calculate the radian to degree value by dividing 180º by the PI number
    * which is 3.141592654 and store this value in the rad_to_deg variable. In order to not have
    * to calculate this value in each loop we have done that just once before the setup void.
    */

    /* Now we can apply the Euler formula. The atan will calculate the arctangent. The
     *  pow(a,b) will elevate the a value to the b power. And finnaly sqrt function
     *  will calculate the rooth square.*/
     /*---X---*/
     Acceleration_angle[0] = atan((Acc_rawY/16384.0)/sqrt(pow((Acc_rawX/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
     /*---Y---*/
     Acceleration_angle[1] = atan(-1*(Acc_rawX/16384.0)/sqrt(pow((Acc_rawY/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
 
   /*Now we read the Gyro data in the same way as the Acc data. The adress for the
    * gyro data starts at 0x43. We can see this adresses if we look at the register map
    * of the MPU6050. In this case we request just 4 values. W don¡t want the gyro for 
    * the Z axis (YAW).*/
    
   Wire.beginTransmission(0x68);
   Wire.write(0x43); //Gyro data first adress
   Wire.endTransmission(false);
   Wire.requestFrom(0x68,4,true); //Just 4 registers
   
   Gyr_rawX=Wire.read()<<8|Wire.read(); //Once again we shif and sum
   Gyr_rawY=Wire.read()<<8|Wire.read();
//   Gyr_rawZ=Wire.read()<<8|Wire.read();
 
   /*Now in order to obtain the gyro data in degrees/seconda we have to divide first
   the raw value by 131 because that's the value that the datasheet gives us*/

   /*---X---*/
   Gyro_angle[0] = Gyr_rawX/131.0; 
   /*---Y---*/
   Gyro_angle[1] = Gyr_rawY/131.0;

   /*Now in order to obtain degrees we have to multiply the degree/seconds
   *value by the elapsedTime.*/
   /*Finnaly we can apply the final filter where we add the acceleration
   *part that afects the angles and ofcourse multiply by 0.98 */

   /*---X axis angle---*/
   Total_angle[0] = 0.98 *(Total_angle[0] + Gyro_angle[0]*elapsedTime) + 0.02*Acceleration_angle[0];
   /*---Y axis angle---*/
   Total_angle[1] = 0.98 *(Total_angle[1] + Gyro_angle[1]*elapsedTime) + 0.02*Acceleration_angle[1];
   
   /*Now we have our angles in degree and values from -10º0 to 100º aprox*/ 
}

void pid_controller(){

  /*///////////////////////////P I D///////////////////////////////////*/
  /*Remember that for the balance we will use just one axis. I've choose the x angle
    *to implement the PID with. That means that the x axis of the IMU has to be paralel to
    *the balance
  */

//   PID Calculation for Roll
  /*First calculate the error between the desired angle and 
    *the real measured angle
  */
  error_roll = roll - desired_roll_angle;
    
  /*Next the proportional value of the PID is just a proportional constant
    *multiplied by the error
  */
  
  pid_p_roll = kp_roll*error_roll;

  /*The integral part should only act if we are close to the
    desired position but we want to fine tune the error. That's
    why I've made a if operation for an error between -2 and 2 degree.
    To integrate we just sum the previous integral value with the
    error multiplied by  the integral constant. This will integrate (increase)
    the value each loop till we reach the 0 point
  */
  if((-3 < error_roll) && (error_roll < 3))
  {
    pid_i_roll = pid_i_roll+(ki_roll*error_roll);
  }
  if(pid_i_roll > 20.0){
    pid_i_roll = 20.0;
  }
  if(pid_i_roll < -20.0){
    pid_i_roll = -20.0;
  }

  /*The last part is the derivate. The derivate acts upon the speed of the error.
    As we know the speed is the amount of error that produced in a certain amount of
    time divided by that time. For taht we will use a variable called previous_error.
    We substract that value from the actual error and divide all by the elapsed time. 
    Finnaly we multiply the result by the derivate constant
  */
//  pid_d_roll = kd_roll*((error_roll - previous_error_roll)/elapsedTime);
//  previous_error_roll = error_roll;

//Using a complementary filter to remove noise
  error_roll = 0.999*previous_error_roll + 0.001*error_roll;
  pid_d_roll = kd_roll*((error_roll - previous_error_roll)/elapsedTime);
  previous_error_roll = error_roll;


  
  /*The final PID values is the sum of each of this 3 parts*/
  pid_roll = pid_p_roll + pid_i_roll + pid_d_roll;
  
  //PID Calculation for Pitch
  /*First calculate the error between the desired angle and 
    *the real measured angle
  */
  error_pitch = pitch - desired_pitch_angle;
    
  /*Next the proportional value of the PID is just a proportional constant
    *multiplied by the error
  */

  pid_p_pitch = kp_pitch*error_pitch;

  /*The integral part should only act if we are close to the
    desired position but we want to fine tune the error. That's
    why I've made a if operation for an error between -2 and 2 degree.
    To integrate we just sum the previous integral value with the
    error multiplied by  the integral constant. This will integrate (increase)
    the value each loop till we reach the 0 point
  */
  if((-3 < error_pitch) && (error_pitch < 3))
  {
    pid_i_pitch = pid_i_pitch+(ki_pitch*error_pitch);
  }
  if(pid_i_pitch > 20.0){
    pid_i_pitch = 20.0;
  }
  if(pid_i_pitch < -20.0){
    pid_i_pitch = -20.0;
  }

  /*The last part is the derivate. The derivate acts upon the speed of the error.
    As we know the speed is the amount of error that produced in a certain amount of
    time divided by that time. For taht we will use a variable called previous_error.
    We substract that value from the actual error and divide all by the elapsed time. 
    Finnaly we multiply the result by the derivate constant
  */

  pid_d_pitch = kd_pitch*((error_pitch - previous_error_pitch)/elapsedTime);
  previous_error_pitch = error_pitch;

  /*The final PID values is the sum of each of this 3 parts*/
  pid_pitch = pid_p_pitch + pid_i_pitch + pid_d_pitch;
}

void clear_pid_corrections(){
    pid_p_roll = 0;
    pid_i_roll = 0;
    pid_d_roll = 0;

    pid_p_pitch = 0;
    pid_i_pitch = 0;
    pid_d_pitch = 0;

    pid_p_yaw = 0;
    pid_i_yaw = 0;
    pid_d_yaw = 0;
}

void loop() {
  
  calculate_elapsed_time();
  get_mpu_6050_data(); 
  pitch = Total_angle[0];
  roll = Total_angle[1] - 1.0;

  pid_controller();
  
  if((-3 <= roll) && (roll <= 3) && (-3 <= pitch) && (pitch <= 3) && !isDroneLevel){
    isDroneLevel = true; 
  }else{
    isDroneLevel = false; 
  }

  /*We know taht the min value of PWM signal is 1000us and the max is 2000. So that
    *tells us that the PID value can/s oscilate more than -1000 and 1000 because when we
    *have a value of 2000us the maximum value taht we could sybstract is 1000 and when
    *we have a value of 1000us for the PWM sihnal, the maximum value that we could add is 1000
    *to reach the maximum 2000us
  */
  if(pid_roll < (-1 * pid_max)){
    pid_roll = (-1 * pid_max);
  }else if(pid_roll > pid_max){
    pid_roll = pid_max;
  }
  
  if(pid_pitch < (-1 * pid_max)){
    pid_pitch = (-1 * pid_max);
  }else if(pid_pitch > pid_max){
    pid_pitch = pid_max;
  }

  rc_read_values();
  
  if((rc_values[RC_CH3] >= 800) && (rc_values[RC_CH3] <= 1000) && (rc_values[RC_CH4] >= 800) && (rc_values[RC_CH4] <= 1000) && !isArmed && isDroneLevel){
    isArmed = true;
    
    clear_pid_corrections();  // this is done to clear saved previous PID corrections particularly the Integral (I) Term

//    Serial.println("ARMED");
    PORTD |= B10000000;
    PORTB |= B00000001;
  }
  if((rc_values[RC_CH3] <= 1000) && (rc_values[RC_CH4] >= 1800) && isArmed){
    isArmed = false;
    
    clear_pid_corrections();  // this is done to clear saved previous PID corrections particularly the Integral (I) Term

//    Serial.println("DISARMED");
    while(throttle1 > 1000){
      throttle1 -= 20;
      throttle2 -= 20;
      throttle3 -= 20;
      throttle4 -= 20;
      esc1.writeMicroseconds(throttle1);
      esc2.writeMicroseconds(throttle2);
      esc3.writeMicroseconds(throttle3);
      esc4.writeMicroseconds(throttle4);
      delay(100);
    }
    if(throttle1 <= 1000){
      throttle1 = 1000;
      throttle2 = 1000;
      throttle3 = 1000;
      throttle4 = 1000;
    }
    esc1.writeMicroseconds(throttle1);
    esc2.writeMicroseconds(throttle2);
    esc3.writeMicroseconds(throttle3);
    esc4.writeMicroseconds(throttle4);
  }
  
  if(isArmed){
    if(!isDroneLevel){
      lEDOnTime = millis() - lEDOnTime2;
      if(isLEDOn && (lEDOnTime >= 100)){
        PORTD |= B10000000;   // Setting digital pin 7 HIGH
        isLEDOn = false;
        lEDOnTime2 = millis();
        lEDOnTime = 0;
      }else if(!isLEDOn && (lEDOnTime >= 100)){
        PORTD &= B01111111;   // Setting digital pin 7 LOW 
        isLEDOn = true;
        lEDOnTime2 = millis();
        lEDOnTime = 0;
      }
    }else{
      PORTD |= B10000000;   // Setting digital pin 7 HIGH
    }
    
    throttle = rc_values[RC_CH3];
    throttle1 = rc_values[RC_CH3];
    throttle2 = rc_values[RC_CH3];
    throttle3 = rc_values[RC_CH3];
    throttle4 = rc_values[RC_CH3];
    
    changeInRoll = ROLL - rc_values[RC_CH1];
    changeInPitch = PITCH - rc_values[RC_CH2];
    // To prevent motor from running when the drone is armed the yaw channel must be checked
    if((rc_values[RC_CH4] > 1250) && (rc_values[RC_CH4] < 1750)){
      changeInYaw = YAW - rc_values[RC_CH4];
    }else{
      changeInYaw = 0;
    }
    
    if(changeInPitch > 100){
      throttle3 = throttle3 + changeInPitch - 100;
      throttle4 = throttle4 + changeInPitch - 100;
    }else if(changeInPitch < -100){
      throttle1 = throttle1 - changeInPitch + 100;
      throttle2 = throttle2 - changeInPitch + 100;
    }

    if(changeInRoll > 100){
      throttle2 = throttle2 + changeInRoll - 100;
      throttle3 = throttle3 + changeInRoll - 100;
    }else if(changeInRoll < -100){
      throttle1 = throttle1 - changeInRoll + 100;
      throttle4 = throttle4 - changeInRoll + 100;
    }

    if(changeInYaw > 100){
      throttle1 = throttle1 + changeInYaw - 100;
      throttle3 = throttle3 + changeInYaw - 100;
      throttle2 = throttle2 - changeInYaw - 100;
      throttle4 = throttle4 - changeInYaw - 100;
    }else if(changeInYaw < -100){
      throttle1 = throttle1 + changeInYaw + 100;
      throttle3 = throttle3 + changeInYaw + 100;
      throttle2 = throttle2 - changeInYaw + 100;
      throttle4 = throttle4 - changeInYaw + 100;
    }

    //Adding PID corrections
    throttle1 = throttle1 - (int)pid_roll;
    throttle4 = throttle4 - (int)pid_roll;
    throttle2 = throttle2 + (int)pid_roll;
    throttle3 = throttle3 + (int)pid_roll;
    
    throttle1 = throttle1 - (int)pid_pitch;
    throttle2 = throttle2 - (int)pid_pitch;
    throttle3 = throttle3 + (int)pid_pitch;
    throttle4 = throttle4 + (int)pid_pitch;

    /*Once again we map the PWM values to be sure that we won't pass the min
    and max values. Yes, we've already maped the PID values. But for example, for 
    throttle value of 1300, if we sum the max PID value we would have 2300us and
    that will mess up the ESC.
  */
    if(throttle1 > 2000) throttle1=2000;
    if(throttle2 > 2000) throttle2=2000;
    if(throttle3 > 2000) throttle3=2000;
    if(throttle4 > 2000) throttle4=2000;

    /*
      * Once again we limit the PWM values to be sure that no motor should stop running
    */
    
    if(throttle1 < 1150) throttle1= 1150;
    if(throttle2 < 1150) throttle2= 1150;
    if(throttle3 < 1150) throttle3= 1150;
    if(throttle4 < 1150) throttle4= 1150;
    
    esc1.writeMicroseconds(throttle1);
    esc2.writeMicroseconds(throttle2);
    esc3.writeMicroseconds(throttle3);
    esc4.writeMicroseconds(throttle4);
  }else{
    esc1.writeMicroseconds(1000);
    esc2.writeMicroseconds(1000);
    esc3.writeMicroseconds(1000);
    esc4.writeMicroseconds(1000);
    
    lEDOnTime = millis() - lEDOnTime2;
    if(isLEDOn && (lEDOnTime >= 500)){
      if(isDroneLevel){
        PORTD |= B10000000;   // Setting digital pin 7 HIGH
      }
      PORTB &= B11111110;     // Setting digital pin 8 LOW
      isLEDOn = false;
      lEDOnTime2 = millis();
      lEDOnTime = 0;
    }else if(!isLEDOn && (lEDOnTime >= 500)){
      PORTD &= B01111111;   // Setting digital pin 7 LOW
      PORTB |= B00000001;     // Setting digital pin 8 HIGH
      isLEDOn = true;
      lEDOnTime2 = millis();
      lEDOnTime = 0;
    }
  } 
}//end of loop void
