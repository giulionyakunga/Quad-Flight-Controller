/*
  Quadcopter Flight Controller Sketch
  This code was written by Julio Nyakunga on 7th of July 2020 
  http://www.mdudegiulio.org/en/Tutorial/Quadcopter_Flight_Controller
  feb 24
*/

#include <EnableInterrupt.h>
#include <Servo.h>
#include <Wire.h>   //Include I2C library00
#include <MPU9250.h>
#include <EEPROM.h>
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

/****************************    Motor minthrottle    *******************************/
    /* Set the minimum throttle command sent to the ESC (Electronic Speed Controller)
       This is the minimum value that allow motors to run at a idle speed  */
    //#define MINTHROTTLE 1300 // for Turnigy Plush ESCs 10A
    //#define MINTHROTTLE 1120 // for Super Simple ESCs 10A
    //#define MINTHROTTLE 1064 // special ESC (simonk)
    //#define MINTHROTTLE 1050 // for brushed ESCs like ladybird
    #define MINTHROTTLE 1150 // (*) (**)

  /****************************    Motor maxthrottle    *******************************/
    /* this is the maximum value for the ESCs at full power, this value can be increased up to 2000 */
    #define MAXTHROTTLE 1850

// Digital pins for RF Channels
#define RC_CH1_INPUT  6    // Roll Channel 
#define RC_CH2_INPUT  5    // Pitch Channel
#define RC_CH3_INPUT  4    // Throttle Channel
#define RC_CH4_INPUT  1     // Yaw/Rudder Channel
#define RC_CH5_INPUT  0     // Camera Channel

#define THROTTLE 1000
int PITCH = 1500;
#define YAW 1500
#define ROLL 1500

int addr = 0;

int changeInThrottle = 0;
int changeInPitch = 0;
int changeInRoll = 0;
int changeInYaw = 0;

int rc_values[RC_NUM_CHANNELS];
uint32_t rc_start[RC_NUM_CHANNELS];
uint32_t trottle_channel;                             //  This variable stores Throttle channel pulse width
volatile uint16_t rc_shared[RC_NUM_CHANNELS];

Servo esc1; // Motor 1
Servo esc2; // Motor 2
Servo esc3; // Motor 3
Servo esc4; // Motor 4

// Digital pins for ESC Signals
int motor_1 = 3;
int motor_2 = 2;
int motor_3 = 9;
int motor_4 = 10;

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

/////////////////PID CONTROLLER OUTPUTS/////////////////
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
double kp_roll = 0.2;  // 0.2
double ki_roll = 0.0;
double kd_roll = 0.4;  // 0.4

double kp_pitch = kp_roll;
double ki_pitch = ki_roll;
double kd_pitch = kd_roll;

double kp_yaw = 0.0;
double ki_yaw = 0.0;
double kd_yaw = 0.0;

float desired_roll_angle = 0.0;         //This is the angle in which we whant the drone to stay steady
float desired_pitch_angle = 0.0;        //This is the angle in which we whant the drone to stay steady
float desired_yaw_angle = 0.0;          //This is the angle in which we whant the drone to stay steady
boolean isDroneLevel = false;           //This boolean variable is used to check if the drone is level at startup
boolean gyroReady = false;
int battery_voltage;
boolean battery_low = false;
boolean is_transmiter_connected = false;
unsigned long loop_timer;
int motor = 1;
int motor_speed = 2000;

MPU9250 IMU (Wire , 0x68);      // MPU9250 is a class and "IMU" is a object, we need to pass parameter to the object "IMU". wire is used for I2C communication, 
                                // second parameter is for I2C address, we left the ADO pin unconnected so its set to low, 0x68 is address,  
                                // if it was high then the address is 0x69

float Acc_rawX, Acc_rawY, Acc_rawZ, acc_total_vector, Gyr_rawX, Gyr_rawY, Gyr_rawZ,Mag_rawX, Mag_rawY, Mag_rawZ;
float speed_X, speed_Y, speed_Z;
float Temperature;
float Acceleration_angle[3];
float Acceleration_angle_2[3];
float Gyro_angle[3];
float Mag_angle[3];
float Total_angle[3];
float gyro_axis_cal[3];
float acc_angle_cal[3];
float pitch, pitchRad, prev_pitch, change_in_pitch, pitch_error, roll, rollRad, roll_error, yaw, yaw_2, yaw_3, yawRad, yaw_error, magnetic_north, magnetic_north_rad;
float elapsedTime, time, timePrev;
boolean gyro_angles_set;
void setup() {                  // put your setup code here, to run once:
  IMU.begin();                  // Initialize the IMU object

//  Serial.begin(SERIAL_PORT_SPEED);
//  mySoftwareSerial.begin(9600);
//  mySoftwareSerial.println("Starting");

  //Setting Pins mode
  DDRB = B11010111;   //Setting PORTB, digital pin 8,9 and 10 to OUTPUT and digital pin 11 and 13 to INPUT (0=INPUT, 1=OUTPUT)  
  DDRD = B10001101;   //Setting PORTD, digital pin 2,3 and 7 to OUTPUT and digital pin 1,4,5 and 6 to INPUT (0=INPUT, 1=OUTPUT)
  
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

  esc1.attach(motor_1);       //attatch the right motor to pin 5
  esc2.attach(motor_2);
  esc3.attach(motor_3);
  esc4.attach(motor_4);

  delay(50);

  // use this code only for arming the ESCs and not when flying the drone
  // arm_escss();
  // motor_speed = 1063;

  // testMotorsss();

  time = millis(); //Start counting time in milliseconds

  // Calculating average Gyro and Accelerometer errors
  for (int cal_int = 0; cal_int < 2000 ; cal_int ++){                           //Take 2000 readings for calibration.
    IMU.readSensor();
    gyro_axis_cal[1] += IMU.getGyroX_rads();
    gyro_axis_cal[2] += IMU.getGyroY_rads();                                       
    gyro_axis_cal[3] += IMU.getGyroZ_rads();                                                                           
    delay(3);                                                               //Wait 3 milliseconds before the next loop.
  }
  //Now that we have 2000 measures, we need to devide by 2000 to get the average gyro offset.
  gyro_axis_cal[1] /= 2000;                                                 //Divide the roll total by 2000.
  gyro_axis_cal[2] /= 2000;                                                 //Divide the pitch total by 2000.
  gyro_axis_cal[3] /= 2000;                                                 //Divide the yaw total by 2000.

  // setting flag to false to indicate that the gyro angles not set when the program begins
  gyro_angles_set = false;

  // flashing LEDs to indication starting of main loop
  for(int n=0; n<5; n++){
    digitalWrite(7, HIGH);
    digitalWrite(8, HIGH);
    delay(100);
    digitalWrite(7, LOW);
    digitalWrite(8, LOW);
    delay(100);
  }
  digitalWrite(7, LOW);
  digitalWrite(8, LOW);
//  isArmed = true;
}

void loop() {

  IMU.readSensor();
  //Accelerometer data code
  Acc_rawX = IMU.getAccelX_mss();
  Acc_rawY = IMU.getAccelY_mss();
  Acc_rawZ = IMU.getAccelZ_mss();

  speed_X = Acc_rawX;
  speed_Y = Acc_rawY;
  speed_Z = Acc_rawZ;

  /* Now we can apply the Euler formula. The atan will calculate the arctangent. The
     *  pow(a,b) will elevate the a value to the b power. And finnaly sqrt function
     *  will calculate the rooth square.*/
  /*---X---*/
  acc_total_vector = sqrt((Acc_rawX*Acc_rawX)+(Acc_rawY*Acc_rawY)+(Acc_rawZ*Acc_rawZ));       //Calculate the total accelerometer vector.

  
  
  if(abs(Acc_rawY) < acc_total_vector){                                        //Prevent the asin function to produce a NaN
    Acceleration_angle[0] = asin(Acc_rawY/acc_total_vector);          //Calculate the pitch angle.
  }

  if(abs(Acc_rawX) < acc_total_vector){                                        //Prevent the asin function to produce a NaN
    Acceleration_angle[1] = asin(Acc_rawX/acc_total_vector);          //Calculate the pitch angle.
  }
  
//  Acceleration_angle[0] = atan((Acc_rawY)/sqrt(pow((Acc_rawX),2) + pow((Acc_rawZ),2)));
  /*---Y---*/
//  Acceleration_angle[1] = atan(-1*(Acc_rawX)/sqrt(pow((Acc_rawY),2) + pow((Acc_rawZ),2)));

  Acceleration_angle[0] = -1*Acceleration_angle[0];
  Acceleration_angle[1] = Acceleration_angle[1];
  
  Acceleration_angle[0] -= 0.038;
  Acceleration_angle[1] -= -0.023;
  
  //Gyroscope data code
  Gyr_rawX = IMU.getGyroX_rads();
  Gyr_rawY = IMU.getGyroY_rads();
  Gyr_rawZ = IMU.getGyroZ_rads();

  // Subtracting the gyro calubrated errors
  Gyr_rawX -= gyro_axis_cal[1];
  Gyr_rawY -= gyro_axis_cal[2];
  Gyr_rawZ -= gyro_axis_cal[3];
  
  /*---X---*/
  Gyro_angle[0] = Gyr_rawX;
  /*---Y---*/
  Gyro_angle[1] = Gyr_rawY;
  /*---Z---*/
  Gyro_angle[2] = Gyr_rawZ;

  //Temperature reading
  Temperature = IMU.getTemperature_C();
//  Serial.print("Temperature: ");
//  Serial.println(Temperature, 2);         // gets the temperature value from the data buffer and returns it in units of C

  //Magnetometer data code
  Mag_rawX = IMU.getMagX_uT()*cos(pitchRad)- IMU.getMagY_uT()*sin(rollRad)*sin(pitchRad) + IMU.getMagZ_uT()*sin(rollRad)*sin(pitchRad);
  Mag_rawY = IMU.getMagY_uT()*cos(rollRad) + IMU.getMagZ_uT()*sin(rollRad);
  Mag_rawZ = IMU.getMagZ_uT();

  Mag_angle[0] = atan2(Mag_rawY,Mag_rawX);
  Mag_angle[0] = -1 * Mag_angle[0];
  Mag_angle[0] = 3.0 * Mag_angle[0];

//  Gyro_angle[1] -= Gyro_angle[0] * sin(Gyro_angle[2]*elapsedTime);                  //If the IMU has yawed transfer the roll angle to the pitch angel.
//  Gyro_angle[0] += Gyro_angle[1] * sin(Gyro_angle[2]*elapsedTime);                  //If the IMU has yawed transfer the pitch angle to the roll angel.


  calculate_elapsed_time();

  /*Now in order to obtain degrees we have to multiply the degree/seconds
  *value by the elapsedTime.*/
  /*Finnaly we can apply the final filter where we add the acceleration
  *part that afects the angles and ofcourse multiply by 0.98 */

  if(gyro_angles_set){
    
//    Total_angle[0] -= Total_angle[1] * sin((Gyro_angle[2] * elapsedTime));                  //If the IMU has yawed transfer the roll angle to the pitch angel.
//    Total_angle[1] += Total_angle[0] * sin((Gyro_angle[2] * elapsedTime));                  //If the IMU has yawed transfer the pitch angle to the roll angel.

    /*---X axis angle---*/
    Total_angle[0] = 0.9996*(Total_angle[0] + Gyro_angle[0]*elapsedTime) + 0.0094*Acceleration_angle[0];
    /*---Y axis angle---*/
    Total_angle[1] = 0.9996*(Total_angle[1] + Gyro_angle[1]*elapsedTime) + 0.0004*Acceleration_angle[1];
    /*---Z axis angle---*/
    //  Total_angle[2] = 0.7 *(Total_angle[2]) + 0.3 *(atan2(Mag_rawY,Mag_rawX));
    Mag_angle[0] = atan2(Mag_rawY,Mag_rawX);
    Mag_angle[0] = -1 * Mag_angle[0];
    Mag_angle[0] = 3.0 * Mag_angle[0];

    //  yaw = yaw + (Gyro_angle[2] * elapsedTime) * 360/(2*3.14);
    Total_angle[2] = 0.96*(Total_angle[2] + Gyro_angle[2] * elapsedTime) + 0.04 * Mag_angle[0];   // multiplying by 0.7 to remove noises
    
  }else{
    //Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started.
    Total_angle[0] = Acceleration_angle[0];
    Total_angle[1] = Acceleration_angle[1];
    Total_angle[2] = Mag_angle[0];
    yaw_3 = Mag_angle[0];
    gyro_angles_set = true;
  }

  rollRad = Total_angle[0];
  roll = rollRad * 360/(2*3.14);
//  roll++;
  
  pitchRad = Total_angle[1];
  pitch = pitchRad * 360/(2*3.14);

//  yaw = Total_angle[2] * 360/(2*3.14);o
  yaw = (Total_angle[2] - yaw_3) * 360/(2*3.14);
  yaw_2 = Mag_angle[0] * 360/(2*3.14);
  
 
//  Serial.print(roll,3);
//  Serial.print("    ");
//  Serial.print(pitch,3);
//  Serial.print("    ");
//  Serial.println(yaw,3);

//  Serial.print(roll,3);
//  Serial.print("\t");
//  Serial.print((Acceleration_angle[0] * 360/(2*3.14)),3);
//  Serial.print("\t");
//  Serial.print(pitch,3);
//  Serial.print("\t");
//  Serial.println((Acceleration_angle[1] * 360/(2*3.14)),3);
//  Serial.print("\t");
//  Serial.print("\t");
//  Serial.print(yaw,3);
//  Serial.print("\t");
//  Serial.println(yaw_2,3);


//  Serial.println(c1);
//  c1++;
//  ac1+=Acceleration_angle[0];
//  ac2+=Acceleration_angle[1];
//  if(c1==2000){
//    ac1=ac1/2000;
//    ac2=ac2/2000;
//    Serial.println("*****************************");
//    Serial.println(c1);
//    Serial.print(ac1,3);
//    Serial.print("\t");
//    Serial.println(ac2,3);
//    c1=0;
//  }

  change_in_pitch = pitch - prev_pitch;
  prev_pitch = pitch;

  magnetic_north_rad = Total_angle[2];
  magnetic_north = magnetic_north_rad * 360/(2*3.14);

  if((-5 <= pitch) && (pitch <= 5) && (-5 <= roll) && (roll <= 5)){
    isDroneLevel = true;
  }else{
    isDroneLevel = false;
  }

  rc_read_values();
  check_rf_connection();

//  (rc_values[RC_CH3] >= 800) && (rc_values[RC_CH3] <= 1050) && (rc_values[RC_CH4] >= 800) && (rc_values[RC_CH4] <= 1100) && (rc_values[RC_CH1] > 1400) && (rc_values[RC_CH1] < 1600) && (rc_values[RC_CH2] > 1400) && (rc_values[RC_CH2] < 1600) && !isArmed && isDroneLevel && gyroReady)

  if( (rc_values[RC_CH3] >= 900) && (rc_values[RC_CH3] <= 1150) && (rc_values[RC_CH4] >= 900) && (rc_values[RC_CH4] <= 1200) && !isArmed && isDroneLevel && gyro_angles_set ){
    isArmed = true;
    clear_pid_corrections();  // this is done to clear saved previous PID corrections particularly the Integral (I) Term
//    Serial.println("ARMED");
    PORTD |= B10000000;       // Setting digital pin 7 HIGH
    PORTB |= B00000001;       // Setting digital pin 8 HIGH
    delay(2000);
  }
  if((rc_values[RC_CH3] >= 800) && (rc_values[RC_CH3] <= 1150) && (rc_values[RC_CH4] > 1900) && isArmed){
    isArmed = false;
    clear_pid_corrections();  // this is done to clear saved previous PID corrections particularly the Integral (I) Term
//    Serial.println("DISARMED");
  }

//  PORTB |= B00000001;     // Setting digital pin 8 HIGH
//    PORTB &= B11111110;     // Setting digital pin 8 LOW
  
  if(isArmed){
    if(is_transmiter_connected){
      PORTD |= B10000000;       // Setting digital pin 7 HIGH
      if(!isDroneLevel){
        lEDOnTime = millis() - lEDOnTime2;
        if(isLEDOn && (lEDOnTime >= 100)){
          PORTB |= B00000001;     // Setting digital pin 8 HIGH
          isLEDOn = false;
          lEDOnTime2 = millis();
          lEDOnTime = 0;
        }else if(!isLEDOn && (lEDOnTime >= 100)){
          PORTB &= B11111110;     // Setting digital pin 8 LOW
          isLEDOn = true;
          lEDOnTime2 = millis();
          lEDOnTime = 0;
        }
      }else{
        PORTB |= B00000001;     // Setting digital pin 8 HIGH
      }
    }else{
      PORTD &= B01111111;       // Setting digital pin 7 LOW
      PORTB &= B11111110;       // Setting digital pin 8 LOW
    }
    
    throttle = rc_values[RC_CH3];
    if (throttle > 1800) throttle = 1800;                          //This prevent motor to get into full throttle
    
    changeInPitch = 1500 - (rc_values[RC_CH2] - 16);       // Addding 52 to trim the roll stick to center
    if(changeInPitch > 50){
      desired_pitch_angle = map(changeInPitch, 50.0, 500.0, 0.0, -10.0);
    }else if(changeInPitch < -50){      
      desired_pitch_angle = map(changeInPitch, -50.0, -500.0, 0.0, 10.0);
    }else{
      desired_pitch_angle = 0.0;
    }

    changeInRoll = ROLL - (rc_values[RC_CH1] + 20);         // Addding 16 to trim the roll stick to center
    if(changeInRoll > 50){
      desired_roll_angle = map(changeInRoll, 50.0, 500.0, 0.0, -10.0);
    }else if(changeInRoll < -50){
      desired_roll_angle = map(changeInRoll, -50.0, -500.0, 0.0, 10.0);
    }
    else{
      desired_roll_angle = 0.0;
    }

    changeInYaw = YAW - rc_values[RC_CH4];
    if(changeInYaw > 100){
      desired_yaw_angle = map(changeInYaw, 100.0, 500.0, 0.0, 10.0);
    }else if(changeInYaw < -100){
      desired_yaw_angle = map(changeInYaw, -100.0, -500.0, 0.0, -10.0);
    }
    else{
      desired_yaw_angle = 0.0;
    }

    pid_controller();

//    Serial.print(rc_values[RC_CH1]);
//    Serial.print("  rp=");
//    Serial.print(rc_values[RC_CH2]);
//    Serial.print("  rt=");
//    Serial.print(rc_values[RC_CH3]);
//    Serial.print("  ry=");
//    Serial.println(rc_values[RC_CH4]);

//    Serial.print("  cr=");
//    Serial.print(changeInRoll);
//    Serial.print("  cp=");
//    Serial.print(changeInPitch);
//
//    Serial.print("   r=");
//    Serial.print(roll,3);
//    Serial.print("   p=");
//    Serial.print(pitch,3);
//    Serial.print("   y=");
//    Serial.print(yaw,3);
//
   
//    Serial.print("  dr=");
//    Serial.print(desired_roll_angle,3);
//    Serial.print("  dp=");
//    Serial.print(desired_pitch_angle,3);
//    Serial.print("  dy=");
//    Serial.print(desired_yaw_angle,3);

//    Serial.print("  pr=");
//    Serial.print((int)pid_p_roll);
//    Serial.print("  pp=");
//    Serial.print((int)pid_p_pitch);
//    Serial.print("  ky=");
//    Serial.println(pid_p_pitch,3);
    

      if(throttle > 1200){
        //Adding PID corrections
        // Throttle signal is the base signal
        throttle1 = throttle - (int)pid_roll - (int)pid_pitch + (int)pid_yaw;
        throttle2 = throttle + (int)pid_roll - (int)pid_pitch - (int)pid_yaw;
        throttle3 = throttle + (int)pid_roll + (int)pid_pitch + (int)pid_yaw; 
        throttle4 = throttle - (int)pid_roll + (int)pid_pitch - (int)pid_yaw;

        if(throttle1 < 1200) throttle1= 1200;
        if(throttle2 < 1200) throttle2= 1200;
        if(throttle3 < 1200) throttle3= 1200;
        if(throttle4 < 1200) throttle4= 1200;

        if(throttle1 > 2000) throttle1=2000;
        if(throttle2 > 2000) throttle2=2000;
        if(throttle3 > 2000) throttle3=2000;
        if(throttle4 > 2000) throttle4=2000;
      }else{
        clear_pid_corrections();
        throttle1 = throttle;
        throttle2 = throttle;
        throttle3 = throttle;
        throttle4 = throttle;
      }

    /*Once again we map the PWM values to be sure that we won't pass the min
    and max values and that we don't mess up the ESC.
  */

    /*
      * Once again we limit the PWM values to be sure that no motor should stop running
    */
    
    //All the information for controlling the motor's is available.
    //The refresh rate is 250Hz. That means the esc's need there pulse every 4ms.
    if((micros() - loop_timer) > 4000){                         //We wait until 4000us are passed.
      loop_timer = micros();                                    //Set the timer for the next loop.
      esc1.writeMicroseconds(throttle1);
      esc2.writeMicroseconds(throttle2);
      esc3.writeMicroseconds(throttle3);
      esc4.writeMicroseconds(throttle4);
    }
    
  }else{
    //The refresh rate is 250Hz. That means the esc's need there pulse every 4ms.
    if((micros() - loop_timer) > 4000){                         //We wait until 4000us are passed.
      loop_timer = micros();                                    //Set the timer for the next loop.
      if(motor == 1){
        motor = 2;
        esc1.writeMicroseconds(900);
      }else if(motor == 2){
        motor = 3;
        esc2.writeMicroseconds(900);
      }else if(motor == 3){
        motor = 4;
        esc3.writeMicroseconds(900);
      }else if(motor == 4){
        motor = 1;
        esc4.writeMicroseconds(900);
      }
    }

    if(is_transmiter_connected){   
      lEDOnTime = millis() - lEDOnTime2;
      if(isLEDOn && (lEDOnTime >= 500)){
        if(isDroneLevel){
          PORTB |= B00000001;     // Setting digital pin 8 HIGH
        }
        PORTD &= B01111111;   // Setting digital pin 7 LOW
        isLEDOn = false;
        lEDOnTime2 = millis();
        lEDOnTime = 0;
      }else if(!isLEDOn && (lEDOnTime >= 500)){
        PORTB &= B11111110;     // Setting digital pin 8 LOW
        PORTD |= B10000000;   // Setting digital pin 7 HIGH
        isLEDOn = true;
        lEDOnTime2 = millis();
        lEDOnTime = 0;
      }
    }else{
      PORTD &= B01111111;       // Setting digital pin 7 LOW
      PORTB &= B11111110;       // Setting digital pin 8 LOW
    }
  }
//  while(battery_low){
//    check_battery_voltage();
//    PORTD |= B10000000;   // Setting digital pin 7 HIGH
//    PORTB |= B00000001;   // Setting digital pin 8 HIGH
//  }

}

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


void test_rf_channels(){
//  RC_CH3 throttle
//  RC_CH2 pitch
//  RC_CH4 Yaw


  if(rc_values[RC_CH2] >= 1200){
    digitalWrite(7,HIGH);
  }else{
    digitalWrite(7,LOW);
  }

  if(rc_values[RC_CH3] >= 1200){
    digitalWrite(8,HIGH);
  }else{
    digitalWrite(8,LOW);
  }
}
void testMotors(){
    Serial.println("Testing motors");
    for (int motor_speed = 1050; motor_speed < 1200; motor_speed++){
      esc1.writeMicroseconds(motor_speed);
      esc2.writeMicroseconds(motor_speed);
      esc3.writeMicroseconds(motor_speed);
      esc4.writeMicroseconds(motor_speed);
      Serial.println(motor_speed);
      delay(1000);
    }
}

void arm_esc() {
  // put your main code here, to run repeatedly:

    delay(1000);
    digitalWrite(7, HIGH);
    digitalWrite(8, HIGH);
    delay(300);
    digitalWrite(7, LOW);
    digitalWrite(8, LOW);
    
    motor_speed = 2000;
    esc1.writeMicroseconds(motor_speed);
    esc2.writeMicroseconds(motor_speed);
    esc3.writeMicroseconds(motor_speed);
    esc4.writeMicroseconds(motor_speed);
    delay(5000);

    digitalWrite(7, HIGH);
    digitalWrite(8, HIGH);
    delay(300);
    digitalWrite(7, LOW);
    digitalWrite(8, LOW);

    motor_speed = 1000;
    esc1.writeMicroseconds(motor_speed);
    esc2.writeMicroseconds(motor_speed);
    esc3.writeMicroseconds(motor_speed);
    esc4.writeMicroseconds(motor_speed);
    
    delay(5000);

    digitalWrite(7, HIGH);
    digitalWrite(8, HIGH);
    delay(2000);

    digitalWrite(7, LOW);
    digitalWrite(8, LOW);

}

float check_battery_voltage(){
  battery_voltage = analogRead(0);
  float voltage = battery_voltage * (5.00 / 1023.00) * 4; //convert the value to a true voltage.
  if (voltage < 14.0) //set the voltage considered low battery here
  {
    isArmed = false;
    battery_low = true;
  }else{
    battery_low = false;
  }
  return voltage;
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
void calc_ch3() { 
  trottle_channel = micros();
  calc_input(RC_CH3, RC_CH3_INPUT); 
}
void calc_ch4() { calc_input(RC_CH4, RC_CH4_INPUT); }
void calc_ch5() { calc_input(RC_CH5, RC_CH5_INPUT); }

void get_mpu_6050_data(){
  
}
  

void pid_controller(){

  /*
   * ******************ROLL PID CALCULATIONS**************************
  */
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

  pid_i_roll = pid_i_roll + ki_roll*error_roll;
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
  error_roll = roll - desired_roll_angle;
  pid_d_roll = kd_roll*((error_roll - previous_error_roll)/elapsedTime);
  previous_error_roll = error_roll;
  
  /*The final PID values is the sum of each of this 3 parts*/
  pid_roll = pid_p_roll + pid_i_roll + pid_d_roll;
  if(pid_roll > pid_max) pid_roll = pid_max;
  else if(pid_roll < pid_max*-1) pid_roll = pid_max*-1;

   /*
   * ******************PITCH PID CALCULATIONS**************************
  */
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

  pid_i_pitch = pid_i_pitch + ki_pitch*error_pitch;
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
  // Calculating the d term for pitch
  error_pitch = pitch - desired_pitch_angle;
  pid_d_pitch = kd_pitch*((error_pitch - previous_error_pitch)/elapsedTime);
  previous_error_pitch = error_pitch;


  /*The final PID values is the sum of each of this 3 parts*/
  pid_pitch = pid_p_pitch + pid_i_pitch + pid_d_pitch;
  if(pid_pitch > pid_max) pid_pitch = pid_max;
  else if(pid_pitch < pid_max*-1) pid_pitch = pid_max*-1;
  
   /*
   * ******************YAW PID CALCULATIONS**************************
  */
  /*First calculate the error between the desired angle and 
    *the real measured angle
  */
  error_yaw = yaw_2 - desired_yaw_angle;
    
  /*Next the proportional value of the PID is just a proportional constant
    *multiplied by the error
  */ 
  pid_p_yaw = kp_yaw*error_yaw;

  /*The integral part should only act if we are close to the
    desired position but we want to fine tune the error. That's
    why I've made a if operation for an error between -2 and 2 degree.
    To integrate we just sum the previous integral value with the
    error multiplied by  the integral constant. This will integrate (increase)
    the value each loop till we reach the 0 point
  */
//  if((-5 < error_yaw) && (error_yaw < 5))
//  {
//    pid_i_yaw = pid_i_yaw+(ki_yaw*error_yaw);
//  }

  pid_i_yaw = pid_i_yaw+(ki_yaw*error_yaw);
  if(pid_i_yaw > 20.0){
    pid_i_yaw = 20.0;
  }
  if(pid_i_yaw < -20.0){
    pid_i_yaw = -20.0;
  }

  /*The last part is the derivate. The derivate acts upon the speed of the error.
    As we know the speed is the amount of error that produced in a certain amount of
    time divided by that time. For taht we will use a variable called previous_error.
    We substract that value from the actual error and divide all by the elapsed time. 
    Finnaly we multiply the result by the derivate constant
  */
  
  // Calculating the d term for yaw
  pid_d_yaw = kd_yaw*((error_yaw - previous_error_yaw)/elapsedTime);
  previous_error_yaw = error_yaw;


  /*The final PID values is the sum of each of this 3 parts*/
  pid_yaw = pid_p_yaw + pid_i_yaw + pid_d_yaw;
  if(pid_yaw > pid_max) pid_yaw = pid_max;
  else if(pid_yaw < pid_max*-1) pid_yaw = pid_max*-1;
}

void clear_pid_corrections(){
    pid_p_roll = 0;
    pid_i_roll = 0;
    pid_d_roll = 0;
    pid_roll = 0;

    pid_p_pitch = 0;
    pid_i_pitch = 0;
    pid_d_pitch = 0;
    pid_pitch = 0;

    pid_p_yaw = 0;
    pid_i_yaw = 0;
    pid_d_yaw = 0;
    pid_pitch = 0;
}

void check_rf_connection(){
  if((micros() - trottle_channel) > 500000){
    is_transmiter_connected = false;
  }else{
    is_transmiter_connected = true;
  }
}
