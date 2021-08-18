#include <SoftwareSerial.h>
#include <Servo.h>
#include <Wire.h>

// String and integer variables for data received from Serial port
String received_string ="";
int received_character = 0;

// loop counter
int loop_counter = 0;
// blink delay time
int blink_delay_time = 500;

//Servo variables for ESC1 and ESC2
Servo ESC1;
Servo ESC2;
Servo ESC3;
Servo ESC4;

// ESC angle of rotation
int esc_angle1= 48;
int esc_angle2= 48;  
int esc_angle3= 48;
int esc_angle4= 48;  
int speedLimit = 131;

// Boolean variables for motor status
boolean isMotorOn = false;
boolean isOn = false;
boolean isOff = false;
boolean isESCArmed = false;
boolean isMotor1On = true;
boolean isMotor2On = true;
boolean isMotor3On = true;
boolean isMotor4On = true;

// Boolean variables for WI-FI status
boolean isWifiModuleActive = false;
boolean isServerModeStarted = false;
boolean isReceivingSerialData = false;

// initializing string variable for web page
String webpage = "";
String tmpString = "";
int n = 0;

// SoftwareSerial variable for Serial communications
SoftwareSerial mySerial(0, 1); // RX, TX

const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;
int totalTime = 0;
int waitedTime = 0;
void setup() {
  // defining indicating lED pins: 7 and 8
  pinMode(7,OUTPUT);
  pinMode(8,OUTPUT);
  digitalWrite(7, LOW);
  digitalWrite(8, LOW);
  
  // initializing mySerial Port for communicating with ESP8266, baud rate: 115200
  mySerial.begin(9600);
  
  while (!mySerial) {
    // wait for mySerial port to connect
  }
  
  mySerial.flush();
  delay(1000);
  
  // defining servo motor pins: 5,6,10 and 11
  ESC1.attach(5, 5, 2200);
  ESC1.write(0);

  ESC2.attach(6, 5, 2200);
  ESC2.write(0);

  ESC3.attach(9, 5, 2200);
  ESC3.write(0);

  ESC4.attach(10, 5, 2200);
  ESC4.write(0);

  delay(100);
  digitalWrite(8, LOW);
  delay(500);
  digitalWrite(8, HIGH);
  delay(1000);
  digitalWrite(8, LOW);
  delay(500);

  if(!isServerModeStarted){
    startEspServerMode();
  }
  digitalWrite(7, LOW);
  digitalWrite(8, LOW);
  delay(1000);
  digitalWrite(7, HIGH);
  digitalWrite(8, HIGH);
  delay(2000);
  digitalWrite(7, LOW);
  digitalWrite(8, LOW);
  delay(1000);
  digitalWrite(7, HIGH);
  digitalWrite(8, HIGH);
  delay(2000);
  digitalWrite(7, LOW);
  digitalWrite(8, LOW);
  delay(1000);
  
  if(isServerModeStarted){
    digitalWrite(8, HIGH);
  }
  totalTime = millis();
}

void loop() {
  while(!mySerial.available()){
    waitedTime = millis() - totalTime;
    if(waitedTime > 2000){
      totalTime = millis();
      if(isOn){
        driveESC();
        delay(2000);
      }
      if(isOff){
        driveESCOff();
        delay(1000);
      }
    }
  }
  if(mySerial.available()){
    while(mySerial.available()){
      received_character = mySerial.read();
      received_string += (char) received_character;
    }
    if(received_character == 10){
          if(searchString(received_string, "drone on")){
            startESC();
            isOn = true;
            isOff = false;
            digitalWrite(7, HIGH);
            received_string += " drone is on";
            sendData(received_string);
          }
          else if(searchString(received_string, "drone off")){
            isOn = false;
            isOff = true;
            digitalWrite(7, LOW);
            received_string += " drone is off";
            sendData(received_string);
          }

          else if(searchString(received_string, "drone restart")){
            restartESC();
            isOn = false;
            isOff = false;
            digitalWrite(7, LOW);
            received_string += " drone is restarting";
            sendData(received_string);
          }

          else if(searchString(received_string, "drone up")){
            speedLimit = speedLimit + 1;
            delay(1000);
            driveESC();
            received_string += " drone is moving up";
            sendData(received_string);
          }

          else if(searchString(received_string, "stop motor 1")){
            stopESC1();
          }

          else if(searchString(received_string, "stop motor 2")){
            stopESC2();
          }

          else if(searchString(received_string, "drone up")){
            droneUP();
//            sendData(received_string);
          }
          else if(searchString(received_string, "drone down")){
            droneDown();
//             sendData(received_string);
          }
          else if(searchString(received_string, "FORWARD")){
            droneForward();
            // sendData(received_string);
          }
          else if(searchString(received_string, "drone backward")){
            droneBackward();
            // sendData(received_string);
          }
          else if(searchString(received_string, "drone right")){
            droneRight();
            // sendData(received_string);
          }
          else if(searchString(received_string, "drone left")){
            droneLeft();
            // sendData(received_string);
          }
          else if(searchString(received_string, "drone rotate clockwise")){
            droneRotateClockwise();
            // sendData(received_string);
          }
          else if(searchString(received_string, "drone rotate anticlockwise")){
            droneRotateAntiClockwise();
            // sendData(received_string);
          }
          else if(searchString(received_string, "post")){
            sendData(tmpString);
          }
          else if(searchString(received_string, "get") || searchString(received_string, "GET")){
            sendWebpage();
          }
          else if(!searchString(received_string, "OK")){
          }
          received_character = 0;
          received_string = "";
    }
  }
}

String trimSerialData(String mainString){
  int n1 = mainString.length();
  int n2 = 0;
  String tempString = "";
  if(n1 > 1){
    for(int k = 0; k < n1; k++){
      if(mainString[k] == '+'){
        for(int m=k+1; m<n1; m++){
          tempString[n2] = mainString[m];
          n2++;
        }
      }
    }
  }
  return tempString;
}

boolean searchString(String mainString, String searchString){
  int n1 = mainString.length();
  int n2 = searchString.length();
  String tempString = "";
  boolean isFound = false;
  if(n1 > n2){
    for(int k = 0; k < n1; k++){
      for(int m = k; m < k+n2; m++){
        tempString += mainString[m];
      }
      if(tempString == searchString){
        isFound = true;
        return true;
      }
      tempString = "";
    }
  }
  return isFound;
}

void changeBaud(){
  mySerial.println("AT");
  delay(50);
  mySerial.println("AT+CIOBAUD=9600");
  mySerial.begin(9600);
  while (!mySerial) {
    // wait for mySerial port to connect
  }
  while(mySerial.available()){
    mySerial.read();
  }
}

boolean startEspServerMode(){
  boolean isWIFIModuleOn = false;
  while(!isWIFIModuleOn){
    mySerial.println("AT\r");
    delay(100);
    if(mySerial.available()){
      while(mySerial.available()){
        received_character = mySerial.read();
        received_string += (char) received_character;
      }
      if(received_character == 10){
        if(searchString(received_string, "OK")){
          isWIFIModuleOn = true;
        }
        received_character = 0;
        received_string = "";
      }
    }
  }
  mySerial.println("AT+CIPMUX=1");
  delay(100);
  if(mySerial.available()){
      while(mySerial.available()){
        received_character = mySerial.read();
        received_string += (char) received_character;
      }
      if(received_character == 10){
        if(searchString(received_string, "OK")){
          isWIFIModuleOn = true;
        }
        received_character = 0;
        received_string = "";
      }
  }
  mySerial.println("AT+CIPSERVER=1,80");
  delay(100);
  if(mySerial.available()){
      while(mySerial.available()){
        received_character = mySerial.read();
        received_string += (char) received_character;
      }
      if(received_character == 10){
        if(searchString(received_string, "OK")){
          isWIFIModuleOn = true;
        }
        received_character = 0;
        received_string = "";
      }
  }
  isServerModeStarted = true;
  return true;
}

void sendData(String dataToSend){
  dataToSend += "\n";
  int n = dataToSend.length();
  mySerial.print("AT+CIPSEND=0,");
  mySerial.println(n);
  delay(5);
  mySerial.println(dataToSend);
  delay(5);
  mySerial.println("AT+CIPCLOSE=0");
  delay(5);
  mySerial.println("AT+CIPCLOSE=1");
  delay(5);
  mySerial.println("AT+CIPCLOSE=2");
  delay(5);
  mySerial.println("AT+CIPCLOSE=3");
}

void sendWebpage(){
  int n =0;
  webpage = "HTTP/1.1 200 OK\r"; n= n + webpage.length();
  webpage = "Date: Sat, 18 Jul 2020 13:46:57 GMT\r";  n= n + webpage.length();
  webpage = "Server: Mdude Drone\r"; n= n + webpage.length();
  webpage = "Vary: Accept-Encoding\r"; n= n + webpage.length();
  webpage = "Content-Encoding: UTF-8\r";  n= n + webpage.length();
  webpage = "Content-Length: 150\r";  n= n + webpage.length();
  webpage = "Keep-Alive: timeout=5, max=100\r"; n= n + webpage.length();
  webpage = "Connection: Keep-Alive\r"; n= n + webpage.length();
  webpage = "Content-Type: text/html; charset=UTF-8\r"; n= n + webpage.length();
  webpage = "<!DOCTYPE html><html><body style='background-color:blue'><h1>Julio Telecoms</h1><h2>Mdude Drone LTD responding HTTP GET</h2></body></html>"; n= n + webpage.length();
  
  n=n+20;
  mySerial.print("AT+CIPSEND=0,");
  mySerial.println(n);
  delay(5);
  mySerial.println("HTTP/1.1 200 OK\r");
  mySerial.println("Date: Sat, 18 Jul 2020 13:46:57 GMT\r");
  mySerial.println("Server: Mdude Drone\r");
  mySerial.println("Vary: Accept-Encoding\r");
  mySerial.println("Content-Encoding: UTF-8\r");
  mySerial.println("Content-Length: 150\r");
  mySerial.println("Keep-Alive: timeout=5, max=100\r");
  mySerial.println("Connection: Keep-Alive\r");
  mySerial.println("Content-Type: text/html; charset=UTF-8\r");
  mySerial.println();
  mySerial.println(webpage);
  delay(5);
  mySerial.println("AT+CIPCLOSE=0");
}

void armESC(){

  esc_angle1 = 19;
  esc_angle2 = 19;
  esc_angle3 = 19;
  esc_angle4 = 19;
  ESC1.write(esc_angle1);
  ESC2.write(esc_angle2);
  ESC3.write(esc_angle3);
  ESC4.write(esc_angle4);
  delay(1000);
  
  esc_angle1 = 20;
  esc_angle2 = 20;
  esc_angle3 = 20;
  esc_angle4 = 20;
  ESC1.write(esc_angle1);
  ESC2.write(esc_angle2);
  ESC3.write(esc_angle3);
  ESC4.write(esc_angle4);
  delay(1000);

  esc_angle1 = 21;
  esc_angle2 = 21;
  esc_angle3 = 21;
  esc_angle4 = 21;
  ESC1.write(esc_angle1);
  ESC2.write(esc_angle2);
  ESC3.write(esc_angle3);
  ESC4.write(esc_angle4);
  delay(1000);

  esc_angle1 = 22;
  esc_angle2 = 22;
  esc_angle3 = 22;
  esc_angle4 = 22;
  ESC1.write(esc_angle1);
  ESC2.write(esc_angle2);
  ESC3.write(esc_angle3);
  ESC4.write(esc_angle4);
  delay(1000);

  esc_angle1 = 23;
  esc_angle2 = 23;
  esc_angle3 = 23;
  esc_angle4 = 23;
  ESC1.write(esc_angle1);
  ESC2.write(esc_angle2);
  ESC3.write(esc_angle3);
  ESC4.write(esc_angle4);
  delay(1000);
  
  esc_angle1 = 125;
  esc_angle2 = 125;
  esc_angle3 = 125;
  esc_angle4 = 125;
  ESC1.write(esc_angle1);
  ESC2.write(esc_angle2);
  ESC3.write(esc_angle3);
  ESC4.write(esc_angle4);
  isESCArmed = true;
  isMotorOn = true;
}

void driveESC(){
  if(esc_angle1 < speedLimit){
    esc_angle1 = esc_angle1 + 1;
    esc_angle2 = esc_angle2 + 1;
    esc_angle3 = esc_angle3 + 1;
    esc_angle4 = esc_angle4 + 1;
  }
  ESC1.write(esc_angle1);
  ESC2.write(esc_angle2);
  ESC3.write(esc_angle3);
  ESC4.write(esc_angle4);
}

void driveESCOff(){
  if(esc_angle1 > 18){
    esc_angle1 = esc_angle1 - 2;
    esc_angle2 = esc_angle2 - 2;
    esc_angle3 = esc_angle3 - 2;
    esc_angle4 = esc_angle4 - 2;
  }else{
    isOff = false;
  }
  ESC1.write(esc_angle1);
  ESC2.write(esc_angle2);
  ESC3.write(esc_angle3);
  ESC4.write(esc_angle4);
}

void restartESC(){
  esc_angle1 = 0;
  esc_angle2 = 0;
  esc_angle3 = 0;
  esc_angle4 = 0;
  ESC1.write(esc_angle1);
  ESC2.write(esc_angle2);
  ESC3.write(esc_angle3);
  ESC4.write(esc_angle4);
}

void startESC(){
  armESC();
  delay(2000);
  driveESC();
}

void stopESC(){
    isMotorOn = false;
    isESCArmed = false;
    esc_angle1 = 0;
    esc_angle2 = 0;
    ESC1.write(esc_angle1);
    ESC2.write(esc_angle2);
}

void stopESC1(){
    esc_angle1 = 0;
    ESC1.write(esc_angle1);
    isMotor1On = false;
}

void stopESC2(){
    esc_angle2 = 0;
    ESC2.write(esc_angle1);
    isMotor2On = false;
}

void droneUP(){
  if(!isMotorOn){
    esc_angle1 = esc_angle1 + 2;
    esc_angle2 = esc_angle2 + 2;
    ESC1.write(esc_angle1);
    ESC2.write(esc_angle2);
    isMotorOn = true;
  }
}

void droneDown(){
  
}

void droneForward(){
  
}

void droneBackward(){
  
}

void droneRight(){
  
}

void droneLeft(){
  
}

void droneRotateClockwise(){
  
}

void droneRotateAntiClockwise(){
  
}
