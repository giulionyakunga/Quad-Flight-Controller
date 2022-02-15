# Quad-Flight-Controller
Quadcopter Flight controller using an ATmega328P Microcontroller.
![Picture of ATmega328P](https://raw.githubusercontent.com/giulionyakunga/Quad-Flight-Controller/main/ATmega328P.jpg)

The aim of this project is to design and build a flight controller circuit for a Quadcopter using an ATmega328P Microcontroller. An ATmega328P Microcontroller is a general purpose microcontroller with 14 digital pins and 6 analogy pins.
The flight controller circuir will be integrated with some peripheral electronic components to safely control the drone. The flight controller will receive control signals from RF Transimitter throught RF Receiver, and decode the signals. The Circuit will also read and process the *IMU* sensor data for drone balancing. Finally the circuit will control speed of blushless motors using PWM signals that will be sent to electronic Speed controllers (ESC).

This project was divided into four main tasks: namely
  1. Flight Controller Circuit designing and building
  2. Coding and programming the Flight controller circuit using Arduino IDE
  3. Mounting the flight controller and other drone components on a DJI F450 frame and
  4. Testing the flight controller 

NOTE
> FC stands for Flight Controller

## Flight controller circuit designing and building
I designed the FC circuit using proteus, then after I printed the circuit and soldered the electronic components on the PCB.
### FC circuit schematic diagram
![Flight Controller Schematic Circuit Diagram](https://raw.githubusercontent.com/giulionyakunga/Quad-Flight-Controller/main/Flight%20Controller%20Schematic%20Circuit%20Diagram.png)
### FC circuit PCB design
![Flight Controller PCB Design](https://raw.githubusercontent.com/giulionyakunga/Quad-Flight-Controller/main/Flight%20Controller%20PCB%20Design.jpg)

### FC circuit contains the following components
1. Atmega328P microcontroller
2. 16Mhz crystal oscillator
3. 2 22p ceramic capacitor
4. 1 Push button switch
5. 1 10K resistor
6. 3 200/220 Resistor
7. 3 LED doide
8. 1x8 female pinheader
9. 2x4 female pinheader
10. 1x4 female pinheader
11. Microcontroller chip holder
12. MPU-9265 (gyroscope, accelerometer and magnetometer)

## Flight controller circuit
![Flight Controller Circuit](https://raw.githubusercontent.com/giulionyakunga/Quad-Flight-Controller/main/Drone%20Circuit%204%20(edited).jpeg)
*The circuit was programmed using Arduino IDE. ```Arduino Code``` for the circuit are in the folder named Quadcopter_Flight_Controller in this repository.* ![View Code](https://github.com/giulionyakunga/Quad-Flight-Controller/blob/main/Quadcopter_Flight_Controller/Quadcopter_Flight_Controller.ino)

## Flight controller circuit mounted on a DJI F450
![Flight Controller Circuit](https://raw.githubusercontent.com/giulionyakunga/Quad-Flight-Controller/main/the%20flight%20controller.jpg


## The Drone
![The Drone](https://raw.githubusercontent.com/giulionyakunga/Quad-Flight-Controller/main/the%20drone_3.jpg)
