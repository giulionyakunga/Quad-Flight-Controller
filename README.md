# Quad-Flight-Controller
Quadcopter Flight controller using an ATmega328P Microcontroller.
![Picture of ATmega328P]()
The aim of this project is to design and build a flight controllere circuit for a Quadcopter using an ATmega328P Microcontroller. An ATmega328P Microcontroller is a general purpose microcontroller with 14 digital pins and 6 analogy pins.
The flight controller circuir will be integrated with some peripheral electronic components to safely control the drone. The flight controller will receive control signals from a RF Transimitter throught a RF Receiver, and decode the signals. The Circuit will also read and process the *IMU* sensor data for drone balancing. Finally the circuit will control speed of motors using Electronic Speed controllers (ESC).

NOTE
> FC stands for Flight Controller"

## FC Circuit Components
1. Atmega328P Microcontroller
2. 16Mhz Crystal Oscillator
3. 2 22p Ceramic Capacitor
4. 1 Push Button Switch
5. 1 10K Resistor
6. 3 200/220 Resistor
7. 3 LED Doide
8. 1x8 Female Pinheader
9. 2x4 Female Pinheader
10. 1x4 Female Pinheader
11. Microcontroller chip Holder
12. Gyroscope (MPU6050) 1 10000

## Flight Controller Circuit
![Flight Controller Circuit](https://raw.githubusercontent.com/giulionyakunga/Quad-Flight-Controller/main/Flight%20Controller%20Circuit.jpg)
*The Circuit was programmed using Arduino IDE. ```Arduino Code``` for the circuit are in the folder named Quadcopter_Flight_Controller in this Repository.*

## The Drone
![The Drone](https://raw.githubusercontent.com/giulionyakunga/Quad-Flight-Controller/main/Drone.jpg)
