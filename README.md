# Index

- 1.[ Introduction](#introduction)
    -
- 2.[ Components](#components)
    -
    - [Materials List](#materials-list)
    - [Arduino mega](#arduino-mega)
    - [Servo motor MG90S](#servo-motor-mg90s)
    - [Bridge H (L293N)](#bridge-h-l293n)
    - [Pixy2 cam](#pixy2-cam)
    - [Ultrasonic sensor (HC-S04)](#ultrasonic-sensor-hc-s04)
    - [Battery and charger](#battery-and-charger)
    - [Motor GA37-520](#motor-ga37-520)
    - [LM2596 HW-411 Step down](#lm2596-hw-411-step-down)
  

- 3.[ Robot floors](#robot-floors)
    -
    - [First and Second Floor](#first-and-second-floor)
        - [Front axel](#front-axle)
        - [Back axel](#back-axel)

    - [Third floor](#third-floor)
    - [Fourth floor](#fourth-floor)
        
- 4.[ 3D desing](models/)
    -
    - [Parts list](models/README.md#parts-list)
    - [Printing settings and recommendations](models/README.md#printing-settings-and-recommendations)
    - [STL files](models/stl/)
    - [Files ready to print](models/ready%20to%20print/)
    - [Full Chasis](models/Full%20chassis%20turkey.step)
  

- 5.[ Circuit diagram](schemes/README.md#circuit-diagram)<!--[Carpeta de Fuentes](../) asi se puede hacer un enlace con una carpeta -->
    -

- 6.[ Codes](src/)
    -
    - [Open challenge](src/Open%20challenge.ino)
    - [Closed challenge](src/Closed%20challenge.ino)

- 7.[ Flowcharts](src/README.md#flowcharts)
    -
    - [Open challenge](src/README.md#open-challenge)
    - [Closed challenge](src/README.md#close-challenge)
    
- 8.[ Vehicle photos](v-photos/README.md)
    -

- 9.[ Performance videos](video/README.md)
    -
    - [Open challenge](video/README.md#open-challenge)
    - [Closed challenge](video/README.md#closed-challenge)

- 10.[ Team photos](t-photos/)
    -
    - [Official photo](t-photos/README.md#official-photo)
    - [Funny photo](t-photos/README.md#funny-photo)

- 11.[ Annex](other/)
    -
    - [First robot](other/First%20robot/)
    - [Second robot](other/Scond%20robot/)
    - [Thirt robot - Final](other/Third%20robot%20-%20Final/)


<!---------------------------------------------espanol-------------------------------------------------------- -->

# Introduction 
Spark is a team from the Salto Angel School participating in the World Robot Olympiad 2024 (WRO) in the future engineers’ category. This team is comprised of three students: Victoria Saez, the new Captain who is in charge of the programming and electronic parts; Sebastian Salina, who is responsible for the designing and mechanical parts of the robot; and finally, Rosa Wong, who attends the documentation and is also a university student at Rafael Urdaneta University.

The World Robot Olympiad (WRO) is an event where young people from different parts of the world meet to show their knowledge and abilities in diverse categories. The future engineers' category is based on the creation of an autonomous robot capable of driving around a game field that measures 3m x 3m using obstacles in it or without them. Every year, the rules and challenges partially change and every four or five years represents a different challenge.

This report explains the components and parts of the robot built and designed by Team Spark to afford all the competition`s challenges.

# Components

## Materials List

| **Pieces**                              | **Quantity** |
|-----------------------------------------|--------------|
| Arduino Mega                            | 1            |
| Pixy2 Cam                               | 1            |
| Ultrasonic Sensor (HC-SR04)             | 3            |
| Bridge H L298N                          | 1            |
| Pull-up Button                          | 1            |
| Motor GA37-520 of 320rpm                | 1            |
| Servo Motor MG90S                       | 1            |
| NI-MH Battery 12V 3000mAh               | 1            |
| LM2596 HW-411 Step Down                 | 2            |
| Switch button                           | 1            |

## Arduino mega

[![A000067-00-front-934x700.jpg](https://i.postimg.cc/Tw4CcFkv/A000067-00-front-934x700.jpg)](https://postimg.cc/2bnQkcR0)

| | | | |
| --- | --- | --- | --- |
| **Microcontroller** | ATmega2560 | **Operating Voltage**    | 5V       |
| **Input Voltage**   | 7-12V      | **Digital I/O Pins**     | 54 (15 provide PWM output)    |
| **Analog Input Pins**   | 16     | **Flash Memory**         | 256 KB            |
| **SRAM**                | 8 KB   | **EEPROM**               | 4 KB              |

A unique controller, the Arduino Mega, was chosen to
manage all the robot`s functions. This model is distinguished
by its great prosecution capability inside the Arduino family,
letting it sort out complete code and manage a wide range of
components thanks to its large quantity of digital pins,
analogic, and PWM.

## Servo motor MG90S
[![mg90s-micro-servomotor-kompatibel-mit-arduino-154289-2.png](https://i.postimg.cc/LsdqjTkQ/mg90s-micro-servomotor-kompatibel-mit-arduino-154289-2.png)](https://postimg.cc/4Y5NgpSp) [![1-2-4-5-10-20pcs-MG90-S-Micro-Servo-Motor-Metal-Gear-Analog-RC-Servomotor-180-jpg.jpg](https://i.postimg.cc/05tbVxdw/1-2-4-5-10-20pcs-MG90-S-Micro-Servo-Motor-Metal-Gear-Analog-RC-Servomotor-180-jpg.jpg)](https://postimg.cc/SjMSKBPS)

| | | | |
| --- | --- | --- | --- |
| **Weight**    | 13.4g  | **Operating speed**  | 0.1 s/60 degree, 0.08 s/60 degree (6V) 
| **Dimension** | 22.5 x 12 x 35.5 mm        | **Operating voltage**| 4.8 V - 6.0 V    |
| **Stall torque**  | 1.8 kgf·cm (4.8V), 2.2 kgf·cm (6V) | **Dead band width**   | 5 μs |

It uses the servomotor MG90S in the turning system to get
precise turns. This model is smaller, lighter, and offers higher
torque than the SG90, which allows it to move the wheels with
the necessary strength for shunting, mainly in obstacle
evasion.

### Code

To make easier its use, we employed the library `<Servo.h>` and created a variable
called `SERVOPIN` where we put the pin that is in use, in this case, it is the 9.

``` ino
#include <Servo.h> // Library to control servomotors.


#define SERVOPIN 9 // Define the pin for the servomotor.


#define TURNGRADE 30 // Define the turning grades for the correct direction
#define CENTERVALUE 90 // Center value for the servomotor (straight).
#define MAXLEFT 156 // Top value towards the left for the servomotor.
#define MAXRIGHT 5 // Top value towards the right for the servomotor.



Servo direction; // Create an instance of the servomotor.


void center() {
  if (DEBUG) Serial3.println("Center"); // Print "Center" if DEBUG is active.
  direction.write(CENTERVALUE); // Place the servomotor in the central value (90 degrees).
}


void turnLeft() {
  if (DEBUG) Serial3.println("Left"); // Print "Left" if DEBUG is active.
  direction.write(MAXLEFT); // Turn the servomotor at the top to the left.
  direction.write(MAXLEFT); // Repeat the writing with precision.
  setMove(500, 1, TURNSPEED); // Perform the movement with 500 ticks in direction1 (forward) to TURNSPEED.
  center(); // Center the servomotor after the turning.
  stop(); // Stop the movement.
}


void turnRight() {
  if (DEBUG) Serial3.println("Right"); // Print "right" if DEBUG is active.
  direction.write(MAXRIGHT); // Turn the servomotor at the top to the right
  direction.write(MAXRIGHT); // Repeat the writing with precision.
  setMove(760, 1, MAXSPEED); // Perform the movement with 760 ticks in direction1 (forward) to MAXSPEED.
  center(); // Center the servomotor after the turning.
  stop(); // Stop the movement.
}


void turnGrade(int grade) {
  if (DEBUG) Serial3.println("turnGrade " + String(CENTERVALUE + grade)); // Print the calculated turn angle.
  direction.write(CENTERVALUE + grade); // Adjust the servomotor angle according to the specify grade.
}

```

This servomotor is used the whole time to perform the turnings that are required
during the challenges.

## Bridge H (L293N)
[![Conexiones-L298-N.png](https://i.postimg.cc/rwJKcztq/Conexiones-L298-N.png)](https://postimg.cc/zyyJFzjQ)

| | | | |
| --- | --- | --- | --- |
| **Number of outputs**  | 4  | **Output current per channel**  | 2A - 3A peak|
| **Working voltage**    | 5V to 35V  | **Current consumption (logical)** | 36mA  |

The L293N acts as an entrepreneur between the Arduino Mega and the motor, translating the instructions into movements. Its power to manage high voltages lets us use motors with nominal voltage, without limiting the controller voltage, which increases the efficiency and potential of the robot.

## Pixy2 cam
[![pixy-v21-camera-sensor.jpg](https://i.postimg.cc/N04gL7dH/pixy-v21-camera-sensor.jpg)](https://postimg.cc/Jysw2JVr)

| | | | |
| --- | --- | --- | --- |
| **Procesador**                  | NXP LPC4330, 204 MHz, dual core | **Imagen sensor**              | Aptina MT9M114, 1296×976       |
| **Field of view horizontaL**        | 60°                          | **Field of view vertical**            | 40°                             |
| **Operating current**                     | 140 mA                       | **Operating voltage**                  | USB (5V) o 6V a 10V            |
| **RAM**                         | 264Kb                        | **Flash**                         | 2Mb                             |
| **Buses**                       | UART serial, SPI, I2C, USB, digital, analog | **Light**               | 20 lumens approx                |

The camera is capable of detecting seven colors
simultaneously. It is equipped with an internal processor,
which lets us explore just the necessary information for the
Arduino or other controllers due to the variety of connections
that allow it. Moreover, it counts with an obturation speed of 60 fps, an excellent gap, and white adjustable LED lights;
thus, the continuous movement and the lower light don’t
represent a huge problem.

### Code

#### Open challenge
With the camera connected to the pins I2C of the Arduino Mega add the library
`<Pixy2.h>`. we create a variable called `firstColorDetected` to save the first color that
is detected and apart a variable called `colorDetected` to indicate if any color is
detected. To start the camera pixy2 uses a `pixi.init()`, in the light of an additional
light is needed for the perception of the colors we will implement the
`pixy.setLamp(1, 0)` command, this will activate the camera LED lights.

``` ino
#include <Pixy2.h> 


Pixy2 pixy; // Create an object of the Pixy2 camera.


byte firstColorDetected = NONE; // Variable to save the first detected color.
bool colorDetected = false; // Indicate if a color is detected.


void flash(byte time) { //Function for indicating that the start button can be pressed 
  for (byte i = 0; i < time; i++) {
    pixy.setLamp(1, 0); // turn on the Pixy2 camera LED. 
    delay(200); // Delay 200 ms
    pixy.setLamp(0, 0); // turn off the Pixy2 camera LED
    delay(200); // Delay 200 ms
  }
}
In the case of the first color detected is blue which is assigned previously as 1, calls the function turnLeft.


           if (pixy.ccc.numBlocks > 0) { // if the Pixy detects an object
      stop(); // Stop the robot
      center(); // Center the servomotor.
      switch (pixy.ccc.blocks[0].m_signature) { // Depending on the signature of the object.
        case BLUE: // If detect a blue object.
          Serial3.println("BLUE"); // Print "BLUE" in the serial monitor.   
          if (!colorDetected) { // if any color has not been detected previously.
            colorDetected = true; // Mark color detected
            firstColorDetected = BLUE; // Stablish the color detected as blue.
          }
          if (colorDetected && firstColorDetected == ORANGE) { // if orange has already been detected.
            Go to orange; // Jump to the orange color section.
          }
Blue: // Tag the blue object.
          center(); // Center the servomotor
          readDistance(currentPosition); // Update 'currentPosition'
          readDistance(lastPosition); // Update 'lastPosition'
          while ((currentPosition[3] < 100)) { // While the left distance is less than 100 cm
            followLine(SPEED2); // Follow the wall to 'SPEED2'.
            readDistance(currentPosition); // Update 'currentPosition'
          }
         turnLeft(); // Turn to the left
          moveCM(40, 1, MAXSPEED); // Move the robot 40 cm forward to high-speed. 
          do {
            followLine(MAXSPEED); // Follow the inner walls to top-speed
            readDistance(currentPosition); // Update 'currentPosition'
          } while (currentPosition[3] > 100); // Repeat while the left distance is more than 100 cm
          break;

```

In the case of the first color detected being orange, assign it as 2 in the camera, and call the turnRight function.

``` ino
 case ORANGE: // If detect an orange object.
          Serial3.println("ORANGE"); // Print "ORANGE" in the serial monitor
          if (!colorDetected) { // If any color has not been detected previously.
            colorDetected = true; // Mark color detected
            firstColorDetected = ORANGE; // Stablish the first color detected as orange 
          }
          if (colorDetected && firstColorDetected == BLUE) { // If blue has already been detected.
            go to blue; // Jump to the blue color section. 
          }
Orange: // Tag the orange object.
          center(); // Center the servomotor.
          readDistance(currentPosition); // Update 'currentPosition'
          readDistance(lastPosition); // Update 'lastPosition'
          while ((currentPosition[2] < 100)) { // while the right distance is less than 100 cm
            followLine(SPEED2); // Follow the inner wall to 'SPEED2'
            readDistance(currentPosition); // Update 'currentPosition'
          }
          turnRight(); // Turn to the right.
          moveCM(40, 1, MAXSPEED); // Move the robot 40 cm forward to high-speed.
          do {
            followLine(MAXSPEED); // Follow the inner Wall to top-speed. 
            readDistance(currentPosition); // Update 'currentPosition'
          } while (currentPosition[2] > 100); // Repeat while the right distance is more than 100 cm
          break;
      }

```


## Ultrasonic sensor (HC-S04)
[![Medidas-de-sensor-ultrasonido-HC-SR04.jpg](https://i.postimg.cc/mgdP54Gq/Medidas-de-sensor-ultrasonido-HC-SR04.jpg)](https://postimg.cc/FYLFY2zg)

| | | | |
| --- | --- | --- | --- |
| **Operating voltage** | 5 V  | **Operating current** | 15mA     |
| **Minimum range**     | 2cm  | **Max. distance**  | 4m   |
| **Measuring angle**   | 15°  | **Output Echo signal**  | 40Hz     |
| **Input Trigger signal**  | Input 10μs TTL | **Pulso Echo** | Output 100-25000 μs TTL |

The ultrasonic sensors are essential in our robot because
let's calculate distances by using ultrasonic vibes. These
sensors own a transmitter that sends the vibe and a receiver
to detect the echo of the reflexive vibes which lets us
determine the distance by the time it takes to come back with
a maximum detection of 4 meters, these sensors are ideal for
the game field size, which measures 3m x 3m. thanks to
them, the robot has a higher perception of its position and
anticipate possible coalitions effectively.

### Code

 To facilitate the frequency conversion of the Vibes reception in cms, use the library `<Ultrasonic.h>` and subsequently, define each sensor pin with Ultrasonic and the name given to the sensor followed by the (Trig, Echo).
``` ino
#include <Ultrasonic.h> // Library to manage ultrasonic sensors.
#define USTRIGHT A13 // Define the signal pin for the right ultrasonic sensor.
#define USERIGHT A12 // Define the echo pin for the right ultrasonic sensor.
#define USTLEFT A14 // Define the signal pin for the left ultrasonic sensor.
#define USELEFT A15 // Define the echo pin for the left ultrasonic sensor.


Ultrasonic USRight(USTRIGHT, USERIGHT); // Instance for the right ultrasonic sensor.
Ultrasonic USLeft(USTLEFT, USELEFT); // Instance to for left ultrasonic sensor.


int firstPosition[4]; // Array to upload the first position detected. 
int lastPosition[4]; // Array to upload the last position detected.
int currentPosition[4]; // Array to upload the current position detected.
int tempPosition[4]; // Array to upload the temporary position detected.


void readDistance(unsigned int* array) {
  array[2] = USRight.read(); // Upload the distance measure by the right ultrasonic sensor.
  array[3] = USLeft.read(); // Upload the distance measure by the left ultrasonic sensor.
}
```

Considering the values of the firstPosition, lastPosition, currentPosition and tempPosition, the robot does an angle correction to evade collisions both with the outer and inner wall staying centered.

``` ino
if (DEBUG2) printDistance(currentPosition); // Print the distances if DEBUG2 is active.
  if (tempPosition[2] < tempPosition[3]) { // If the right distance is less than the left position
    grade = map(currentPosition[2], tempPosition[2] - LMIN, tempPosition[2] + LMIN, TURNGRADE, -TURNGRADE); // Map the angle correction to the right.
    if (DEBUG) Serial3.println("1 Actual: " + String(currentPosition[2]) + ". Inicial: " + String(tempPosition[2])); // Depuration
  } else { // If the left distance is less or equal to the right 
    grade = map(currentPosition[3], tempPosition[3] - LMIN, tempPosition[3] + LMIN, -TURNGRADE, TURNGRADE); // Map the angle correction to the left.
    if (DEBUG) Serial3.println("2 Actual: " + String(currentPosition[3]) + ". Inicial: " + String(tempPosition[3])); // Depuration
  }
 if (DEBUG) Serial3.println("Grades : " + String(grade)); // Print the angle calculated correction.
  if (grade > TURNGRADE) { // Limit the higher turning angle.
    grade = TURNGRADE;
  } else if (grade < TURNGRADE * -1) { // Limit the lower turning angle
    grade = -TURNGRADE;
  }

```

## Battery and charger
[![REV-31-1302-12-VSlim-Battery-New-FINAL-87390.jpg](https://i.postimg.cc/L80sVVtF/REV-31-1302-12-VSlim-Battery-New-FINAL-87390.jpg)](https://postimg.cc/pmzxWzcs) [![AA-Ni-Mh-Battery-Charger-noflag-08282.jpg](https://i.postimg.cc/CMHL20GN/AA-Ni-Mh-Battery-Charger-noflag-08282.jpg)](https://postimg.cc/qgqTt9Zh)

| | | | |
| --- | --- | --- | --- |
| **Voltage**  | 12V  | **Capacity**           | 3000mAh  |
| **Weight**   | 567g | **Connector**          | XT30         |
| **Replaceable Fuse** | 20A ATM |             |              |

After an extensive period of using NI-MH Batteries, we
have chosen a 12V battery and 3000 mAh Rev Robotics. This
new battery offers significant advantages.

Firstly, it gives better autonomy, letting the robot work
through longer periods without the charging needs. Its 12
voltages are enough to completely provide the energetic
needs of the robot, ensuring a favorable performance.

In addition, the battery includes a versatile charger with
two charging modes; a lower one (0.9 A) and a faster one (1.8
A), allowing flexibility in the time charging. Also, it is allowed
on international trips by airplane because it is made with
nickel which is safe for its transport.

Lastly, it is a prefabricated battery and offers a superior
level of security and efficiency as compared to the settings of
individual batteries.


## Motor GA37-520
[![51-J-66l-USg-L.jpg](https://i.postimg.cc/dV0214sb/51-J-66l-USg-L.jpg)](https://postimg.cc/xcWJpGb5) 

| | | | |
| --- | --- | --- | --- |
| **Nominal voltage**  | 12V | **Rated power** | 8,3W    |
| **Rated current**| 1,5A | **Reduction**    | 18.8:1  |
| **No-load speed** | 320rpm | **Rated speed**| 224rpm  |
| **No-load current** | ≥120mA | **Rated current**  | ≥400mA |
| **Rated torque** | 3.1Kg.cm  | **Par nominal**  | 0.67Kg.cm    |

From a robotic kit, we have obtained two motors. Each one
to 12V, they are capable of going at a nominal speed of
250rpm and a 3,5 kg torque. Therefore using just a 12V motor
is enough to give it the speed, control, and attraction needed,
as well as greater management thanks to its encoder which
Let us have better precision in the turns that the robot should
do in the game field.

### Code

#### Open challenge

Firstly, we defined the variables that go in the H bridge and at
the same time connected the motor that is called `IN1` and IN2,
equally, we defined 3 different speeds (`MAXSPEED`,
`TURNSSPEED`, `SPEED2`) that could be inside of a range
between 255 and 0.

``` ino
#define IN1 4 // Define IN1 to the motor control pin.
#define IN2 5 // Define IN2 to the motor control pin.


#define MAXSPEED 250 // Top-Speed of the motor.
#define TURNSPEED 180 // Turnings Speed.
#define SPEED2 140 // Secundary Speed.

void stop() {
  if (DEBUG) Serial3.println("Stop"); // Print "Stop" in the serial monitor if the mode depuration is active.
  digitalWrite(IN1, LOW); // Turn off the motor in the IN1 direction.
  digitalWrite(IN2, LOW); // Turn off the motor in the IN2 direction.
}

void forward(byte speed) {
  if (DEBUG2) Serial3.println("Forward " + String(speed)); // Print "Forward" and the speed if DEBUG2 is active.
  analogWrite(IN1, speed); // Assign the speed in IN1 to go forward.
  digitalWrite(IN2, LOW); 
}
void backward(byte speed) {
  if (DEBUG) Serial3.println("Backward " + String(speed)); // Print "Backward" and the speed if DEBUG is active.
  digitalWrite(IN1, LOW); // turn off IN1 to return
  analogWrite(IN2, speed);

}
```

## LM2596 HW-411 Step down
[![XL6009-Module-Pinout.jpg](https://i.postimg.cc/Vv6fMrJk/XL6009-Module-Pinout.jpg)](https://postimg.cc/LYcFFskc)

| | | | |
| --- | --- | --- | --- |
| **Input**                   | 4.5-40VDC                   | **Output**                   | 1.5-35VDC                     |
| **Rated Current**           | 2A (Surge 3A)               | **Regulation**               | 0.5%                          |

To protect the robot`s electronic components from
possible damages caused by higher voltages that are not
required, for this, we have incorporated a voltage regulator in
our system. This device assures a constant and stable energy
supply, which prevents failures in the components, and also
increases the stability and movement of the robot in its
performance.

# Robot floors
The robot is made up of four floors
listed from button to top.

## First and Second Floor
Both floors are connected themselves including the Steering system and the Driving
system.

### Front Axle
The front axle of our robot is located in a steering system boosted by a servo
(Mg90s). this system counts with two wings, a superior one and a lower one, that
allow an appropriate Caster, between the wings we can find the pivots that count
with two bearings that facilitate the free movement of the wheels and avoid an
incorrect Camber, both pivots include a steering arm which is connected through
the Tie Rod.

Movement transmission is carried out by the servo connected to the Tie Rod. This
is connected at the same time with the Steering Arm of the right Wheel to make the
movement transmission we use the Steering Linkage that is connected to the left
steering arm. The Tie Rod has a screw thread which allows an adjustable Toe. The
tire rod and the steering linkage count with the rod end at the ends give us a wide
work range. making it much easier to integrate the servo with the steering system.

### Back Alxe
In the back of the robot, we can find the driven system (the wheels and the motor).
This motor transmits the movement through a chain and two sprockets a small one
with 10 teeth that is connected to the shaft and the bigger one with 15 teeth which
is connected to the motor. in this setting the velocity increases x1.5, resulting in a
rated speed of 540RPM. also, the ring for the wheels was modified to fit with a
5mm hexagonal shaft, which makes the movement transmission and the assembly
easier.

## Third Floor.
On the third floor, the robot has different electronic components that are essential. It is located the battery which provides the needed energy to the proper function of the robot. As well as the camera that let get images and detect obstacles. Besides, the ultrasonic sensors and the laser sensor are included, and each one sets in its respective support that helps with its positioning and functioning.


## Fourth floor.
On the fourth robot`s floor is located the controller and near it the shield, which organizes all the electric connections, including the feeding. The connections are soldered, avoiding accidental disconnection, and minimizing the errors and accidents during the robot's manipulation.
