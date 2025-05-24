 🚀 Proyecto de Arduino - [Ledma Tech_MK1!]

## 📖 Introducción
carrito autonomo controladom por arduino mega que tiene que realizar varios funciones complejas como la deteccion de colores y esquivar 
obstaculos ademas para pasar las pruebas es un prototipo hecho para la clasificacionde la WRO en el que se realizaran
diferentes pruebas para poder clasificar 
este carro se lleva cabo en una fundacion pra poder aprender y mejorar nuestros conocimientos de robotica y compresion der la misma 


## 🔧 Materiales y Componentes
Lista detallada de los componentes utilizados:
- **Chasis de referencias de modelado 3D**
- **Motor DC 12V 1A**
- **Servo motor Rev robotics REV-41-1097**
- **Arduino Mega 2640**
- **Arduino cam OV2076**
- **Batería de litio 18650 (7 UND)**
- **Puente H L293D**
- **Módulo Memoria Micro SD Adaptador Arduino Shield SPI**
- **Sensor de ultrasonido de distancia Arduino HC-SR04 (3 UND)**
- **Controlador de velocidad PWM motores DC 4.5V-35V 5A 90W**


**Chasis de referencias de modelado 3D**
  https://cults3d.com/es/modelo-3d/juegos/voyager-mk1-3d-printed-1-10-offroad-rc-chassis

![Imagen1](https://github.com/user-attachments/assets/f2a3010a-b9ae-4fb4-b983-11a7d8c863ec)

**archivos de impresion de chasis fotos de la impresion**

![Imagen de WhatsApp 2025-05-20 a las 09 24 45_103739bd](https://github.com/user-attachments/assets/cd348690-413e-4cba-aaee-bd8124d99b3c)

![Imagen de WhatsApp 2025-05-17 a las 11 36 43_54ba59b0](https://github.com/user-attachments/assets/eca676e0-80c4-478e-9d03-1438da985e14)


**Servo motor Rev robotics REV-41-1097**
https://www.revrobotics.com/rev-41-1097/

![servo motor rev](https://github.com/user-attachments/assets/b42de167-ad23-4dbd-b86a-e949ff78e697)


**Arduino Mega 2060**

![descarga](https://github.com/user-attachments/assets/63b1213b-3a6c-4e2e-802f-76436bbadba5)

**Arduino cam OV2640**

![descarga (1)](https://github.com/user-attachments/assets/12fd795a-7a83-4307-b64f-ecd3a42b0802)

**Batería de litio 18650**

![batrerias de litio](https://github.com/user-attachments/assets/74d2a0b1-1fb8-4fd4-a26b-50ba482d4100)

**Puente H L293D**

![puente h](https://github.com/user-attachments/assets/74b50415-35cb-41c1-8fc4-b2f70ee2c634)

**Módulo Memoria Micro SD Adaptador Arduino Shield SPI**

![adaptador sd](https://github.com/user-attachments/assets/9da28bd9-b0fc-495d-b1a7-11de1e568101)

 **Sensor de ultrasonido de distancia Arduino HC-SR04**

![ultrasinico](https://github.com/user-attachments/assets/902c8c8b-0308-4dfd-9ffe-1c58e8a6bf10)

**Controlador de velocidad PWM motores DC 4.5V-35V 5A 90W**

![regulador](https://github.com/user-attachments/assets/452a675d-c2d1-484c-8777-c530555d7ff3)


## 🔨 Instalación y Montaje
Pasos detallados para ensamblar y conectar los componentes:
1. Ensamblar los motores y sensores en el chasis.
2. Conectar correctamente los módulos a la placa Arduino Mega 2060.
3. Configurar la cámara Arduino OV2076.
4. Cargar el código en la placa y probar el funcionamiento.
   
5. codigo del carrito (avance del codigoabsuelto a cambios)

#define enA 10//Enable1 L298 Pin enA 
#define in1 9 //Motor1  L298 Pin in1 
#define in2 8 //Motor1  L298 Pin in1 
#define in3 7 //Motor2  L298 Pin in1 
#define in4 6 //Motor2  L298 Pin in1 
#define enB 5 //Enable2 L298 Pin enB 

#define L_S A0 //ir sensor Left
#define R_S A1 //ir sensor Right

#define echo A2    //Echo pin
#define trigger A3 //Trigger pin

#define servo A5

int Set=15;
int distance_L, distance_F, distance_R; 

void servoPulse(int pin, int angle);
long Ultrasonic_read();
void Check_side();
void forword();
void turnRight();
void turnLeft();
void stopMovement();
void stop();

void setup(){ // put your setup code here, to run once

Serial.begin(9600); // start serial communication at 9600bps

pinMode(R_S, INPUT); // declare if sensor as input  
pinMode(L_S, INPUT); // declare ir sensor as input

pinMode(echo, INPUT );// declare ultrasonic sensor Echo pin as input
pinMode(trigger, OUTPUT); // declare ultrasonic sensor Trigger pin as Output  

pinMode(enA, OUTPUT); // declare as output for L298 Pin enA 
pinMode(in1, OUTPUT); // declare as output for L298 Pin in1 
pinMode(in2, OUTPUT); // declare as output for L298 Pin in2 
pinMode(in3, OUTPUT); // declare as output for L298 Pin in3   
pinMode(in4, OUTPUT); // declare as output for L298 Pin in4 
pinMode(enB, OUTPUT); // declare as output for L298 Pin enB 

analogWrite(enA, 200); // Write The Duty Cycle 0 to 255 Enable Pin A for Motor1 Speed 
analogWrite(enB, 200); // Write The Duty Cycle 0 to 255 Enable Pin B for Motor2 Speed 

pinMode(servo, OUTPUT);

 for (int angle = 45; angle <= 90; angle += 5)  {
   servoPulse(servo, angle);  }
 for (int angle = 90; angle >= 0; angle -= 5)  {
   servoPulse(servo, angle);  }
 for (int angle = 0; angle <= 45; angle += 5)  {
   servoPulse(servo, angle);  }

distance_F = Ultrasonic_read();

delay(500);
}


void loop(){  
//==============================================
//     Line Follower and Obstacle Avoiding
//==============================================  

distance_F = Ultrasonic_read();
    Serial.print("D F="); Serial.println(distance_F);

    stopMovement(); // Detener el movimiento

//if Right Sensor and Left Sensor are at White color then it will call forword function
 if((digitalRead(R_S) == 0)&&(digitalRead(L_S) == 0)){
  if(distance_F > Set){forword();}
                  else{Check_side();}  
 }  
 
//if Right Sensor is Black and Left Sensor is White then it will call turn Right function
else if((digitalRead(R_S) == 1)&&(digitalRead(L_S) == 0)){turnRight();}  

//if Right Sensor is White and Left Sensor is Black then it will call turn Left function
else if((digitalRead(R_S) == 0)&&(digitalRead(L_S) == 1)){turnLeft();} 
    
delay(10);
}

  
void servoPulse (int pin, int angle) 
int pwm = (angle*11) + 500;      // Convert angle to microseconds
 digitalWrite(pin, HIGH);
 delayMicroseconds(pwm);
 digitalWrite(pin, LOW);
 delay(50); // Refresh cycle of servo
}


//**********************Ultrasonic_read****************************
long Ultrasonic_read()
  digitalWrite(trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  long time = pulseIn (echo, HIGH);
  return time / 29 / 2;
}

void compareDistance()
    if(distance_L > distance_R){
  turnLeft();
  delay(500);
  forword();
  delay(600);
  turnRight();
  delay(500);
  forword();
  delay(600);
  turnRight();
  delay(400);
  }
  else{
  turnRight();
  delay(500);
  forword();
  delay(600);
  turnLeft();
  delay(500);
  forword();
  delay(600);  
  turnLeft();
  delay(400);
  }
}

void Check_side()
    Stop();
    delay(100);
 for (int angle = 45; angle <= 90; angle += 5)  {
   servoPulse(servo, angle);  }
    delay(300);
    distance_R = Ultrasonic_read();
    Serial.print("D R=");Serial.println(distance_R);
    delay(100);
  for (int angle = 90; angle >= 0; angle -= 5)  {
   servoPulse(servo, angle);  }
    delay(500);
    distance_L = Ultrasonic_read();
    Serial.print("D L=");Serial.println(distance_L);
    delay(100);
 for (int angle = 0; angle <= 45; angle += 5)  {
   servoPulse(servo, angle);  }
    delay(300);
    compareDistance();
}

void forword()  //forword
digitalWrite(in1, LOW); //Left Motor backword Pin 
digitalWrite(in2, HIGH); //Left Motor forword Pin 
digitalWrite(in3, HIGH); //Right Motor forword Pin 
digitalWrite(in4, LOW); //Right Motor backword Pin 
}

void backword() //backword
digitalWrite(in1, HIGH); //Left Motor backword Pin 
digitalWrite(in2, LOW); //Left Motor forword Pin 
digitalWrite(in3, LOW); //Right Motor forword Pin 
digitalWrite(in4, HIGH); //Right Motor backword Pin 
}

void turnRight() //turnRight
digitalWrite(in1, LOW); //Left Motor backword Pin 
digitalWrite(in2, HIGH); //Left Motor forword Pin 
digitalWrite(in3, LOW); //Right Motor forword Pin 
digitalWrite(in4, HIGH); //Right Motor backword Pin 
}

void turnLeft() //turnLeft
digitalWrite(in1, HIGH); //Left Motor backword Pin 
digitalWrite(in2, LOW); //Left Motor forword Pin 
digitalWrite(in3, HIGH); //Right Motor forword Pin 
digitalWrite(in4, LOW); //Right Motor backword Pin 
}

void stop() //stop
digitalWrite(in1, LOW); //Left Motor backword Pin 
digitalWrite(in2, LOW); //Left Motor forword Pin 
digitalWrite(in3, LOW); //Right Motor forword Pin 
digitalWrite(in4, LOW); //Right Motor backword Pin 
}

## 📷 Imágenes del Proyecto





