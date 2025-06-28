#include <Servo.h>

// --- Pines de Hardware ---
#define ENA 3             // Pin PWM para el control de velocidad del motor (ej. 3, 5, 6, 9, 10, 11 en Arduino UNO)
#define IN1 4             // Pin de control de dirección del motor 1
#define IN2 5             // Pin de control de dirección del motor 2
#define PIN_SERVO 6       // Pin para el servo de dirección

// --- Parámetros de Calibración del Servo ---
#define POS_CENTRO 94      // Ángulo del servo para ruedas rectas (ajustar si es necesario)
#define ANGULO_GIRO 30     // Grados a sumar/restar de POS_CENTRO para giros
#define ANGULO_DERECHA (POS_CENTRO + ANGULO_GIRO + 18) // Ángulo para girar a la derecha
#define ANGULO_IZQUIERDA (POS_CENTRO - ANGULO_GIRO - 8) // Ángulo para girar a la izquierda

// --- Parámetros de Velocidad del Motor de Tracción ---
#define VELOCIDAD_MINIMA 80   // Velocidad mínima del motor (0-255 PWM)
#define VELOCIDAD_MEDIA 150  // Velocidad media del motor
#define VELOCIDAD_MAXIMA 220 // Velocidad máxima del motor

// --- Objeto Servo ---
Servo miServo; // Objeto servo

// --- Prototipos de Funciones ---
void motorDelante(int velocidad);
void motorParar();
void setSteeringAngle(int angle);

void setup() {
  // Inicializar comunicación serial para depuración
  Serial.begin(9600);
  Serial.println("Iniciando prueba de servo y motor de traccion...");
  Serial.println("----------------------------------------");

  // Configurar pines del motor
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  motorParar(); // Asegurarse de que el motor esté parado al inicio

  // Adjuntar el servo
  miServo.attach(PIN_SERVO);
  setSteeringAngle(POS_CENTRO); // Poner las ruedas rectas al inicio
  delay(1000); // Pequeña pausa para que el servo se estabilice
}

void loop() {
  // --- Prueba de Dirección (Servo) ---

  Serial.println("Estableciendo direccion: Centro (ruedas rectas)");
  setSteeringAngle(POS_CENTRO);
  motorDelante(VELOCIDAD_MEDIA); // Moverse mientras prueba la dirección
  delay(2000); // Mantener la posición por 2 segundos

  Serial.println("Estableciendo direccion: Izquierda");
  setSteeringAngle(ANGULO_IZQUIERDA);
  delay(2000); // Mantener la posición por 2 segundos

  Serial.println("Estableciendo direccion: Derecha");
  setSteeringAngle(ANGULO_DERECHA);
  delay(2000); // Mantener la posición por 2 segundos

  // Volver al centro para la prueba de velocidad
  Serial.println("Volviendo a direccion: Centro");
  setSteeringAngle(POS_CENTRO);
  delay(1000);

  // --- Prueba de Velocidad (Motor de Tracción) ---

  Serial.println("Probando velocidad: Minima");
  motorDelante(VELOCIDAD_MINIMA);
  delay(3000); // Moverse a velocidad mínima por 3 segundos

  Serial.println("Probando velocidad: Media");
  motorDelante(VELOCIDAD_MEDIA);
  delay(3000); // Moverse a velocidad media por 3 segundos

  Serial.println("Probando velocidad: Maxima");
  motorDelante(VELOCIDAD_MAXIMA);
  delay(3000); // Moverse a velocidad máxima por 3 segundos

  Serial.println("Deteniendo motor");
  motorParar(); // Detener el motor
  delay(3000); // Pausa de 3 segundos antes de repetir el ciclo
}

/**
 * @brief Mueve el motor de tracción hacia adelante a una velocidad dada.
 * @param velocidad Valor PWM (0-255) para la velocidad del motor.
 */
void motorDelante(int velocidad) {
  digitalWrite(IN1, HIGH); // Configura la dirección del motor
  digitalWrite(IN2, LOW);
  analogWrite(ENA, constrain(velocidad, 0, 255)); // Aplica la velocidad PWM, asegurando que esté en el rango 0-255
}

/**
 * @brief Detiene completamente el motor de tracción.
 */
void motorParar() {
  digitalWrite(IN1, LOW); // Quita la señal de dirección
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);    // Establece la velocidad a 0 (detiene el motor)
}

/**
 * @brief Establece el ángulo del servomotor de dirección.
 * @param angle El ángulo deseado en grados (0-180).
 */
void setSteeringAngle(int angle) {
  // miServo.write() envía un pulso para mover el servo a un ángulo específico.
  // constrain() asegura que el ángulo esté dentro de un rango seguro (0 a 180 grados).
  miServo.write(constrain(angle, 0, 180));
}
