#include <Arduino.h> // No es estrictamente necesario en IDE de Arduino, pero buena práctica

// --- Pines de Hardware para los Sensores Ultrasonicos ---
#define TRIG_FRONTAL 7    // Pin TRIG del sensor ultrasónico frontal
#define ECHO_FRONTAL 8    // Pin ECHO del sensor ultrasónico frontal
#define TRIG_DERECHO 9    // Pin TRIG del sensor ultrasónico derecho
#define ECHO_DERECHO 10   // Pin ECHO del sensor ultrasónico derecho
#define TRIG_IZQUIERDO 11 // Pin TRIG del sensor ultrasónico izquierdo
#define ECHO_IZQUIERDO 12 // Pin ECHO del sensor ultrasónico izquierdo

// --- Variables para almacenar las distancias ---
float frontal_dist;
float derecha_dist;
float izquierda_dist;

// --- Prototipo de la función para leer el sensor ---
float leerUltrasonico(uint8_t trigPin, uint8_t echoPin);

void setup() {
  // Inicializar la comunicación serial a 9600 baudios
  // Asegúrate de que el monitor serial en tu IDE de Arduino esté configurado a la misma velocidad.
  Serial.begin(9600); 
  Serial.println("Iniciando lectura de sensores ultrasónicos...");
  Serial.println("----------------------------------------");

  // Configurar los pines TRIG como salida y ECHO como entrada para cada sensor
  pinMode(TRIG_FRONTAL, OUTPUT);
  pinMode(ECHO_FRONTAL, INPUT);

  pinMode(TRIG_DERECHO, OUTPUT);
  pinMode(ECHO_DERECHO, INPUT);

  pinMode(TRIG_IZQUIERDO, OUTPUT);
  pinMode(ECHO_IZQUIERDO, INPUT);
}

void loop() {
  // Leer la distancia de cada sensor
  frontal_dist = leerUltrasonico(TRIG_FRONTAL, ECHO_FRONTAL);
  derecha_dist = leerUltrasonico(TRIG_DERECHO, ECHO_DERECHO);
  izquierda_dist = leerUltrasonico(TRIG_IZQUIERDO, ECHO_IZQUIERDO);

  // Imprimir las distancias en el monitor serial
  Serial.print("Frontal: ");
  Serial.print(frontal_dist);
  Serial.print(" cm | ");

  Serial.print("Derecha: ");
  Serial.print(derecha_dist);
  Serial.print(" cm | ");

  Serial.print("Izquierda: ");
  Serial.print(izquierda_dist);
  Serial.println(" cm");

  // Pequeña pausa para permitir lecturas estables y no saturar el monitor serial
  delay(100); 
}

/**
 * @brief Lee la distancia de un sensor ultrasónico HC-SR04.
 * * @param trigPin El pin TRIG del sensor.
 * @param echoPin El pin ECHO del sensor.
 * @return La distancia medida en centímetros. Retorna 400 cm si la lectura es inconsistente (0 o > 400).
 */
float leerUltrasonico(uint8_t trigPin, uint8_t echoPin) {
  // Limpiar el pin TRIG (asegurarse de que esté en LOW antes de enviar el pulso)
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2); // Pequeña pausa

  // Establecer el pin TRIG en HIGH durante 10 microsegundos para enviar el pulso
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW); // Volver el pin TRIG a LOW

  // Medir la duración del pulso en el pin ECHO (tiempo que tarda la señal en ir y volver)
  long duracion = pulseIn(echoPin, HIGH);

  // Calcular la distancia en centímetros
  // Velocidad del sonido en el aire es aproximadamente 0.034 cm/microsegundo.
  // La distancia es (velocidad * tiempo) / 2 (porque el sonido viaja de ida y vuelta).
  float distanciaCm = duracion * 0.034 / 2;

  // Filtrado simple: si la distancia es 0 o un valor muy alto (fuera de rango usual),
  // se considera una lectura inválida y se devuelve un valor grande.
  if (distanciaCm == 0 || distanciaCm > 400) { 
    return 400.0; // Retorna 400 cm para indicar que no hay obstáculo cercano o lectura fuera de rango
  }
  return distanciaCm; // Retorna la distancia calculada
}
