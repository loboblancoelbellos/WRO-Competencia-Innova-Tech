#include <Servo.h>
#include <math.h> // Para la función cos() y radians()

// --- Pines de Hardware ---
#define SW_ARRANQUE 2     // Pin para el botón de arranque (con pull-up interno)
#define ENA 3             // Pin PWM para el control de velocidad del motor (Debe ser un pin PWM, ej. 3, 5, 6, 9, 10, 11 en Arduino UNO)
#define IN1 4             // Pin de control de dirección del motor 1
#define IN2 5             // Pin de control de dirección del motor 2
#define PIN_SERVO 6       // Pin para el servo de dirección
#define TRIG_FRONTAL 7    // Pin TRIG del sensor ultrasónico frontal
#define ECHO_FRONTAL 8    // Pin ECHO del sensor ultrasónico frontal
#define TRIG_DERECHO 9    // Pin TRIG del sensor ultrasónico derecho
#define ECHO_DERECHO 10   // Pin ECHO del sensor ultrasónico derecho
#define TRIG_IZQUIERDO 11 // Pin TRIG del sensor ultrasónico izquierdo
#define ECHO_IZQUIERDO 12 // Pin ECHO del sensor ultrasónico izquierdo

// --- Parámetros de Calibración y Comportamiento ---
#define POS_CENTRO 94      // Ángulo del servo para ir recto (ajustar si es necesario)
#define ANGULO_MAX_GIRO 38 // Máximo ángulo de giro para el servo (e.g., 38 grados a cada lado del centro)
// Los ángulos de giro se calculan dinámicamente en setSteeringAngle para mayor flexibilidad.

#define DISTANCIA_PARADA_FRONTA 60 // Distancia en cm para detenerse frente a un muro (al inicio y en cualquier obstáculo)
#define DISTANCIA_SEGURA 200        // Distancia en cm a partir de la cual la velocidad comienza a reducirse
#define DISTANCIA_DESEADA_PARED_FIJA 20 // Distancia deseada al muro exterior fijo para el control PID (en cm)
#define UMBRAL_DETECCION_ESQUINA 100 // Distancia en cm que indica una 'apertura' en el sensor lateral para detectar esquina
#define UMBRAL_RECTA_SIMILARIDAD 5.0 // Umbral en cm para considerar que las paredes laterales son paralelas (fin de giro)

// --- Constantes PID para control de dirección ---
#define KP 0.8  // Ganancia Proporcional (ajustar)
#define KI 0.01 // Ganancia Integral (ajustar)
#define KD 0.05 // Ganancia Derivativa (ajustar)

// --- Parámetros de Velocidad ---
#define VELOCIDAD_MAXIMA 200 // Velocidad máxima del motor (0-255 PWM)
#define VELOCIDAD_MINIMA 50  // Velocidad mínima del motor para movimiento
#define VELOCIDAD_GIRO 100   // Velocidad del motor al girar en una esquina

// --- Variables Globales ---
Servo miServo; // Objeto servo

// Lecturas de los sensores ultrasónicos
float frontal_dist;
float derecha_dist;
float izquierda_dist;

// Almacena el ángulo actual del servo para corrección trigonométrica
int currentSteeringAngle = POS_CENTRO; 

// Variables de estado del robot
bool detenerCompletamente = false;
bool iniciado = false; // Bandera para saber si el robot ha iniciado el desafío
bool enFaseDeGiro = false; // Indica si el robot está actualmente ejecutando un giro de esquina

// Variables para el control de vueltas
int seccionesEsquinaPasadas = 0;
const int MAX_SECCIONES_ESQUINA = 12; // 3 vueltas * 4 esquinas/vuelta = 12 secciones de esquina

// Variable para la dirección del desafío (CLO = Clockwise, CCW = Counter-Clockwise)
enum DireccionDesafio { DESCONOCIDA, CLOCKWISE, COUNTER_CLOCKWISE };
DireccionDesafio direccionActual = DESCONOCIDA;

// Variables PID
float errorAnterior = 0;
float sumaErrores = 0;
unsigned long tiempoAnteriorPID = 0;

// --- Prototipos de Funciones ---
float leerUltrasonico(uint8_t trigPin, uint8_t echoPin);
float getCorrectedLateralDistance(float raw_distance, int servo_angle_val, float sensor_offset_cm);
void motorDelante(int velocidad);
void motorAtras(int velocidad);
void motorParar();
void setSteeringAngle(int angle);
void avanzarHastaMuroInicial();
void determinarDireccionDesafio();
void controlPID();
void detectarYContarEsquina();
void manejarEsquina();
void ajustarVelocidadEnRecta();

// --- Configuración Inicial ---
void setup() {
  // Configuración de pines de motor
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  motorParar(); // Asegurarse de que el motor esté parado al inicio

  // Configuración de pines de sensores ultrasónicos
  pinMode(TRIG_FRONTAL, OUTPUT);
  pinMode(ECHO_FRONTAL, INPUT);
  pinMode(TRIG_DERECHO, OUTPUT);
  pinMode(ECHO_DERECHO, INPUT);
  pinMode(TRIG_IZQUIERDO, OUTPUT);
  pinMode(ECHO_IZQUIERDO, INPUT);

  // Configuración del botón de arranque con pull-up interno
  pinMode(SW_ARRANQUE, INPUT_PULLUP);

  // Adjuntar el servo
  miServo.attach(PIN_SERVO);
  setSteeringAngle(POS_CENTRO); // Poner las ruedas rectas al inicio
  delay(500); // Pequeña pausa para que el servo se estabilice

  // Iniciar comunicación serial para depuración
  Serial.begin(9600);
  Serial.println("Sistema iniciado. Esperando boton de arranque...");
}

// --- Bucle Principal ---
void loop() {
  // Leer el estado del botón de arranque
  if (digitalRead(SW_ARRANQUE) == LOW) { // Si el botón está presionado (LOW por pull-up)
    if (!iniciado) {
      Serial.println("Boton de arranque presionado. Iniciando desafio.");
      delay(500); // Antirebote
      iniciado = true; // El robot ha iniciado
      detenerCompletamente = false; // Reiniciar estado de detención
    }
  }

  if (iniciado && !detenerCompletamente) {
    // 1. Leer sensores
    frontal_dist = leerUltrasonico(TRIG_FRONTAL, ECHO_FRONTAL);
    derecha_dist = leerUltrasonico(TRIG_DERECHO, ECHO_DERECHO);
    izquierda_dist = leerUltrasonico(TRIG_IZQUIERDO, ECHO_IZQUIERDO);

    // Imprimir lecturas para depuración
    Serial.print("Frontal: "); Serial.print(frontal_dist);
    Serial.print(" Derecha: "); Serial.print(derecha_dist);
    Serial.print(" Izquierda: "); Serial.println(izquierda_dist);

    // --- Lógica del Desafío ---

    // Paso 1: Avanzar hasta el muro inicial y determinar la dirección si aún no se ha hecho
    if (direccionActual == DESCONOCIDA) {
      Serial.println("Avanzando hacia el muro inicial...");
      avanzarHastaMuroInicial(); // Avanza y se detiene
      // Una vez detenido frente al muro, determina la dirección
      determinarDireccionDesafio();
      Serial.print("Direccion del desafio determinada: ");
      if (direccionActual == CLOCKWISE) {
        Serial.println("CLOCKWISE");
      } else if (direccionActual == COUNTER_CLOCKWISE) {
        Serial.println("COUNTER_CLOCKWISE");
      }
      // Después de determinar la dirección y el giro inicial, el robot debe empezar a seguir el carril
      // Podría ser necesario un pequeño avance o ajuste aquí antes del bucle principal de seguimiento.
      // Por simplicidad, asumimos que el giro de determinarDireccionDesafio lo deja bien posicionado.
    } else {
      // Si la dirección ya está determinada, el robot está en el ciclo de seguimiento de carril y vueltas
      if (!enFaseDeGiro) { // Solo ajustar velocidad y PID si no está en una fase de giro activa
        ajustarVelocidadEnRecta(); // Ajustar velocidad basada en la distancia frontal (generalizado)
        controlPID();              // Mantenerse en el centro del carril
      }
      detectarYContarEsquina();  // Detectar y contar esquinas (llama a manejarEsquina si detecta)
    }

    // Comprobar si se han completado las 3 vueltas
    if (seccionesEsquinaPasadas >= MAX_SECCIONES_ESQUINA) {
      Serial.println("3 vueltas completadas! Deteniendo el robot.");
      motorParar();
      detenerCompletamente = true; // Detener completamente el robot
      iniciado = false; // Resetear el estado para que no vuelva a entrar en el bucle principal
    }
  } else if (detenerCompletamente) {
    motorParar(); // Asegurarse de que el motor esté parado
    setSteeringAngle(POS_CENTRO); // Ruedas rectas al finalizar
    delay(100); // Pequeña pausa para no saturar el serial
  }
}

// --- Implementación de Funciones Auxiliares ---

// Función para leer la distancia de un sensor ultrasónico
float leerUltrasonico(uint8_t trigPin, uint8_t echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duracion = pulseIn(echoPin, HIGH);
  float distanciaCm = duracion * 0.034 / 2;
  // Filtrado simple para lecturas inconsistentes (puede ser necesario un filtro más avanzado)
  if (distanciaCm == 0 || distanciaCm > 400 || distanciaCm < 2) { // El sensor suele dar 0 o valores muy altos/muy bajos si no detecta nada
    return 400; // Un valor grande para indicar que no hay obstáculo cercano o lectura fuera de rango
  }
  return distanciaCm;
}

/**
 * @brief Corrige la distancia leída por un sensor lateral basándose en el ángulo de dirección del servo.
 * Esto compensa cuando el sensor no está perpendicular a la pared debido a un giro.
 * @param raw_distance La lectura de distancia cruda del sensor lateral.
 * @param servo_angle_val El ángulo actual del servo de dirección (currentSteeringAngle).
 * @param sensor_offset_cm Opcional: distancia del sensor al eje de giro del vehículo. 
 * (Por simplicidad, en este ejemplo se asume 0, lo cual es una idealización.
 * Para mayor precisión, mide esta distancia).
 * @return La distancia estimada corregida a la pared.
 */
float getCorrectedLateralDistance(float raw_distance, int servo_angle_val, float sensor_offset_cm = 0.0) {
  // Calcular el ángulo de desviación del servo respecto al centro.
  float angle_deviation_degrees = abs(servo_angle_val - POS_CENTRO);
  
  // Convertir grados a radianes para la función cos()
  float angle_deviation_radians = radians(angle_deviation_degrees);
  
  // Fórmula trigonométrica simplificada: corrected_dist = raw_dist * cos(ángulo_desviación)
  // Esto asume que el sensor está en el eje de giro. Si hay un offset, la fórmula es más compleja.
  float corrected_dist = raw_distance * cos(angle_deviation_radians);

  // Si hay un offset significativo (distancia del sensor al eje de giro en el sentido del avance)
  // y un ángulo, esto también influye, pero lo omitimos para mantener la complejidad.
  // Una corrección más completa implicaría:
  // corrected_dist = (raw_distance - sensor_offset_x * sin(angle_deviation_radians)) * cos(angle_deviation_radians);
  // O, si el offset es lateral: corrected_dist = raw_distance / cos(angle_deviation_radians) + offset_y * sin(angle_deviation_radians);
  // Para la mayoría de los casos de uso con sensores laterales montados cerca de las ruedas delanteras,
  // raw_distance * cos(angle_deviation_radians) es una buena aproximación.

  return corrected_dist;
}

// Función para mover el motor hacia adelante
void motorDelante(int velocidad) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, constrain(velocidad, 0, 255)); // Limitar velocidad entre 0 y 255
}

// Función para mover el motor hacia atrás (no se usa en este desafío, pero útil)
void motorAtras(int velocidad) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, constrain(velocidad, 0, 255));
}

// Función para detener el motor
void motorParar() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0); // Detener el motor
}

// Función para establecer el ángulo del servo de dirección
void setSteeringAngle(int angle) {
  // Limitar el ángulo para evitar que el servo exceda sus límites mecánicos
  currentSteeringAngle = constrain(angle, POS_CENTRO - ANGULO_MAX_GIRO, POS_CENTRO + ANGULO_MAX_GIRO); 
  miServo.write(currentSteeringAngle);
}

// --- Funciones de Lógica del Desafío ---

// El robot avanza hasta que el sensor frontal detecta un muro a DISTANCIA_PARADA_FRONTA
// La velocidad es proporcional a la distancia del muro.
void avanzarHastaMuroInicial() {
  // TODO: El usuario ya ha marcado esta función para revisión. La lógica actual es de
  // velocidad proporcional a la distancia frontal y detención.
  // Esto ya cumple con "velocidad en los tramos rectos sea proporcional a la distancia de la pared y se detenga a la distancia especificada"
  // para el contexto de un muro frontal.
  float velActual;

  while (frontal_dist > DISTANCIA_PARADA_FRONTA) {
    frontal_dist = leerUltrasonico(TRIG_FRONTAL, ECHO_FRONTAL);
    Serial.print("Avanzando... Frontal: "); Serial.println(frontal_dist);

    // Ajustar velocidad: más rápido cuando está lejos, más lento cuando está cerca
    // Mapea la distancia frontal desde DISTANCIA_SEGURA hasta DISTANCIA_PARADA_FRONTA
    // a un rango de velocidad desde VELOCIDAD_MAXIMA hasta VELOCIDAD_MINIMA.
    velActual = map(frontal_dist, DISTANCIA_PARADA_FRONTA, DISTANCIA_SEGURA, VELOCIDAD_MINIMA, VELOCIDAD_MAXIMA);
    velActual = constrain(velActual, VELOCIDAD_MINIMA, VELOCIDAD_MAXIMA); // Asegurar que la velocidad esté dentro del rango definido
    motorDelante(velActual);
    setSteeringAngle(POS_CENTRO); // Mantener ruedas rectas

    delay(50); // Pequeña pausa para lecturas estables
  }
  motorParar(); // Detener completamente al alcanzar la distancia de parada
  Serial.println("Detenido frente al muro inicial.");
  delay(1000); // Pausa para estabilizar después de la parada
}

// Determina la dirección del desafío (Clockwise o Counter-Clockwise)
// Asume que al detenerse frente al muro inicial, un lado estará más "abierto" que el otro.
void determinarDireccionDesafio() {
  // Después de detenerse frente al muro, leer de nuevo los sensores laterales
  derecha_dist = leerUltrasonico(TRIG_DERECHO, ECHO_DERECHO);
  izquierda_dist = leerUltrasonico(TRIG_IZQUIERDO, ECHO_IZQUIERDO);

  Serial.print("Determinado direccion: Derecha: "); Serial.print(derecha_dist);
  Serial.print(", Izquierda: "); Serial.println(izquierda_dist);

  float diferencia = abs(derecha_dist - izquierda_dist);
  float margen_error_ultrasonico = 10.0; // Margen para la precisión del sensor

  if (diferencia > margen_error_ultrasonico) {
    if (derecha_dist > izquierda_dist) {
      direccionActual = CLOCKWISE;
      Serial.println("Direccion: CLOCKWISE");
      setSteeringAngle(POS_CENTRO + ANGULO_MAX_GIRO + 10); // Preparar para el giro inicial a la derecha (ajuste fino +10)
    } else {
      direccionActual = COUNTER_CLOCKWISE;
      Serial.println("Direccion: COUNTER_CLOCKWISE");
      setSteeringAngle(POS_CENTRO - ANGULO_MAX_GIRO); // Preparar para el giro inicial a la izquierda
    }
    // No avanzamos aquí, la primera 'manejarEsquina' o el ciclo principal lo hará.
  } else {
    Serial.println("No se pudo determinar la direccion del desafio, reintentando...");
    // Considerar un comportamiento de respaldo si la dirección no se puede determinar.
    // Por ahora, se mantendrá en DESCONOCIDA y el bucle principal intentará de nuevo.
  }
}

// Control PID para mantener el robot en el centro del carril
void controlPID() {
  float error = 0;
  float corrected_izquierda_dist = getCorrectedLateralDistance(izquierda_dist, currentSteeringAngle);
  float corrected_derecha_dist = getCorrectedLateralDistance(derecha_dist, currentSteeringAngle);

  // El error se calcula como la diferencia entre la distancia deseada y la distancia actual al muro fijo.
  // El muro fijo es el exterior.
  if (direccionActual == CLOCKWISE) {
    // Si va en sentido horario, el muro fijo está a la izquierda.
    error = corrected_izquierda_dist - DISTANCIA_DESEADA_PARED_FIJA;
  } else if (direccionActual == COUNTER_CLOCKWISE) {
    // Si va en sentido anti-horario, el muro fijo está a la derecha.
    error = corrected_derecha_dist - DISTANCIA_DESEADA_PARED_FIJA;
  } else {
    // Si la dirección es desconocida, no se aplica PID.
    setSteeringAngle(POS_CENTRO); // Mantener ruedas rectas por defecto.
    return;
  }

  unsigned long tiempoActualPID = millis();
  float deltaTime = (tiempoActualPID - tiempoAnteriorPID) / 1000.0; // Convertir a segundos
  if (deltaTime == 0) deltaTime = 0.001; // Evitar division por cero
  tiempoAnteriorPID = tiempoActualPID;

  // Componente Proporcional
  float p_term = KP * error;

  // Componente Integral (anti-windup simple)
  sumaErrores += error * deltaTime;
  // Limitar la suma de errores para evitar saturación del integrador
  if (sumaErrores > 200) sumaErrores = 200;
  if (sumaErrores < -200) sumaErrores = -200;
  float i_term = KI * sumaErrores;

  // Componente Derivativo
  float d_term = KD * ((error - errorAnterior) / deltaTime);
  errorAnterior = error;

  // Calcular la corrección total del PID
  float correccion = p_term + i_term + d_term;

  // Ajustar el ángulo del servo
  int anguloServo = POS_CENTRO;

  if (direccionActual == CLOCKWISE) {
    // Para CW, muro fijo está a la izquierda.
    // Si error > 0 (muy lejos de la pared izquierda), necesita girar a la izquierda (ángulo < POS_CENTRO)
    // Si error < 0 (muy cerca de la pared izquierda), necesita girar a la derecha (ángulo > POS_CENTRO)
    anguloServo = POS_CENTRO - (int)correccion; // Invertir corrección para que un error positivo gire a la izquierda
  } else if (direccionActual == COUNTER_CLOCKWISE) {
    // Para CCW, muro fijo está a la derecha.
    // Si error > 0 (muy lejos de la pared derecha), necesita girar a la derecha (ángulo > POS_CENTRO)
    // Si error < 0 (muy cerca de la pared derecha), necesita girar a la izquierda (ángulo < POS_CENTRO)
    anguloServo = POS_CENTRO + (int)correccion; // Error positivo debe girar a la derecha
  }

  setSteeringAngle(anguloServo);
  Serial.print("Error PID: "); Serial.print(error);
  Serial.print(", Corr: "); Serial.print(correccion);
  Serial.print(", Angulo Servo: "); Serial.println(anguloServo);
}

// Detecta el paso por una sección de esquina y actualiza el contador de vueltas.
void detectarYContarEsquina() {
  bool esEsquinaDetectada = false;

  // Detección de esquina: uno de los sensores laterales debe ver una apertura (distancia grande)
  // mientras el frontal está relativamente libre.
  if (direccionActual == CLOCKWISE) {
    // Si se mueve CW, la esquina es un giro a la derecha. El sensor derecho debería ver la "apertura".
    if (derecha_dist > UMBRAL_DETECCION_ESQUINA && frontal_dist > DISTANCIA_PARADA_FRONTA + 10) { 
      esEsquinaDetectada = true;
    }
  } else if (direccionActual == COUNTER_CLOCKWISE) {
    // Si se mueve CCW, la esquina es un giro a la izquierda. El sensor izquierdo debería ver la "apertura".
    if (izquierda_dist > UMBRAL_DETECCION_ESQUINA && frontal_dist > DISTANCIA_PARADA_FRONTA + 10) {
      esEsquinaDetectada = true;
    }
  }

  // Lógica para contar la esquina solo una vez por esquina y manejar el giro
  static bool enTramoRectoPrevioEsquina = true; // Estado para evitar doble conteo y asegurar transición
  
  if (esEsquinaDetectada && enTramoRectoPrevioEsquina && !enFaseDeGiro) {
    Serial.println("¡Potencial esquina detectada! Iniciando manejo de esquina...");
    enFaseDeGiro = true; // Entrar en fase de giro
    enTramoRectoPrevioEsquina = false; // Resetear para la siguiente esquina
    manejarEsquina(); // Ejecutar la lógica de giro
    // Después de manejar la esquina, el robot ya debería estar en la siguiente recta, listo para PID.
    seccionesEsquinaPasadas++;
    Serial.print("Esquina completada. Secciones pasadas: "); Serial.println(seccionesEsquinaPasadas);
    enFaseDeGiro = false; // Salir de la fase de giro una vez que se completó
  } else if (!esEsquinaDetectada && !enTramoRectoPrevioEsquina && !enFaseDeGiro) {
    // Una vez que el robot ha salido de la esquina y no está en fase de giro
    // y los sensores ya no detectan una esquina, se considera que está en una recta de nuevo.
    enTramoRectoPrevioEsquina = true; 
  }
}

/**
 * TODO: el robot debe girar las ruedas de la direccion y entonces empezar a moverse, el giro de la esquina termina cuando el sensor ultrasonico adelante no 
 * tiene obstaculos y ambos sensores ultrasonicos, izquierdo y derecho tienen paredes a los lados, puede usarse el umbral de error para comparar.
 */
void manejarEsquina() {
  Serial.println("Manejando esquina...");
  motorParar(); // Detener brevemente antes de iniciar el giro o ajustar el ángulo
  delay(100); // Pequeña pausa para estabilizar

  // 1. Girar las ruedas a la dirección correcta para la curva
  if (direccionActual == CLOCKWISE) {
    setSteeringAngle(POS_CENTRO + ANGULO_MAX_GIRO); // Ángulo para giro a la derecha
    Serial.println("Girando ruedas a la derecha...");
  } else if (direccionActual == COUNTER_CLOCKWISE) {
    setSteeringAngle(POS_CENTRO - ANGULO_MAX_GIRO); // Ángulo para giro a la izquierda
    Serial.println("Girando ruedas a la izquierda...");
  }
  delay(500); // Dar tiempo al servo para alcanzar la posición

  motorDelante(VELOCIDAD_GIRO); // Moverse a velocidad reducida durante el giro

  // 2. Esperar a que el robot complete el giro basándose en las lecturas de los sensores
  // El giro termina cuando:
  // a) El sensor frontal no detecta obstáculos cercanos (el camino está despejado)
  // b) Ambos sensores laterales (corregidos) detectan paredes a una distancia similar y razonable,
  //    indicando que el robot ha entrado en una nueva sección recta.

  unsigned long tiempoInicioGiro = millis();
  const long TIMEOUT_GIRO = 5000; // Timeout para evitar bucles infinitos (5 segundos)

  while (millis() - tiempoInicioGiro < TIMEOUT_GIRO) {
    frontal_dist = leerUltrasonico(TRIG_FRONTAL, ECHO_FRONTAL);
    derecha_dist = leerUltrasonico(TRIG_DERECHO, ECHO_DERECHO);
    izquierda_dist = leerUltrasonico(TRIG_IZQUIERDO, ECHO_IZQUIERDO);

    float corrected_derecha = getCorrectedLateralDistance(derecha_dist, currentSteeringAngle);
    float corrected_izquierda = getCorrectedLateralDistance(izquierda_dist, currentSteeringAngle);

    Serial.print("Girando... Frontal: "); Serial.print(frontal_dist);
    Serial.print(", C_Der: "); Serial.print(corrected_derecha);
    Serial.print(", C_Izq: "); Serial.println(corrected_izquierda);

    // Condición de salida del giro:
    // 1. Camino frontal despejado (ya no está 'pegado' al muro de la esquina)
    // 2. Ambos lados detectan paredes a una distancia razonable (está en el carril)
    // 3. Las distancias laterales corregidas son "similares" (ha enderezado y está en una recta)
    if (frontal_dist > DISTANCIA_SEGURA && 
        corrected_derecha < UMBRAL_DETECCION_ESQUINA && corrected_derecha > (DISTANCIA_DESEADA_PARED_FIJA / 2) &&
        corrected_izquierda < UMBRAL_DETECCION_ESQUINA && corrected_izquierda > (DISTANCIA_DESEADA_PARED_FIJA / 2) &&
        abs(corrected_derecha - corrected_izquierda) < UMBRAL_RECTA_SIMILARIDAD) {
      Serial.println("Fin del giro detectado: En nueva recta.");
      break; // Salir del bucle de giro
    }
    delay(50); // Pequeña pausa
  }
  
  if (millis() - tiempoInicioGiro >= TIMEOUT_GIRO) {
    Serial.println("Timeout en el giro. Puede que no haya detectado el fin de la esquina correctamente.");
  }

  motorParar(); // Detener al final del giro para asegurar la estabilidad
  setSteeringAngle(POS_CENTRO); // Volver las ruedas rectas
  delay(200); // Pequeña pausa antes de continuar
}

/**
 * TODO: Ajustar la función
 * la velocidad del robot debe ser proporcional a la distancia y debe detenerse frente al muro a la distancia segura para así cruzar
 */
void ajustarVelocidadEnRecta() {
  // Esta función controla la velocidad general del robot en tramos rectos.
  // La velocidad es inversamente proporcional a la distancia del muro frontal:
  // más rápido cuando el camino está libre, y disminuye progresivamente si se detecta un obstáculo frontal.
  // La detención completa en DISTANCIA_PARADA_FRONTA es manejada implícitamente por el loop principal
  // que llama a esta función y luego chequea si frontal_dist > DISTANCIA_PARADA_FRONTA.
  // Si frontal_dist es <= DISTANCIA_PARADA_FRONTA, el robot debería haberse detenido o estar a punto de hacerlo.

  if (frontal_dist > DISTANCIA_PARADA_FRONTA) {
    // Mapear la distancia frontal a un valor de velocidad.
    // Si frontal_dist es VELOCIDAD_SEGURA, la velocidad será VELOCIDAD_MAXIMA.
    // Si frontal_dist es DISTANCIA_PARADA_FRONTA, la velocidad será VELOCIDAD_MINIMA.
    int velocidadAdaptativa = map(frontal_dist, DISTANCIA_PARADA_FRONTA, DISTANCIA_SEGURA, VELOCIDAD_MINIMA, VELOCIDAD_MAXIMA);
    
    // Asegurar que la velocidad esté dentro de los límites definidos
    velocidadAdaptativa = constrain(velocidadAdaptativa, VELOCIDAD_MINIMA, VELOCIDAD_MAXIMA);
    motorDelante(velocidadAdaptativa);
    Serial.print("Velocidad adaptativa en recta: "); Serial.println(velocidadAdaptativa);
  } else {
    // Si la distancia frontal es igual o menor a DISTANCIA_PARADA_FRONTA, detenerse.
    motorParar(); 
    Serial.println("Muro frontal detectado, deteniendo en ajustarVelocidadEnRecta.");
  }
}
