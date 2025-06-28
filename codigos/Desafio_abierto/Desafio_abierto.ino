#include <Servo.h>

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
#define ANGULO_GIRO_DERECHA POS_CENTRO + ANGULO_MAX_GIRO + 10 // Ángulo para girar a la derecha
#define ANGULO_GIRO_IZQUIERDA POS_CENTRO - ANGULO_MAX_GIRO // Ángulo para girar a la izquierda
#define ANGULO_MAX_GIRO 38 // Máximo ángulo de giro para el servo (e.g., 30 grados a cada lado del centro)

#define DISTANCIA_PARADA_FRONTA 20 // Distancia en cm para detenerse frente a un muro
#define DISTANCIA_SEGURA 60        // Distancia en cm para reducir la velocidad (usado en el avance inicial)
#define DISTANCIA_DESEADA_PARED_FIJA 30 // Distancia deseada al muro exterior fijo para el control PID (en cm)
#define UMBRAL_DETECCION_ESQUINA 100 // Distancia en cm que indica una 'apertura' en el sensor lateral para detectar esquina

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

// Variables de estado del robot
bool detenerCompletamente = false;
bool iniciado = false; // Bandera para saber si el robot ha iniciado el desafío

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
      ajustarVelocidadEnRecta(); // Ajustar velocidad basada en la distancia frontal (generalizado)
      controlPID();              // Mantenerse en el centro del carril
      detectarYContarEsquina();  // Detectar y contar esquinas
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
    // Serial.println("Robot detenido. Desafio finalizado."); // Evitar spam en el serial
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
  if (distanciaCm == 0 || distanciaCm > 400) { // El sensor suele dar 0 o valores muy altos si no detecta nada
    return 400; // Un valor grande para indicar que no hay obstáculo cercano
  }
  return distanciaCm;
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
  miServo.write(constrain(angle, POS_CENTRO - ANGULO_MAX_GIRO, POS_CENTRO + ANGULO_MAX_GIRO)); // Limitar el ángulo
}

// --- Funciones de Lógica del Desafío ---

// El robot avanza hasta que el sensor frontal detecta un muro a DISTANCIA_PARADA_FRONTA
// La velocidad es proporcional a la distancia del muro.
void avanzarHastaMuroInicial() {
  motorDelante(VELOCIDAD_MINIMA); // Iniciar con una velocidad mínima constante
  long ultimaLecturaFrontal = millis(); // Para evitar lecturas muy seguidas
  float velActual = VELOCIDAD_MAXIMA;

  while (frontal_dist > DISTANCIA_PARADA_FRONTA) {
    frontal_dist = leerUltrasonico(TRIG_FRONTAL, ECHO_FRONTAL);
    Serial.print("Avanzando... Frontal: "); Serial.println(frontal_dist);

    // Ajustar velocidad proporcionalmente a la distancia
    // Cuanto más cerca, más lento. Usa un mapeo lineal.
    // De DISTANCIA_SEGURA a DISTANCIA_PARADA_FRONTA, la velocidad irá de VELOCIDAD_MAXIMA a VELOCIDAD_MINIMA.
    if (frontal_dist < DISTANCIA_SEGURA) {
      velActual = map(frontal_dist, DISTANCIA_PARADA_FRONTA, DISTANCIA_SEGURA, VELOCIDAD_MINIMA, VELOCIDAD_MAXIMA);
      velActual = constrain(velActual, VELOCIDAD_MINIMA, VELOCIDAD_MAXIMA); // Asegurar que no baja de min o excede max
      motorDelante(velActual);
    } else {
      motorDelante(VELOCIDAD_MAXIMA); // Velocidad máxima si está lejos del muro
    }
    setSteeringAngle(POS_CENTRO); // Mantener ruedas rectas

    delay(50); // Pequeña pausa para lecturas estables
  }
  motorParar(); // Detener completamente
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

  // La pista es cuadrada. En el punto de inicio (tras la parada frontal), el robot está en una esquina.
  // Uno de los sensores laterales debería ver una gran distancia (pista abierta), el otro la pared.
  // Podríamos tener una pista de 600mm o 1000mm de ancho.
  // Si derecha_dist es significativamente mayor que izquierda_dist, la pista abierta está a la derecha -> giro CW
  // Si izquierda_dist es significativamente mayor que derecha_dist, la pista abierta está a la izquierda -> giro CCW

  // Se asume que el robot se detiene en el centro del carril al inicio del muro frontal.
  // Y que los sensores laterales verán una pared y una apertura.
  // Se necesita un umbral de diferencia para ser significativo.
  float diferencia = abs(derecha_dist - izquierda_dist);
  float margen_error_ultrasonico = 10.0; // Margen para la precisión del sensor

  if (diferencia > margen_error_ultrasonico) {
    if (derecha_dist > izquierda_dist) {
      direccionActual = CLOCKWISE;
      Serial.println("Direccion: CLOCKWISE");
      setSteeringAngle(ANGULO_GIRO_DERECHA); // Preparar para el giro inicial
    } else {
      direccionActual = COUNTER_CLOCKWISE;
      Serial.println("Direccion: COUNTER_CLOCKWISE");
      setSteeringAngle(ANGULO_GIRO_IZQUIERDA); // Preparar para el giro inicial
    }
    motorDelante(VELOCIDAD_GIRO); // Avanzar un poco para iniciar el giro
    delay(500); // Tiempo para ejecutar el giro inicial (ajustar)
    motorParar();
    setSteeringAngle(POS_CENTRO); // Volver al centro
  } else {
    // Si las distancias son similares, algo no está bien o no es un punto de determinación claro.
    // Podría significar que está en una recta y no en una esquina detectable de esta manera.
    // Para este desafío, asumimos que el punto de partida es un muro frontal seguido de una esquina.
    Serial.println("No se pudo determinar la direccion del desafio, reintentando...");
    // Podría implementar una estrategia de "prueba y error" o mover el robot un poco
    // Por ahora, se mantendrá en DESCONOCIDA y el bucle principal intentará de nuevo.
  }
}


// Control PID para mantener el robot en el centro del carril
void controlPID() {
  float error = 0;
  // El error se calcula como la diferencia entre la distancia deseada y la distancia actual al muro fijo.
  // El muro fijo es el exterior.
  if (direccionActual == CLOCKWISE) {
    // Si va en sentido horario, el muro fijo está a la izquierda.
    error = izquierda_dist - DISTANCIA_DESEADA_PARED_FIJA;
  } else if (direccionActual == COUNTER_CLOCKWISE) {
    // Si va en sentido anti-horario, el muro fijo está a la derecha.
    error = derecha_dist - DISTANCIA_DESEADA_PARED_FIJA;
  } else {
    // Si la dirección es desconocida, no se aplica PID.
    setSteeringAngle(POS_CENTRO); // Mantener ruedas rectas por defecto.
    return;
  }

  unsigned long tiempoActualPID = millis();
  float deltaTime = (tiempoActualPID - tiempoAnteriorPID) / 1000.0; // Convertir a segundos
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
  // Si el error es positivo, el robot está muy lejos del muro fijo, necesita girar hacia el muro fijo.
  // Si el error es negativo, el robot está muy cerca del muro fijo, necesita girar lejos del muro fijo.
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
  Serial.print("Error: "); Serial.print(error);
  Serial.print(", Correccion: "); Serial.print(correccion);
  Serial.print(", Angulo Servo: "); Serial.println(anguloServo);
}

// Detecta el paso por una sección de esquina y actualiza el contador de vueltas.
// La detección de esquina es crítica y puede requerir ajuste.
// Una forma simple es detectar cuando un sensor lateral (el que está viendo el muro fijo)
// de repente ve una distancia muy grande (indicando una apertura o el fin de la pared recta)
// y el sensor frontal también ve una distancia grande (indicando que puede girar).
void detectarYContarEsquina() {
  bool esEsquinaDetectada = false;

  // Supongamos que en una esquina, el sensor frontal ve 'infinito' o un valor muy grande
  // y el sensor lateral que estaba midiendo la pared fija deja de verla o ve una apertura
  // significativa.
  // También podríamos usar la distancia del sensor lateral que da al 'vacío' de la pista
  // Si de repente ve una pared cercana, significa que está saliendo de la curva.
  
  // En el Open Challenge, las paredes interiores son móviles. La referencia es el muro fijo exterior.
  // Cuando el robot entra en una esquina, la distancia al muro exterior debería cambiar de forma predecible.
  // O el sensor lateral que estaba viendo la pared interior (móvil) de repente ve una gran apertura.

  if (direccionActual == CLOCKWISE) {
    // Muro fijo a la izquierda. Al entrar a una esquina, la pared derecha debería desaparecer
    // o el frontal ver una distancia grande para girar a la derecha.
    if (derecha_dist > UMBRAL_DETECCION_ESQUINA && frontal_dist > DISTANCIA_SEGURA) { // Si hay espacio para girar a la derecha
      // Y si estaba pegado a la pared izquierda (muro fijo) y de repente la izquierda se abre o la frontal es grande
      // Es una detección simplificada: si el frontal está libre y el lado opuesto al giro se abre.
      esEsquinaDetectada = true;
    }
  } else if (direccionActual == COUNTER_CLOCKWISE) {
    // Muro fijo a la derecha. Al entrar a una esquina, la pared izquierda debería desaparecer
    // o el frontal ver una distancia grande para girar a la izquierda.
    if (izquierda_dist > UMBRAL_DETECCION_ESQUINA && frontal_dist > DISTANCIA_SEGURA) { // Si hay espacio para girar a la izquierda
      esEsquinaDetectada = true;
    }
  }

  // Lógica para contar la esquina solo una vez por esquina
  static bool enEsquina = false; // Estado para evitar contar múltiples veces la misma esquina
  if (esEsquinaDetectada && !enEsquina) {
    seccionesEsquinaPasadas++;
    Serial.print("Esquina detectada! Secciones pasadas: "); Serial.println(seccionesEsquinaPasadas);
    enEsquina = true; // El robot ha entrado en la esquina
    manejarEsquina(); // Ejecutar la lógica de giro
  } else if (!esEsquinaDetectada && enEsquina) {
    // El robot ha salido de la esquina, resetear la bandera
    enEsquina = false;
  }
}

// Ejecuta el giro en la esquina
void manejarEsquina() {
  motorDelante(VELOCIDAD_GIRO); // Velocidad reducida para el giro
  if (direccionActual == CLOCKWISE) {
    setSteeringAngle(ANGULO_GIRO_DERECHA);
  } else if (direccionActual == COUNTER_CLOCKWISE) {
    setSteeringAngle(ANGULO_GIRO_IZQUIERDA);
  }

  // Ejecutar el giro por un tiempo determinado o hasta que los sensores indiquen el fin de la curva.
  // Para una implementación robusta, esto debería basarse en lecturas de sensores (frontal ve pared, lateral ve pared).
  // Por simplicidad, un delay fijo:
  Serial.println("Manejando esquina...");
  delay(800); // Tiempo de giro (ajustar según el robot y la curva)

  setSteeringAngle(POS_CENTRO); // Volver las ruedas rectas
  motorDelante(VELOCIDAD_MAXIMA); // Acelerar para la recta siguiente
  delay(200); // Pequeño avance post-giro
}

// Ajusta la velocidad en los tramos rectos proporcional a la distancia del muro frontal.
// Esto podría ser útil para desacelerar si hay un obstáculo inesperado o si se acerca a un muro.
// Sin embargo, para tramos rectos normales de carril, una velocidad constante puede ser mejor.
// La interpretación del usuario "velocidad en los tramos rectos sea proporcional a la distancia de la pared"
// se aplica aquí como una 'velocidad adaptativa' al frontal más que un control de velocidad constante.
void ajustarVelocidadEnRecta() {
  // Si el sensor frontal detecta un muro cercano, reducir la velocidad.
  if (frontal_dist < DISTANCIA_SEGURA) {
    int velocidadAdaptativa = map(frontal_dist, DISTANCIA_PARADA_FRONTA, DISTANCIA_SEGURA, VELOCIDAD_MINIMA, VELOCIDAD_MAXIMA);
    motorDelante(constrain(velocidadAdaptativa, VELOCIDAD_MINIMA, VELOCIDAD_MAXIMA));
    Serial.print("Velocidad adaptativa: "); Serial.println(velocidadAdaptativa);
  } else {
    motorDelante(VELOCIDAD_MAXIMA); // Velocidad máxima si el camino está libre
  }
}
