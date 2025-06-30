
## **Diagrama Electrico**

![circuit_image](https://github.com/user-attachments/assets/2e3a6b02-0ed8-4931-bf9c-d51e61286a32)



## Diagrama de Flujo del CircuitoEste documento presenta un diagrama de flujo que ilustra la lógica operativa de un circuito, basado en la imagen de referencia proporcionada.

    A[Inicio] --> B{Encendido};
    B --> C[Verificar Voltaje de Batería];
    C -- Voltaje OK --> D[Inicializar Arduino UNO];
    C -- Voltaje Bajo --> E[Indicar Batería Baja / Apagar];
    D --> F[Inicializar Sensores Ultrasónicos];
    F --> G[Inicializar Servo Motor];
    F --> H[Inicializar Controlador de Motor];
    G --> I[Iniciar Bucle Principal];
    I --> J[Leer Distancia del Sensor 1];
    J --> K[Leer Distancia del Sensor 2];
    J --> L[Leer Distancia del Sensor 3];
    K --> M{Obstáculo Detectado?};
    M -- Sí --> N[Mover Servo Motor];
    N --> O[Controlar Motor DC a Través del Controlador];
    M -- No --> P[Continuar Monitoreando];
    O --> Q[Esperar / Retrasar];
    Q --> I;
    P --> I;


**Explicación Detallada del Diagrama de Flujo**:El diagrama de flujo desglosa los pasos inferidos del funcionamiento del circuito:Inicio:Representa el punto de partida para la secuencia de operaciones del circuito.

**ENCENDIDO**:Se refiere al momento en que el circuito recibe energía. Basado en la imagen original, esto se logra a través de las baterías de iones de litio (18650 Li-ion).Verificar Voltaje de Batería:Aunque no se visualiza un componente específico para esta tarea en el circuito, se asume como una buena práctica. La lógica aquí es asegurar que la fuente de energía sea adecuada para el funcionamiento continuo.Voltaje OK: Si el nivel de voltaje es suficiente, el sistema procede a inicializar el microcontrolador.Voltaje Bajo: Si el voltaje es crítico, el sistema podría emitir una advertencia visual o sonora, o incluso apagarse para proteger los componentes y prolongar la vida útil de las baterías.

**Inicializar Arduino UNO**:El corazón del circuito, el microcontrolador Arduino UNO, comienza su secuencia de arranque. Esto incluye la configuración de sus puertos (pines de entrada/salida) y la preparación para la ejecución del código.Inicializar Sensores Ultrasónicos Los tres sensores HC-SR04 se configuran para su función principal: emitir pulsos de sonido y medir el tiempo que tardan en regresar para calcular distancias.Inicializar Servo Motor El servo motor se establece en una posición predeterminada, que generalmente es su punto central o una posición de reposo inicial. Inicializar Controlador de Motor El módulo controlador de motor L298N se prepara para recibir señales del Arduino y controlar el motor de corriente continua (DC).

**Iniciar Bucle Principal**:El programa del Arduino entra en un ciclo de ejecución continuo. Este bucle es crucial, ya que permite que el circuito opere de forma autónoma y reactiva.Leer Distancia del Sensor 1, 2, 3:Dentro del bucle principal, cada sensor ultrasónico realiza una lectura de distancia. Estas lecturas son fundamentales para la detección de objetos.Obstáculo Detectado?

Este es un punto de decisión crítico. El sistema procesa las lecturas de los sensores para determinar si hay algún objeto dentro de un rango de proximidad que requiera una acción.Sí Si se identifica un obstáculo Mover Servo Motor El servo motor ajusta su posición. Esto podría ser para reorientar un sensor, activar una parte móvil o iniciar una acción de evasión.Controlar Motor DC a Través del Controlador: El motor DC se activa o se le da una instrucción (por ejemplo, avanzar, detenerse, retroceder, girar) para reaccionar al obstáculo.Esperar / Retrasar: Una breve pausa permite que las acciones iniciadas se completen o para estabilizar el sistema antes de la siguiente iteración de lectura.No: Si no se detectan obstáculos, el sistema simplemente continúa su proceso de monitoreo.Bucle Independientemente del resultado de la detección de obstáculos, el flujo regresa al paso de lectura de distancias. Esto asegura un monitoreo constante del entorno y una capacidad de respuesta en tiempo real del circuito.
