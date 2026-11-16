#define MOTOR_LEFT_IN1  17
#define MOTOR_LEFT_IN2  16
#define MOTOR_RIGHT_IN1 2
#define MOTOR_RIGHT_IN2 13

// --- Configuración de PWM ---
#define PWM_FREQ       15000
#define PWM_RESOLUTION 8     // 8 bits (Nota: Tu comentario decía 12 bits, pero el valor es 8)

// --- CONFIGURACIÓN ENCODER 1 ---
#define ENCODER1_A  25
#define ENCODER1_B  26

// --- CONFIGURACIÓN ENCODER 2 (¡Ocupa pines I2C nativos!) ---
#define ENCODER2_A  21
#define ENCODER2_B  22

long encoder1Pulsos=0;
long encoder2Pulsos=0;

float erroranterior=0.0;
float derivativo=0.0;
float integral=0.0;

long tiempo=0;
long tiempoAnterior=0; // Asegúrate de inicializar esto para el cálculo de dt

float distancianecesaria=500.0;

// === NUEVAS VARIABLES PARA ACELERACIÓN ===
float aceleracion = 10.0;     // Unidades de PWM por segundo (puedes variar este valor)
float maximo_actual = 0.0;    // Comienza en 0 para la rampa de aceleración
float maximo_final = 50.0;    // El límite de velocidad que tenías originalmente (30)

void IRAM_ATTR leerEncoder1AB() {
  int estadoA = digitalRead(ENCODER1_A);
  int estadoB = digitalRead(ENCODER1_B);
  if (estadoA == estadoB) { encoder1Pulsos++; } else { encoder1Pulsos--; }
}

void IRAM_ATTR leerEncoder2AB() {
  int estadoA = digitalRead(ENCODER2_A);
  int estadoB = digitalRead(ENCODER2_B);
  if (estadoA == estadoB) { encoder2Pulsos--; } else { encoder2Pulsos++; }
}

void setMotor(int chIN1, int chIN2, float v, int maximo, float porcentaje) {
  v = constrain(v, -maximo, maximo);  
  v = v * porcentaje;
  
  if (v < 0)      { ledcWrite(chIN1, (uint32_t)(-v)); ledcWrite(chIN2, 0); }
  else if (v > 0) { ledcWrite(chIN1, 0); ledcWrite(chIN2, (uint32_t)v); }
  else            { ledcWrite(chIN1, 0); ledcWrite(chIN2, 0); }
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  ledcSetup(0, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_LEFT_IN1, 0);

  ledcSetup(1, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_LEFT_IN2, 1);

  ledcSetup(2, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_RIGHT_IN1, 2);

  ledcSetup(3, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_RIGHT_IN2, 3);
  
  pinMode(ENCODER1_A, INPUT);
  pinMode(ENCODER1_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER1_A), leerEncoder1AB, CHANGE);

  pinMode(ENCODER2_A, INPUT);
  pinMode(ENCODER2_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_A), leerEncoder2AB, CHANGE);

  tiempoAnterior = micros(); // Inicialización correcta para evitar saltos en dt
  tiempo = millis();

  delay(5000);
}

void loop() {
    unsigned long tiempoActual = micros();
    float dt = (tiempoActual - tiempoAnterior) / 1000000.0;
  
    if (dt >= 0.001) {
      tiempoAnterior = tiempoActual; // Actualizar tiempoAnterior aquí fija el dt correctamente

      float media = ((float)encoder1Pulsos + (float)encoder2Pulsos) / 2.0;
      float distancia = media * 0.01067;

      //Serial.println(distancia);

      float Kp = 2.0;
      float Kd = 0.1;
      float Ki = 0.0;

      float error = distancianecesaria - distancia;

      derivativo = (error - erroranterior) / dt;
           
      integral += error * dt;
      integral = constrain(integral, -60, 60); 

      float pid = (Kp * error) + (Kd * derivativo) + (Ki * integral);

      float pidD = pid;   // Corrección motor derecho
      float pidI = pid;   // Corrección motor izquierdo

      if (abs(error) > 0.3) {
          // Incrementa el límite máximo gradualmente usando el valor de aceleración y dt
          if (maximo_actual < maximo_final) {
              maximo_actual += aceleracion * dt;
              if (maximo_actual > maximo_final) {
                  maximo_actual = maximo_final;
              }
          }

          // Se aplica 'maximo_actual' en lugar del 30 fijo anterior
          setMotor(0, 1, pidD, (int)maximo_actual, 1.128);   // Rueda Derecha
          setMotor(2, 3, pidI, (int)maximo_actual, 1.0);     // Rueda Izquierda
              
      } else {
          setMotor(0, 1, 0, 0, 0.0);
          setMotor(2, 3, 0, 0, 0.0);
          maximo_actual = 0.0; // Resetea la rampa para el próximo movimiento
      }
        
      erroranterior = error; // Guardar el error al final del cálculo
      tiempo = millis();
    }
}
