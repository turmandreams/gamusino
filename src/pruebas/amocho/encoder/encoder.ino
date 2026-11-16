
#define MOTOR_LEFT_IN1  17
#define MOTOR_LEFT_IN2  16
#define MOTOR_RIGHT_IN1 2
#define MOTOR_RIGHT_IN2 13

// --- Configuración de PWM ---
#define PWM_FREQ       15000
#define PWM_RESOLUTION 8     // 8 bits (Nota: Tu comentario decía 12 bits, pero el valor es 8)

#define ENCODERMI_A  22
#define ENCODERMI_B  21
#define ENCODERMD_A  25
#define ENCODERMD_B  26



long contMI=0;
long contMD=0;

const double METROS_POR_PULSO = 0.0000112047; // Basado en diámetro de 21mm y 5888 pulsos/rev

double veldeseada = 0.0; // Velocidad objetivo en m/s (acepta valores negativos)

struct PID {
  double Kp = 0.0;
  double Ki = 0.0;
  double Kd = 0.0;   
  double integral = 0.0;
  double errorprev = 0.0;
};


PID PIDMI;
PID PIDMD;

long tiempo=0;
long tiempoAnterior=0; // Asegúrate de inicializar esto para el cálculo de dt


void IRAM_ATTR leerEncoderMI() {
  int estadoA = digitalRead(ENCODERMI_A);
  int estadoB = digitalRead(ENCODERMI_B);
  if (estadoA == estadoB) { contMI++; } else { contMI--; }
}

void IRAM_ATTR leerEncoderMD() {
  int estadoA = digitalRead(ENCODERMD_A);
  int estadoB = digitalRead(ENCODERMD_B);
  if (estadoA == estadoB) { contMD++; } else { contMD--; }
}

void setMotor(int chIN1, int chIN2, float pwm, int maximo) {
  
  pwm = constrain(pwm,-maximo, maximo);  
    
  if (pwm < 0)      { ledcWrite(chIN1, (uint32_t)(-pwm)); ledcWrite(chIN2, 0); }
  else if (pwm > 0) { ledcWrite(chIN1, 0); ledcWrite(chIN2, (uint32_t)pwm); }
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
  
  pinMode(ENCODERMI_A, INPUT);
  pinMode(ENCODERMI_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODERMI_A), leerEncoderMI, CHANGE);

  pinMode(ENCODERMD_A, INPUT);
  pinMode(ENCODERMD_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODERMD_A), leerEncoderMD, CHANGE);

  tiempoAnterior = micros(); // Inicialización correcta para evitar saltos en dt
  tiempo = millis();

 
}

void loop() {
  
    Serial.print("Contador MI : ");
    Serial.println(contMI);

    Serial.print("Contador MD : ");
    Serial.println(contMD);

    delay(100);
    
}
