//Succion

#define PIN_SUCCION   12


//WS2812

#include <Adafruit_NeoPixel.h>

#define PIN_DATOS   27  // El pin donde conectaste el DIN
#define NUM_LEDS     4  // Tienes 4 LEDs conectados

// Inicializamos la tira de LEDs
Adafruit_NeoPixel tira(NUM_LEDS, PIN_DATOS, NEO_GRB + NEO_KHZ800);


//IMU

#include <SPI.h>
#include <LSM6DSRSensor.h>

#define SPI_SCK  18
#define SPI_MISO 19
#define SPI_MOSI 23
#define SPI_CS   5

SPIClass dev_spi(VSPI);
LSM6DSRSensor AccGyr(&dev_spi, SPI_CS);

// Ángulos finales filtrados (grados)
float anguloX = 0.0; // Roll
float anguloY = 0.0; // Pitch
float anguloZ = 0.0; // Yaw

// Sesgos del giroscopio (dps)
float sesgoX = 0.0;
float sesgoY = 0.0;
float sesgoZ = 0.0;


float yawDeseado = 0.0;          // ángulo objetivo (grados)

float turn=0;

float dt=0.0;

//motores

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

long encoder1Pulsos=1;
long encoder2Pulsos=1;

float erroranterior=0.0;
float derivativo=0.0;
float integral=0.0;

long tiempo=0;
long tiempoAnterior=0; // Asegúrate de inicializar esto para el cálculo de dt

float distancianecesaria=500.0;


// === NUEVAS VARIABLES PARA ACELERACIÓN ===
float aceleracion = 10.0;     // Unidades de PWM por segundo (puedes variar este valor)
float maximo_actual = 0.0;    // Comienza en 0 para la rampa de aceleración
float maximo_final = 40.0;    // El límite de velocidad que tenías originalmente (30)

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


void calculaturn(){

    int32_t gyro_raw[3];
  
    AccGyr.Get_G_Axes(gyro_raw);
  
  //  float gx = (gyro_raw[0] / 1000.0) - sesgoX;
  //  float gy = (gyro_raw[1] / 1000.0) - sesgoY;
    float gz = (gyro_raw[2] / 1000.0) - sesgoZ;

    anguloZ += gz * dt;

    
    const float KpYaw    = 10.0;     // por grado de error de yaw
    const float KdYaw    = 0.7;      // sobre gz (dps) para frenar el giro
    const float TURN_MAX = 40;      // máxima corrección de giro (de 4000)
    const float YAW_SIGN = 1.0;      // pon -1.0 si gira al revés

    float errYaw = yawDeseado - anguloZ;

    turn = YAW_SIGN * (KpYaw * errYaw - KdYaw * gz);
    turn = constrain(turn, -TURN_MAX, TURN_MAX);

    if(abs(errYaw)<=0.1){ turn=0.0;  }
}

// Función auxiliar para iluminar toda la tira de un solo golpe
void cambiarColor(uint32_t color, int espera) {
  for(int i=0; i<NUM_LEDS; i++) {
    tira.setPixelColor(i, color);
  }
  tira.show();
  delay(espera);
}


void setMotor(int chIN1, int chIN2, float v, int maximo, float porcentaje) {
  v = constrain(v, -maximo, maximo);  
  v = v * porcentaje;
  v = constrain(v, -255,255);  
    
  if (v < 0)      { ledcWrite(chIN1, (uint32_t)(-v)); ledcWrite(chIN2, 0); }
  else if (v > 0) { ledcWrite(chIN1, 0); ledcWrite(chIN2, (uint32_t)v); }
  else            { ledcWrite(chIN1, 0); ledcWrite(chIN2, 0); }
}

void setup() {
  
  Serial.begin(115200);
  while (!Serial);

  tira.begin();           // Inicializa el hardware de los LEDs
  
  cambiarColor(tira.Color(10,10,10),10);
  
  dev_spi.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SPI_CS);
  if (AccGyr.begin() != 0) {
    Serial.println("¡Error con LSM6DSR!");
    while (1);
  }

  // Activar sensores
  AccGyr.Enable_X(); // Acelerómetro
  AccGyr.Enable_G(); // Giroscopio

  // Configuración de escalas y ODR solicitados
  AccGyr.Set_G_FS(4000);       // Giroscopio: +/- 4000 dps
  AccGyr.Set_G_ODR(1666.0f);   // 1666 Hz

  AccGyr.Set_X_FS(4);          // Acelerómetro: +/- 4 g
  AccGyr.Set_X_ODR(1666.0f);   // 1666 Hz

  // --- Calibración Inicial ---
  Serial.println("Calibrando sesgo... Mantén el sensor completamente quieto.");
  cambiarColor(tira.Color(10,0,0),10);
  delay(5000);
  
 
  const int muestras = 2000;
  int32_t gyro_raw[3];
  double sumaX = 0, sumaY = 0, sumaZ = 0;

  for (int i = 0; i < muestras; i++) {
    AccGyr.Get_G_Axes(gyro_raw);
    sumaX += (gyro_raw[0] / 1000.0);
    sumaY += (gyro_raw[1] / 1000.0);
    sumaZ += (gyro_raw[2] / 1000.0);
    delayMicroseconds(600);
  }

  sesgoX = sumaX / (double)muestras;
  sesgoY = sumaY / (double)muestras;
  sesgoZ = sumaZ / (double)muestras;

  // Ángulo inicial estimado desde el acelerómetro
  int32_t accel_raw[3];
  AccGyr.Get_X_Axes(accel_raw);
  float ax = accel_raw[0] / 1000.0;
  float ay = accel_raw[1] / 1000.0;
  float az = accel_raw[2] / 1000.0;

  anguloX = atan2(ay, az) * 180.0 / M_PI;
  anguloY = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI;


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


//  ledcSetup(5,500,PWM_RESOLUTION);
//  ledcAttachPin(PIN_SUCCION,5);

  cambiarColor(tira.Color(0,0,10),10);

//  ledcWrite(5,80);
  
  delay(5000);
  
}

void loop() {
  
    unsigned long tiempoActual = micros();
    dt = (tiempoActual - tiempoAnterior) / 1000000.0;
  
    if (dt >= 0.001) {
      tiempoAnterior = tiempoActual; // Actualizar tiempoAnterior aquí fija el dt correctamente

      float media = ((float)encoder1Pulsos + (float)encoder2Pulsos) / 2.0;
      float distancia = media * 0.01067;

      float p1=((float)encoder2Pulsos/(float)encoder1Pulsos); if(p1>1.1){ p1=1.1;}
      float p2=((float)encoder1Pulsos/(float)encoder2Pulsos); if(p2>1.1){ p2=1.1;}
      
      //Serial.println(distancia);
      
      float error = distancianecesaria - distancia;

      if (abs(error) > 0.3) {

          calculaturn();

          float Kp = 1.0;
          float Kd = 0.01;
          float Ki = 0.0;

          derivativo = (error - erroranterior) / dt;
           
          integral += error * dt;
          integral = constrain(integral, -60, 60); 

          float pid = (Kp * error) + (Kd * derivativo) + (Ki * integral);

          turn=0.0;
          
          float pidD = pid+turn;   // Corrección motor derecho
          float pidI = pid-turn;   // Corrección motor izquierdo
          
          if (maximo_actual < maximo_final) {
              maximo_actual += aceleracion * dt;
              if (maximo_actual > maximo_final) {
                  maximo_actual = maximo_final;
              }
          }

          int turnmax=abs(turn);
          
          // Se aplica 'maximo_actual' en lugar del 30 fijo anterior
          setMotor(0, 1,pidD,(int)maximo_actual+turnmax, p1);   // Rueda Derecha
          setMotor(2, 3,pidI,(int)maximo_actual+turnmax, p2);     // Rueda Izquierda
              
      } else {
          //ledcWrite(5,0);
          cambiarColor(tira.Color(0,20,0),10);  
          setMotor(0, 1, 0, 0, 0.0);
          setMotor(2, 3, 0, 0, 0.0);
          maximo_actual = 0.0; // Resetea la rampa para el próximo movimiento
      }
        
      erroranterior = error; // Guardar el error al final del cálculo
      tiempo = millis();
    }
}
