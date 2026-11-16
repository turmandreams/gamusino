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


#define MOTOR_LEFT_IN1  2
#define MOTOR_LEFT_IN2  13
#define MOTOR_RIGHT_IN1 17
#define MOTOR_RIGHT_IN2 16

// --- Configuración de PWM ---
#define PWM_FREQ       15000
#define PWM_RESOLUTION 8     // 12 bits

int tiempo=0;

float erroranterior=0;
float integral=0.0;

unsigned long tiempoAnterior = 0;

// Filtro complementario
const float ALFA = 0.9999;

float correcionY=6.18;

// --- Giro (yaw) ---
float yawDeseado = 0.0;          // ángulo objetivo (grados)



#define numpixeles 4
int pinleds=27;
boolean esperarleds=false;

struct LED {
    byte R; 
    byte G; 
    byte B; 
};

struct LED led[numpixeles];


void resetLeds(){
   int i;
   for(i=0; i < numpixeles; i++){
      led[i].R = 0;
      led[i].G = 0;
      led[i].B = 0;   
   }
}

void setColorLed(int i,int R,int G,int B){
   led[i].R=R;
   led[i].G=G;
   led[i].B=B;
}


void actualizaLeds(){

   esperarleds=true;
   
   noInterrupts();
   
   int i,j;
        
    
   for(i=0;i<numpixeles;i++){
    
      for(j=0;j<8;j++)
      {      
         if(bitRead(led[i].G,j)){
             digitalWrite(pinleds,HIGH);
             asm volatile ("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n");
             asm volatile ("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n");
             asm volatile ("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n");
             asm volatile ("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n");
             asm volatile ("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n");
             digitalWrite(pinleds,LOW);
         }else{
             digitalWrite(pinleds,HIGH);
             digitalWrite(pinleds,LOW);
             asm volatile ("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n");
             asm volatile ("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n");
             asm volatile ("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n");
         }
      }
      for(j=0;j<8;j++)
      {      
         if(bitRead(led[i].R,j)){
             digitalWrite(pinleds,HIGH);
             asm volatile ("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n");
             asm volatile ("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n");
             asm volatile ("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n");
             asm volatile ("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n");
             asm volatile ("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n");
             digitalWrite(pinleds,LOW);
         }else{
             digitalWrite(pinleds,HIGH);                         
             digitalWrite(pinleds,LOW);
             asm volatile ("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n");
             asm volatile ("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n");
             asm volatile ("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n");
         }
      }
      for(j=0;j<8;j++)
      {
         if(bitRead(led[i].B,j)){
             digitalWrite(pinleds,HIGH);
             asm volatile ("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n");
             asm volatile ("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n");
             asm volatile ("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n");
             asm volatile ("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n"); 
             asm volatile ("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n");
             digitalWrite(pinleds,LOW);
         }else{
             digitalWrite(pinleds,HIGH);
             digitalWrite(pinleds,LOW);
             asm volatile ("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n");
             asm volatile ("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n");
             asm volatile ("nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n");
         }
      } 
   }

   
   interrupts();

   esperarleds=false;
}


void setMotor(int chIN1, int chIN2, float v) {
  v = constrain(v, -255,255);  
  if (v < 0)      { ledcWrite(chIN1,(uint32_t)(-v)); ledcWrite(chIN2,0); }
  else if (v > 0) { ledcWrite(chIN1,0); ledcWrite(chIN2,(uint32_t)v); }
  else            { ledcWrite(chIN1,0); ledcWrite(chIN2,0); }
}


void setup() {
  Serial.begin(115200);
  while (!Serial);

  pinMode(pinleds,OUTPUT);

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
  delay(3000);
  
  setColorLed(0,10,0,0);
  setColorLed(1,10,0,0);
  setColorLed(2,10,0,0);
  setColorLed(3,10,0,0);    
  actualizaLeds(); 

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

  Serial.println("Calibración completada.");
  Serial.println("Envía un ángulo (ej. 90) + Enter para girar.");
  tiempoAnterior = micros();


  ledcSetup(0, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_LEFT_IN1,0);

  ledcSetup(1, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_LEFT_IN2,1);

  ledcSetup(2, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_RIGHT_IN1,2);

  ledcSetup(3, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_RIGHT_IN2,3);

  tiempo=millis();

  setColorLed(0,0,0,10);
  setColorLed(1,0,0,10);
  setColorLed(2,0,0,10);
  setColorLed(3,0,0,10);    
  actualizaLeds(); 

/*
  delay(5000);

  ledcWrite(0,0);ledcWrite(1,2000);   //Izquierda Adelante
  delay(1000);
  
  ledcWrite(0,0);ledcWrite(1,0);     //Derecha Adelante
  ledcWrite(2,0);ledcWrite(3,2000);  
  delay(1000);

  
  ledcWrite(2,0);ledcWrite(3,0);     //Izquierda Atras
  ledcWrite(1,0);ledcWrite(0,2000);  
  delay(1000);

  ledcWrite(1,0);ledcWrite(0,0);    //Derecha Atras
  ledcWrite(3,0);ledcWrite(2,2000);  
  delay(1000);
  
  ledcWrite(3,0);ledcWrite(2,0);
  delay(1000);

  */
  
}


void loop() {
  
  unsigned long tiempoActual = micros();
  float dt = (tiempoActual - tiempoAnterior) / 1000000.0;

  // Muestreo: ejecutar cada 1 ms (~1000 Hz)
  if (dt >= 0.001) {
    tiempoAnterior = tiempoActual;

    int32_t gyro_raw[3];
  //  int32_t accel_raw[3];

    AccGyr.Get_G_Axes(gyro_raw);
  //  AccGyr.Get_X_Axes(accel_raw);

    // 1. Giroscopio a dps restando sesgo
    float gx = (gyro_raw[0] / 1000.0) - sesgoX;
    float gy = (gyro_raw[1] / 1000.0) - sesgoY;
    float gz = (gyro_raw[2] / 1000.0) - sesgoZ;

    // 2. Acelerómetro a G
  //  float ax = accel_raw[0] / 1000.0;
  //  float ay = accel_raw[1] / 1000.0;
  //  float az = accel_raw[2] / 1000.0;

    // 3. Ángulos directos del acelerómetro
  //  float accelAnguloX = atan2(ay, az) * 180.0 / M_PI;
  //  float accelAnguloY = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI;

    // 4. Filtro complementario para X e Y
  //  anguloX = ALFA * (anguloX + gx * dt) + (1.0 - ALFA) * accelAnguloX;
  //  anguloY = ALFA * (anguloY + gy * dt) + (1.0 - ALFA) * accelAnguloY;

    // 5. Integración directa para Z (Yaw)
    anguloZ += gz * dt;

    // Mantenimiento de Z en el rango [-180°, 180°]
    //if (anguloZ > 180.0) anguloZ -= 360.0;
    //if (anguloZ < -180.0) anguloZ += 360.0;

    if((millis()-tiempo)>3000){
        yawDeseado+=90;
        tiempo=millis();
    }

    const float KpYaw    = 10.0;     // por grado de error de yaw
    const float KdYaw    = 0.7;      // sobre gz (dps) para frenar el giro
    const float TURN_MAX = 90;      // máxima corrección de giro (de 4000)
    const float YAW_SIGN = 1.0;      // pon -1.0 si gira al revés

    // --- Control de giro: error de yaw por el camino más corto ---
    float errYaw = yawDeseado - anguloZ;
   // while (errYaw > 180.0)  errYaw -= 360.0;
   // while (errYaw < -180.0) errYaw += 360.0;

    float turn = YAW_SIGN * (KpYaw * errYaw - KdYaw * gz);
    turn = constrain(turn, -TURN_MAX, TURN_MAX);

    //Serial.println(anguloZ);
    
    Serial.println(errYaw);

    //Serial.println(gz);
       
    if(abs(errYaw)>7){

       float pidD=(-turn*100)/100;   // Corrección motor derecho
       float pidI=(turn*100)/100;    // Corrección motor izquierdo
      
       // Equilibrio + giro diferencial
       setMotor(0, 1, pidD);   // Rueda Derecha
       setMotor(2, 3, pidI);   // Rueda Izquierda

/*
      if(errYaw>0){
        setMotor(0,1,-20);
        setMotor(2,3,20);
      }else{
        setMotor(0,1,20);
        setMotor(2,3,-20);
      }
*/      
      
    }else{

               
        ledcWrite(0,0);
        ledcWrite(1,0);
        ledcWrite(2,0);
        ledcWrite(3,0);

       
    }
    
  }

}
