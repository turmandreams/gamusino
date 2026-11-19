
boolean simulador=false;


//IR


int valorcon[4];
int valorsin[4];

#define pinIR_IA 36
#define pinIR_DW 39
#define pinIR_IW 34
#define pinIR_DA 35

#define pinIR_I 32
#define pinIR_D 33



//succion

#define PIN_SUCCION   12  
#define canalSuccion  4  // va a ser el canal 4 del pwm


//ws2812

#include <Adafruit_NeoPixel.h>

#define PIN_DATOS   27  // El pin donde conectaste el DIN
#define NUM_LEDS     4  // Tienes 4 LEDs conectados

// Inicializamos la tira de LEDs
Adafruit_NeoPixel leds(NUM_LEDS, PIN_DATOS, NEO_GRB + NEO_KHZ800);



//imu

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
double gzgrados=0.0;   // velocidad de giro medida por el giroscopio (grados/s), sin derivar el angulo

// Sesgos del giroscopio (dps)
float sesgoX = 0.0;
float sesgoY = 0.0;
float sesgoZ = 0.0;

float yawDeseado=0.0;

//motores

#define pinMI1 2
#define pinMI2 13
#define pinMD1 17
#define pinMD2 16

#define canalMI1 0
#define canalMI2 1
#define canalMD1 2
#define canalMD2 3

// --- Configuración de PWM ---
#define PWM_FREQ       15000
#define PWM_RESOLUTION 8     // 8 bits (Nota: Tu comentario decía 12 bits, pero el valor es 8)


//encoders

#include <ESP32Encoder.h>

// Instancias para ambos encoders
ESP32Encoder encoderMI;
ESP32Encoder encoderMD;

// Pines para el Encoder 1
const int pinencoderMIA = 22; 
const int pinencoderMIB = 21;

// Pines para el Encoder 2
const int pinencoderMDA = 25; 
const int pinencoderMDB = 26;

long contMIant = 0;
long contMDant = 0;

const double M_POR_P = 0.000005336; // Basado en diámetro de 20mm y 5888 pulsos/rev
const double METROS_POR_PULSO = 5.336; // Como se divide entre microsegundos lo multiplico por 1.000.000 , para luego pasarlo a m/s

double veldeseada = 0.8; // Velocidad objetivo en m/s (acepta valores negativos)
double velpermitida = 0.0;
double velinicial=0.0;
double aceleracion=2.0; //aceleracion m/s²
long t=0.0;

double velmediaMI=0.0;
double velmediaMD=0.0;
double velmedia=0.0;


struct PID {
  double kp = 0.0;
  double ki = 0.0;
  double kd = 0.0;
  double error=0.0;   
  double integral = 0.0;
  double errorant = 0.0;
  double instante = 0.0;
  int maximo=0.0;
  int maximointegral=0.0;
};



PID PIDMI;
PID PIDMD;
PID PIDYAW;

float x=0.0;
float y=0.0;

float xfin=0.0;
float yfin=0.0;


long instante=0; 

long tiempo=0;

int estado=0;
boolean para=false;
boolean sigue=true;
int terminado=0;

int menu=0;
int opcion=0;


void angulohaciaobjetivo() {
  float deltaX = xfin - x;
  float deltaY = yfin - y;
  
  if (sqrt(deltaX*deltaX + deltaY*deltaY) < 0.03) return;
  
  float radianes = atan2(deltaY, deltaX); 
  float grados = radianes * RAD_TO_DEG;

  yawDeseado=grados;
  
}


// =========================================================================
// TABLAS DE CALIBRACIÓN (PWM vs VELOCIDAD)
// =========================================================================

// --- MOTOR IZQUIERDO ---
const int NUM_PUNTOS_IZQ = 12;
const int pwmTablaIzq[NUM_PUNTOS_IZQ] = {
  0,  23,   25,   28,   31,   34,   38,   42,   48,   52,   63,   71
};
const float velTablaIzq[NUM_PUNTOS_IZQ] = {
  0.00, 0.85, 0.98, 1.09, 1.15, 1.28, 1.34, 1.44, 1.51, 1.59, 1.66, 1.67
};

// --- MOTOR DERECHO ---
const int NUM_PUNTOS_DER = 12;
const int pwmTablaDer[NUM_PUNTOS_DER] = {
  0,  24,   26,   28,   31,   34,   38,   43,   48,   54,   61,   73
};
const float velTablaDer[NUM_PUNTOS_DER] = {
  0.00, 0.82, 0.92, 1.03, 1.17, 1.31, 1.41, 1.50, 1.57, 1.65, 1.71, 1.79
};


int calcularPWM(float velObjetivo, const float velTabla[], const int pwmTabla[], int numPuntos) {
  
  if (velObjetivo <= 0.0) {  return 0;  }

  if (velObjetivo <= velTabla[1]) {  return pwmTabla[1];  }
  
  if (velObjetivo >= velTabla[numPuntos - 1]) {  return pwmTabla[numPuntos - 1]; }
  
  for (int i = 1; i < numPuntos - 1; i++) {
    if (velObjetivo >= velTabla[i] && velObjetivo <= velTabla[i + 1]) {      
      float proporcion = (velObjetivo - velTabla[i]) / (velTabla[i + 1] - velTabla[i]);
      float pwmCalculado = pwmTabla[i] + proporcion * (pwmTabla[i + 1] - pwmTabla[i]);
      return round(pwmCalculado);
    }
  }
  return 0;
}

int getPWMIzquierda(float velocidad) {
  return calcularPWM(velocidad, velTablaIzq, pwmTablaIzq, NUM_PUNTOS_IZQ);
}

int getPWMDerecha(float velocidad) {
  return calcularPWM(velocidad, velTablaDer, pwmTablaDer, NUM_PUNTOS_DER);
}



// Distancia (m) que recorre el robot desde que se activa el freno (frenazo) hasta que se para, segun la velocidad (m/s)
// que llevaba. Medida en el simulador: con el freno puesto, la distancia sale casi lineal con la velocidad (~0.1 s).
// Rehacer esta tabla con el robot real (frenar a varias velocidades y medir lo que avanza).
const int PWMMINGIRO=32;   // PWM minimo con el que se mueven las dos ruedas a la vez al girar parado
const int NUM_PUNTOS_FRENADA = 10;
const float velTablaFrenada[NUM_PUNTOS_FRENADA]  = {0.0, 0.378, 0.58, 0.90, 1.14, 1.265, 1.37, 1.468, 1.57, 1.63};
const float distTablaFrenada[NUM_PUNTOS_FRENADA] = {0.0, 0.031, 0.055, 0.090, 0.118, 0.130, 0.140, 0.152, 0.165, 0.169};

float distanciaFrenada(float velocidad) {

  if (velocidad <= 0.0) { return 0.0; }

  //por encima de la ultima medida se sigue con la pendiente de unos 0.1 m por m/s
  if (velocidad >= velTablaFrenada[NUM_PUNTOS_FRENADA - 1]) {
    return distTablaFrenada[NUM_PUNTOS_FRENADA - 1] + 0.1 * (velocidad - velTablaFrenada[NUM_PUNTOS_FRENADA - 1]);
  }

  for (int i = 0; i < NUM_PUNTOS_FRENADA - 1; i++) {
    if (velocidad >= velTablaFrenada[i] && velocidad <= velTablaFrenada[i + 1]) {
      float proporcion = (velocidad - velTablaFrenada[i]) / (velTablaFrenada[i + 1] - velTablaFrenada[i]);
      return distTablaFrenada[i] + proporcion * (distTablaFrenada[i + 1] - distTablaFrenada[i]);
    }
  }
  return 0.0;
}


void pwmmotor(int canal1,int canal2,int pwm,int maximo) {
  
  pwm = constrain(pwm,-maximo, maximo);  
  int min=30;
  if(veldeseada==0.0){ min=0; }
  //el valor de 30 es por que en ese valor nos aseguramos que los dos motores se pueden mover
  
  if (pwm < -min)      { ledcWrite(canal1,-pwm); ledcWrite(canal2, 0); }
  else if (pwm > min) { ledcWrite(canal1,0);ledcWrite(canal2,pwm); }
  else              { ledcWrite(canal1,0);ledcWrite(canal2, 0); }   
      
}

void frenazo() {

  digitalWrite(pinMI1,HIGH);digitalWrite(pinMI2,HIGH);
  digitalWrite(pinMD1,HIGH);digitalWrite(pinMD2,HIGH);
  ledcWrite(0,255);ledcWrite(1,255);
  ledcWrite(2,255);ledcWrite(3,255);   

}

void paramotores() {
  digitalWrite(pinMI1,LOW);digitalWrite(pinMI2,LOW);
  digitalWrite(pinMD1,LOW);digitalWrite(pinMD2,LOW);  
  ledcWrite(0,0);ledcWrite(1,0);
  ledcWrite(2,0);ledcWrite(3,0);   

}


void inicializaPID(PID &pide,double kp,double ki,double kd,int maximo,int maximointegral){
  pide.kp=kp;
  pide.ki=ki;
  pide.kd=kd;
  pide.integral=0.0;
  pide.errorant=0.0;  
  pide.instante=micros(); 
  pide.maximo=maximo;   
  pide.maximointegral=maximointegral;   
}

void modificaPID(PID &pide,double kp,double ki,double kd,int maximo,int maximointegral){
  pide.kp=kp;
  pide.ki=ki;
  pide.kd=kd;
  pide.maximo=maximo;   
  pide.maximointegral=maximointegral;   
}


void resetPID(PID &pide){
  pide.integral=0.0;
  pide.errorant=0.0; 
  pide.instante=micros();     
}


double calulaPID(PID &pide,double error,double instante){

  double dt=(instante-pide.instante)/1000000.0;
  pide.instante=instante;
  if(dt<=0.0) return 0;

  pide.integral += error * dt;
  pide.integral = constrain(pide.integral,-pide.maximointegral,pide.maximointegral); 

  double derivativa = (error - pide.errorant)/dt;
  pide.errorant = error;

  double salida = (pide.kp*error) + (pide.ki*pide.integral) + (pide.kd*derivativa);

  return constrain((int)salida,-pide.maximo,pide.maximo);
}


double calulaPIDderivada(PID &pide,double error,double instante,double derivada){

  double dt=(instante-pide.instante)/1000000.0;
  pide.instante=instante;
  if(dt<=0.0) return 0;

  pide.integral += error * dt;
  pide.integral = constrain(pide.integral,-pide.maximointegral,pide.maximointegral); 

  pide.errorant = error;

  double salida = (pide.kp*error) + (pide.ki*pide.integral) + (pide.kd*derivada);

  return constrain((int)salida,-pide.maximo,pide.maximo);
}



void inicializamotores(){

  ledcSetup(canalMI1,PWM_FREQ,PWM_RESOLUTION);
  ledcAttachPin(pinMI1,canalMI1);

  ledcSetup(canalMI2, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(pinMI2,canalMI2);

  ledcSetup(canalMD1, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(pinMD1,canalMD1);

  ledcSetup(canalMD2, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(pinMD2,canalMD2);
  
  ledcSetup(canalSuccion, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PIN_SUCCION,canalSuccion);

   
}

void inicializaencoders(){

  // Habilitar las resistencias de pull-up internas para todos los encoders
  ESP32Encoder::useInternalWeakPullResistors = puType::up;

  encoderMI.attachFullQuad(pinencoderMIA,pinencoderMIB);
  encoderMI.setCount(0);

  encoderMD.attachFullQuad(pinencoderMDA,pinencoderMDB);
  encoderMD.setCount(0);
  
  
  
}

void inicializaimu(){

  dev_spi.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SPI_CS);
  if (AccGyr.begin() != 0) {
    Serial.println("¡Error con LSM6DSR!");
    
    while (1) {  //si hay fallo parpadean los leds 
      cambiarleds(leds.Color(60,0,0)); delay(200);
      cambiarleds(0);                  delay(200);
    }    
  }

  // Activar sensores
  AccGyr.Enable_X(); // Acelerómetro
  AccGyr.Enable_G(); // Giroscopio

  // Configuración de escalas y ODR solicitados
  AccGyr.Set_G_FS(4000);       // Giroscopio: +/- 4000 dps
  AccGyr.Set_G_ODR(1666.0f);   // 1666 Hz

  AccGyr.Set_X_FS(4);          // Acelerómetro: +/- 4 g
  AccGyr.Set_X_ODR(1666.0f);   // 1666 Hz

  
    
}

void calibraimu(){
 
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
  anguloZ = 0;
  
}


void actualizaanguloz(double dt){

    int32_t gyro_raw[3];
  //  int32_t accel_raw[3];

    AccGyr.Get_G_Axes(gyro_raw);
  //  AccGyr.Get_X_Axes(accel_raw);

    float gx = (gyro_raw[0] / 1000.0) - sesgoX;
    float gy = (gyro_raw[1] / 1000.0) - sesgoY;
    float gz = (gyro_raw[2] / 1000.0) - sesgoZ;

  //  float ax = accel_raw[0] / 1000.0;
  //  float ay = accel_raw[1] / 1000.0;
  //  float az = accel_raw[2] / 1000.0;

  //  float accelAnguloX = atan2(ay, az) * 180.0 / M_PI;
  //  float accelAnguloY = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI;

  //  anguloX = ALFA * (anguloX + gx * dt) + (1.0 - ALFA) * accelAnguloX;
  //  anguloY = ALFA * (anguloY + gy * dt) + (1.0 - ALFA) * accelAnguloY;

   // Serial.println(gz);
    
    gzgrados=gz;
    anguloZ -= (gz*dt)/1000000.0;
    
}


float angulozaradianes() {  return anguloZ * (PI / 180.0); }

void calculaxy(float numpasos){
    float distanciarecorrida=numpasos*M_POR_P;
    float radianes=angulozaradianes();
    
    x += distanciarecorrida * cos(radianes);
    y += distanciarecorrida * sin(radianes);  
}


void inicializaleds(){
  leds.begin();       
  leds.setBrightness(30);
  leds.show();             
}

void cambiarleds(uint32_t color) {
  for(int i=0; i<NUM_LEDS; i++) {
    leds.setPixelColor(i, color);
  }
  leds.show();  
}

void resetleds() {
  for(int i=0; i<NUM_LEDS; i++) {
    leds.setPixelColor(i,leds.Color(0,0,0));
  }  
}

void enciendeleds(boolean l4,boolean l3,boolean l2,boolean l1,uint32_t color){

     if(l1){ leds.setPixelColor(0,color); }else{ leds.setPixelColor(0,leds.Color(0,0,0)); }
     if(l2){ leds.setPixelColor(1,color); }else{ leds.setPixelColor(1,leds.Color(0,0,0)); }
     if(l3){ leds.setPixelColor(2,color); }else{ leds.setPixelColor(2,leds.Color(0,0,0)); }
     if(l4){ leds.setPixelColor(3,color); }else{ leds.setPixelColor(3,leds.Color(0,0,0)); }

     leds.show();
}

void apagaleds(){
    resetleds();
    leds.show();  
}

void parpadeoleds(boolean l4,boolean l3,boolean l2,boolean l1,uint32_t color){

   apagaleds();delay(100);    
   enciendeleds(l4,l3,l2,l1,leds.Color(0,0,20));delay(100); 
   apagaleds();delay(100);    
   enciendeleds(l4,l3,l2,l1,leds.Color(0,0,20));delay(100); 
   apagaleds();delay(100);    
     
}

void inicializaIR(){
  
    pinMode(pinIR_IA,INPUT);
    pinMode(pinIR_DW,INPUT);
    pinMode(pinIR_IW,INPUT);
    pinMode(pinIR_DA,INPUT);
    
    pinMode(pinIR_I,OUTPUT);
    pinMode(pinIR_D,OUTPUT);

   

}

void pruebaencoders(){  //menu=0 opcion=0

   parpadeoleds(0,0,0,1,leds.Color(0,0,20));  
   
   t=millis();
    
   while(millis()-t<10000){

    resetleds();
       
    long contMI = encoderMI.getCount();
    long difMI = contMI - contMIant;
    contMIant=contMI;
    
    long contMD = encoderMD.getCount();
    long difMD = contMD - contMDant;
    contMDant=contMD;

    if(difMD>400){
        leds.setPixelColor(0,leds.Color(0,20,0));leds.show();
    }else if(difMD<-400){
        leds.setPixelColor(1,leds.Color(0,20,0));leds.show();
    }

    if(difMI>400){
        leds.setPixelColor(3,leds.Color(0,20,0));leds.show();
    }else if(difMI<-400){
        leds.setPixelColor(2,leds.Color(0,20,0));leds.show();
    }

    
    delay(100);
    
   }

   parpadeoleds(0,0,0,1,leds.Color(0,0,20));
   
}


void pruebagz(){  //menu=0 opcion=1

  
   parpadeoleds(0,0,1,0,leds.Color(0,0,20));

   cambiarleds(leds.Color(20,0,0));
   delay(3000);
   calibraimu(); 

   cambiarleds(leds.Color(0,0,20));
   delay(1000);

      
   tiempo=millis();

   sigue=true;

   anguloZ=0.0;
   yawDeseado=0.0;

   inicializaPID(PIDYAW,4.0,0.0,0.4,40,150); 
    
   while(sigue){

      double ahora=micros();        
      double dt=ahora-instante;
        
      if (dt >= 100) {   //se revisa cada 100 us, se revisa que los motores vayan a la velocidad que tienen que ir    
          
          instante=ahora;   
         
          actualizaanguloz(dt);
          
          int turn=calulaPID(PIDYAW,yawDeseado-anguloZ,ahora);
                  
          pwmmotor(canalMI1,canalMI2,turn,255);  
          pwmmotor(canalMD1,canalMD2,-turn,255);
          
      
      }

      if((millis()-tiempo)>2000) {
          yawDeseado+=90;
          if(yawDeseado>360){
            
              sigue=false;
          }
          tiempo=millis();
      }

   }

   resetleds();

   parpadeoleds(0,0,1,0,leds.Color(0,0,20));
   
}


void pruebamotores() {  //menu=0 opcion=2
  
   parpadeoleds(0,0,1,1,leds.Color(0,0,20));   
   
   cambiarleds(leds.Color(20,0,0));
   delay(3000);

   ledcWrite(0,0);ledcWrite(1,40);delay(1000);
   ledcWrite(1,0);ledcWrite(0,40);delay(1000);
   ledcWrite(0,0);
   
   ledcWrite(2,0);ledcWrite(3,40);delay(1000);
   ledcWrite(3,0);ledcWrite(2,40);delay(1000);
   
   ledcWrite(2,0); 
   
   parpadeoleds(0,0,1,1,leds.Color(0,0,20));
   
}


void calibrarimu(){  //menu=0 opcion=3

  
   parpadeoleds(0,1,0,0,leds.Color(0,0,20));  

   cambiarleds(leds.Color(20,0,0));
   delay(3000);
   calibraimu(); 

   cambiarleds(leds.Color(0,0,20));
   delay(1000);

   resetleds();
   
   parpadeoleds(0,1,0,0,leds.Color(0,0,20));
   
}


void pruebasuccion(){  //menu=0 opcion=4

   parpadeoleds(0,1,0,1,leds.Color(0,0,20));
   
   cambiarleds(leds.Color(20,0,0));
   delay(3000);
   
   ledcWrite(4,60);
   delay(5000);

   ledcWrite(4,0);
      
   parpadeoleds(0,1,0,1,leds.Color(0,0,20));
   
   
}

void comprobarIR(){  //menu=0 opcion=5

   parpadeoleds(0,1,1,0,leds.Color(0,0,20));

   
   t=millis();
    
   while(millis()-t<15000){

      resetleds();leds.show();         
      
      digitalWrite(pinIR_I,LOW);
      digitalWrite(pinIR_D,LOW);
      delay(1);

    
      valorsin[0]=analogRead(pinIR_IA);
      valorsin[1]=analogRead(pinIR_DW);
      valorsin[2]=analogRead(pinIR_IW);
      valorsin[3]=analogRead(pinIR_DA);

    
      digitalWrite(pinIR_I,HIGH);
      digitalWrite(pinIR_D,HIGH);

      delay(1);

      valorcon[0]=analogRead(pinIR_IA);
      valorcon[1]=analogRead(pinIR_DW);
      valorcon[2]=analogRead(pinIR_IW);
      valorcon[3]=analogRead(pinIR_DA);
    
      if((valorcon[0]-valorsin[0])>3500) { leds.setPixelColor(0,leds.Color(0,20,0));leds.show(); }
      if((valorcon[1]-valorsin[1])>3500) { leds.setPixelColor(1,leds.Color(0,20,0));leds.show(); }
      if((valorcon[2]-valorsin[2])>3500) { leds.setPixelColor(2,leds.Color(0,20,0));leds.show(); }
      if((valorcon[3]-valorsin[3])>3500) { leds.setPixelColor(3,leds.Color(0,20,0));leds.show(); }
      
   }

      
   parpadeoleds(0,1,1,0,leds.Color(0,0,20));
   
   
}


void pruebarecorredistancia(){


   if(!simulador){    

       parpadeoleds(0,1,1,1,leds.Color(0,0,20));
    
       cambiarleds(leds.Color(20,0,0));
       delay(3000);
          
       calibraimu(); 
     
       cambiarleds(leds.Color(0,0,20));
       delay(1000);
       
       cambiarleds(leds.Color(20,0,0));  //se pone en rojo por que empieza a andar

   }

  inicializaPID(PIDMI,10,0.005,0.0,255,150);
  inicializaPID(PIDMD,10,0.005,0.0,255,150);
  
  //inicializaPID(PIDYAW,4.0,0.0,0.4,40,150);
  //inicializaPID(PIDYAW,2.0,0.0,0.2,15,150);
  inicializaPID(PIDYAW,8.0,1.0,0.3,40,150); 

  veldeseada=1.6;
  aceleracion=50.0;
  velinicial=0.0;
  yawDeseado=0.0;
  anguloZ=0.0;
  x=0.0;
  y=0.0;
  xfin=1.62;
  yfin=0.0;
  estado=0;

  boolean sigue=true;

  //le mandamos un pulso a los motores para provocar que se muevan
  pwmmotor(canalMI1,canalMI2,100,255);  
  pwmmotor(canalMD1,canalMD2,100,255);
 
  t=millis();
  tiempo=millis();

  long contMI = encoderMI.getCount();contMIant = contMI;      
  long contMD = encoderMD.getCount();contMDant = contMD;

  velmediaMI=0.0;velmediaMD=0.0;velmedia=0.0;

  double ahora=micros();instante=ahora; 

  while(sigue){

    ahora=micros();        
    double dt=ahora-instante;
      
    if (dt >= 100) {   //se revisa cada 100 us, se revisa que los motores vayan a la velocidad que tienen que ir    
        
        instante=ahora;   

        //se realiza el control de velocidad MI y MD
        //se obtiene pwmMI y pwmMD
          
        contMI = encoderMI.getCount();
        long numMI = contMI - contMIant;
        contMIant = contMI;
      
        contMD = encoderMD.getCount();
        long numMD = contMD - contMDant;
        contMDant = contMD;
      
        double velMI = (numMI * METROS_POR_PULSO) / dt;
        double velMD = (numMD * METROS_POR_PULSO) / dt;
    
        velmediaMI=(velmediaMI*0.8)+(velMI*0.2);
        velmediaMD=(velmediaMD*0.8)+(velMD*0.2);

        velmedia=(velmediaMI+velmediaMD)/2;

        velpermitida=velinicial+((t*aceleracion)/1000);
        if(velpermitida>veldeseada) { velpermitida=veldeseada; }

       // velpermitida=veldeseada;

        int pwmbaseI=getPWMIzquierda(velpermitida);
        int pwmbaseD=getPWMDerecha(velpermitida);
    
        int pwmMI=pwmbaseI+(int)calulaPID(PIDMI,velpermitida-velmediaMI,ahora);
        int pwmMD=pwmbaseD+(int)calulaPID(PIDMD,velpermitida-velmediaMD,ahora);

        if(veldeseada==0.0){
            pwmMI=0;
            pwmMD=0;          
        }
          
        
        actualizaanguloz(dt);
        calculaxy((numMI+numMD)/2.0);
        
        if(estado==0){
          angulohaciaobjetivo();
        }
        
        int turn=calulaPIDderivada(PIDYAW,yawDeseado-anguloZ,ahora,gzgrados);   //d(error)/dt = +gz

        if(veldeseada==0.0){          
          if(fabs(yawDeseado-anguloZ)<0.1){ turn=0; }
          else if(turn>0 && turn<PWMMINGIRO){ turn=PWMMINGIRO; }
          else if(turn<0 && turn>-PWMMINGIRO){ turn=-PWMMINGIRO; }
        }

        if(!para){
          pwmmotor(canalMI1,canalMI2,pwmMI+turn,255);  
          pwmmotor(canalMD1,canalMD2,pwmMD-turn,255);
        }else{
          pwmmotor(canalMI1,canalMI2,0,0);  
          pwmmotor(canalMD1,canalMD2,0,0);
        }
          


    }
    

     // Serial.println(pwmMI);
  
     if(estado==0){
        //se frena cuando lo que falta hasta xfin es lo que recorre al frenar a esta velocidad
        double vmedia=(velmediaMI+velmediaMD)/2.0;
        if(x+distanciaFrenada(vmedia)>=xfin){
            frenazo();
            veldeseada=0.0;
            estado++;   
            cambiarleds(leds.Color(0,30,0));   //se ha pasado de la distancia       
    
            tiempo=millis();  
            t=millis();
            velinicial=0.0;
            veldeseada=0.0;
            yawDeseado=0.0;
            terminado=0;   
                      
        }
        
     }else if(estado==1){
  
        if(abs(yawDeseado-anguloZ)<0.5){    
          if(abs(velmedia)<0.2){  
            terminado++;
            if(terminado>30){            
              yawDeseado+=180.0;
              estado++; 
              terminado=0; 
            }        
          }else{ terminado=0;}
        }else{ terminado=0;}
                  
     }else if(estado==2){
        if(abs(yawDeseado-anguloZ)<0.5){    
          if(abs(velmedia)<0.2){
            terminado++;
            if(terminado>30){ 
              frenazo();  
              estado++; 
              cambiarleds(leds.Color(0,0,30));         
              sigue=false;  
            }         
           }else{ terminado=0;}
        }else{ terminado=0;}
                  
     }   
   
     
  }  

  paramotores();

  if(!simulador){ parpadeoleds(0,1,1,1,leds.Color(0,0,20)); }
  
}


void setup() {
  Serial.begin(115200);
  while (!Serial);

  inicializaleds();
  inicializamotores();    
  inicializaencoders();
  inicializaimu();
  inicializaIR();

  cambiarleds(leds.Color(20,0,0));delay(300);
  cambiarleds(leds.Color(0,20,0));delay(300);
  cambiarleds(leds.Color(0,0,20));delay(300);

   
  tiempo=millis();
   

  if(simulador) { calibraimu();}
  
}

void loop() {    

    if(simulador) {
       pruebarecorredistancia();
       return;       
    }

    boolean entra=false;
     
    long contMI = encoderMI.getCount();
    long difMI = contMI - contMIant;
    
    long contMD = encoderMD.getCount();
    long difMD = contMD - contMDant;
    
    Serial.print("Dif MI : ");Serial.println(difMI);
    Serial.print("Dif MD : ");Serial.println(difMD);

    if(difMD>1000) { 
        if(menu==0){
          opcion++; if(opcion>6){ opcion=0; }
        }
    }else if(difMD<-1000) { 
        if(menu==0){
          opcion--; if(opcion<0){ opcion=6; }
        }
    }

    if(difMI>1000) { 
        entra=true;
    }else if(difMD<-1000) { 
      /*
        if(menu==0){ menu=1;}
        else { menu=0; }
      */
    }

    
    resetleds();

    if(menu==0){  ///Menu para ejecutar pruebas
        uint32_t color=leds.Color(0,0,20);
              
        if(opcion==0){    ///Prueba de encoders

            enciendeleds(0,0,0,1,color);            
            if(entra){ pruebaencoders(); }
            
        }else if(opcion==1){    ///Prueba de imu
          
            enciendeleds(0,0,1,0,color);                 
            if(entra){ pruebagz(); }
               
        }else if(opcion==2){    ///Prueba de motores
            enciendeleds(0,0,1,1,color);                    
            leds.show();  
            if(entra){ pruebamotores(); }        
        }else if(opcion==3){    ///Calibrar IMU
            enciendeleds(0,1,0,0,color);            
            if(entra){ calibrarimu(); }          
        }else if(opcion==4){    ///prueba succion
            enciendeleds(0,1,0,1,color);            
            if(entra){ pruebasuccion(); }          
        }else if(opcion==5){    ///comprobar IR
            enciendeleds(0,1,1,0,color);            
            if(entra){ comprobarIR(); } 
        }else if(opcion==6){    ///prueba correrdistancia
            enciendeleds(0,1,1,1,color);            
            if(entra){ pruebarecorredistancia(); }         
        }          
      
     }

    contMIant = encoderMI.getCount();
    contMDant = encoderMD.getCount();
    delay(200);
    
}
