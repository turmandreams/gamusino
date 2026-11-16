
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


void pwmmotor(int canal1,int canal2,int pwm,int maximo) {
  
  pwm = constrain(pwm,-maximo, maximo);  

  //el valor de 30 es por que en ese valor nos aseguramos que los dos motores se pueden mover
  
  if (pwm < -30)      { ledcWrite(canal1,-pwm); ledcWrite(canal2, 0); }
  else if (pwm > 30) { ledcWrite(canal1,0);ledcWrite(canal2,pwm); }
  else              { ledcWrite(canal1,0);ledcWrite(canal2, 0); }
    
  
    
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


void resetPID(PID &pide){
  pide.integral=0.0;
  pide.errorant=0.0; 
  pide.instante=micros();     
}


double calulaPID(PID &pide,double error,double instante){

  double dt=instante-pide.instante;
  
  pide.integral += error * dt;
  pide.integral = constrain(pide.integral,-pide.maximointegral,pide.maximointegral); 
  
  double derivativa = (error - pide.errorant)/dt;  
  pide.errorant = error;
    
  double salida = (pide.kp*error) + (pide.ki*pide.integral) + (pide.kd*derivativa);

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

void setup() {
  Serial.begin(115200);
  while (!Serial);

  inicializaleds();
  inicializamotores();    
  inicializaencoders();
  inicializaimu();

  cambiarleds(leds.Color(30,0,0));
  Serial.println("Se va a realizar la calibración !!!");
  delay(3000);

  calibraimu();  
  cambiarleds(leds.Color(0,0,30));
  Serial.println("Calibración realizada !!!");
    
  tiempo=millis();

  
  inicializaPID(PIDMI,10,0.005,0.0,255,150);
  inicializaPID(PIDMD,10,0.005,0.0,255,150);

  inicializaPID(PIDYAW,1.0,0.0,0.2,10,150);
  


  instante = micros(); // Inicialización correcta para evitar saltos en dt

  veldeseada=1.6;
  aceleracion=3.0;
  velinicial=0;
  yawDeseado=0.0;
  xfin=1.5;
  yfin=0.0;
  
  t=millis();
  
}

void loop() {
    
    double ahora=micros();        
    double dt=ahora-instante;
      
    if (dt >= 100) {   //se revisa cada 100 us, se revisa que los motores vayan a la velocidad que tienen que ir    
        
        instante=ahora;   

        //se realiza el control de velocidad MI y MD
        //se obtiene pwmMI y pwmMD
          
        long contMI = encoderMI.getCount();
        long numMI = contMI - contMIant;
        contMIant = contMI;
      
        long contMD = encoderMD.getCount();
        long numMD = contMD - contMDant;
        contMDant = contMD;
      
        double velMI = (numMI * METROS_POR_PULSO) / dt;
        double velMD = (numMD * METROS_POR_PULSO) / dt;
    
        velmediaMI=(velmediaMI*0.8)+(velMI*0.2);
        velmediaMD=(velmediaMD*0.8)+(velMD*0.2);

        velpermitida=velinicial+((t*aceleracion)/1000);
        if(velpermitida>veldeseada) { velpermitida=veldeseada; }

       // velpermitida=veldeseada;

        int pwmbaseI=getPWMIzquierda(veldeseada);
        int pwmbaseD=getPWMDerecha(veldeseada);
    
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
        
        int turn=calulaPID(PIDYAW,yawDeseado-anguloZ,ahora);

        
        if(!para){
          pwmmotor(canalMI1,canalMI2,pwmMI+turn,255);  
          pwmmotor(canalMD1,canalMD2,pwmMD-turn,255);
        }else{
          pwmmotor(canalMI1,canalMI2,0,0);  
          pwmmotor(canalMD1,canalMD2,0,0);
        }
          


    }
    

   // Serial.println(pwmMI);
            
    if((millis()-tiempo)>800){

        //pwmmotor(canalMI1,canalMI2,0,0);  
        //pwmmotor(canalMD1,canalMD2,0,0);

        //delay(2000);

        estado++;

        if(estado==1){
          inicializaPID(PIDYAW,4.0,0.0,0.4,40,150);
          t=millis();
          velinicial=velpermitida;
          veldeseada=0.0;         
        }else if(estado==2){                            
          velinicial=0.0;
          veldeseada=0.0;
          yawDeseado+=180;          
        }else if(estado<10){
          velinicial=0.0;
          veldeseada=0.0;          
        }else{          
          //para=true;
          //estado--;          
          inicializaPID(PIDYAW,1.0,0.0,0.2,10,150);
          estado=0;
          anguloZ=0.0;
          veldeseada=1.6;
          velinicial=0;
          yawDeseado=0.0;
          x=0.0;y=0.0;
          xfin=1.5;yfin=0.0;
          t=millis();          
        }

        Serial.print("Contador MI : ");
        Serial.println(encoderMI.getCount());
        Serial.print("Contador MD : ");
        Serial.println(encoderMD.getCount());
          
        Serial.print("X : ");
        Serial.println(x);
        Serial.print("Y : ");
        Serial.println(y);


        /*
        Serial.print("Velocidad MI : ");
        Serial.println(velmediaMI);
        Serial.print("Velocidad MD : ");
        Serial.println(velmediaMD);
        */
        
        Serial.print("\nYaw : ");
        Serial.println(anguloZ);
        Serial.print("\n");
        
                        
        tiempo=millis();

        
    }

   
    
}
