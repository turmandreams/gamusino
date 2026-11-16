// gamusino_1 - VERSIÓN CORREGIDA
// Mismo comportamiento que el original, solo con los fallos arreglados.
// Cada cambio va marcado con  // [FIX n]  (ver CAMBIOS.md)

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
#define PWM_RESOLUTION 8     // 8 bits -> PWM 0..255

// [FIX 10] en core 3.x se escribe por canal con ledcWriteChannel
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  #define ESCRIBE_PWM ledcWriteChannel
#else
  #define ESCRIBE_PWM ledcWrite
#endif


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

// [FIX 5] OJO: con rueda de 20 mm y 5888 pulsos/vuelta sale pi*0.020/5888 = 1.067e-5 m/pulso.
// 5.336e-6 corresponde a 11776 pulsos/vuelta (o rueda de 10 mm). Se deja el valor original
// hasta medirlo: empuja el robot 1 m a mano y mira cuántos pulsos cuenta.
const double M_POR_P = 0.000005336;
const double METROS_POR_PULSO = M_POR_P * 1000000.0; // [FIX 5] derivado de M_POR_P para que no se desincronicen

double veldeseada = 1.6; // Velocidad objetivo en m/s
double velpermitida = 0.0;
double velinicial=0.0;
double aceleracion=5.0; //aceleracion m/s²
unsigned long t=0;      // [FIX 9] tipo correcto para millis()

double velmediaMI=0.0;
double velmediaMD=0.0;

struct PID {
  double kp = 0.0;
  double ki = 0.0;
  double kd = 0.0;
  double error=0.0;
  double integral = 0.0;
  double errorant = 0.0;
  double instante = 0.0;     // en microsegundos
  int maximo=0;
  int maximointegral=0;
};



PID PIDMI;
PID PIDMD;
PID PIDYAW;

float x=0.0;
float y=0.0;

float xfin=0.0;
float yfin=0.0;


unsigned long instante=0; // [FIX 9]

unsigned long tiempo=0;   // [FIX 9]

int estado=0;
boolean para=false;


// [FIX 4] lleva cualquier ángulo a -180..180
float normaliza180(float a){
  while(a > 180.0)  a -= 360.0;
  while(a < -180.0) a += 360.0;
  return a;
}


void angulohaciaobjetivo() {
  float deltaX = xfin - x;
  float deltaY = yfin - y;

  // [FIX 6] cerca del objetivo atan2 de valores mínimos da direcciones al azar: no recalcular
  if (sqrt(deltaX*deltaX + deltaY*deltaY) < 0.03) return;

  float radianes = atan2(deltaY, deltaX);
  float grados = radianes * RAD_TO_DEG;

  yawDeseado=grados;
}


// [FIX 3] siempre devuelve un valor y respeta el signo de la velocidad
int veltopwmdef(float vel){
    float v = fabs(vel);
    int s = (vel < 0) ? -1 : 1;
    if (v < 0.001)       return 0;
    if(v<=0.4){          return s*10;  }
    else if(v<=0.6){     return s*20;  }
    else if(v<=1.0){     return s*30;  }
    else if(v<=1.3){     return s*40;  }
    else if(v<=1.4){     return s*50;  }
    else if(v<=1.5){     return s*60;  }
    else if(v<=1.6){     return s*70;  }
    else if(v<=1.65){    return s*80;  }
    else if(v<=1.70){    return s*90;  }
    return s*100;                       // por encima de 1.70 m/s
}

void pwmmotor(int canal1,int canal2,int pwm,int maximo) {

  pwm = constrain(pwm,-maximo, maximo);

  if (pwm < 0)      { ESCRIBE_PWM(canal1,-pwm); ESCRIBE_PWM(canal2, 0); }
  else if (pwm > 0) { ESCRIBE_PWM(canal1,0);ESCRIBE_PWM(canal2,pwm); }
  else              { ESCRIBE_PWM(canal1,0);ESCRIBE_PWM(canal2, 0); }

}

void inicializaPID(PID &pide,double kp,double ki,double kd,int maximo,int maximointegral){
  pide.kp=kp;
  pide.ki=ki;
  pide.kd=kd;
  pide.integral=0.0;
  pide.errorant=0.0;
  pide.instante=micros();   // [FIX 1] arranca con la hora actual, no con 0
  pide.maximo=maximo;
  pide.maximointegral=maximointegral;
}


void resetPID(PID &pide){
  pide.integral=0.0;
  pide.errorant=0.0;
  pide.instante=micros();
}


// [FIX 1] el PID se pasa POR REFERENCIA (&) para que guarde integral, error anterior e instante.
// dt se calcula en SEGUNDOS: ki y kd quedan en unidades normales (por segundo).
double calulaPID(PID &pide,double error,double ahora){

  double dt=(ahora-pide.instante)/1000000.0;
  pide.instante=ahora;
  if (dt <= 0.0) return 0.0;

  pide.integral += error * dt;
  pide.integral = constrain(pide.integral,-pide.maximointegral,pide.maximointegral);

  double derivativa = (error - pide.errorant)/dt;
  pide.errorant = error;

  double salida = (pide.kp*error) + (pide.ki*pide.integral) + (pide.kd*derivativa);

  return constrain(salida,(double)-pide.maximo,(double)pide.maximo);  // [FIX 9] sin truncar a int aquí
}

void inicializamotores(){
  // [FIX 10] compatible con core ESP32 2.x y 3.x
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttachChannel(pinMI1,PWM_FREQ,PWM_RESOLUTION,canalMI1);
  ledcAttachChannel(pinMI2,PWM_FREQ,PWM_RESOLUTION,canalMI2);
  ledcAttachChannel(pinMD1,PWM_FREQ,PWM_RESOLUTION,canalMD1);
  ledcAttachChannel(pinMD2,PWM_FREQ,PWM_RESOLUTION,canalMD2);
#else
  ledcSetup(canalMI1,PWM_FREQ,PWM_RESOLUTION);
  ledcAttachPin(pinMI1,canalMI1);

  ledcSetup(canalMI2, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(pinMI2,canalMI2);

  ledcSetup(canalMD1, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(pinMD1,canalMD1);

  ledcSetup(canalMD2, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(pinMD2,canalMD2);
#endif
}


void inicializaencoders(){

  // Habilitar las resistencias de pull-up internas para todos los encoders
  ESP32Encoder::useInternalWeakPullResistors = puType::up;

  encoderMI.attachFullQuad(pinencoderMIA,pinencoderMIB);
  encoderMI.setCount(0);

  encoderMD.attachFullQuad(pinencoderMDA,pinencoderMDB);
  encoderMD.setCount(0);

}

void cambiarleds(uint32_t color);

void inicializaimu(){

  dev_spi.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SPI_CS);
  if (AccGyr.begin() != 0) {
    Serial.println("¡Error con LSM6DSR!");
    // [FIX 11] avisa con LED rojo parpadeando en vez de colgarse sin decir nada
    while (1) {
      cambiarleds(leds.Color(60,0,0)); delay(200);
      cambiarleds(0);                  delay(200);
    }
  }

  // Activar sensores
  AccGyr.Enable_X(); // Acelerómetro
  AccGyr.Enable_G(); // Giroscopio

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
    AccGyr.Get_G_Axes(gyro_raw);

    float gz = (gyro_raw[2] / 1000.0) - sesgoZ;

    anguloZ -= (gz*dt)/1000000.0;
    anguloZ = normaliza180(anguloZ);   // [FIX 4]
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

  // [FIX 1] al calcular ahora dt en segundos, ki y kd cambian de escala: REAJUSTAR en pruebas.
  inicializaPID(PIDMI,10,0.005,0.0,255,150);
  inicializaPID(PIDMD,10,0.005,0.0,255,150);

  inicializaPID(PIDYAW,1.0,0.0,0.2,10,150);

  instante = micros();

  veldeseada=1.6;
  aceleracion=20.0;
  velinicial=0;
  yawDeseado=0.0;
  xfin=1.5;
  yfin=0.0;

  t=millis();

}

void loop() {

    unsigned long ahora=micros();
    double dt=(double)(ahora-instante);   // [FIX 9] resta sin signo: sobrevive al desbordamiento de micros()

    if (dt >= 50) {   //se revisa cada 50 us
        instante=ahora;

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

        // [FIX 2] rampa con el tiempo TRANSCURRIDO desde t, y también hacia abajo (frenada)
        double transcurrido=(millis()-t)/1000.0;
        if (veldeseada >= velinicial) {
          velpermitida = velinicial + transcurrido*aceleracion;
          if(velpermitida>veldeseada) velpermitida=veldeseada;
        } else {
          velpermitida = velinicial - transcurrido*aceleracion;
          if(velpermitida<veldeseada) velpermitida=veldeseada;
        }

        int pwmbase=veltopwmdef(velpermitida);

        int pwmMI=pwmbase+(int)calulaPID(PIDMI,velpermitida-velmediaMI,ahora);
        int pwmMD=pwmbase+(int)calulaPID(PIDMD,velpermitida-velmediaMD,ahora);

        // [FIX 2] solo se cortan motores cuando la rampa ya ha llegado a 0
        if(veldeseada==0.0 && velpermitida==0.0){
            pwmMI=0;
            pwmMD=0;
            resetPID(PIDMI);   // que la integral no se quede cargada para la siguiente arrancada
            resetPID(PIDMD);
        }

        actualizaanguloz(dt);
        calculaxy((numMI+numMD)/2.0);

        if(estado==0){
          angulohaciaobjetivo();
        }

        int turn=(int)calulaPID(PIDYAW,normaliza180(yawDeseado-anguloZ),ahora);  // [FIX 4]

        if(!para){
          pwmmotor(canalMI1,canalMI2,pwmMI+turn,255);
          pwmmotor(canalMD1,canalMD2,pwmMD-turn,255);
        }else{
          pwmmotor(canalMI1,canalMI2,0,0);
          pwmmotor(canalMD1,canalMD2,0,0);
        }

    }

    if((millis()-tiempo)>800){

        estado++;

        if(estado==1){
          inicializaPID(PIDYAW,5.0,0.1,1.0,40,150);
          t=millis();
          velinicial=velpermitida;
          veldeseada=0.0;
        }else if(estado==2){
          velinicial=0.0;
          veldeseada=0.0;
          yawDeseado=normaliza180(yawDeseado+180);   // [FIX 4]
        }else if(estado<20){
          velinicial=0.0;
          veldeseada=0.0;
        }else{
          // [FIX 8] 'para' se queda a true: el robot se detiene tras la primera ida y vuelta,
          // como en el original. Quitar esta línea si se quiere que repita.
          para=true;
          inicializaPID(PIDYAW,1.0,0.0,0.2,10,150);
          estado=0;
          anguloZ=0.0;
          x=0.0; y=0.0;                            // [FIX 7] reinicia también la odometría
          veldeseada=1.6;
          velinicial=0;
          yawDeseado=0.0;
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

        Serial.print("\nYaw : ");
        Serial.println(anguloZ);
        Serial.print("\n");

        tiempo=millis();
    }
}
