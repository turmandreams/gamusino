
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


//const double METROS_POR_PULSO = 0,000005336; // Basado en diámetro de 20mm y 5888 pulsos/rev
const double METROS_POR_PULSO = 5.336; // Como se divide entre microsegundos lo multiplico por 1.000.000 , para luego pasarlo a m/s

double velmediaMI=0.0;
double velmediaMD=0.0;


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


long tiempo;

double instante;

int pwm=0;



void pwmmotor(int canal1,int canal2,int pwm,int maximo) {
  
  pwm = constrain(pwm,-maximo, maximo);  
    
  if (pwm < 0)      { ledcWrite(canal1,-pwm); ledcWrite(canal2, 0); }
  else if (pwm > 0) { ledcWrite(canal1,0);ledcWrite(canal2,pwm); }
  else              { ledcWrite(canal1,0);ledcWrite(canal2, 0); }
    
}


void setup() {
  
  Serial.begin(115200);

  ledcSetup(canalMI1,PWM_FREQ,PWM_RESOLUTION);
  ledcAttachPin(pinMI1,canalMI1);

  ledcSetup(canalMI2, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(pinMI2,canalMI2);

  ledcSetup(canalMD1, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(pinMD1,canalMD1);

  ledcSetup(canalMD2, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(pinMD2,canalMD2);
  
  
  // Habilitar las resistencias de pull-up internas para todos los encoders
  ESP32Encoder::useInternalWeakPullResistors = puType::up;

  // Configuración del Encoder 1
  encoderMI.attachFullQuad(pinencoderMIA,pinencoderMIB);
  encoderMI.setCount(0);

  // Configuración del Encoder 2
  encoderMD.attachFullQuad(pinencoderMDA,pinencoderMDB);
  encoderMD.setCount(0);

  delay(3000);
  
}

void loop() {

  double ahora=micros();        
  double dt=ahora-instante;
  instante=ahora;   

  long contMI = encoderMI.getCount();
  long numMI = contMI - contMIant;
  contMIant = contMI;
  
  long contMD = encoderMD.getCount();
  long numMD = contMD - contMDant;
  contMDant = contMD;
  
  double velMI = (numMI * METROS_POR_PULSO) / dt;
  double velMD = (numMD * METROS_POR_PULSO) / dt;

  velmediaMI=(velmediaMI*0.9)+(velMI*0.1);
  velmediaMD=(velmediaMD*0.9)+(velMD*0.1);
    
  if (millis() - tiempo >= 1000) {
      
    /*
    Serial.print("PWM : ");
    Serial.println(pwm);

    Serial.print("velmediaMI :");
    Serial.println(velmediaMI);
    
    Serial.print("velmediaMD :");
    Serial.println(velmediaMD);

    */

    Serial.print(pwm);
    Serial.print(","); 
    Serial.print(velmediaMI);
    Serial.print(",");    
    Serial.println(velmediaMD);
   

    pwm+=1;

    if(pwm>=250){
        pwm=0;
        pwmmotor(canalMI1,canalMI2,pwm,255);  
        pwmmotor(canalMD1,canalMD2,pwm,255);
    
        delay(100000);      
    }

    pwmmotor(canalMI1,canalMI2,pwm,255);  
    pwmmotor(canalMD1,canalMD2,pwm,255);
    
 
    tiempo=millis();
    
  }

  delay(1);
}
