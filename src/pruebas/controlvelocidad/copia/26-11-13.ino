
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

//const double METROS_POR_PULSO = 0,000005336; // Basado en diámetro de 20mm y 5888 pulsos/rev
const double METROS_POR_PULSO = 5.336; // Como se divide entre microsegundos lo multiplico por 1.000.000 , para luego pasarlo a m/s

double veldeseada = 0.8; // Velocidad objetivo en m/s (acepta valores negativos)

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


long instante=0; // Asegúrate de inicializar esto para el cálculo de dt

long tiempo=0;

int veltopwmdef(float vel){

    if(vel<=0.4){        return 10;  }
    else if(vel<=0.6){   return 20;  }
    else if(vel<=1.0){   return 30;  }
    else if(vel<=1.3){   return 40;  }
    else if(vel<=1.4){   return 50;  }
    else if(vel<=1.5){   return 60;  }
    else if(vel<=1.6){   return 70;  }
    else if(vel<=1.65){  return 80;  }
    else if(vel<=1.70){  return 90;  }
    
  
}

void pwmmotor(int canal1,int canal2,int pwm,int maximo) {
  
  pwm = constrain(pwm,-maximo, maximo);  
    
  if (pwm < 0)      { ledcWrite(canal1,-pwm); ledcWrite(canal2, 0); }
  else if (pwm > 0) { ledcWrite(canal1,0);ledcWrite(canal2,pwm); }
  else              { ledcWrite(canal1,0);ledcWrite(canal2, 0); }
    
}

void inicializaPID(PID &pide,double kp,double ki,double kd,int maximo,int maximointegral){
  pide.kp=kp;
  pide.ki=ki;
  pide.kd=kd;
  pide.integral=0.0;
  pide.errorant=0.0;   
  pide.maximo=maximo;   
  pide.maximointegral=maximointegral;   
}


void resetPID(PID &pide){
  pide.integral=0.0;
  pide.errorant=0.0;      
}


double calulaPID(PID pide,double error,double instante){

  double dt=instante-pide.instante;
  
  pide.integral += error * dt;
  pide.integral = constrain(pide.integral,-pide.maximointegral,pide.maximointegral); 
  
  double derivativa = (error - pide.errorant)/dt;  
  pide.errorant = error;
    
  double salida = (pide.kp*error) + (pide.ki*pide.integral) + (pide.kd*derivativa);

  return constrain((int)salida,-pide.maximo,pide.maximo);
    
}



void setup() {
  Serial.begin(115200);
  while (!Serial);

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

  
  inicializaPID(PIDMI,10,0.05,0.0,255,150);
  inicializaPID(PIDMD,10,0.05,0.0,255,150);


  delay(3000);

  tiempo=millis();

  instante = micros(); // Inicialización correcta para evitar saltos en dt
 
  
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

    velmediaMI=(velmediaMI*0.8)+(velMI*0.2);
    velmediaMD=(velmediaMD*0.8)+(velMD*0.2);

    int pwmMI=(int)calulaPID(PIDMI,veldeseada-velmediaMI,ahora);
    int pwmMD=(int)calulaPID(PIDMD,veldeseada-velmediaMD,ahora);

    int pwmbase=veltopwmdef(veldeseada);
    
    pwmmotor(canalMI1,canalMI2,pwmbase+pwmMI,255);  
    pwmmotor(canalMD1,canalMD2,pwmbase+pwmMD,255);


   // Serial.println(pwmMI);
            
    if((millis()-tiempo)>500){

        pwmmotor(canalMI1,canalMI2,0,0);  
        pwmmotor(canalMD1,canalMD2,0,0);

        delay(5000);
    
        Serial.print("Velocidad MI : ");
        Serial.println(velmediaMI);
        Serial.print("Velocidad MD : ");
        Serial.println(velmediaMD);
                
        tiempo=millis();

        
    }

    delay(1);
    
}
