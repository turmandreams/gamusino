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

unsigned long ultimoTiempo = 0;

long contMIant = 0;
long contMDant = 0;



void setup() {
  Serial.begin(115200);
  
  // Habilitar las resistencias de pull-up internas para todos los encoders
  ESP32Encoder::useInternalWeakPullResistors = puType::up;

  // Configuración del Encoder 1
  encoderMI.attachFullQuad(pinencoderMIA,pinencoderMIB);
  encoderMI.setCount(0);

  // Configuración del Encoder 2
  encoderMD.attachFullQuad(pinencoderMDA,pinencoderMDB);
  encoderMD.setCount(0);
  
}

void loop() {
  unsigned long tiempoActual = millis();
  
  if (tiempoActual - ultimoTiempo >= 100) {
    unsigned long deltaTiempo = tiempoActual - ultimoTiempo;

    long contMI = encoderMI.getCount();
    long deltaPasos1 = contMI - contMIant;
    float rpmMI = (deltaPasos1 / 512.0) * (60000.0 / deltaTiempo);
    
    long contMD = encoderMD.getCount();
    long deltaPasos2 = contMD - contMDant;
    float rpmMD = (deltaPasos2 / 512.0) * (60000.0 / deltaTiempo);
    
    
    Serial.print("contMI: ");
    Serial.print(contMI);
    Serial.print(" | RPM: ");
    Serial.print(rpmMI);
    
    Serial.print("  ///  Enc2 - Pasos: ");
    Serial.print(contMD);
    Serial.print(" | RPM: ");
    Serial.println(rpmMD);
    
    // Actualizar históricos
    contMIant = contMI;
    contMDant = contMD;
    ultimoTiempo = tiempoActual;
    
  }
  
}
