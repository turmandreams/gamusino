
// --- CONFIGURACIÓN ENCODER 1 ---
#define ENCODER1_A  25
#define ENCODER1_B  26

// --- CONFIGURACIÓN ENCODER 2 (¡Ocupa pines I2C nativos!) ---
#define ENCODER2_A  21
#define ENCODER2_B  22


long encoder1Pulsos=0;
long encoder2Pulsos=0;

int tiempo=0;

void IRAM_ATTR leerEncoder1AB() {
  int estadoA = digitalRead(ENCODER1_A);
  int estadoB = digitalRead(ENCODER1_B);
  if (estadoA == estadoB) {
    encoder1Pulsos++;
  } else {
    encoder1Pulsos--;
  }
}

// ISR para Encoder 2
void IRAM_ATTR leerEncoder2AB() {
  int estadoA = digitalRead(ENCODER2_A);
  int estadoB = digitalRead(ENCODER2_B);
  if (estadoA == estadoB) {
    encoder2Pulsos--;
  } else {
    encoder2Pulsos++;
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // Configuración de Hardware Encoder 1
  pinMode(ENCODER1_A, INPUT);
  pinMode(ENCODER1_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER1_A), leerEncoder1AB, CHANGE);

  // Configuración de Hardware Encoder 2
  pinMode(ENCODER2_A, INPUT);
  pinMode(ENCODER2_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_A), leerEncoder2AB, CHANGE);


  tiempo=millis();
  
}

//><

void loop() {

    // Una vuelta de la rueda equivale a 5888 en el valor de encoderPulsos
    
    if((millis()-tiempo)>100) {

      Serial.print(encoder1Pulsos);
      Serial.print(",");
      Serial.println(encoder2Pulsos);

      Serial.print("Distancia recorrida (mm): ");

      float media=((float)encoder1Pulsos+(float)encoder2Pulsos)/2.0;
      
      float distancia=media*0.01067;

      Serial.println(distancia);

      
      tiempo=millis();
      
    }
    
    
 
}
