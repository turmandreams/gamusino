#include <SPI.h>
#include <LSM6DSRSensor.h>

// Pines compartidos del Bus VSPI
#define SPI_SCK   18
#define SPI_MISO  19
#define SPI_MOSI  23

// Pines Chip Select individuales (Únicos)
#define LSM_CS      5
#define AS5040_CS   4

// --- CONFIGURACIÓN ENCODER 1 ---
#define ENCODER1_A  25
#define ENCODER1_B  26

// --- CONFIGURACIÓN ENCODER 2 (¡Ocupa pines I2C nativos!) ---
#define ENCODER2_A  21
#define ENCODER2_B  22

SPIClass dev_spi(VSPI);
LSM6DSRSensor AccGyr(&dev_spi, LSM_CS);

// Temporización e intervalos de ejecución (No bloqueantes)
unsigned long cronoIMU = 0;
unsigned long cronoRPM = 0;
unsigned long cronoSerial = 0;

const unsigned long INTERVALO_IMU_MS = 10;     // Muestreo IMU a 100Hz
const unsigned long INTERVALO_RPM_MS = 100;    // Cálculo de velocidad cada 100ms
const unsigned long INTERVALO_SERIAL_MS = 200; // Impresión por pantalla cada 200ms

// Variables LSM6DSR (IMU)
float anguloZ = 0.0;
float sesgoZ = 0.0;

// Configuración SPI AS5040
SPISettings as5040Settings(1000000, MSBFIRST, SPI_MODE1);

// Variables Encoder 1
volatile long encoder1Pulsos = 0;
long ultimosPulsosRPM1 = 0;
float encoder1RPM = 0.0;

// Variables Encoder 2
volatile long encoder2Pulsos = 0;
long ultimosPulsosRPM2 = 0;
float encoder2RPM = 0.0;

// Variables AS5040 (SPI)
float encoderAnguloAbs = 0.0;
bool imanFalloStatus = false;

// ISR para Encoder 1
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
    encoder2Pulsos++;
  } else {
    encoder2Pulsos--;
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

  pinMode(AS5040_CS, OUTPUT);
  digitalWrite(AS5040_CS, HIGH);

  // Inicializar bus SPI compartido
  dev_spi.begin(SPI_SCK, SPI_MISO, SPI_MOSI, LSM_CS);

  if (AccGyr.begin() != 0) {
    Serial.println("¡Error inicializando LSM6DSR!");
    while (1);
  }
  
  AccGyr.Enable_G();
  AccGyr.Set_G_FS(4000);
  AccGyr.Set_G_ODR(1666.0f);

  // --- Calibración Avanzada de la IMU ---
  Serial.println("Calibrando IMU... Deja el dispositivo completamente quieto.");
  delay(1000);
  
  const int muestras = 1000;
  int32_t gyro_raw[3];
  long long sumaZ = 0;
  
  for(int i = 0; i < muestras; i++) {
    AccGyr.Get_G_Axes(gyro_raw);
    sumaZ += gyro_raw[2];
    delayMicroseconds(1500); 
  }
  sesgoZ = ((double)sumaZ / muestras) / 1000.0;
  
  Serial.print("Calibración completada. Sesgo Z calculado: ");
  Serial.println(sesgoZ, 4);

  // Sincronizar todos los cronómetros
  unsigned long tiempoActual = millis();
  cronoIMU = micros();
  cronoRPM = tiempoActual;
  cronoSerial = tiempoActual;
  
  //ledcSetup(5,500,8);
  //ledcAttachPin(12,5);
  //ledcWrite(5,20);

}

void loop() {
  unsigned long actualMs = millis();
  unsigned long actualUs = micros();

  // ========================================================
  // TAREA 1: MUESTREO DE IMU
  // ========================================================
  if (actualUs - cronoIMU >= (INTERVALO_IMU_MS * 1000)) {
    float dt = (actualUs - cronoIMU) / 1000000.0;
    cronoIMU = actualUs;

    int32_t gyro_raw[3];
    AccGyr.Get_G_Axes(gyro_raw);
    
    float velocidadZ_dps = (gyro_raw[2] / 1000.0) - sesgoZ;
    
    if (abs(velocidadZ_dps) < 0.5) {
      velocidadZ_dps = 0.0;
    }
    anguloZ += velocidadZ_dps * dt;
  }

  // ========================================================
  // TAREA 2: CÁLCULO DE RPM PARA ENCODER 1 Y ENCODER 2
  // ========================================================
  if (actualMs - cronoRPM >= INTERVALO_RPM_MS) {
    // Sección crítica: proteger la lectura de variables volátiles
    noInterrupts();
    long pulsosActuales1 = encoder1Pulsos;
    long pulsosActuales2 = encoder2Pulsos;
    interrupts();

    float deltaTiempoMinutos = (actualMs - cronoRPM) / 60000.0;
    
    // Encoder 1 RPM
    long deltaPulsos1 = pulsosActuales1 - ultimosPulsosRPM1;
    encoder1RPM = (deltaPulsos1 / 512.0) / deltaTiempoMinutos;
    ultimosPulsosRPM1 = pulsosActuales1;

    // Encoder 2 RPM
    long deltaPulsos2 = pulsosActuales2 - ultimosPulsosRPM2;
    encoder2RPM = (deltaPulsos2 / 512.0) / deltaTiempoMinutos;
    ultimosPulsosRPM2 = pulsosActuales2;
    
    cronoRPM = actualMs;
  }

  // ========================================================
  // TAREA 3: LECTURA DEL ENCODER AS5040 POR BUS SPI
  // ========================================================
  if (actualMs - cronoSerial >= INTERVALO_SERIAL_MS) {
    dev_spi.beginTransaction(as5040Settings);
    digitalWrite(AS5040_CS, LOW);
    delayMicroseconds(2); 
    uint16_t datosRaw = dev_spi.transfer16(0x0000);
    digitalWrite(AS5040_CS, HIGH);
    dev_spi.endTransaction();

    uint16_t encoderRaw = (datosRaw >> 6) & 0x03FF;
    encoderAnguloAbs = (encoderRaw * 360.0) / 1024.0;
    imanFalloStatus = ((datosRaw & 0x0020) && (datosRaw & 0x0010));
  }

  // ========================================================
  // TAREA 4: SALIDA DE DATOS HACIA EL PUERTO SERIAL
  // ========================================================
  if (actualMs - cronoSerial >= INTERVALO_SERIAL_MS) {
    noInterrupts();
    long impPulsos1 = encoder1Pulsos;
    long impPulsos2 = encoder2Pulsos;
    interrupts();

    float encoder1AnguloRel = (impPulsos1 * 360.0) / 512.0;
    float encoder2AnguloRel = (impPulsos2 * 360.0) / 512.0;

    Serial.print("IMU_Z:");       Serial.print(anguloZ, 1);             Serial.print("°\t| ");
    Serial.print("AS5040_Abs:");   Serial.print(encoderAnguloAbs, 1);    Serial.print("°\t|| ");
    
    // Datos Encoder 1
    Serial.print("E1_RPM:");       Serial.print(encoder1RPM, 1);         Serial.print(" (");
    Serial.print(encoder1AnguloRel, 1); Serial.print("°)\t| ");
    
    // Datos Encoder 2
    Serial.print("E2_RPM:");       Serial.print(encoder2RPM, 1);         Serial.print(" (");
    Serial.print(encoder2AnguloRel, 1); Serial.print("°)");

    if (imanFalloStatus) {
      Serial.print(" [¡ERR IMÁN!]");
    }
    Serial.println();
    
    cronoSerial = actualMs;
  }
}
