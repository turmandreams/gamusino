#include <Adafruit_NeoPixel.h>

#define PIN_DATOS   27  // El pin donde conectaste el DIN
#define NUM_LEDS     4  // Tienes 4 LEDs conectados

// Inicializamos la tira de LEDs
Adafruit_NeoPixel tira(NUM_LEDS, PIN_DATOS, NEO_GRB + NEO_KHZ800);

void setup() {
  tira.begin();           // Inicializa el hardware de los LEDs
  tira.setBrightness(50); // Ajusta el brillo (de 0 a 255) para no encandilar
  tira.show();            // Apaga todos los LEDs al inicio
}

void loop() {
  // Secuencia de colores básicos: Rojo, Verde, Azul
  
  // 1. Encender todos en ROJO
  cambiarColor(tira.Color(255, 0, 0), 500);
  
  // 2. Encender todos en VERDE
  cambiarColor(tira.Color(0, 255, 0), 500);
  
  // 3. Encender todos en AZUL
  cambiarColor(tira.Color(0, 0, 255), 500);
  
  // 4. Efecto de barrido (uno por uno) en Amarillo
  for(int i=0; i<NUM_LEDS; i++) {
    tira.setPixelColor(i, tira.Color(255, 255, 0)); // Amarillo
    tira.show();
    delay(200);
  }
  delay(500);
}

// Función auxiliar para iluminar toda la tira de un solo golpe
void cambiarColor(uint32_t color, int espera) {
  for(int i=0; i<NUM_LEDS; i++) {
    tira.setPixelColor(i, color);
  }
  tira.show();
  delay(espera);
}
