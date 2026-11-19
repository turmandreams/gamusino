#include <WifiEspNowBroadcast.h>
#include <WiFi.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <esp_task_wdt.h>


#include <Adafruit_NeoPixel.h>

#define PIN_DATOS   27  // El pin donde conectaste el DIN
#define NUM_LEDS     4  // Tienes 4 LEDs conectados

// Inicializamos la tira de LEDs
Adafruit_NeoPixel tira(NUM_LEDS, PIN_DATOS, NEO_GRB + NEO_KHZ800);



const int pinM2P2=13;
const int pinM2P1=2;
const int pinM1P2=16;
const int pinM1P1=17;

const int canalM1P1=0;
const int canalM1P2=1;
const int canalM2P1=2;
const int canalM2P2=3;

const int ZONA_MUERTA = 20;   // en unidades de -128..127

int tiempo=0;

TaskHandle_t Task1;  // declaramos para poder ejecutar en otro core

void motor(int canalP1,int canalP2, int vel) {
  vel = constrain(vel, -255, 255);
  if (vel >= 0) {
    ledcWrite(canalP1, 0);
    ledcWrite(canalP2, vel);    
  } else {
    ledcWrite(canalP2, 0);
    ledcWrite(canalP1, -vel);
  }
}

void processRx(const uint8_t mac[WIFIESPNOW_ALEN], const uint8_t* buf, size_t count, void* arg){

    for(int i=0; i<NUM_LEDS; i++) {  tira.setPixelColor(i,tira.Color(0,0,100));  }
    tira.show();
  
         
    Serial.print(buf[0]);Serial.print(",");Serial.print(buf[1]);Serial.print(",");Serial.println(buf[2]);

    int x=buf[1]-128;
    int y=buf[0]-128;
 
    if (abs(x) < ZONA_MUERTA) {x=0;}
    if (abs(y) < ZONA_MUERTA) {y=0;}
    
    int velIzq = y + x;
    int velDer = y - x;

    // Escalar de -128..127 a -255..255 para aprovechar todo el PWM
    velIzq = map(velIzq, -128, 127, -255, 255);
    velDer = map(velDer, -128, 127, -255, 255);
  
    motor(canalM1P1,canalM1P2, velIzq);
    motor(canalM2P1,canalM2P2, velDer);
  
    //Serial.println(dato);

   
    tiempo=millis();

        
    return;
    
}


void wdt(){
    esp_task_wdt_init(30,false);
    esp_task_wdt_reset();
    yield(); 
}


void setup() {

  tira.begin();           // Inicializa el hardware de los LEDs
  tira.setBrightness(25); // Ajusta el brillo (de 0 a 255) para no encandilar
  tira.show();            // Apaga todos los LEDs al inicio

  for(int i=0; i<NUM_LEDS; i++) {  tira.setPixelColor(i,tira.Color(0,100,0));  }
  tira.show();
  

  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  
  pinMode(pinM1P1,OUTPUT);digitalWrite(pinM1P1,LOW);
  pinMode(pinM1P2,OUTPUT);digitalWrite(pinM1P2,LOW);
  pinMode(pinM2P1,OUTPUT);digitalWrite(pinM2P1,LOW);
  pinMode(pinM2P2,OUTPUT);digitalWrite(pinM2P2,LOW);
 

  ledcSetup(canalM1P1,3000,8); ledcAttachPin(pinM1P1,canalM1P1);ledcWrite(canalM1P1, 0);  
  ledcSetup(canalM1P2,3000,8); ledcAttachPin(pinM1P2,canalM1P2);ledcWrite(canalM1P2, 0);     
  ledcSetup(canalM2P1,3000,8); ledcAttachPin(pinM2P1,canalM2P1);ledcWrite(canalM2P1, 0);     
  ledcSetup(canalM2P2,3000,8); ledcAttachPin(pinM2P2,canalM2P2);ledcWrite(canalM2P2, 0);     

  Serial.begin(115200);

  Serial.println("ATROZOS CARGANDO !!!");

  
  WiFi.persistent(false);
  bool ok = WifiEspNowBroadcast.begin("MANDO",3);
  if (!ok) {
     // Serial.println("WifiEspNowBroadcast.begin() failed");
      ESP.restart();
  }
  
  WifiEspNowBroadcast.onReceive(processRx, nullptr);

   xTaskCreatePinnedToCore(
                    Task1code,   /* Task function. */
                    "Task1",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task1,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */                  

  delay(200);
  
   
  tiempo=millis();
  
}


void Task1code( void * pvParameters ){   
  
  for(;;){         

   WifiEspNowBroadcast.loop();       
   
   if((millis()-tiempo)>1000){ 

      ledcWrite(canalM1P1,0);  
      ledcWrite(canalM1P2,0);  
      ledcWrite(canalM2P1,0);  
      ledcWrite(canalM2P2,0);  
             
      Serial.println("Se para");
      tiempo=millis();
        
   }
    wdt();     
  } 
}


void loop() {
    
    delay(10);

     wdt();
    
}
