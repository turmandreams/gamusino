// =====================================================================
//  gamusino_fork  -  robot ratón, versión reorganizada y mejorada
//  Basado en gamusino_1. Ver CAMBIOS.md y MANUAL.md.
//
//  Al encender: calibra el giroscopio y queda EN REPOSO esperando orden.
//    - Botón BOOT o comando 'r'  -> hace la ruta (ir, frenar, girar 180, volver)
//    - Comando 'a'               -> autoajuste PID de ruedas (robot EN EL AIRE)
//    - Comando 'g'               -> autoajuste PID de giro   (robot EN EL SUELO)
//    - 'p' ver ganancias, 'b' borrar guardadas, 's' parar, 'h' ayuda
//  Monitor serie a 115200, "Nueva línea".
// =====================================================================

#include <Adafruit_NeoPixel.h>
#include <SPI.h>
#include <LSM6DSRSensor.h>
#include <ESP32Encoder.h>
#include <Preferences.h>

// ------------------------- CONFIGURACIÓN -----------------------------
// LEDs
#define PIN_DATOS   27
#define NUM_LEDS     4

// Botón BOOT de la placa ESP32-WROOM-32 (GPIO0, activo a nivel bajo)
#define PIN_BOTON    0

// IMU (SPI)
#define SPI_SCK  18
#define SPI_MISO 19
#define SPI_MOSI 23
#define SPI_CS   5

// Motores (puente H, 2 pines por motor)
#define PIN_MI1 2
#define PIN_MI2 13
#define PIN_MD1 17
#define PIN_MD2 16
#define CH_MI1 0
#define CH_MI2 1
#define CH_MD1 2
#define CH_MD2 3
#define PWM_FREQ  15000
#define PWM_BITS  8
#define PWM_MAX   255

// Encoders
#define PIN_ENC_MI_A 22
#define PIN_ENC_MI_B 21
#define PIN_ENC_MD_A 25
#define PIN_ENC_MD_B 26

// Mecánica  (MEDIR: ver MANUAL.md, paso 2)
const float DIAM_RUEDA_M      = 0.020;   // diámetro de rueda
const float PULSOS_POR_VUELTA = 5888;    // ya en cuadratura x4 (attachFullQuad)
const float M_POR_PULSO       = PI * DIAM_RUEDA_M / PULSOS_POR_VUELTA;
const int   SIGNO_ENC_MI      = 1;       // pon -1 si un encoder cuenta al revés
const int   SIGNO_ENC_MD      = 1;
const int   SIGNO_YAW         = -1;      // el original hacía anguloZ -= gz

// Periodos de los lazos
const uint32_t T_CONTROL_US   = 1000;    // 1 kHz: velocidad, yaw y odometría
const uint32_t T_TELEMETRIA_MS= 200;

// Movimiento
const float VEL_MAX      = 1.6;   // m/s
const float ACEL         = 2.0;   // m/s² (subida y bajada)
const float TOL_DIST     = 0.02;  // m: se considera llegado
const float TOL_YAW      = 3.0;   // °: giro terminado
const float XFIN = 1.5, YFIN = 0.0;
const uint32_t ESPERA_MS = 1000;
const bool  REPETIR = false;
const bool  ARRANQUE_AUTOMATICO = false;  // true = hace la ruta nada más calibrar (como el original)

// Autoajuste (método del relé)
const float AT_VEL        = 0.8;   // m/s alrededor de la que oscilan las ruedas
const float AT_H_RUEDAS   = 25;    // PWM que sube/baja el relé en las ruedas
const float AT_EPS_VEL    = 0.02;  // m/s de histéresis
const float AT_H_GIRO     = 70;    // PWM diferencial del relé al girar
const float AT_EPS_YAW    = 1.0;   // ° de histéresis
const int   AT_CICLOS     = 6;     // ciclos que se promedian (tras descartar 2)
const float AT_TIMEOUT_S  = 20;    // por ronda
const int   AT_RONDAS     = 3;     // rondas seguidas; se usa la mediana
const float AT_AMP_RUEDAS = 0.10;  // m/s de oscilación buscada (se reajusta el relé entre rondas)
const float AT_AMP_GIRO   = 4.0;   // ° de oscilación buscada

// ------------------------- UTILIDADES --------------------------------
static inline float norm180(float a){
  while (a >  180.f) a -= 360.f;
  while (a < -180.f) a += 360.f;
  return a;
}

// PID con: dt en segundos, antisaturación (anti-windup),
// derivada sobre la medida (sin picos al cambiar la consigna) y filtrada.
struct PID {
  float kp=0, ki=0, kd=0;
  float salidaMax=0, integralMax=0;
  float integral=0, medidaAnt=0, dFilt=0;
  bool  primero=true;

  void config(float p,float i,float d,float sMax,float iMax){
    kp=p; ki=i; kd=d; salidaMax=sMax; integralMax=iMax; reset();
  }
  void reset(){ integral=0; dFilt=0; primero=true; }

  float paso(float consigna,float medida,float dt,bool angular=false){
    float e = consigna - medida;
    if (angular) e = norm180(e);
    float dMed = 0;
    if (!primero){
      float dm = medida - medidaAnt;
      if (angular) dm = norm180(dm);
      dMed = dm / dt;
    }
    primero=false; medidaAnt=medida;
    dFilt = 0.8f*dFilt + 0.2f*dMed;

    float pTerm = kp*e, dTerm = -kd*dFilt;
    float iNuevo = constrain(integral + e*dt, -integralMax, integralMax);
    float u = pTerm + ki*iNuevo + dTerm;
    if ((u < salidaMax && u > -salidaMax) || (u*e < 0)) integral = iNuevo;
    return constrain(pTerm + ki*integral + dTerm, -salidaMax, salidaMax);
  }
};

// Perfil de velocidad: la consigna se acerca al objetivo a ACEL m/s².
struct Rampa {
  float actual=0;
  float paso(float objetivo,float acel,float dt){
    float maxCambio = acel*dt;
    actual += constrain(objetivo-actual, -maxCambio, maxCambio);
    return actual;
  }
};

// Relé para autoajuste (Åström-Hägglund): fuerza una oscilación y mide
// su periodo Tu y su amplitud a. Ku = 4h / (pi·a).
struct Rele {
  float sp=0, h=0, eps=0;
  bool  alto=true, armado=false;
  float vmax=-1e9, vmin=1e9, t=0, tUlt=0;
  int   ciclos=0, n=0;
  float sumT=0, sumA=0;

  void inicio(float s,float hh,float e){
    sp=s; h=hh; eps=e; alto=true; armado=false;
    vmax=-1e9; vmin=1e9; t=0; tUlt=0; ciclos=0; n=0; sumT=0; sumA=0;
  }
  float paso(float m,float dt){            // devuelve +h o -h
    t += dt;
    if (m>vmax) vmax=m;
    if (m<vmin) vmin=m;
    if (alto && m > sp+eps) alto=false;
    else if (!alto && m < sp-eps){         // empieza un ciclo nuevo
      alto=true;
      if (armado){
        ciclos++;
        if (ciclos>2 && n<AT_CICLOS){ sumT += t-tUlt; sumA += (vmax-vmin)/2; n++; }
      }
      armado=true; tUlt=t; vmax=-1e9; vmin=1e9;
    }
    return alto ? h : -h;
  }
  bool  listo() const { return n>=AT_CICLOS; }
  float Tu()    const { return sumT/n; }
  float amp()   const { return sumA/n; }
  float Ku()    const { return 4*h/(PI*amp()); }
};

// Alimentación directa (feedforward): PWM aproximado para una velocidad.
const float FF_V[]   = {0.0, 0.4, 0.6, 1.0, 1.3, 1.4, 1.5, 1.6, 1.65, 1.70};
const float FF_PWM[] = {0,   10,  20,  30,  40,  50,  60,  70,  80,   90};
float feedforward(float v){
  float a = fabs(v), s = (v<0)?-1:1;
  if (a < 0.001) return 0;
  const int n = sizeof(FF_V)/sizeof(FF_V[0]);
  if (a >= FF_V[n-1]) return s*FF_PWM[n-1];
  for (int i=1;i<n;i++) if (a<=FF_V[i]){
    float k=(a-FF_V[i-1])/(FF_V[i]-FF_V[i-1]);
    return s*(FF_PWM[i-1]+k*(FF_PWM[i]-FF_PWM[i-1]));
  }
  return 0;
}

// ------------------------- GANANCIAS (guardadas en flash) ------------
struct Ganancias {
  float mi_kp, mi_ki;          // velocidad rueda izquierda
  float md_kp, md_ki;          // velocidad rueda derecha
  float g_kp, g_ki, g_kd;      // giro en el sitio
  float y_kp, y_kd;            // corrección de rumbo en marcha
};
const Ganancias G_DEFECTO = {120,600, 120,600, 3.0,2.0,0.08, 1.5,0.05};
Ganancias G = G_DEFECTO;
bool gananciasGuardadas = false;
Preferences prefs;
const uint32_t VERSION_G = 1;

void cargaGanancias(){
  prefs.begin("gamusino", true);
  if (prefs.getUInt("ver",0)==VERSION_G && prefs.getBytesLength("g")==sizeof(G)){
    prefs.getBytes("g",&G,sizeof(G)); gananciasGuardadas=true;
  }
  prefs.end();
}
void guardaGanancias(){
  prefs.begin("gamusino", false);
  prefs.putBytes("g",&G,sizeof(G)); prefs.putUInt("ver",VERSION_G);
  prefs.end(); gananciasGuardadas=true;
}
void borraGanancias(){
  prefs.begin("gamusino", false); prefs.clear(); prefs.end();
  G=G_DEFECTO; gananciasGuardadas=false;
}

// ------------------------- HARDWARE ----------------------------------
Adafruit_NeoPixel leds(NUM_LEDS, PIN_DATOS, NEO_GRB + NEO_KHZ800);
SPIClass dev_spi(VSPI);
LSM6DSRSensor imu(&dev_spi, SPI_CS);
ESP32Encoder encMI, encMD;

void ledsColor(uint8_t r,uint8_t g,uint8_t b){
  for (int i=0;i<NUM_LEDS;i++) leds.setPixelColor(i, leds.Color(r,g,b));
  leds.show();
}

void pwmInit(){
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttachChannel(PIN_MI1,PWM_FREQ,PWM_BITS,CH_MI1);
  ledcAttachChannel(PIN_MI2,PWM_FREQ,PWM_BITS,CH_MI2);
  ledcAttachChannel(PIN_MD1,PWM_FREQ,PWM_BITS,CH_MD1);
  ledcAttachChannel(PIN_MD2,PWM_FREQ,PWM_BITS,CH_MD2);
#else
  const int pin[4]={PIN_MI1,PIN_MI2,PIN_MD1,PIN_MD2};
  for (int c=0;c<4;c++){ ledcSetup(c,PWM_FREQ,PWM_BITS); ledcAttachPin(pin[c],c); }
#endif
}
static inline void pwmEscribe(int canal,int v){
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWriteChannel(canal,v);
#else
  ledcWrite(canal,v);
#endif
}
void motor(int c1,int c2,float pwm){
  int p = (int)constrain(pwm, -PWM_MAX, PWM_MAX);
  if      (p < 0){ pwmEscribe(c1,-p); pwmEscribe(c2,0); }
  else if (p > 0){ pwmEscribe(c1,0);  pwmEscribe(c2,p); }
  else           { pwmEscribe(c1,0);  pwmEscribe(c2,0); }
}
void motoresParar(){ motor(CH_MI1,CH_MI2,0); motor(CH_MD1,CH_MD2,0); }

void fallo(const char* msg){
  motoresParar();
  Serial.println(msg);
  while (true){ ledsColor(60,0,0); delay(200); ledsColor(0,0,0); delay(200); }
}

// ------------------------- ESTADO ------------------------------------
float sesgoGz = 0;
float yaw = 0, gzActual = 0;   // grados, °/s
float x = 0, y = 0;            // metros
float velMI = 0, velMD = 0;    // m/s filtradas
long  cntMIant = 0, cntMDant = 0;

PID pidMI, pidMD, pidYawMarcha, pidYawGiro;
Rampa rampa;
Rele releMI, releMD, releYaw;
float yawAuto0 = 0, tAuto = 0;
int   ronda = 0;
float kuI_r[AT_RONDAS], tuI_r[AT_RONDAS], kuD_r[AT_RONDAS], tuD_r[AT_RONDAS];
float kuY_r[AT_RONDAS], tuY_r[AT_RONDAS];
float hMI = 0, hMD = 0, hYaw = 0;

// mediana de n valores (n pequeño, máx. 8)
float mediana(const float* v,int n){
  float a[8]; for(int i=0;i<n;i++) a[i]=v[i];
  for(int i=1;i<n;i++){ float k=a[i]; int j=i-1; while(j>=0&&a[j]>k){a[j+1]=a[j];j--;} a[j+1]=k; }
  return (n%2) ? a[n/2] : 0.5f*(a[n/2-1]+a[n/2]);
}
// dispersión relativa (máx-mín)/mediana: avisa si las rondas no coinciden

float dispersion(const float* v,int n){
float mn=v[0],mx=v[0]; for(int i=1;i<n;i++){ mn=min(mn,v[i]); mx=max(mx,v[i]); }
float m=mediana(v,n); return m!=0 ? (mx-mn)/fabs(m) : 0;
}
// nueva fuerza del relé para acercar la amplitud a la buscada
float reajustaH(float h,float amp,float objetivo,float hMin,float hMax){
if (amp<=0) return h;
return constrain(h*objetivo/amp, hMin, hMax);
}
enum Fase { REPOSO, IR, FRENAR, GIRAR, ESPERAR, VOLVER, AUTO_RUEDAS, AUTO_GIRO };
Fase fase = REPOSO;
float objX = XFIN, objY = YFIN;
float yawObjetivo = 0;
uint32_t tFase = 0;
const char* nombreFase(int f){
switch(f){ case REPOSO:return "REPOSO"; case IR:return "IR"; case FRENAR:return "FRENAR";
case GIRAR:return "GIRAR"; case ESPERAR:return "ESPERAR"; case VOLVER:return "VOLVER";
case AUTO_RUEDAS:return "AUTO_RUEDAS"; default:return "AUTO_GIRO"; }
}
void aplicaGanancias(){
pidMI.config(G.mi_kp, G.mi_ki, 0, PWM_MAX, 0.2);
pidMD.config(G.md_kp, G.md_ki, 0, PWM_MAX, 0.2);
pidYawGiro.config(G.g_kp, G.g_ki, G.g_kd, 120, 30);
pidYawMarcha.config(G.y_kp, 0, G.y_kd, 40, 50);
}
void imprimeGanancias(){
Serial.printf("Ganancias (%s):\n", gananciasGuardadas ? "guardadas en flash" : "por defecto");
Serial.printf("  Rueda izq : kp=%.2f ki=%.2f\n", G.mi_kp, G.mi_ki);
Serial.printf("  Rueda der : kp=%.2f ki=%.2f\n", G.md_kp, G.md_ki);
Serial.printf("  Giro      : kp=%.3f ki=%.3f kd=%.4f\n", G.g_kp, G.g_ki, G.g_kd);
Serial.printf("  Rumbo     : kp=%.3f kd=%.4f\n", G.y_kp, G.y_kd);
}
void cambiaFase(int f_int){
Fase f = (Fase)f_int;
fase=f; tFase=millis();
pidYawMarcha.reset(); pidYawGiro.reset();
switch(f){
case REPOSO:          motoresParar(); rampa.actual=0; pidMI.reset(); pidMD.reset();
ledsColor(10,10,10); break;
case IR: case VOLVER: ledsColor(0,30,0); break;
case FRENAR:          ledsColor(30,20,0); break;
case GIRAR:           ledsColor(0,0,30); break;
case ESPERAR:         ledsColor(20,0,20); break;
case AUTO_RUEDAS: case AUTO_GIRO: ledsColor(0,25,25); break;
}
}
void empiezaRuta(){
x=0; y=0; yaw=0; yawObjetivo=0; objX=XFIN; objY=YFIN;
rampa.actual=0; pidMI.reset(); pidMD.reset();
Serial.println(">> Ruta");
cambiaFase(IR);
}
void empiezaAutoRuedas(){
float b = feedforward(AT_VEL);
float h = min(AT_H_RUEDAS, b);            // sin invertir el motor
hMI = hMD = h; ronda = 0;
releMI.inicio(AT_VEL, hMI, AT_EPS_VEL);
releMD.inicio(AT_VEL, hMD, AT_EPS_VEL);
tAuto=0;
Serial.printf(">> Autoajuste RUEDAS (robot en el aire), %d rondas.\n", AT_RONDAS);
Serial.printf("   Base %.0f PWM, rele +-%.0f\n", b, h);
cambiaFase(AUTO_RUEDAS);
}
void empiezaAutoGiro(){
yawAuto0 = yaw; hYaw = AT_H_GIRO; ronda = 0;
releYaw.inicio(0, hYaw, AT_EPS_YAW); tAuto=0;
Serial.printf(">> Autoajuste GIRO (robot en el suelo), %d rondas. Rele +-%.0f PWM\n", AT_RONDAS, hYaw);
cambiaFase(AUTO_GIRO);
}
void abortaAuto(const char* motivo){
motoresParar();
Serial.printf("!! Autoajuste cancelado: %s\n", motivo);
cambiaFase(REPOSO);
for (int i=0;i<3;i++){ ledsColor(60,0,0); delay(150); ledsColor(0,0,0); delay(150); }
ledsColor(10,10,10);
}
// ------------------------- SETUP -------------------------------------
void calibraGiro(){
const int N=2000; double s=0; int32_t g[3];
for (int i=0;i<N;i++){ imu.Get_G_Axes(g); s += g[2]/1000.0; delayMicroseconds(600); }
sesgoGz = s/N;
}
void ayuda(){
Serial.println(F("Comandos: r=ruta  s=parar  a=autoajuste ruedas (en el aire)"));
Serial.println(F("          g=autoajuste giro (en el suelo)  p=ver ganancias"));
Serial.println(F("          b=borrar ganancias guardadas  h=ayuda   | Boton BOOT = ruta"));
}
void setup(){
Serial.begin(115200);
uint32_t t0=millis(); while(!Serial && millis()-t0<2000);
leds.begin(); leds.setBrightness(30); ledsColor(0,0,0);
pwmInit(); motoresParar();
pinMode(PIN_BOTON, INPUT_PULLUP);
ESP32Encoder::useInternalWeakPullResistors = puType::up;
encMI.attachFullQuad(PIN_ENC_MI_A,PIN_ENC_MI_B); encMI.setCount(0);
encMD.attachFullQuad(PIN_ENC_MD_A,PIN_ENC_MD_B); encMD.setCount(0);
dev_spi.begin(SPI_SCK,SPI_MISO,SPI_MOSI,SPI_CS);
if (imu.begin()!=0) fallo("¡Error con LSM6DSR!");
imu.Enable_X(); imu.Enable_G();
imu.Set_G_FS(2000);  imu.Set_G_ODR(1666.0f);
imu.Set_X_FS(4);     imu.Set_X_ODR(1666.0f);
ledsColor(30,0,0);
Serial.println("Calibrando: no mover el robot");
delay(1500);
calibraGiro();
Serial.printf("Sesgo gz = %.3f dps\n", sesgoGz);
cargaGanancias();
aplicaGanancias();
imprimeGanancias();
ayuda();
cambiaFase(REPOSO);
if (ARRANQUE_AUTOMATICO) empiezaRuta();
}
// ------------------------- LAZO DE CONTROL ---------------------------
void control(float dt){
// 1) encoders -> velocidades
long cMI = encMI.getCount()*SIGNO_ENC_MI, cMD = encMD.getCount()*SIGNO_ENC_MD;
long dMI = cMI-cntMIant, dMD = cMD-cntMDant; cntMIant=cMI; cntMDant=cMD;
float vMI = d*MIM_POR_PULSO/dt, vMD = dMDM_POR_PULSO/dt;
velMI = 0.7fvelMI + 0.3fvMI;
velMD = 0.7fvelMD + 0.3fvMD;
// 2) giroscopio -> yaw
int32_t g[3]; imu.Get_G_Axes(g);
gzActual = SIGNO_YAW*(g[2]/1000.0f - sesgoGz);
yaw = norm180(yaw + gzActual*dt);
// 3) odometría
float dist = 0.5f*(dMI+dMD)M_POR_PULSO;
float r = yawDEG_TO_RAD;
x += distcosf(r); y += distsinf(r);
// 4) modos de autoajuste: mandan directamente en los motores
if (fase==AUTO_RUEDAS){
tAuto += dt;
float b = feedforward(AT_VEL);
motor(CH_MI1,CH_MI2, b + releMI.paso(velMI,dt));
motor(CH_MD1,CH_MD2, b + releMD.paso(velMD,dt));
if (tAuto > AT_TIMEOUT_S){
abortaAuto(releMI.ciclos+releMD.ciclos < 2 ?
"las ruedas no oscilan: sube AT_H_RUEDAS o revisa encoders/feedforward" : "tiempo agotado");
return;
}
if (releMI.listo() && releMD.listo()){
kuI_r[ronda]=releMI.Ku(); tuI_r[ronda]=releMI.Tu();
kuD_r[ronda]=releMD.Ku(); tuD_r[ronda]=releMD.Tu();
Serial.printf("Ronda %d/%d izq: Ku=%.1f Tu=%.3fs amp=%.3f h=%.0f | der: Ku=%.1f Tu=%.3fs amp=%.3f h=%.0f\n",
ronda+1, AT_RONDAS, kuI_r[ronda], tuI_r[ronda], releMI.amp(), hMI,
kuD_r[ronda], tuD_r[ronda], releMD.amp(), hMD);
ronda++;
if (ronda < AT_RONDAS){
// afina: ajusta la fuerza del relé para que la oscilación quede cerca de AT_AMP_RUEDAS
float b = feedforward(AT_VEL);
hMI = reajustaH(hMI, releMI.amp(), AT_AMP_RUEDAS, 5, b);
hMD = reajustaH(hMD, releMD.amp(), AT_AMP_RUEDAS, 5, b);
releMI.inicio(AT_VEL, hMI, AT_EPS_VEL);
releMD.inicio(AT_VEL, hMD, AT_EPS_VEL);
tAuto = 0;
return;                              // sigue oscilando sin parar los motores
}
motoresParar();
// Tyreus-Luyben (PI con poco sobreimpulso): kp = Ku/3.2 ; Ti = 2.2·Tu, con la mediana de las rondas
float kuI=mediana(kuI_r,AT_RONDAS), tuI=mediana(tuI_r,AT_RONDAS);
float kuD=mediana(kuD_r,AT_RONDAS), tuD=mediana(tuD_r,AT_RONDAS);
G.mi_kp = kuI/3.2f; G.mi_ki = G.mi_kp/(2.2ftuI);
G.md_kp = kuD/3.2f; G.md_ki = G.md_kp/(2.2ftuD);
float dI=dispersion(kuI_r,AT_RONDAS), dD=dispersion(kuD_r,AT_RONDAS);
Serial.printf("Mediana izq: Ku=%.1f Tu=%.3fs (dispersion %.0f%%)\n", kuI, tuI, 100dI);
Serial.printf("Mediana der: Ku=%.1f Tu=%.3fs (dispersion %.0f%%)\n", kuD, tuD, 100dD);
if (dI>0.25 || dD>0.25)
Serial.println("Aviso: las rondas no coinciden (>25%). Repite 'a' o revisa holguras/encoders.");
guardaGanancias(); aplicaGanancias(); imprimeGanancias();
Serial.println("<< Autoajuste de ruedas terminado y guardado");
cambiaFase(REPOSO);
}
return;
}
if (fase==AUTO_GIRO){
tAuto += dt;
float m = norm180(yaw - yawAuto0);
if (fabs(m) > 90){ abortaAuto("se va de giro: revisa SIGNO_YAW o el sentido de los motores"); return; }
float giro = releYaw.paso(m, dt);
motor(CH_MI1,CH_MI2,  giro);
motor(CH_MD1,CH_MD2, -giro);
if (tAuto > AT_TIMEOUT_S){
abortaAuto(releYaw.ciclos < 2 ? "no oscila: sube AT_H_GIRO" : "tiempo agotado");
return;
}
if (releYaw.listo()){
kuY_r[ronda]=releYaw.Ku(); tuY_r[ronda]=releYaw.Tu();
Serial.printf("Ronda %d/%d giro: Ku=%.2f Tu=%.3fs amp=%.2f grados h=%.0f\n",
ronda+1, AT_RONDAS, kuY_r[ronda], tuY_r[ronda], releYaw.amp(), hYaw);
ronda++;
if (ronda < AT_RONDAS){
hYaw = reajustaH(hYaw, releYaw.amp(), AT_AMP_GIRO, 30, 160);
releYaw.inicio(0, hYaw, AT_EPS_YAW);
tAuto = 0;
return;
}
motoresParar();
// Ziegler-Nichols "algo de sobreimpulso": kp=0.33Ku, Ti=0.5Tu, Td=0.33Tu, con la mediana de las rondas
float ku=mediana(kuY_r,AT_RONDAS), tu=mediana(tuY_r,AT_RONDAS);
float dY=dispersion(kuY_r,AT_RONDAS);
if (dY>0.25) Serial.println("Aviso: las rondas no coinciden (>25%). Repite 'g' en un suelo uniforme.");
G.g_kp = 0.33fku; G.g_ki = G.g_kp/(0.5ftu); G.g_kd = G.g_kp0.33ftu;
G.y_kp = 0.5fG.g_kp; G.y_kd = 0.5fG.g_kd;        // en marcha, más suave
Serial.printf("Giro: Ku=%.2f PWM/grado  Tu=%.3fs  amp=%.2f grados\n", ku, tu, releYaw.amp());
guardaGanancias(); aplicaGanancias(); imprimeGanancias();
Serial.println("<< Autoajuste de giro terminado y guardado");
cambiaFase(REPOSO);
}
return;
}
if (fase==REPOSO) return;
// 5) ruta: máquina de estados -> velocidad lineal objetivo y corrección de giro
float vObj = 0, giro = 0;
float dx = objX-x, dy = objY-y, d = sqrtf(dxdx+dydy);
switch (fase){
case IR: case VOLVER: {
float vFreno = sqrtf(2ACELmax(d-TOL_DIST,0.0f));   // v² = 2·a·d
vObj = min(VEL_MAX, vFreno);
if (d > 0.05) yawObjetivo = atan2f(dy,dx)RAD_TO_DEG;
giro = pidYawMarcha.paso(yawObjetivo, yaw, dt, true);
if (d < TOL_DIST || (dxcosf(r)+dy*sinf(r)) < 0) cambiaFase(FRENAR);
} break;
case FRENAR:
vObj = 0;
giro = pidYawMarcha.paso(yawObjetivo, yaw, dt, true);
if (fabs(rampa.actual)<0.01 && fabs(velMI)+fabs(velMD)<0.02){
yawObjetivo = norm180(yawObjetivo+180); cambiaFase(GIRAR);
}
break;
case GIRAR:
vObj = 0;
giro = pidYawGiro.paso(yawObjetivo, yaw, dt, true);
if (fabs(norm180(yawObjetivo-yaw)) < TOL_YAW && fabs(gzActual) < 20) cambiaFase(ESPERAR);
break;
case ESPERAR:
vObj = 0;
giro = pidYawGiro.paso(yawObjetivo, yaw, dt, true);
if (millis()-tFase > ESPERA_MS){
bool enOrigen = (objX==0 && objY==0);
if (enOrigen && !REPETIR){ Serial.println("<< Ruta terminada"); cambiaFase(REPOSO); return; }
objX = enOrigen ? XFIN : 0; objY = enOrigen ? YFIN : 0;
cambiaFase(enOrigen ? IR : VOLVER);
}
break;
default: break;
}
// 6) rampa + PID de velocidad por rueda + feedforward
float v = rampa.paso(vObj, ACEL, dt);
float uMI = feedforward(v) + pidMI.paso(v, velMI, dt);
float uMD = feedforward(v) + pidMD.paso(v, velMD, dt);
if (v == 0 && vObj == 0){ uMI = uMD = 0; pidMI.reset(); pidMD.reset(); }
motor(CH_MI1,CH_MI2, uMI + giro);
motor(CH_MD1,CH_MD2, uMD - giro);
}
// ------------------------- ÓRDENES -----------------------------------
void ordenes(){
while (Serial.available()){
char c = tolower(Serial.read());
bool ocupado = (fase!=REPOSO);
switch (c){
case 'r': if (ocupado) Serial.println("Ocupado: 's' para parar"); else empiezaRuta(); break;
case 'a': if (ocupado) Serial.println("Ocupado: 's' para parar"); else empiezaAutoRuedas(); break;
case 'g': if (ocupado) Serial.println("Ocupado: 's' para parar"); else empiezaAutoGiro(); break;
case 's': Serial.println(">> Parado"); cambiaFase(REPOSO); break;
case 'p': imprimeGanancias(); break;
case 'b': borraGanancias(); aplicaGanancias(); Serial.println("Ganancias borradas: por defecto"); imprimeGanancias(); break;
case 'h': case '?': ayuda(); break;
default: break;   // ignora saltos de línea y otros
}
}
// Botón BOOT: en reposo arranca la ruta; durante cualquier movimiento, para.
static bool antes = HIGH; static uint32_t tPuls = 0;
bool ahora = digitalRead(PIN_BOTON);
if (antes==HIGH && ahora==LOW && millis()-tPuls > 300){
tPuls = millis();
if (fase==REPOSO) empiezaRuta(); else { Serial.println(">> Parado (boton)"); cambiaFase(REPOSO); }
}
antes = ahora;
}
// ------------------------- LOOP --------------------------------------
uint32_t tCtrl = 0, tTel = 0;
void loop(){
ordenes();
uint32_t ahora = micros();
if (ahora - tCtrl >= T_CONTROL_US){
float dt = (ahora - tCtrl)/1e6f;
tCtrl = ahora;
if (dt < 0.01f) control(dt);
}
if (fase!=REPOSO && millis() - tTel >= T_TELEMETRIA_MS){
tTel = millis();
Serial.printf("fase=%s x=%.3f y=%.3f yaw=%.1f vMI=%.2f vMD=%.2f vRef=%.2f\n",
nombreFase(fase), x, y, yaw, velMI, velMD, rampa.actual);
}
}
