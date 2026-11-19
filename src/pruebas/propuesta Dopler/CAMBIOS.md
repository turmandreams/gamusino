# Gamusino (robot ratón): revisión del programa

Placa: ESP32-WROOM-32 (core ESP32 instalado en este PC: 2.0.17).

Ficheros:
- `gamusino_1_corregido/gamusino_1_corregido.ino`: el programa original con **solo los fallos arreglados**. Se comporta igual que antes. Cada cambio está marcado en el código con `[FIX n]`.
- `gamusino_fork/gamusino_fork.ino`: versión **reorganizada y mejorada** (ver la segunda parte de este documento).

> No he podido compilarlos: no hay `arduino-cli` en este PC. Ábrelos en el Arduino IDE y compílalos antes de cargarlos.

---

## 1. Problemas encontrados y correcciones

| # | Problema | Efecto en el robot | Corrección |
|---|---|---|---|
| 1 | `calulaPID(PID pide, …)` recibe el PID **por copia**. `integral`, `errorant` e `instante` no se guardan nunca. `pide.instante` siempre vale 0, así que `dt` es el tiempo desde que arrancó la placa. | La integral y la derivada no hacen lo que deberían; en la práctica solo funciona la parte proporcional. En el yaw la derivada es casi nula. | Se pasa por referencia (`PID &pide`), se guarda el instante y `dt` va en **segundos**. Se inicializa `instante` con `micros()`. **Hay que reajustar `ki` y `kd`**, porque ahora sí actúan. |
| 2 | Rampa: `velpermitida = velinicial + t*aceleracion/1000` con `t = millis()` absoluto, no el tiempo transcurrido. Además la fórmula solo sube. | No hay rampa: la consigna salta de golpe a `veldeseada` y el robot patina al arrancar. Al frenar, la velocidad cae a 0 de golpe. | Se usa `millis()-t`, con rampa también de bajada. Los motores se cortan cuando la rampa llega a 0, no antes. Se resetea la integral al parar. |
| 3 | `veltopwmdef()` **no devuelve nada** por encima de 1,70 m/s, y con velocidades negativas devuelve +10. | Comportamiento indefinido (PWM basura). Marcha atrás con el feedforward en sentido contrario. | Siempre devuelve un valor y respeta el signo. Para velocidad 0 devuelve 0. |
| 4 | El yaw no se normaliza: `atan2` da ±180, pero `anguloZ` y `yawDeseado += 180` crecen sin límite. | Al cruzar ±180 el error salta casi 360° y el robot gira por el lado largo o da vueltas. | Función `normaliza180()` para `anguloZ`, `yawDeseado` y el error del PID. |
| 5 | `M_POR_P = 5.336e-6`, pero con rueda de 20 mm y 5888 pulsos por vuelta sale π·0,020/5888 = **1,067e-5**: el doble. `METROS_POR_PULSO` estaba escrito aparte. | Si el dato real es 5888, el robot recorre el doble de lo que cree: x/y y la velocidad salen a la mitad. | En la versión corregida **se deja el valor original** (puede que la cuenta sea 11776 por vuelta) y `METROS_POR_PULSO` se calcula a partir de `M_POR_P`. **Mídelo:** empuja el robot 1 m a mano y mira cuántos pulsos cuenta. |
| 6 | `angulohaciaobjetivo()` recalcula el rumbo aunque esté encima del objetivo. | Al llegar, `atan2` de diferencias minúsculas da direcciones al azar y el robot culebrea. | No se recalcula a menos de 3 cm del objetivo. |
| 7 | Al reiniciar el ciclo se pone `anguloZ = 0` pero no `x` ni `y`. | La siguiente ida calcula el rumbo con una posición vieja. | Se reinician también `x` e `y`. |
| 8 | `para = true` no vuelve nunca a `false`, y `estado--` justo antes de `estado = 0` no sirve. | El robot se para para siempre tras la primera ida y vuelta. | Se mantiene, por si era intencionado, con un comentario. Se quita el `estado--`. |
| 9 | Tipos: `long t = 0.0`, `int maximo = 0.0`, `instante` con signo, `(int)salida` dentro del PID. | Al desbordarse `micros()` (cada ~71 min) `dt` sale negativo. El PID trunca su salida. | `unsigned long` para los tiempos, resta sin signo, y el PID devuelve `double`. |
| 10 | `ledcSetup` y `ledcAttachPin` no existen en el core ESP32 3.x. | No compila si actualizas el core. | `#if` para 2.x y 3.x (`ledcAttachChannel` / `ledcWriteChannel`). |
| 11 | Si la IMU falla, `while(1);` se queda colgado sin avisar. | Robot muerto sin saber por qué si no hay USB. | LED rojo parpadeando. |

### Avisos que no son fallos del código
- **Pines de arranque de la ESP32-WROOM-32:** GPIO2 (motor MI1) y GPIO5 (CS de la IMU) intervienen en el arranque. Si la placa no arranca o no carga el programa con el robot montado, desconecta el driver de GPIO2 al programar o mueve ese motor a otro pin (por ejemplo, 4 o 14).
- **El lazo cada 50 µs** cuenta muy pocos pulsos por muestra, así que la velocidad sale muy ruidosa. Además, el giroscopio solo da un dato nuevo cada 600 µs. En la versión corregida se deja así para no cambiar el comportamiento; el fork lo pasa a 1 ms.
- **`while (!Serial);`** en la ESP32 no bloquea, pero en otras placas sí. En el fork espera como mucho 2 s.

---

## 2. Versión fork: qué mejora y por qué

| Mejora | Por qué |
|---|---|
| **Lazo de control fijo a 1 kHz** (`T_CONTROL_US`) | Más pulsos por muestra, velocidad menos ruidosa; coincide con la frecuencia de datos del giroscopio. Libera CPU. |
| **PID nuevo**: `dt` en segundos, **anti-windup**, derivada sobre la medida y filtrada, modo angular | La integral no se dispara cuando el motor está saturado. La derivada no da picos al cambiar la consigna (por ejemplo al pedir +180°). El error angular siempre va por el camino corto. |
| **Dos PID de yaw**: uno suave para ir recto y otro fuerte para girar en el sitio | Antes se cambiaban las ganancias a mano en cada estado. |
| **Rampa trapezoidal real** (`Rampa`) en subida y bajada | Arranques y frenadas sin patinar, que estropean la odometría. |
| **Frenada calculada por distancia** (v² = 2·a·d) | El robot empieza a frenar justo a tiempo para parar en el punto, no por tiempo. |
| **Máquina de estados por eventos** (`IR → FRENAR → GIRAR → ESPERAR → VOLVER → FIN`) | Antes todo iba por temporizador de 800 ms: si el robot iba más lento o más rápido, se desincronizaba. Ahora cada fase termina cuando se cumple su condición (llegado, parado, girado). |
| **Detecta si se ha pasado del objetivo** | No vuelve atrás en zigzag si se pasa unos milímetros. |
| **Feedforward interpolado** (no por escalones) y con signo | El PID trabaja menos y sin saltos al cruzar 0,4, 0,6, … m/s. |
| **Mecánica parametrizada**: diámetro, pulsos por vuelta y signo de cada encoder y del giroscopio | `M_POR_PULSO` se calcula solo y hay un único sitio donde ajustar cada cosa. **Ojo:** aquí uso 20 mm y 5888 pulsos, es decir 1,067e-5 (ver punto 5): comprueba cuál es el valor correcto. |
| **Giroscopio a ±2000 dps** (antes ±4000) | El doble de resolución; un robot ratón no gira a más de 2000 °/s. |
| **LED por fase**: verde va, naranja frena, azul gira, morado espera, blanco fin, rojo parpadeando error | Sabes qué está haciendo sin cable USB. |
| **Telemetría en una línea cada 200 ms** | Se lee bien en el Serial Plotter o para pasar a CSV; no afecta al control. |
| **Parada segura** al fallar la IMU y al terminar | Los motores se ponen a 0 antes de cualquier bloqueo. |
| **Autoajuste del PID por relé** (`a` ruedas, `g` giro), con **3 rondas**, reajuste de la fuerza del relé entre rondas, mediana y aviso de dispersión | Las ganancias salen del propio robot y no a ojo. Varias rondas eliminan las medidas malas. |
| **Ganancias guardadas en flash** (`Preferences`) | Se conservan al apagar y al volver a cargar el programa. `b` vuelve a las de por defecto. |
| **Arranque en reposo y órdenes por serie y botón BOOT** | Permite ajustar sin que el robot salga corriendo. `ARRANQUE_AUTOMATICO = true` recupera el comportamiento original. |

Instrucciones de uso: **`MANUAL.md`**.

### Ganancias de partida del fork (hay que ajustarlas)
Unidades: error en m/s o en grados; salida en PWM (0–255).
- Velocidad por rueda: `kp=120, ki=600`. Empieza con `ki=0`, sube `kp` hasta que siga bien la consigna sin vibrar, y luego añade `ki`.
- Yaw en marcha: `kp=1.5, kd=0.05`.
- Yaw girando: `kp=3, ki=2, kd=0.08`, con salida máxima 120.

### Antes de la primera prueba (con las ruedas en el aire)
1. Comprueba que los dos encoders cuentan positivo hacia delante. Si no, cambia `SIGNO_ENC_MI` o `SIGNO_ENC_MD` a -1.
2. Gira el robot a mano hacia la izquierda: el yaw debe subir. Si baja, cambia `SIGNO_YAW`.
3. Mide los metros por pulso (punto 5).
