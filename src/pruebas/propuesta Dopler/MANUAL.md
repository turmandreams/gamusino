# Gamusino fork: manual de uso

Programa: `gamusino_fork/gamusino_fork.ino` · Placa: ESP32-WROOM-32

---

## 1. Cargar el programa

1. **Arduino IDE** → Placa: *ESP32 Dev Module*.
2. **Bibliotecas necesarias** (Gestor de bibliotecas):
   - *Adafruit NeoPixel*
   - *STM32duino LSM6DSR*
   - *ESP32Encoder* (madhephaestus)
   - `Preferences` ya viene con el core ESP32.
3. **Subir el programa.** Si no entra en modo de carga con el robot montado, desconecta el motor de GPIO2 mientras subes el programa (ver `CAMBIOS.md`).
4. **Abrir el Monitor serie:** **115200 baudios**, fin de línea *Nueva línea*.

---

## 2. Qué hace al encender

1. **LED rojo:** calibra el giroscopio durante unos 3 s. **No muevas el robot.**
2. **LED blanco tenue:** **REPOSO**. Espera una orden y los motores están parados.
3. Por el monitor serie imprime las ganancias en uso (por defecto o las guardadas) y la lista de comandos.

> Si quieres que haga la ruta nada más encender, como el programa original, pon `ARRANQUE_AUTOMATICO = true`.

---

## 3. Órdenes

Se escribe la letra en el monitor serie y se pulsa Enter.

| Orden | Qué hace | Cuándo |
|---|---|---|
| `r` | Ruta: ir al punto (1,5 m ; 0), frenar, girar 180°, esperar, volver y girar | En reposo |
| `a` | **Autoajuste de las ruedas** | Robot **en el aire** |
| `g` | **Autoajuste del giro** | Robot **en el suelo** |
| `s` | Parar todo y volver a reposo | Siempre |
| `p` | Ver las ganancias del PID | Siempre |
| `b` | Borrar las ganancias guardadas y volver a las de por defecto | Siempre |
| `h` | Ayuda | Siempre |
| **Botón BOOT** | En reposo arranca la ruta; en marcha **para** | Sin USB |

> **No mantengas pulsado BOOT al encender o al resetear:** la ESP32 entraría en modo de carga de programa.

---

## 4. Colores del LED

| Color | Estado |
|---|---|
| Rojo fijo | Calibrando el giroscopio |
| Rojo parpadeando sin parar | Fallo de la IMU (revisa el cableado SPI) |
| 3 parpadeos rojos | Autoajuste cancelado (el motivo sale por el monitor serie) |
| Blanco tenue | Reposo |
| Verde | Avanzando (ida o vuelta) |
| Naranja | Frenando |
| Azul | Girando 180° |
| Morado | Esperando |
| Cian | Autoajuste en marcha |

---

## 5. Primera puesta en marcha (en este orden)

### Paso 1: comprobar sentidos (robot en reposo, con la mano)
Envía `r` con el robot **en el aire** y mira la telemetría unos segundos; luego envía `s`.
- **`vMI` y `vMD` deben salir positivas** con las ruedas girando hacia delante. Si una sale negativa, cambia su `SIGNO_ENC_MI` o `SIGNO_ENC_MD` a `-1`.
- Si una rueda gira hacia atrás, el motor está cruzado: intercambia sus dos pines en la configuración, o los cables.

Después, en el suelo, gira el robot a mano **hacia la izquierda** con la ruta en marcha:
- **El `yaw` debe subir.** Si baja, cambia `SIGNO_YAW` (de -1 a 1).

### Paso 2: metros por pulso
1. Marca el suelo, pon el robot en reposo y anota los pulsos (envía `r` y `s` al momento, o añade un `Serial.println(encMI.getCount())`).
2. Empújalo a mano **1 m** en línea recta.
3. Ajusta `DIAM_RUEDA_M` o `PULSOS_POR_VUELTA` hasta que **x ≈ 1,00**. Con 20 mm y 5888 pulsos, el programa usa 1,067e-5 m por pulso; el original usaba 5,336e-6.

### Paso 3: autoajuste de las ruedas (`a`)
1. Pon el robot **con las ruedas en el aire** (sobre un soporte).
2. Envía `a`. El LED se pone cian y las ruedas aceleran y frenan solas alrededor de 0,8 m/s durante unos segundos.
3. Al terminar, el programa imprime `Ku`, `Tu` y la amplitud de cada rueda, **guarda las ganancias** y vuelve a reposo.

**Si se cancela:**
- *"las ruedas no oscilan"*: sube `AT_H_RUEDAS` (por ejemplo a 35) o revisa la tabla `FF_PWM`. Con el PWM base del feedforward la rueda debe acercarse a 0,8 m/s.
- *"tiempo agotado"*: oscila pero de forma irregular. Revisa los encoders y prueba a subir `AT_EPS_VEL` a 0,04.

### Paso 4: autoajuste del giro (`g`)
1. Robot **en el suelo**, con al menos 30 cm libres alrededor.
2. Envía `g`. El robot gira a izquierda y derecha alrededor de su rumbo inicial durante unos segundos.
3. Al terminar, imprime `Ku` y `Tu`, **guarda** las ganancias del giro y las del rumbo en marcha, y vuelve a reposo.

**Si se cancela:**
- *"se va de giro"*: gira cada vez más hacia un lado. `SIGNO_YAW` o el sentido de los motores está al revés (paso 1).
- *"no oscila"*: no vence el rozamiento del suelo. Sube `AT_H_GIRO` (por ejemplo a 90).

### Paso 5: probar la ruta (`r` o botón BOOT)
Pon el robot en el origen mirando hacia +x y pulsa BOOT. Va a 1,5 m, gira y vuelve.
- **Se queda corto o se pasa:** revisa el paso 2.
- **Culebrea al ir recto:** baja `y_kp` (envía `p` para ver los valores; los cambios a mano se hacen en `G_DEFECTO` y después `b`).
- **Patina al arrancar o al frenar:** baja `ACEL`.

---

## 6. Cómo funciona el autoajuste (método del relé)

En lugar de un PID, el programa aplica un **relé**: potencia alta cuando la medida está por debajo del objetivo y baja cuando está por encima. Eso fuerza una **oscilación estable**, y de ella se miden:
- `Tu`: el periodo de la oscilación.
- `a`: su amplitud.
- `Ku = 4·h / (π·a)`: la ganancia crítica, donde `h` es lo que sube o baja el relé.

### Varias rondas para afinar
Cada autoajuste hace **3 rondas seguidas** (`AT_RONDAS`) sin parar los motores:
1. **Ronda 1:** usa la fuerza de relé de partida (`AT_H_RUEDAS` / `AT_H_GIRO`).
2. **Entre rondas** reajusta esa fuerza para que la oscilación quede cerca de la amplitud buscada (`AT_AMP_RUEDAS` = 0,10 m/s, `AT_AMP_GIRO` = 4°). Con una oscilación demasiado grande la medida sale exagerada; con una demasiado pequeña, el rozamiento la falsea.
3. **Al final** usa la **mediana** de las rondas, de modo que una ronda mala no estropea el resultado, y muestra la **dispersión** entre ellas.

Si la dispersión pasa del 25 %, avisa: repite el autoajuste. Si sigue saliendo alta, hay algo mecánico (holguras, encoder que pierde pulsos, suelo irregular).

**Afinado recomendado:**
1. Haz `a` 2 o 3 veces seguidas y compara los `Ku`/`Tu` de las medianas. Si coinciden en ±10 %, ya está.
2. Igual con `g`, en el **mismo suelo** donde vaya a correr.
3. Repite el autoajuste si cambias ruedas, batería, peso o suelo.
4. Retoque fino a mano, si hace falta: con `p` ves los valores. Si sobrepasa, baja `kp` un 20 %; si va perezoso, súbelo. Para fijar unos valores a mano, ponlos en `G_DEFECTO`, carga el programa y envía `b`.

### Cálculo
En cada ronda se descartan los 2 primeros ciclos y se promedian 6. Con la mediana de las rondas se calculan las ganancias así:

| Lazo | Regla | Fórmulas |
|---|---|---|
| Velocidad de cada rueda (PI) | Tyreus-Luyben (poco sobreimpulso) | kp = Ku/3,2 · ki = kp/(2,2·Tu) |
| Giro en el sitio (PID) | Ziegler-Nichols "algo de sobreimpulso" | kp = 0,33·Ku · ki = kp/(0,5·Tu) · kd = kp·0,33·Tu |
| Rumbo en marcha (PD) | La mitad del giro | kp = 0,5·kp_giro · kd = 0,5·kd_giro |

Las ganancias se guardan en la **memoria flash** de la ESP32: **se conservan al apagar** y al volver a cargar el programa. Para volver a las de por defecto, envía `b`.

---

## 7. Parámetros que puedes tocar (arriba del programa)

| Parámetro | Para qué |
|---|---|
| `XFIN`, `YFIN` | Punto de destino de la ruta (m) |
| `VEL_MAX`, `ACEL` | Velocidad máxima y aceleración |
| `TOL_DIST`, `TOL_YAW` | Cuándo considera que ha llegado o que ha terminado de girar |
| `ESPERA_MS`, `REPETIR` | Pausa tras girar; repetir la ruta sin fin |
| `ARRANQUE_AUTOMATICO` | Hacer la ruta al encender |
| `AT_VEL`, `AT_H_RUEDAS`, `AT_H_GIRO` | Punto de trabajo y fuerza inicial del relé en el autoajuste |
| `AT_RONDAS`, `AT_AMP_RUEDAS`, `AT_AMP_GIRO` | Número de rondas (máx. 8) y oscilación buscada al afinar |
| `FF_V`, `FF_PWM` | Tabla de velocidad a PWM base (feedforward) |
| `G_DEFECTO` | Ganancias que se usan si no hay nada guardado |

---

## 8. Telemetría

Mientras se mueve, imprime una línea cada 200 ms:

```
fase=IR x=0.532 y=0.004 yaw=0.3 vMI=1.21 vMD=1.19 vRef=1.20
```

En el **Serial Plotter** del IDE se ven como gráficas: `vRef` es la consigna y `vMI`/`vMD` lo que hacen las ruedas. Si siguen bien a `vRef`, el PID de las ruedas está bien ajustado.
