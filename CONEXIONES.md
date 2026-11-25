# Guía de conexiones — Camera tracker pan-tilt

## Material

| Cant. | Componente | Notas |
|---|---|---|
| 1 | Arduino UNO (o Nano) + cable USB | |
| 1 | Protoboard | |
| 2 | Servo SG90 | uno para el eje X (pan) y otro para el Y (tilt) |
| 1 | Fuente externa de 5 V, ≥ 2 A | cargador USB de 5 V/2 A con cable pelado, o 4 pilas AA NiMH (4,8 V). **No uses 9 V ni 12 V: quema los SG90.** |
| 1 | Condensador electrolítico de 470–1000 µF, ≥ 10 V | absorbe los picos de corriente de los servos |
| 3 | LED (verde, rojo, amarillo) + 3 resistencias de 220 Ω | opcional: indican el modo |
| 1 | Sensor PIR HC-SR501 | opcional: solo para el filtro IR (tecla `i`) |
| — | Cables dupont macho-macho | |

Colores del cable del SG90: **marrón = GND**, **rojo = +5 V**, **naranja = señal**.

> Haz todas las conexiones **con el USB y la fuente desconectados**.

## Paso a paso

Llamamos **raíl +** a la línea roja de la protoboard y **raíl −** a la azul.
Muchas protoboards tienen los raíles cortados a la mitad: si es tu caso, une las dos mitades con un cable.

### Alimentación de los servos

1. **Fuente 5 V (+)** → **raíl +** de la protoboard.
2. **Fuente 5 V (−)** → **raíl −** de la protoboard.
3. **Arduino GND** → **raíl −**. *Imprescindible*: sin tierra común la señal no llega bien y los servos tiemblan o no se mueven.
4. **Condensador**: pata larga (+) → **raíl +**; pata de la franja (−) → **raíl −**. Colócalo cerca de donde enchufas los servos. Respeta la polaridad.

> ⚠️ No conectes el + de la fuente externa al pin **5V** del Arduino. El Arduino se alimenta por USB; las dos fuentes solo comparten GND.

### Servo eje X (pan, gira a izquierda/derecha)

5. Marrón → **raíl −**
6. Rojo → **raíl +**
7. Naranja → **Arduino D9**

### Servo eje Y (tilt, sube/baja)

8. Marrón → **raíl −**
9. Rojo → **raíl +**
10. Naranja → **Arduino D10**

### LEDs de estado (opcional)

Para cada LED: pin del Arduino → resistencia de 220 Ω → pata **larga** del LED; pata **corta** del LED → **raíl −**.

11. **D4** → 220 Ω → LED **verde** (modo libre)
12. **D5** → 220 Ω → LED **rojo** (objetivo fijado)
13. **D6** → 220 Ω → LED **amarillo** (objetivo perdido, parpadea)

### Sensor PIR (opcional)

14. PIR **VCC** → **Arduino 5V**
15. PIR **GND** → **raíl −**
16. PIR **OUT** → **Arduino D2**

### Encendido

17. Conecta el **USB** del Arduino al PC.
18. Conecta la **fuente de 5 V** de los servos.

## Resumen de pines

| Arduino | Va a |
|---|---|
| D9 | Señal (naranja) servo **pan / eje X** |
| D10 | Señal (naranja) servo **tilt / eje Y** |
| D4 / D5 / D6 | LED verde / rojo / amarillo (con 220 Ω) |
| D2 | OUT del PIR |
| 5V | VCC del PIR (solo el PIR) |
| GND | Raíl − (común con la fuente de los servos) |

```
 Fuente 5V (+) ──┬───────────── raíl + ── rojo servo X ── rojo servo Y
                 │ [C 1000µF]
 Fuente 5V (−) ──┴──┬────────── raíl − ── marrón servo X ── marrón servo Y ── cátodos LED ── GND PIR
 Arduino GND ───────┘
 Arduino D9  ──────────────── naranja servo X (pan)
 Arduino D10 ──────────────── naranja servo Y (tilt)
 Arduino D4/D5/D6 ── 220Ω ── LED verde/rojo/amarillo
 Arduino D2  ──────────────── OUT PIR      Arduino 5V ── VCC PIR
```

### Sin fuente externa (solo para pruebas)

Puedes conectar el **raíl +** al pin **5V** del Arduino en lugar de a la fuente (y el raíl − a GND).
Funciona con el condensador puesto, pero el USB da ~500 mA y dos SG90 pueden pedir más de 1 A al arrancar:
si el Arduino se reinicia solo, la cámara se desconecta o los servos tiemblan, usa la fuente externa.

## Montaje mecánico

1. Carga el sketch (abajo) **antes** de poner los brazos a los servos: al encender, ambos se colocan a 90°.
2. Con los servos en 90°, pon el brazo del servo pan mirando al frente y el del tilt horizontal.
3. Si el soporte choca en algún extremo, ajusta `PAN_MIN/PAN_MAX` y `TILT_MIN/TILT_MAX` en `servos/servos.ino`.

## Cargar el programa y ejecutar

1. Abre `servos/servos.ino` en el Arduino IDE → Placa **Arduino UNO** → Puerto **COMx** → **Subir**.
   Cierra el Monitor Serie después (si está abierto, Python no puede usar el puerto).
2. En PowerShell, dentro de la carpeta del proyecto:

```powershell
.\venv\Scripts\Activate.ps1
```

```powershell
python seguidor.py
```

   El puerto del Arduino se detecta solo; si no, añade `--port COM3`. La primera vez descarga `yolo26n.pt` y el detector de caras.

3. Calibración rápida:
   - Si la cámara se mueve al lado contrario del objetivo: `--inv-pan` y/o `--inv-tilt`.
   - Si se queda corta o se pasa al corregir: ajusta `--hfov` al campo de visión de tu cámara (60–78° es lo típico en webcams).
   - El rectángulo gris de la imagen es la zona tranquila: los servos solo actúan cuando el objetivo sale de ahí (`--borde`, por defecto 0.55) y paran cuando vuelve al centro (`--vuelta`).
