#!/usr/bin/env python3
"""
Calibra el rango real de los servos (no usa la cámara) y lo guarda en la EEPROM del Arduino.

Para cada extremo: mueve el servo despacio hasta que deje de girar, zumbe o el soporte
toque; vuelve 3-5° atrás y marca el límite. seguidor.py lee estos límites al arrancar.

Teclas:
  a / d  -> pan  izquierda / derecha       w / s -> tilt arriba / abajo
  + / -  -> paso de 5° / 1°                 c     -> al centro
  1 / 2  -> marcar PAN mínimo / máximo      3 / 4 -> marcar TILT mínimo / máximo
  g      -> guardar en el Arduino            q / ESC -> salir
"""
import argparse
import time

import cv2
import numpy as np

from seguidor import Arduino, clamp, texto


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=None, help="COM3 o /dev/ttyACM0 (por defecto: autodetecta)")
    args = ap.parse_args()

    ard = Arduino(args.port)
    if ard.ser is None:
        print("Hace falta el Arduino conectado con servos/servos.ino cargado.")
        return
    lim = list(ard.limites)
    pan, tilt = (lim[0] + lim[1]) / 2, (lim[2] + lim[3]) / 2
    paso = 1
    msg, t_msg = "", 0.0
    cv2.namedWindow("Calibrar servos")

    while True:
        ard.envia(f"R,{round(pan * 10)},{round(tilt * 10)}")  # también mantiene vivo el watchdog
        img = np.full((330, 640, 3), 30, np.uint8)
        real = ard.hist[-1][1:] if ard.hist else (pan, tilt)
        lineas = [
            (f"PAN  {pan:5.1f} deg  (real {real[0]:5.1f})   limites marcados: {lim[0]} - {lim[1]}", (255, 255, 255)),
            (f"TILT {tilt:5.1f} deg  (real {real[1]:5.1f})   limites marcados: {lim[2]} - {lim[3]}", (255, 255, 255)),
            (f"Guardados en el Arduino: pan {ard.limites[0]}-{ard.limites[1]}  "
             f"tilt {ard.limites[2]}-{ard.limites[3]}", (0, 200, 0)),
            (f"Paso: {paso} deg", (200, 200, 200)),
            ("a/d pan   w/s tilt   +/- paso   c centro", (200, 200, 200)),
            ("1/2 marcar PAN min/max   3/4 marcar TILT min/max", (200, 200, 200)),
            ("g guardar en Arduino   q salir", (200, 200, 200)),
            ("Lleva cada extremo hasta que el servo pare o zumbe,", (0, 200, 255)),
            ("vuelve 3-5 grados atras y marca el limite.", (0, 200, 255)),
        ]
        for i, (t, col) in enumerate(lineas):
            texto(img, t, (12, 28 + i * 32), col, 0.55)
        if time.perf_counter() - t_msg < 3:
            texto(img, msg, (12, 318), (0, 255, 255), 0.6, 2)
        cv2.imshow("Calibrar servos", img)

        k = cv2.waitKey(60) & 0xFF
        if k in (27, ord("q")) or cv2.getWindowProperty("Calibrar servos", cv2.WND_PROP_VISIBLE) < 1:
            break
        if k == ord("a"):
            pan -= paso
        elif k == ord("d"):
            pan += paso
        elif k == ord("w"):
            tilt -= paso
        elif k == ord("s"):
            tilt += paso
        elif k == ord("+"):
            paso = 5
        elif k == ord("-"):
            paso = 1
        elif k == ord("c"):
            pan, tilt = (lim[0] + lim[1]) / 2, (lim[2] + lim[3]) / 2
        elif k in (ord("1"), ord("2")):
            lim[k - ord("1")] = round(pan)
        elif k in (ord("3"), ord("4")):
            lim[k - ord("1")] = round(tilt)
        elif k == ord("g"):
            if lim[1] - lim[0] < 20 or lim[3] - lim[2] < 20:
                msg = "Rango invalido: el minimo debe ser al menos 20 grados menor que el maximo"
            else:
                ard.error = None
                ard.listo.clear()
                ard.envia("L,{},{},{},{}".format(*lim))
                ok = ard.listo.wait(1.5) and ard.error is None
                msg = "Limites guardados en el Arduino" if ok else "El Arduino rechazo los limites"
            t_msg = time.perf_counter()
        pan, tilt = clamp(pan, 0, 180), clamp(tilt, 0, 180)

    ard.envia("C")
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
