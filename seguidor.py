#!/usr/bin/env python3
"""
Seguidor de objetos con cámara USB + servos pan-tilt vía Arduino.
Controles:
  Mouse hover  -> resalta el recuadro bajo el cursor
  Clic izq.    -> fija el objeto (se ocultan los demás)
  Clic der. / r-> libera (modo libre)
  i            -> activa/desactiva filtro IR (solo personas + PIR)
  c            -> centra los servos
  q / ESC      -> salir
"""
import argparse
import threading
import time

import cv2
from ultralytics import YOLO

try:
    import serial
except ImportError:
    serial = None


# ---------------------- Enlace con Arduino ----------------------
class Arduino:
    def __init__(self, port, baud=115200):
        self.ser = None
        self.pir = True  # sin Arduino no se bloquea el filtro
        if port and serial:
            try:
                self.ser = serial.Serial(port, baud, timeout=0.1)
                time.sleep(2.0)  # el UNO se reinicia al abrir el puerto
                self.pir = False
                threading.Thread(target=self._lee, daemon=True).start()
                print(f"[OK] Arduino en {port}")
            except Exception as e:
                print(f"[AVISO] No se pudo abrir {port}: {e}. Modo simulación.")
        else:
            print("[AVISO] Sin puerto serie: modo simulación (no se mueven servos).")

    def _lee(self):
        while True:
            try:
                linea = self.ser.readline().decode(errors="ignore").strip()
            except Exception:
                return
            if linea.startswith("IR,"):
                self.pir = linea.endswith("1")

    def envia(self, txt):
        if self.ser:
            try:
                self.ser.write((txt + "\n").encode())
            except Exception:
                pass


# ---------------------- Captura en hilo (baja latencia) ----------------------
class Camara:
    def __init__(self, idx, w, h):
        self.cap = cv2.VideoCapture(idx)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not self.cap.isOpened():
            raise RuntimeError(f"No se pudo abrir la cámara {idx}")
        self.frame = None
        self.run = True
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while self.run:
            ok, f = self.cap.read()
            if ok:
                self.frame = f

    def lee(self):
        f = self.frame
        self.frame = None
        return f

    def cierra(self):
        self.run = False
        time.sleep(0.1)
        self.cap.release()


# ---------------------- Estado del mouse ----------------------
mouse = {"x": -1, "y": -1, "clic": False, "libera": False}


def cb_mouse(evento, x, y, flags, param):
    mouse["x"], mouse["y"] = x, y
    if evento == cv2.EVENT_LBUTTONDOWN:
        mouse["clic"] = True
    elif evento == cv2.EVENT_RBUTTONDOWN:
        mouse["libera"] = True


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cam", type=int, default=0)
    ap.add_argument("--port", default=None, help="COM3 o /dev/ttyACM0")
    ap.add_argument("--model", default="yolov8n.pt")
    ap.add_argument("--imgsz", type=int, default=320)
    ap.add_argument("--conf", type=float, default=0.35)
    ap.add_argument("--w", type=int, default=640)
    ap.add_argument("--h", type=int, default=480)
    ap.add_argument("--kp", type=float, default=5.0, help="grados por cuadro con error máximo")
    ap.add_argument("--inv-pan", action="store_true", help="invierte el sentido del PAN")
    ap.add_argument("--inv-tilt", action="store_true", help="invierte el sentido del TILT")
    ap.add_argument("--zona-muerta", type=float, default=0.08)
    ap.add_argument("--perdido-max", type=int, default=90, help="cuadros antes de volver a libre")
    args = ap.parse_args()

    PAN_MIN, PAN_MAX = 10, 170
    TILT_MIN, TILT_MAX = 40, 140
    sg_pan = -1 if args.inv_pan else 1
    sg_tilt = -1 if args.inv_tilt else 1

    ard = Arduino(args.port)
    cam = Camara(args.cam, args.w, args.h)
    model = YOLO(args.model)

    cv2.namedWindow("Seguidor")
    cv2.setMouseCallback("Seguidor", cb_mouse)

    pan, tilt = 90.0, 90.0
    ard.envia("C")
    ard.envia("M0")

    modo = 0                    # 0 libre, 1 fijado, 2 perdido
    id_fijo, cls_fijo = None, None
    ultimo_c = None             # último centro conocido del objetivo
    perdido = 0
    filtro_ir = False
    ult_envio = (None, None)
    t_prev, fps = time.time(), 0.0
    t_hb = 0

    while True:
        frame = cam.lee()
        if frame is None:
            if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                break
            time.sleep(0.003)
            continue

        H, W = frame.shape[:2]
        clases = [0] if filtro_ir else None
        r = model.track(frame, persist=True, tracker="bytetrack.yaml",
                        imgsz=args.imgsz, conf=args.conf, classes=clases,
                        verbose=False)[0]

        dets = []
        if r.boxes is not None and r.boxes.id is not None:
            xyxy = r.boxes.xyxy.cpu().numpy()
            ids = r.boxes.id.int().cpu().tolist()
            cl = r.boxes.cls.int().cpu().tolist()
            cf = r.boxes.conf.cpu().tolist()
            for b, i, c, p in zip(xyxy, ids, cl, cf):
                x1, y1, x2, y2 = map(int, b)
                dets.append(dict(id=i, cls=c, conf=p, box=(x1, y1, x2, y2),
                                 area=(x2 - x1) * (y2 - y1)))

        # ---- Hover: el recuadro más pequeño bajo el cursor ----
        hover = None
        mx, my = mouse["x"], mouse["y"]
        for d in dets:
            x1, y1, x2, y2 = d["box"]
            if x1 <= mx <= x2 and y1 <= my <= y2:
                if hover is None or d["area"] < hover["area"]:
                    hover = d

        # ---- Eventos de mouse ----
        if mouse["libera"]:
            mouse["libera"] = False
            modo, id_fijo = 0, None
            ard.envia("M0")
        if mouse["clic"]:
            mouse["clic"] = False
            if modo == 0 and hover is not None:
                id_fijo, cls_fijo = hover["id"], hover["cls"]
                x1, y1, x2, y2 = hover["box"]
                ultimo_c = ((x1 + x2) / 2, (y1 + y2) / 2)
                perdido = 0
                modo = 1
                ard.envia("M1")

        # ---- Lógica de seguimiento ----
        objetivo = None
        if modo in (1, 2):
            objetivo = next((d for d in dets if d["id"] == id_fijo), None)
            if objetivo is None and ultimo_c is not None:
                # Reacquisición: misma clase, cercano a la última posición
                mejor, dmin = None, 0.25 * W
                for d in dets:
                    if d["cls"] != cls_fijo:
                        continue
                    x1, y1, x2, y2 = d["box"]
                    dist = ((x1 + x2) / 2 - ultimo_c[0]) ** 2 + ((y1 + y2) / 2 - ultimo_c[1]) ** 2
                    dist **= 0.5
                    if dist < dmin:
                        mejor, dmin = d, dist
                if mejor is not None:
                    objetivo = mejor
                    id_fijo = mejor["id"]

            if objetivo is not None:
                if modo == 2:
                    modo = 1
                    ard.envia("M1")
                perdido = 0
                x1, y1, x2, y2 = objetivo["box"]
                cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
                ultimo_c = (cx, cy)
                ex = (cx - W / 2) / (W / 2)     # -1..1
                ey = (cy - H / 2) / (H / 2)
                if abs(ex) > args.zona_muerta:
                    pan = clamp(pan - sg_pan * args.kp * ex, PAN_MIN, PAN_MAX)
                if abs(ey) > args.zona_muerta:
                    tilt = clamp(tilt + sg_tilt * args.kp * ey, TILT_MIN, TILT_MAX)
            else:
                perdido += 1
                if modo == 1:
                    modo = 2
                    ard.envia("M2")
                if perdido > args.perdido_max:
                    modo, id_fijo = 0, None
                    ard.envia("M0")

        # ---- Enviar posición solo si cambió (y latido de 1 Hz) ----
        pos = (int(round(pan)), int(round(tilt)))
        ahora = time.time()
        if pos != ult_envio or ahora - t_hb > 1.0:
            ard.envia(f"P,{pos[0]},{pos[1]}")
            if ahora - t_hb > 1.0:
                ard.envia(f"M{modo}")
                t_hb = ahora
            ult_envio = pos

        # ---- Dibujo ----
        mostrar_libre = (not filtro_ir) or ard.pir
        if modo == 0:
            if mostrar_libre:
                for d in dets:
                    x1, y1, x2, y2 = d["box"]
                    es_h = d is hover
                    col = (0, 255, 255) if es_h else (0, 200, 0)
                    cv2.rectangle(frame, (x1, y1), (x2, y2), col, 3 if es_h else 1)
                    nombre = model.names[d["cls"]]
                    cv2.putText(frame, f"{nombre} #{d['id']} {d['conf']:.2f}",
                                (x1, max(15, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX,
                                0.5, col, 1, cv2.LINE_AA)
        else:
            if objetivo is not None:
                x1, y1, x2, y2 = objetivo["box"]
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 3)
                nombre = model.names[objetivo["cls"]]
                cv2.putText(frame, f"FIJADO: {nombre} #{objetivo['id']}",
                            (x1, max(15, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (0, 0, 255), 2, cv2.LINE_AA)
            else:
                cv2.putText(frame, "OBJETIVO PERDIDO - buscando...", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)

        cv2.drawMarker(frame, (W // 2, H // 2), (255, 255, 255), cv2.MARKER_CROSS, 20, 1)
        dt = time.time() - t_prev
        t_prev = time.time()
        fps = 0.9 * fps + 0.1 * (1.0 / dt if dt > 0 else 0)
        txt_modo = ["LIBRE", "FIJADO", "PERDIDO"][modo]
        cv2.putText(frame,
                    f"{txt_modo} | FPS {fps:.1f} | IR {'ON' if filtro_ir else 'OFF'}"
                    f" (PIR {'1' if ard.pir else '0'}) | pan {pos[0]} tilt {pos[1]}",
                    (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

        cv2.imshow("Seguidor", frame)
        k = cv2.waitKey(1) & 0xFF
        if k in (27, ord("q")):
            break
        elif k == ord("r"):
            modo, id_fijo = 0, None
            ard.envia("M0")
        elif k == ord("i"):
            filtro_ir = not filtro_ir
        elif k == ord("c"):
            pan, tilt = 90.0, 90.0
            ard.envia("C")

    ard.envia("M0")
    cam.cierra()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()