#!/usr/bin/env python3
"""
Seguidor con cámara USB + YOLO26 + servos pan-tilt SG90 vía Arduino (servos/servos.ino).

- Modo libre: detecta y etiqueta objetos; en personas marca la cara.
- Clic sobre un objeto: se fija, los demás desaparecen. En personas se apunta a la cara.
- Los servos solo se mueven cuando el objetivo va a salir del encuadre y lo
  devuelven al centro (histéresis --borde / --vuelta).

Controles:
  Ratón encima  -> resalta el recuadro       Clic izq.     -> fija el objetivo
  Clic der. / r -> libera (modo libre)       i             -> filtro IR (solo personas + PIR)
  c             -> centra los servos         q / ESC       -> salir
"""
import argparse
import math
import sys
import threading
import time
import urllib.request
from collections import deque
from pathlib import Path

import cv2
from ultralytics import YOLO

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None

AQUI = Path(__file__).resolve().parent
YUNET = AQUI / "face_detection_yunet_2023mar.onnx"
YUNET_URL = ("https://github.com/opencv/opencv_zoo/raw/main/models/"
             "face_detection_yunet/face_detection_yunet_2023mar.onnx")
VIDS_ARDUINO = {0x2341, 0x2A03, 0x1A86, 0x0403, 0x10C4}  # Arduino, CH340, FTDI, CP210x
PERSONA = 0
MODOS = ("LIBRE", "FIJADO", "PERDIDO")


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


# ---------------------- Enlace con Arduino ----------------------
class Arduino:
    def __init__(self, port, baud=115200):
        self.ser = None
        self.pir = True                     # sin Arduino el filtro IR no oculta nada
        self.limites = (10, 170, 40, 140)   # se sustituyen por los del READY
        self.hist = deque(maxlen=40)        # (t, pan, tilt) posiciones reales reportadas
        if serial is None:
            print("[AVISO] pyserial no instalado: modo simulación.")
            return
        port = port or next((p.device for p in list_ports.comports() if p.vid in VIDS_ARDUINO), None)
        if not port:
            print("[AVISO] No se encontró Arduino (usa --port COMx): modo simulación.")
            return
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
        except serial.SerialException as e:
            print(f"[AVISO] No se pudo abrir {port}: {e}. Modo simulación.")
            return
        self._listo = threading.Event()
        threading.Thread(target=self._lee, daemon=True).start()
        if self._listo.wait(4.0):  # el UNO se reinicia al abrir el puerto
            print(f"[OK] Arduino en {port}, límites pan/tilt {self.limites}")
        else:
            print(f"[AVISO] {port} no respondió READY: ¿cargaste servos/servos.ino?")

    def _lee(self):
        while True:
            try:
                p = self.ser.readline().decode(errors="ignore").strip().split(",")
            except (serial.SerialException, OSError):
                print("[ERROR] Arduino desconectado.")
                self.ser = None
                return
            try:
                if p[0] == "A" and len(p) == 3:
                    self.hist.append((time.perf_counter(), int(p[1]) / 10, int(p[2]) / 10))
                elif p[0] == "IR" and len(p) == 2:
                    self.pir = p[1] == "1"
                elif p[0] == "READY" and len(p) == 5:
                    self.limites = tuple(int(v) for v in p[1:])
                    self._listo.set()
            except ValueError:
                pass  # línea corrupta

    def posicion_en(self, t):
        """Ángulo real de los servos en el instante t (cuando se capturó el cuadro)."""
        pos = None
        for ti, pan, tilt in list(self.hist):
            if ti > t:
                break
            pos = (pan, tilt)
        return pos or (self.hist[0][1:] if self.hist else None)

    def envia(self, txt):
        if self.ser:
            try:
                self.ser.write((txt + "\n").encode())
            except (serial.SerialException, OSError):
                pass


# ---------------------- Captura en hilo (baja latencia) ----------------------
class Camara:
    def __init__(self, idx, w, h):
        api = cv2.CAP_DSHOW if sys.platform == "win32" else cv2.CAP_ANY  # DSHOW abre mucho más rápido en Windows
        self.cap = cv2.VideoCapture(idx, api)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not self.cap.isOpened():
            raise RuntimeError(f"No se pudo abrir la cámara {idx}")
        self.dato = None  # (frame, t_captura)
        self.run = True
        self.hilo = threading.Thread(target=self._loop, daemon=True)
        self.hilo.start()

    def _loop(self):
        while self.run:
            ok, f = self.cap.read()
            if ok:
                self.dato = (f, time.perf_counter())

    def lee(self):
        d, self.dato = self.dato, None
        return d

    def cierra(self):
        self.run = False
        self.hilo.join(timeout=1.0)
        self.cap.release()


# ---------------------- Caras (YuNet, incluido en OpenCV) ----------------------
class Caras:
    def __init__(self):
        self.det = None
        if not YUNET.exists():
            try:
                print("[INFO] Descargando detector de caras YuNet (~230 KB)...")
                tmp = YUNET.with_suffix(".part")
                urllib.request.urlretrieve(YUNET_URL, tmp)
                tmp.replace(YUNET)
            except OSError as e:
                print(f"[AVISO] Sin detector de caras ({e}); se apuntará a la cabeza estimada.")
                return
        self.det = cv2.FaceDetectorYN.create(str(YUNET), "", (320, 320), 0.7, 0.3, 50)

    def detecta(self, img, ox=0, oy=0):
        """Caras en img como [(x1, y1, x2, y2, score)], desplazadas por (ox, oy)."""
        if self.det is None or img.size == 0:
            return []
        h, w = img.shape[:2]
        s = min(1.0, 320 / max(w, h))  # YuNet va sobrado a 320 px y así cuesta pocos ms en CPU
        if s < 1:
            img = cv2.resize(img, (round(w * s), round(h * s)))
        self.det.setInputSize((img.shape[1], img.shape[0]))
        _, f = self.det.detect(img)
        if f is None:
            return []
        return [(ox + r[0] / s, oy + r[1] / s, ox + (r[0] + r[2]) / s, oy + (r[1] + r[3]) / s, r[14])
                for r in f]


def asigna_caras(dets, caras):
    """A cada persona le asigna la mejor cara cuyo centro cae en el 60 % superior de su caja."""
    for d in dets:
        if d["cls"] != PERSONA:
            continue
        x1, y1, x2, y2 = d["box"]
        dentro = [c for c in caras
                  if x1 <= (c[0] + c[2]) / 2 <= x2 and y1 <= (c[1] + c[3]) / 2 <= y1 + 0.6 * (y2 - y1)]
        if dentro:
            d["cara"] = tuple(map(int, max(dentro, key=lambda c: c[4])[:4]))


def zona_mira(d):
    """Caja a la que se apunta: la cara; si no se ve, la cabeza estimada; en objetos, la caja entera."""
    if d.get("cara"):
        return d["cara"]
    x1, y1, x2, y2 = d["box"]
    if d["cls"] == PERSONA:
        hw = 0.2 * (x2 - x1)
        cx = (x1 + x2) / 2
        return int(cx - hw), y1, int(cx + hw), int(y1 + 0.22 * (y2 - y1))
    return d["box"]


def histograma(frame, box):
    x1, y1, x2, y2 = box
    roi = frame[max(0, y1):y2, max(0, x1):x2]
    if roi.size == 0:
        return None
    h = cv2.calcHist([cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)], [0, 1], None, [16, 16], [0, 180, 0, 256])
    return cv2.normalize(h, h).flatten()


# ---------------------- Control de un eje del pan-tilt ----------------------
class Eje:
    def __init__(self, lo, hi, fov, signo):
        self.lo, self.hi, self.signo = lo, hi, signo
        self.tan = math.tan(math.radians(fov / 2))
        self.cmd = 90.0
        self.activo = False

    def actualiza(self, e, saliendo, real, borde, vuelta):
        """e: posición del objetivo en -1..1 desde el centro; real: ángulo del servo al capturar el cuadro."""
        if saliendo or abs(e) > borde:
            self.activo = True
        elif abs(e) < vuelta:
            self.activo = False
        if self.activo:
            # Ángulo absoluto donde está el objetivo: no acumula error aunque el servo vaya con retraso.
            self.cmd = clamp(real + self.signo * math.degrees(math.atan(e * self.tan)), self.lo, self.hi)


# ---------------------- Estado del ratón ----------------------
mouse = {"x": -1, "y": -1, "clic": False, "libera": False}


def cb_mouse(evento, x, y, flags, param):
    mouse["x"], mouse["y"] = x, y
    if evento == cv2.EVENT_LBUTTONDOWN:
        mouse["clic"] = True
    elif evento == cv2.EVENT_RBUTTONDOWN:
        mouse["libera"] = True


def texto(img, t, org, col, esc=0.5, grosor=1):
    cv2.putText(img, t, org, cv2.FONT_HERSHEY_SIMPLEX, esc, (0, 0, 0), grosor + 2, cv2.LINE_AA)
    cv2.putText(img, t, org, cv2.FONT_HERSHEY_SIMPLEX, esc, col, grosor, cv2.LINE_AA)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cam", type=int, default=0)
    ap.add_argument("--port", default=None, help="COM3 o /dev/ttyACM0 (por defecto: autodetecta)")
    ap.add_argument("--model", default="yolo26n.pt", help="se descarga solo la primera vez")
    ap.add_argument("--imgsz", type=int, default=416)
    ap.add_argument("--conf", type=float, default=0.4)
    ap.add_argument("--w", type=int, default=640)
    ap.add_argument("--h", type=int, default=480)
    ap.add_argument("--hfov", type=float, default=60.0, help="campo de visión horizontal de tu cámara (°)")
    ap.add_argument("--borde", type=float, default=0.55,
                    help="0..1: los servos actúan cuando el objetivo pasa de esta distancia al centro")
    ap.add_argument("--vuelta", type=float, default=0.12, help="0..1: dejan de actuar al volver a esta zona")
    ap.add_argument("--inv-pan", action="store_true", help="invierte el sentido del PAN")
    ap.add_argument("--inv-tilt", action="store_true", help="invierte el sentido del TILT")
    ap.add_argument("--perdido-max", type=float, default=3.0, help="segundos sin verlo antes de liberar")
    args = ap.parse_args()

    ard = Arduino(args.port)
    cam = Camara(args.cam, args.w, args.h)
    model = YOLO(args.model)
    caras = Caras()

    W, H = args.w, args.h
    vfov = math.degrees(2 * math.atan(math.tan(math.radians(args.hfov / 2)) * H / W))
    pmin, pmax, tmin, tmax = ard.limites
    eje_pan = Eje(pmin, pmax, args.hfov, 1 if args.inv_pan else -1)
    eje_tilt = Eje(tmin, tmax, vfov, -1 if args.inv_tilt else 1)

    cv2.namedWindow("Seguidor")
    cv2.setMouseCallback("Seguidor", cb_mouse)
    ard.envia("C")
    ard.envia("M0")

    modo = 0
    fijo = None            # dict: id, cls, centro, hist, t_visto
    filtro_ir = False
    ult_envio, t_hb = None, 0.0
    t_prev, fps = time.perf_counter(), 0.0

    def cambia_modo(m):
        nonlocal modo
        if m != modo:
            modo = m
            ard.envia(f"M{m}")

    while True:
        dato = cam.lee()
        if dato is None:
            if (cv2.waitKey(1) & 0xFF) in (27, ord("q")):
                break
            time.sleep(0.002)
            continue
        frame, t_cap = dato
        H, W = frame.shape[:2]

        if modo:
            clases = [fijo["cls"]]          # fijado: solo su clase, más rápido y sin confusiones
        else:
            clases = [PERSONA] if filtro_ir else None
        r = model.track(frame, persist=True, tracker="bytetrack.yaml", imgsz=args.imgsz,
                        conf=args.conf, classes=clases, verbose=False)[0]

        dets = []
        if r.boxes is not None and r.boxes.id is not None:
            for b, i, c, p in zip(r.boxes.xyxy.int().tolist(), r.boxes.id.int().tolist(),
                                  r.boxes.cls.int().tolist(), r.boxes.conf.tolist()):
                dets.append(dict(id=i, cls=c, conf=p, box=tuple(b),
                                 area=(b[2] - b[0]) * (b[3] - b[1])))

        # ---- Hover: el recuadro más pequeño bajo el cursor ----
        mx, my = mouse["x"], mouse["y"]
        bajo = [d for d in dets if d["box"][0] <= mx <= d["box"][2] and d["box"][1] <= my <= d["box"][3]]
        hover = min(bajo, key=lambda d: d["area"]) if bajo and modo == 0 else None

        # ---- Eventos de ratón ----
        if mouse["libera"]:
            mouse["libera"] = False
            fijo = None
            cambia_modo(0)
        if mouse["clic"]:
            mouse["clic"] = False
            if hover is not None:
                x1, y1, x2, y2 = hover["box"]
                fijo = dict(id=hover["id"], cls=hover["cls"], centro=((x1 + x2) / 2, (y1 + y2) / 2),
                            hist=histograma(frame, hover["box"]), t_visto=t_cap)
                cambia_modo(1)

        # ---- Seguimiento ----
        objetivo = None
        if modo:
            objetivo = next((d for d in dets if d["id"] == fijo["id"]), None)
            if objetivo is None:
                # Reacquisición: misma clase, cerca de donde se vio y con colores parecidos.
                radio = 0.5 * W
                mejor, s_mejor = None, 1e9
                for d in dets:
                    x1, y1, x2, y2 = d["box"]
                    dist = math.dist(((x1 + x2) / 2, (y1 + y2) / 2), fijo["centro"])
                    h = histograma(frame, d["box"])
                    corr = cv2.compareHist(fijo["hist"], h, cv2.HISTCMP_CORREL) if (
                        h is not None and fijo["hist"] is not None) else 0.5
                    s = dist / radio + (1 - corr)
                    if dist < radio and corr > 0.4 and s < s_mejor:
                        mejor, s_mejor = d, s
                if mejor is not None:
                    objetivo = mejor
                    fijo["id"] = mejor["id"]

            if objetivo is not None:
                cambia_modo(1)
                x1, y1, x2, y2 = objetivo["box"]
                fijo["centro"] = ((x1 + x2) / 2, (y1 + y2) / 2)
                fijo["t_visto"] = t_cap
                h = histograma(frame, objetivo["box"])
                if h is not None and fijo["hist"] is not None:
                    fijo["hist"] = 0.9 * fijo["hist"] + 0.1 * h

                if objetivo["cls"] == PERSONA:
                    bw, bh = x2 - x1, y2 - y1
                    cx1, cy1 = max(0, int(x1 - 0.1 * bw)), max(0, y1)
                    recorte = frame[cy1:min(H, int(y1 + 0.6 * bh)), cx1:min(W, int(x2 + 0.1 * bw))]
                    asigna_caras([objetivo], caras.detecta(recorte, cx1, cy1))

                # ---- Servos: solo actúan si el objetivo se está saliendo ----
                ax1, ay1, ax2, ay2 = zona_mira(objetivo)
                ex = ((ax1 + ax2) / 2 - W / 2) / (W / 2)
                ey = ((ay1 + ay2) / 2 - H / 2) / (H / 2)
                m = 0.04
                sale_x = (ax1 < m * W and ex < 0) or (ax2 > (1 - m) * W and ex > 0)
                sale_y = (ay1 < m * H and ey < 0) or (ay2 > (1 - m) * H and ey > 0)
                real = ard.posicion_en(t_cap) or (eje_pan.cmd, eje_tilt.cmd)
                eje_pan.actualiza(ex, sale_x, real[0], args.borde, args.vuelta)
                eje_tilt.actualiza(ey, sale_y, real[1], args.borde, args.vuelta)
            else:
                # Perdido: los servos se quedan apuntando a donde se le vio por última vez.
                cambia_modo(2)
                if t_cap - fijo["t_visto"] > args.perdido_max:
                    fijo = None
                    cambia_modo(0)
        elif dets:
            asigna_caras(dets, caras.detecta(frame) if any(d["cls"] == PERSONA for d in dets) else [])

        # ---- Enviar posición solo si cambió (y latido de 1 Hz para el watchdog) ----
        pos = (round(eje_pan.cmd * 10), round(eje_tilt.cmd * 10))
        ahora = time.perf_counter()
        if pos != ult_envio or ahora - t_hb > 1.0:
            ard.envia(f"P,{pos[0]},{pos[1]}")
            ult_envio = pos
            if ahora - t_hb > 1.0:
                ard.envia(f"M{modo}")
                t_hb = ahora

        # ---- Dibujo ----
        if modo == 0:
            if not filtro_ir or ard.pir:
                for d in dets:
                    x1, y1, x2, y2 = d["box"]
                    col = (0, 255, 255) if d is hover else (0, 200, 0)
                    cv2.rectangle(frame, (x1, y1), (x2, y2), col, 3 if d is hover else 1)
                    if d.get("cara"):
                        fx1, fy1, fx2, fy2 = d["cara"]
                        cv2.rectangle(frame, (fx1, fy1), (fx2, fy2), (255, 200, 0), 2)
                    texto(frame, f"{model.names[d['cls']]} {d['conf']:.2f}", (x1, max(15, y1 - 6)), col)
        elif objetivo is not None:
            x1, y1, x2, y2 = objetivo["box"]
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
            ax1, ay1, ax2, ay2 = zona_mira(objetivo)
            cv2.rectangle(frame, (ax1, ay1), (ax2, ay2), (0, 0, 255), 3)
            cv2.drawMarker(frame, ((ax1 + ax2) // 2, (ay1 + ay2) // 2), (0, 0, 255), cv2.MARKER_CROSS, 14, 2)
            texto(frame, f"FIJADO: {model.names[objetivo['cls']]}", (x1, max(15, y1 - 8)), (0, 0, 255), 0.6, 2)
        else:
            texto(frame, "OBJETIVO PERDIDO - buscando...", (10, 50), (0, 255, 255), 0.7, 2)

        cv2.drawMarker(frame, (W // 2, H // 2), (255, 255, 255), cv2.MARKER_CROSS, 20, 1)
        bx, by = int(W / 2 * (1 - args.borde)), int(H / 2 * (1 - args.borde))
        cv2.rectangle(frame, (bx, by), (W - bx, H - by), (90, 90, 90), 1)  # dentro: servos quietos

        dt = ahora - t_prev
        t_prev = ahora
        fps = 0.9 * fps + 0.1 / dt if dt > 0 else fps
        texto(frame, f"{MODOS[modo]} | FPS {fps:.1f} | IR {'ON' if filtro_ir else 'OFF'} "
                     f"(PIR {int(ard.pir)}) | pan {pos[0] / 10:.1f} tilt {pos[1] / 10:.1f}",
              (10, 20), (255, 255, 255))

        cv2.imshow("Seguidor", frame)
        k = cv2.waitKey(1) & 0xFF
        if k in (27, ord("q")) or cv2.getWindowProperty("Seguidor", cv2.WND_PROP_VISIBLE) < 1:
            break
        if k == ord("r"):
            fijo = None
            cambia_modo(0)
        elif k == ord("i"):
            filtro_ir = not filtro_ir
        elif k == ord("c"):
            eje_pan.cmd = eje_tilt.cmd = 90.0
            ard.envia("C")

    ard.envia("M0")
    cam.cierra()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
