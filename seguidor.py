#!/usr/bin/env python3
"""
Seguidor con cámara USB + YOLO26 + servos pan-tilt SG90 vía Arduino (servos/servos.ino).

- Modo libre: detecta y etiqueta objetos con un ID estable (YOLO26 + BoT-SORT); en personas, la cara.
- Clic sobre un objeto: se fija y los demás desaparecen. Se guarda su "huella" para no confundirlo:
    * Personas: se sigue SOLO LA CARA. Huella facial SFace: otra cara nunca se acepta.
    * Objetos: huella visual MobileNetV3: otro objeto de la misma clase no se acepta.
  VitTrack lo sigue en cada cuadro; YuNet/YOLO corrigen la caja y lo recuperan si se pierde.
- Los servos solo se mueven cuando el objetivo va a salir del encuadre y lo
  devuelven al centro (histéresis --borde / --vuelta). Rango de los servos: calibrar.py.

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
import numpy as np
from ultralytics import YOLO

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None

try:  # oculta los avisos internos de OpenCV (MSMF, "Targets are not supported"...)
    cv2.utils.logging.setLogLevel(cv2.utils.logging.LOG_LEVEL_ERROR)
except AttributeError:
    pass

AQUI = Path(__file__).resolve().parent
TRACKER = AQUI / "tracker.yaml"
ZOO = "https://github.com/opencv/opencv_zoo/raw/main/models/"
VIDS_ARDUINO = {0x2341, 0x2A03, 0x1A86, 0x0403, 0x10C4}  # Arduino, CH340, FTDI, CP210x
PERSONA = 0
MODOS = ("LIBRE", "FIJADO", "PERDIDO")


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def iou(a, b):
    ix = max(0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0, min(a[3], b[3]) - max(a[1], b[1]))
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - ix * iy
    return ix * iy / union if union > 0 else 0.0


def centro(b):
    return (b[0] + b[2]) / 2, (b[1] + b[3]) / 2


def dentro(p, b):
    return b[0] <= p[0] <= b[2] and b[1] <= p[1] <= b[3]


def amplia(b, f, W, H):
    """Caja b escalada f veces respecto a su centro y recortada al cuadro."""
    cx, cy = centro(b)
    hw, hh = (b[2] - b[0]) * f / 2, (b[3] - b[1]) * f / 2
    return max(0, int(cx - hw)), max(0, int(cy - hh)), min(W, int(cx + hw)), min(H, int(cy + hh))


def parte_alta(b):
    """60 % superior de una persona: donde puede estar la cara."""
    return b[0], b[1], b[2], int(b[1] + 0.6 * (b[3] - b[1]))


def caja(fila):
    """Caja (x1, y1, x2, y2) de una fila de YuNet."""
    return int(fila[0]), int(fila[1]), int(fila[0] + fila[2]), int(fila[1] + fila[3])


def modelo_zoo(carpeta, nombre):
    """Descarga (solo la primera vez) un modelo ONNX del opencv_zoo. Devuelve su ruta o None."""
    ruta = AQUI / nombre
    if not ruta.exists():
        try:
            print(f"[INFO] Descargando {nombre}...")
            tmp = ruta.with_suffix(".part")
            urllib.request.urlretrieve(f"{ZOO}{carpeta}/{nombre}", tmp)
            tmp.replace(ruta)
        except OSError as e:
            print(f"[AVISO] No se pudo descargar {nombre}: {e}")
            return None
    return ruta


def carga_yolo(nombre, openvino, imgsz):
    """YOLO normal o, con --openvino, exportado a OpenVINO (bastante más rápido en CPU)."""
    model = YOLO(nombre)
    if not openvino:
        return model
    ov = Path(model.ckpt_path).with_name(Path(nombre).stem + "_openvino_model")
    try:
        if not ov.exists():
            print("[INFO] Exportando a OpenVINO (solo la primera vez, tarda ~1 min)...")
            model.export(format="openvino", imgsz=imgsz)
        return YOLO(str(ov), task="detect")
    except Exception as e:  # la exportación instala paquetes y puede fallar de muchas formas
        print(f"[AVISO] OpenVINO no disponible ({e}); se usa el modelo normal.")
        return model


# ---------------------- Enlace con Arduino ----------------------
class Arduino:
    def __init__(self, port, baud=115200):
        self.ser = None
        self.pir = True                     # sin Arduino el filtro IR no oculta nada
        self.limites = (10, 170, 40, 140)   # se sustituyen por los del READY (calibrados en EEPROM)
        self.hist = deque(maxlen=40)        # (t, pan, tilt) posiciones reales reportadas
        self.error = None
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
        self.listo = threading.Event()
        threading.Thread(target=self._lee, daemon=True).start()
        if not self.listo.wait(4.0):  # el UNO se reinicia al abrir el puerto
            self.envia("?")
        if self.listo.wait(1.0):
            print(f"[OK] Arduino en {port}, límites pan {self.limites[:2]} tilt {self.limites[2:]}")
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
                    self.listo.set()
                elif p[0] == "ERR":
                    self.error = ",".join(p)
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
        # Sin --cam se prueba primero la 1 (la USB externa; la 0 suele ser la integrada del portátil).
        # Algunas webcams USB solo funcionan con MSMF y otras solo con DirectShow: se prueban ambos.
        apis = [cv2.CAP_MSMF, cv2.CAP_DSHOW] if sys.platform == "win32" else [cv2.CAP_ANY]
        self.cap = None
        for i in ([idx] if idx is not None else [1, 0]):
            for api in apis:
                cap = cv2.VideoCapture(i, api)
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                if cap.isOpened() and cap.read()[0]:
                    self.cap = cap
                    print(f"[OK] Cámara {i} ({cap.getBackendName()}) "
                          f"{int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}")
                    break
                cap.release()
            if self.cap:
                break
        if self.cap is None:
            raise RuntimeError(f"No se pudo abrir la cámara {idx if idx is not None else '1 ni 0'}")
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


# ---------------------- Caras: YuNet (detección) + SFace (identidad) ----------------------
class Caras:
    def __init__(self):
        ruta = modelo_zoo("face_detection_yunet", "face_detection_yunet_2023mar.onnx")
        self.det = cv2.FaceDetectorYN.create(str(ruta), "", (320, 320), 0.6, 0.3, 50) if ruta else None
        ruta = modelo_zoo("face_recognition_sface", "face_recognition_sface_2021dec.onnx")
        self.rec = cv2.FaceRecognizerSF.create(str(ruta), "") if ruta else None
        if self.det is None:
            print("[AVISO] Sin detector de caras: se apuntará a la cabeza estimada.")
        if self.rec is None:
            print("[AVISO] Sin reconocimiento facial: las caras se distinguirán solo por posición.")

    def detecta(self, frame, region=None):
        """Filas de YuNet (x, y, w, h, 5 puntos de referencia, score) en coordenadas del cuadro."""
        if self.det is None:
            return []
        H, W = frame.shape[:2]
        x1, y1, x2, y2 = region or (0, 0, W, H)
        img = frame[y1:y2, x1:x2]
        if img.size == 0:
            return []
        h, w = img.shape[:2]
        # 320 px basta y cuesta pocos ms; los recortes pequeños se amplían para no perder caras lejanas.
        s = min(2.0, 320 / max(w, h))
        if s != 1:
            img = cv2.resize(img, (max(1, round(w * s)), max(1, round(h * s))))
        self.det.setInputSize((img.shape[1], img.shape[0]))
        _, f = self.det.detect(img)
        if f is None:
            return []
        f = f.copy()
        f[:, :14] /= s
        f[:, [0, 4, 6, 8, 10, 12]] += x1
        f[:, [1, 5, 7, 9, 11, 13]] += y1
        return list(f)

    def huella(self, frame, fila):
        """Vector de identidad de la cara (normalizado): coseno > ~0,36 = misma persona."""
        if self.rec is None:
            return None
        v = self.rec.feature(self.rec.alignCrop(frame, fila.reshape(1, -1).astype(np.float32))).flatten()
        return v / (np.linalg.norm(v) + 1e-9)


# ---------------------- Apariencia de objetos: MobileNetV3 (torchvision) ----------------------
class Apariencia:
    """Huella visual de un recorte: distingue objetos de la misma clase (dos tazas, dos móviles...)."""
    MEDIA = np.array([0.485, 0.456, 0.406], np.float32)
    DESV = np.array([0.229, 0.224, 0.225], np.float32)

    def __init__(self):
        self.red = None
        try:
            import torch
            import torchvision
            pesos = torchvision.models.MobileNet_V3_Small_Weights.IMAGENET1K_V1  # ~10 MB, solo la 1ª vez
            m = torchvision.models.mobilenet_v3_small(weights=pesos)
            self.red = torch.nn.Sequential(m.features, m.avgpool, torch.nn.Flatten()).eval()
            self.torch = torch
        except Exception as e:  # sin internet la 1ª vez, o torchvision incompatible
            print(f"[AVISO] Sin MobileNet ({e}): los objetos se distinguirán solo por color.")

    def __call__(self, frame, cajas):
        if self.red is None:
            vs = [histograma(frame, b) for b in cajas]
            return [np.zeros(256, np.float32) if v is None else v for v in vs]
        recortes = []
        for x1, y1, x2, y2 in cajas:
            roi = frame[max(0, y1):max(0, y2), max(0, x1):max(0, x2)]
            roi = cv2.resize(roi, (128, 128)) if roi.size else np.zeros((128, 128, 3), np.uint8)
            recortes.append(cv2.cvtColor(roi, cv2.COLOR_BGR2RGB))
        x = (np.stack(recortes).astype(np.float32) / 255 - self.MEDIA) / self.DESV
        with self.torch.inference_mode():
            f = self.red(self.torch.from_numpy(x).permute(0, 3, 1, 2)).numpy()
        return list(f / (np.linalg.norm(f, axis=1, keepdims=True) + 1e-9))


def asigna_caras(dets, caras):
    """A cada persona le asigna la mejor cara (x1, y1, x2, y2, score) cuyo centro cae en su 60 % superior."""
    for d in dets:
        if d["cls"] != PERSONA:
            continue
        alta = parte_alta(d["box"])
        dentro_ = [c for c in caras if dentro(centro(c), alta)]
        if dentro_:
            d["cara"] = tuple(map(int, max(dentro_, key=lambda c: c[4])[:4]))


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
    roi = frame[max(0, y1):max(0, y2), max(0, x1):max(0, x2)]
    if roi.size == 0:
        return None
    h = cv2.calcHist([cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)], [0, 1], None, [16, 16], [0, 180, 0, 256])
    return cv2.normalize(h, h).flatten()


# ---------------------- Objetivo fijado ----------------------
class Objetivo:
    """Objetivo fijado con identidad propia.
    VitTrack lo sigue en cada cuadro; una detección solo se acepta como "él" si su huella
    (SFace en caras, MobileNet en objetos) se parece a las guardadas al fijarlo."""
    VIT_OK = 0.3           # puntuación mínima de VitTrack para fiarse
    VIT_SEGURO = 0.55      # con esta puntuación VitTrack se confirma a sí mismo
    SIN_CONFIRMAR = 1.5    # s que VitTrack puede ir solo, con puntuación dudosa
    YOLO_CADA = 3          # cada cuántos cuadros YOLO corrige un objeto
    SIM_CARA = 0.40        # coseno SFace para aceptar una cara como la suya (umbral del modelo: 0,363)
    SIM_CARA_OTRA = 0.25   # por debajo, seguro que es otra persona
    SIM_OBJ = 0.75         # coseno MobileNet para recuperar el objeto
    SIM_OBJ_OTRO = 0.50    # por debajo, seguro que es otro objeto
    MARGEN = 0.05          # al recuperar, el mejor debe destacar sobre el segundo

    def __init__(self, vit, caras, apar, frame, d, t):
        self.vit, self.caras, self.apar = vit, caras, apar
        self.cls, self.id = d["cls"], d["id"]
        self.es_persona = d["cls"] == PERSONA
        self.galeria = []  # huellas del objetivo (varias poses/ángulos)
        box = d["box"]
        if self.es_persona:  # se sigue solo la cara
            filas = caras.detecta(frame, parte_alta(box))
            if filas:
                fila = max(filas, key=lambda f: f[14])
                box = caja(fila)
                self._aprende(caras.huella(frame, fila))
            else:
                box = zona_mira(d)  # de espaldas: cabeza; la huella se aprende al verle la cara
        else:
            self._aprende(apar(frame, [box])[0])
        self.box = box
        self.t_visto = self.t_conf = t
        self.perdido = False
        self.n = 0
        self.mira = None
        self.sim = None
        self._ancla(frame, box)

    def _aprende(self, h):
        if h is None or (self.galeria and self.parecido(h) > 0.92):
            return  # nada nuevo que aprender
        self.galeria.append(h)
        if len(self.galeria) > 8:
            del self.galeria[1]  # la [0] (la del clic) se conserva siempre

    def parecido(self, h):
        if h is None or not self.galeria:
            return None
        return max(float(np.dot(h, g)) for g in self.galeria)

    def _ancla(self, frame, box):
        if self.vit is not None and box[2] - box[0] > 4 and box[3] - box[1] > 4:
            self.vit.init(frame, (box[0], box[1], box[2] - box[0], box[3] - box[1]))

    def necesita_yolo(self):
        if self.es_persona:
            return False  # personas: todo se hace con las caras
        return self.perdido or self.vit is None or self.n % self.YOLO_CADA == 0

    def actualiza(self, frame, t, dets):
        """Caja del objetivo en este cuadro (la cara si es persona) o None si se ha perdido.
        dets: detecciones de YOLO de este cuadro, o None si este cuadro no se pasó por YOLO."""
        self.n += 1
        H, W = frame.shape[:2]
        vbox = None
        if self.vit is not None:
            ok, r = self.vit.update(frame)
            score = self.vit.getTrackingScore() if ok else 0.0
            if score >= self.VIT_SEGURO:
                self.t_conf = t
            if score >= self.VIT_OK and t - self.t_conf < self.SIN_CONFIRMAR:
                vbox = (r[0], r[1], r[0] + r[2], r[1] + r[3])

        if self.es_persona:
            nuevo, vbox = self._cara(frame, vbox, W, H)
        else:
            nuevo, vbox = self._objeto(frame, vbox, dets)

        if nuevo is not None:
            self._ancla(frame, nuevo)  # plantilla fresca y caja corregida: 0,7 ms
            self.box, self.t_conf = nuevo, t
        elif vbox is not None:
            self.box = vbox
        else:
            self.perdido = True
            return None
        self.perdido = False
        self.t_visto = t
        return self.box

    def _cara(self, frame, vbox, W, H):
        ref = vbox or (None if self.perdido else self.box)
        region = amplia(ref, 3.0, W, H) if ref else None  # perdido: busca en todo el cuadro
        mejor = h_mejor = sim_mejor = None
        s_mejor = -1e9
        for fila in self.caras.detecta(frame, region):
            b = caja(fila)
            h = self.caras.huella(frame, fila)
            sim = self.parecido(h)
            if sim is None:  # aún sin huella: la cara más cercana a donde se le sigue
                if not ref or math.dist(centro(b), centro(ref)) > 1.5 * (ref[2] - ref[0]):
                    continue
                s = -math.dist(centro(b), centro(ref))
            else:
                if vbox and sim < self.SIM_CARA_OTRA and dentro(centro(vbox), b):
                    vbox = None  # VitTrack se ha pasado a la cara de otra persona
                if sim < (self.SIM_CARA if ref else self.SIM_CARA + 0.05):  # recuperar exige más
                    continue
                s = sim
            if s > s_mejor:
                mejor, h_mejor, s_mejor, sim_mejor = b, h, s, sim
        if mejor is not None:
            self._aprende(h_mejor)
            self.sim = sim_mejor
        return mejor, vbox

    def _objeto(self, frame, vbox, dets):
        if dets is None:
            return None, vbox
        mismos = [d["box"] for d in dets if d["cls"] == self.cls]
        if not mismos:
            return None, vbox
        hs = self.apar(frame, mismos)
        sims = [self.parecido(h) or 0.0 for h in hs]
        if vbox:
            # Re-anclaje: la detección que solapa con VitTrack, si de verdad se parece.
            i = max(range(len(mismos)), key=lambda k: iou(mismos[k], vbox))
            if iou(mismos[i], vbox) > 0.3:
                if sims[i] >= self.SIM_OBJ - 0.1:
                    self._aprende(hs[i])
                    self.sim = sims[i]
                    return mismos[i], vbox
                if sims[i] < self.SIM_OBJ_OTRO:
                    vbox = None  # VitTrack está sobre otro objeto
            if vbox:
                return None, vbox
        # Recuperación: el más parecido, solo si destaca claramente sobre el segundo.
        orden = sorted(range(len(mismos)), key=lambda k: sims[k], reverse=True)
        k = orden[0]
        segundo = sims[orden[1]] if len(orden) > 1 else -1.0
        if sims[k] >= self.SIM_OBJ and sims[k] - segundo >= self.MARGEN:
            self._aprende(hs[k])
            self.sim = sims[k]
            return mismos[k], None
        return None, None

    def suaviza_mira(self, zona, alfa=0.6):
        """Media exponencial de la caja: quita el temblor sin apenas retraso."""
        if self.mira is None:
            self.mira = tuple(map(float, zona))
        else:
            self.mira = tuple(m + alfa * (z - m) for m, z in zip(self.mira, zona))
        return tuple(map(int, self.mira))


# ---------------------- Control de un eje del pan-tilt ----------------------
class Eje:
    def __init__(self, lo, hi, fov, signo):
        self.lo, self.hi, self.signo = lo, hi, signo
        self.tan = math.tan(math.radians(fov / 2))
        self.cmd = (lo + hi) / 2
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

    def en_limite(self):
        return self.activo and (self.cmd <= self.lo + 0.5 or self.cmd >= self.hi - 0.5)


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
    ap.add_argument("--cam", type=int, default=None, help="índice de cámara (por defecto: la USB externa, si no la 0)")
    ap.add_argument("--port", default=None, help="COM3 o /dev/ttyACM0 (por defecto: autodetecta)")
    ap.add_argument("--model", default="yolo26n.pt",
                    help="yolo26n.pt (rápido) o yolo26s.pt (más preciso, más lento); se descarga solo")
    ap.add_argument("--openvino", action="store_true", help="acelera YOLO en CPU con OpenVINO")
    ap.add_argument("--imgsz", type=int, default=480)
    ap.add_argument("--conf", type=float, default=0.35, help="confianza mínima para mostrar en modo libre")
    ap.add_argument("--w", type=int, default=640)
    ap.add_argument("--h", type=int, default=480)
    ap.add_argument("--hfov", type=float, default=60.0, help="campo de visión horizontal de tu cámara (°)")
    ap.add_argument("--borde", type=float, default=0.55,
                    help="0..1: los servos actúan cuando el objetivo pasa de esta distancia al centro")
    ap.add_argument("--vuelta", type=float, default=0.12, help="0..1: dejan de actuar al volver a esta zona")
    ap.add_argument("--inv-pan", action="store_true", help="invierte el sentido del PAN")
    ap.add_argument("--inv-tilt", action="store_true", help="invierte el sentido del TILT")
    ap.add_argument("--perdido-max", type=float, default=5.0, help="segundos sin verlo antes de liberar")
    args = ap.parse_args()

    ard = Arduino(args.port)
    cam = Camara(args.cam, args.w, args.h)
    model = carga_yolo(args.model, args.openvino, args.imgsz)
    caras = Caras()
    apar = Apariencia()
    vit = None
    ruta_vit = modelo_zoo("object_tracking_vittrack", "object_tracking_vittrack_2023sep.onnx")
    if ruta_vit:
        p = cv2.TrackerVit_Params()
        p.net = str(ruta_vit)
        vit = cv2.TrackerVit_create(p)  # uno solo, se re-inicializa en cada fijado
    else:
        print("[AVISO] Sin VitTrack: el objetivo se seguirá solo con las detecciones.")

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
    fijo = None            # Objetivo fijado
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

        # ---- YOLO: siempre en modo libre; fijado, solo cuando el objetivo lo necesita ----
        dets = None
        if fijo is None or fijo.necesita_yolo():
            clases = [fijo.cls] if fijo else ([PERSONA] if filtro_ir else None)
            # conf bajo a propósito: el tracker usa las detecciones débiles para no soltar el track;
            # --conf solo filtra lo que se dibuja.
            r = model.track(frame, persist=True, tracker=str(TRACKER), imgsz=args.imgsz,
                            conf=0.1, classes=clases, verbose=False)[0]
            dets = []
            if r.boxes is not None and r.boxes.id is not None:
                for b, i, c, p in zip(r.boxes.xyxy.int().tolist(), r.boxes.id.int().tolist(),
                                      r.boxes.cls.int().tolist(), r.boxes.conf.tolist()):
                    dets.append(dict(id=i, cls=c, conf=p, box=tuple(b),
                                     area=(b[2] - b[0]) * (b[3] - b[1])))
        visibles = [d for d in dets if d["conf"] >= args.conf] if fijo is None else []

        # ---- Hover: el recuadro más pequeño bajo el cursor ----
        mx, my = mouse["x"], mouse["y"]
        bajo = [d for d in visibles if dentro((mx, my), d["box"])]
        hover = min(bajo, key=lambda d: d["area"]) if bajo else None

        # ---- Eventos de ratón ----
        if mouse["libera"]:
            mouse["libera"] = False
            fijo = None
            cambia_modo(0)
        if mouse["clic"]:
            mouse["clic"] = False
            if hover is not None:
                fijo = Objetivo(vit, caras, apar, frame, hover, t_cap)
                cambia_modo(1)
                continue  # el siguiente cuadro ya se procesa como fijado

        # ---- Seguimiento ----
        mira = None
        if fijo:
            box = fijo.actualiza(frame, t_cap, dets)
            if box is not None:
                cambia_modo(1)
                mira = fijo.suaviza_mira(box)

                # ---- Servos: solo actúan si el objetivo se está saliendo ----
                ax1, ay1, ax2, ay2 = mira
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
                fijo.mira = None
                if t_cap - fijo.t_visto > args.perdido_max:
                    fijo = None
                    cambia_modo(0)
        elif visibles and any(d["cls"] == PERSONA for d in visibles):
            asigna_caras(visibles, [(*caja(f), f[14]) for f in caras.detecta(frame)])

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
                for d in visibles:
                    col = (0, 255, 255) if d is hover else (0, 200, 0)
                    # En personas se enmarca solo la cara (o el cuerpo si está de espaldas).
                    x1, y1, x2, y2 = d.get("cara") or d["box"]
                    cv2.rectangle(frame, (x1, y1), (x2, y2), col, 3 if d is hover else 2)
                    texto(frame, f"{model.names[d['cls']]} #{d['id']} {d['conf']:.2f}",
                          (x1, max(15, y1 - 6)), col)
        elif mira is not None:
            ax1, ay1, ax2, ay2 = mira
            cv2.rectangle(frame, (ax1, ay1), (ax2, ay2), (0, 0, 255), 3)
            cv2.drawMarker(frame, ((ax1 + ax2) // 2, (ay1 + ay2) // 2), (0, 0, 255), cv2.MARKER_CROSS, 14, 2)
            nombre = "cara" if fijo.es_persona else model.names[fijo.cls]
            sim = f" ({fijo.sim:.2f})" if fijo.sim is not None else ""
            texto(frame, f"FIJADO: {nombre} #{fijo.id}{sim}", (ax1, max(15, ay1 - 8)), (0, 0, 255), 0.6, 2)
        else:
            texto(frame, "OBJETIVO PERDIDO - buscando...", (10, 50), (0, 255, 255), 0.7, 2)

        limites = [n for n, e in (("PAN", eje_pan), ("TILT", eje_tilt)) if e.en_limite()]
        if limites:
            texto(frame, f"LIMITE {'/'.join(limites)}: el servo no puede girar mas",
                  (10, H - 12), (0, 140, 255), 0.6, 2)

        cv2.drawMarker(frame, (W // 2, H // 2), (255, 255, 255), cv2.MARKER_CROSS, 20, 1)
        bx, by = int(W / 2 * (1 - args.borde)), int(H / 2 * (1 - args.borde))
        cv2.rectangle(frame, (bx, by), (W - bx, H - by), (90, 90, 90), 1)  # dentro: servos quietos

        dt = ahora - t_prev
        t_prev = ahora
        fps = 0.9 * fps + 0.1 / dt if dt > 0 else fps
        texto(frame, f"{MODOS[modo]} | FPS {fps:.1f} | IR {'ON' if filtro_ir else 'OFF'} "
                     f"(PIR {int(ard.pir)}) | pan {pos[0] / 10:.1f} [{eje_pan.lo}-{eje_pan.hi}] "
                     f"tilt {pos[1] / 10:.1f} [{eje_tilt.lo}-{eje_tilt.hi}]",
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
            eje_pan.cmd, eje_tilt.cmd = (eje_pan.lo + eje_pan.hi) / 2, (eje_tilt.lo + eje_tilt.hi) / 2
            ard.envia("C")

    ard.envia("M0")
    cam.cierra()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
