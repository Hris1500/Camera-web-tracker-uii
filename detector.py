import argparse
import sys
from pathlib import Path


def cargar_catalogo(ruta):
    catalogo = {}
    for numero, linea in enumerate(ruta.read_text(encoding="utf-8-sig").splitlines(), 1):
        if not linea.strip() or linea.lstrip().startswith("#"):
            continue
        ingles, separador, espanol = (parte.strip() for parte in linea.partition("|"))
        if not separador or not ingles or not espanol or ingles in catalogo:
            raise ValueError(f"Línea {numero} inválida en {ruta.name}. Usa inglés|español y evita duplicados.")
        catalogo[ingles] = espanol
    if not catalogo:
        raise ValueError("Agrega al menos un objeto al catálogo.")
    return catalogo


OBJETOS_SELECCIONABLES = {
    "bottle", "soda can", "cup", "bowl", "book", "notebook",
    "pencil", "pen", "calculator", "headphones", "plate", "vase",
}


def seleccionar_en_centro(cajas, nombres, x, y):
    candidatas = (
        (confianza, int(identificador))
        for x1, y1, x2, y2, identificador, confianza, clase in cajas
        if nombres[int(clase)] in OBJETOS_SELECCIONABLES and x1 <= x <= x2 and y1 <= y <= y2
    )
    mejor = max(candidatas, default=None)
    return mejor[1] if mejor is not None else None


def main():
    parser = argparse.ArgumentParser(description="Detecta objetos con tu webcam.")
    parser.add_argument("--camara", type=int, default=0, help="Indice de la webcam")
    args = parser.parse_args()

    import cv2
    import numpy as np
    from ultralytics import YOLOWorld

    print("Cargando el modelo YOLO...", flush=True)
    carpeta = Path(__file__).resolve().parent
    catalogo = cargar_catalogo(carpeta / "catalogo.txt")
    nombres = list(catalogo)
    etiquetas = dict(enumerate(catalogo.values()))
    pesos = carpeta / "yolov8s-worldv2.pt"
    modelo = YOLOWorld(str(pesos))
    modelo.set_classes(nombres.copy())
    camara = cv2.VideoCapture(args.camara)
    ventana = "Detector de objetos | F seleccionar | R soltar | Esc salir"
    seleccionado = None
    aviso = ""
    try:
        if not camara.isOpened():
            raise RuntimeError(
                "No se pudo abrir la camara. Revisa los permisos de Windows, "
                "cierra otras apps que la usen o prueba con --camara 1."
            )
        camara.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        camara.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        while True:
            disponible, imagen = camara.read()
            if not disponible:
                raise RuntimeError("La camara dejo de entregar imagen.")
            resultado = modelo.track(
                imagen, persist=True, tracker="bytetrack.yaml",
                imgsz=416, conf=0.4, device="cpu", verbose=False,
            )[0]
            cajas = resultado.boxes
            # Las filas con ID tienen formato xyxy, ID, confianza, clase.
            filas = cajas.data.tolist() if cajas is not None and cajas.is_track else []
            resultado.names = etiquetas
            objeto = None
            visibles = resultado
            if seleccionado is not None:
                for indice, fila in enumerate(filas):
                    if int(fila[4]) == seleccionado:
                        objeto = fila
                        visibles = resultado[indice]
                        break

            vista = imagen.copy()
            if seleccionado is None or objeto is not None:
                anotada = visibles.plot(pil=True)
                vista = cv2.cvtColor(np.asarray(anotada), cv2.COLOR_RGB2BGR)

            alto, ancho = vista.shape[:2]
            centro_x, centro_y = ancho // 2, alto // 2
            if seleccionado is None:
                cv2.circle(vista, (centro_x, centro_y), 4, (255, 255, 255), -1)

            estado = aviso or "Centra un objeto y pulsa F"
            if seleccionado is not None:
                estado = "Objeto perdido | R para volver a elegir"
            if objeto is not None:
                x1, y1, x2, y2 = map(int, objeto[:4])
                cv2.rectangle(vista, (x1, y1), (x2, y2), (60, 255, 60), 3)
                estado = "En seguimiento"
            cv2.rectangle(vista, (0, 0), (ancho, 34), (28, 28, 28), -1)
            cv2.putText(vista, estado, (10, 23), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.imshow(ventana, vista)

            tecla = cv2.waitKey(1) & 0xFF
            if tecla in (27, ord("q"), ord("Q")):
                break
            if seleccionado is None and tecla in (ord("f"), ord("F")):
                nuevo = seleccionar_en_centro(filas, nombres, centro_x, centro_y)
                if nuevo is not None:
                    seleccionado = nuevo
                aviso = "Centra una botella, lata, libro u otro objeto compatible"
            elif tecla in (ord("r"), ord("R")):
                seleccionado = None
                aviso = ""
            if cv2.getWindowProperty(ventana, cv2.WND_PROP_VISIBLE) < 1:
                break
    finally:
        camara.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    except Exception as error:
        sys.exit(f"Error: {error}")



