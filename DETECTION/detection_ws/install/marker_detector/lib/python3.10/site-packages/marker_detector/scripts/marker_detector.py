#!/usr/bin/env python3
# marcador_led_detector.py
import cv2
import numpy as np
import collections
from pathlib import Path

# ----------------------- PARÁMETROS AJUSTABLES -----------------------
HSV_MIN = (55, 100, 180)       # rango verde puro – AJUSTA a tu LED
HSV_MAX = (80, 255, 255)
MIN_AREA = 2_000               # píxeles contorno mínimo
TEMPLATE_SIZE = 200            # px del warp cuadrado
TEMPLATE_PATH = Path("template_200.png")  # blanco = LED encendido
CAM_ID = 0                     # índice cámara
FPS_DESIRED = 60
FRAME_W, FRAME_H = 640, 480
# ---------------------------------------------------------------------

def build_template(path: Path, size: int = TEMPLATE_SIZE) -> np.ndarray:
    """Genera (o carga) imagen plantilla binaria 8-bit, 0/255."""
    if path.exists():
        t = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        return cv2.threshold(t, 127, 255, cv2.THRESH_BINARY)[1]
    # crear desde cero (por si no la tienes guardada)
    t = np.zeros((size, size), np.uint8)
    pad = int(size * .05)        # margen LED
    # marco
    cv2.rectangle(t, (pad, pad), (size-pad-1, size-pad-1), 255, thickness=20)
    # diagonales
    cv2.line(t, (pad, pad), (size-pad-1, size-pad-1), 255, 20)
    cv2.line(t, (size-pad-1, pad), (pad, size-pad-1), 255, 20)
    # “mordida” superior
    m_w = int(size * .2)
    cv2.rectangle(t, (size//2-m_w//2, 0), (size//2+m_w//2, pad+5), 0, -1)
    cv2.imwrite(str(path), t)
    return t

TEMPLATE = build_template(TEMPLATE_PATH)
TEMPLATE_EDGES = cv2.Canny(TEMPLATE, 80, 160)

def verify_quad(cnt) -> bool:
    """Heurística rápida: contorno ≈ cuadrado y suficientemente grande."""
    peri = cv2.arcLength(cnt, True)
    approx = cv2.approxPolyDP(cnt, 0.03*peri, True)
    if len(approx) != 4 or not cv2.isContourConvex(approx):
        return False
    area = cv2.contourArea(approx)
    return area >= MIN_AREA

def sort_corners(pts: np.ndarray) -> np.ndarray:
    """Devuelve esquinas ordenadas: [tl, tr, br, bl]"""
    pts = pts.reshape(-1, 2)
    s = pts.sum(1)
    tl = pts[np.argmin(s)]
    br = pts[np.argmax(s)]
    diff = np.diff(pts, axis=1)
    tr = pts[np.argmin(diff)]
    bl = pts[np.argmax(diff)]
    return np.array([tl, tr, br, bl], np.float32)

def init_kalman():
    """8-estado: x,y,w,h + dx,dy,dw,dh"""
    kf = cv2.KalmanFilter(8, 4)
    dt = 1/FPS_DESIRED
    kf.transitionMatrix = np.array([
        [1,0,0,0, dt,0, 0,0],
        [0,1,0,0, 0,dt, 0,0],
        [0,0,1,0, 0,0, dt,0],
        [0,0,0,1, 0,0, 0,dt],
        [0,0,0,0, 1,0, 0,0],
        [0,0,0,0, 0,1, 0,0],
        [0,0,0,0, 0,0, 1,0],
        [0,0,0,0, 0,0, 0,1],
    ], np.float32)
    kf.measurementMatrix = np.zeros((4,8), np.float32)
    kf.measurementMatrix[:4, :4] = np.eye(4, dtype=np.float32)
    kf.processNoiseCov = np.eye(8, dtype=np.float32) * 1e-3
    kf.measurementNoiseCov = np.eye(4, dtype=np.float32) * 1e-1
    kf.errorCovPost *= 10.
    return kf

def main():
    cap = cv2.VideoCapture(CAM_ID)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
    cap.set(cv2.CAP_PROP_FPS, FPS_DESIRED)

    kf = init_kalman()
    prev_gray = None
    prev_quad = None
    fps_queue = collections.deque(maxlen=30)

    # suposición cámara intrínseca (rellena con los datos reales)
    fx = fy = 700.0
    cx, cy = FRAME_W/2, FRAME_H/2
    K = np.array([[fx,0,cx],[0,fy,cy],[0,0,1]], np.float32)
    dist = np.zeros(5)  # sin distorsión

    # 3D model: cuadrado lado 1 m centrado
    model = np.array([
        [-0.5,  0.5, 0],
        [ 0.5,  0.5, 0],
        [ 0.5, -0.5, 0],
        [-0.5, -0.5, 0],
    ], np.float32)

    while True:
        ok, frame = cap.read()
        if not ok:
            break
        tic = cv2.getTickCount()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # -------- ROI por Kalman o full frame --------
        if prev_quad is not None:
            x,y,w,h = map(int, kf.predict()[:4])
            x0 = max(0, x-2*w); y0 = max(0, y-2*h)
            x1 = min(FRAME_W, x+3*w); y1 = min(FRAME_H, y+3*h)
            roi_hsv = hsv[y0:y1, x0:x1]
        else:
            kf.predict()
            x0=y0=0; x1=FRAME_W; y1=FRAME_H
            roi_hsv = hsv

        # -------- Umbral & contornos --------
        mask = cv2.inRange(roi_hsv, HSV_MIN, HSV_MAX)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,
                                cv2.getStructuringElement(
                                    cv2.MORPH_ELLIPSE,(7,7)))

        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
        found_quad = None
        for c in cnts:
            if not verify_quad(c): continue
            quad = sort_corners(c)
            quad += (x0, y0)  # compensar ROI
            # warp -> edges -> template match
            M = cv2.getPerspectiveTransform(
                    quad, np.float32([[0,0],[TEMPLATE_SIZE-1,0],
                                      [TEMPLATE_SIZE-1,TEMPLATE_SIZE-1],
                                      [0,TEMPLATE_SIZE-1]]))
            warp = cv2.warpPerspective(
                    mask, M, (TEMPLATE_SIZE, TEMPLATE_SIZE))
            w_edges = cv2.Canny(warp, 80, 160)
            score = cv2.matchTemplate(
                    w_edges, TEMPLATE_EDGES,
                    cv2.TM_CCOEFF_NORMED)[0][0]
            if score > 0.60:
                found_quad = quad
                break

        # -------- tracking fallback --------
        if found_quad is None and prev_quad is not None:
            p0 = prev_quad.astype(np.float32)
            p1, st, _ = cv2.calcOpticalFlowPyrLK(
                prev_gray, gray, p0, None,
                winSize=(21,21), maxLevel=2,
                criteria=(cv2.TERM_CRITERIA_EPS|
                          cv2.TERM_CRITERIA_COUNT, 30, 0.01))
            if st.sum() >= 3:
                found_quad = sort_corners(p1)

        # -------- actualizar estado --------
        if found_quad is not None:
            prev_quad = found_quad
            prev_gray = gray.copy()
            x,y,w,h = cv2.boundingRect(found_quad.astype(int))
            kf.correct(np.array([[x],[y],[w],[h]],np.float32))

            # Dibujar contorno
            cv2.polylines(frame,[found_quad.astype(int)],True,(0,255,0),2)

            # SolvePnP
            ok, rvec, tvec = cv2.solvePnP(
                    model, found_quad, K, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if ok:
                cv2.putText(frame, f"t = {tvec.ravel()}", (10,30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0),1,
                            cv2.LINE_AA)
        else:
            prev_quad = None

        # -------- Mostrar FPS --------
        toc = (cv2.getTickCount()-tic)/cv2.getTickFrequency()
        fps_queue.append(1.0/toc)
        fps = np.mean(fps_queue)
        cv2.putText(frame, f"{fps:4.1f} FPS", (FRAME_W-100, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255),1)

        cv2.imshow("LED Marker", frame)
        if cv2.waitKey(1) & 0xFF == 27:   # Esc para salir
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
