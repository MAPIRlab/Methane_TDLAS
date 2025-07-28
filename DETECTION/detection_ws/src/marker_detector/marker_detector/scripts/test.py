#!/usr/bin/env python3

import cv2
import vmbpy as vmb
import numpy as np
import queue
import math
import time
import cv2, numpy as np, time
from .LensConnect_Controller import UsbConnect, ScanUsbWithLensInfo
from . import DefVal as DV
from . import LensCtrl
import threading

# Controles de zoom con flechas ↑ ↓
zoom_step = 200
zoom_min, zoom_max = 4111, 17911
current_zoom = 4111  # valor inicial

focus_thread = None
focus_lock = threading.Lock()
is_focusing = False
key_queue = queue.Queue(maxsize=1)
last_frame = None
last_mask = None
last_roi_box = None
is_okey = False

def keyboard_zoom_worker():
    global current_zoom
    while True:
        try:
            key = key_queue.get(timeout=0.1)
        except queue.Empty:
            continue
        if key == 82:  # Flecha ↑
            current_zoom = min(current_zoom + zoom_step, zoom_max)
            LensCtrl.ZoomMove(current_zoom)
            print(f"⬆️ Zoom aumentado a {current_zoom}")
        elif key == 84:  # Flecha ↓
            current_zoom = max(current_zoom - zoom_step, zoom_min)
            LensCtrl.ZoomMove(current_zoom)
            print(f"⬇️ Zoom disminuido a {current_zoom}")


def connect_lens(idx: int = 0):
    ScanUsbWithLensInfo()
    if UsbConnect(idx) != DV.RET_SUCCESS:
        print("❌ No se pudo conectar al LensConnect")
        return None
    print("ℹ️ Inicializando Zoom")
    LensCtrl.ZoomInit()
    print("ℹ️ Inicializando Focus")
    LensCtrl.FocusInit()
    print("ℹ️ Inicializando Iris")
    LensCtrl.IrisInit()
    print("ℹ️ Moiendo zoom a 4111")
    LensCtrl.ZoomMove(4111)
    print("ℹ️ Moiendo focus a 2880")
    LensCtrl.FocusMove(2880)
    print("ℹ️ Moiendo iris a 0")
    LensCtrl.IrisMove(0)
    print("🔌 LensConnect listo")
    return True

def extract_green_RG8(bayer):
    """
    Devuelve solo el canal verde de un BayerRG8 (RG/GB).
    """
    g = np.zeros_like(bayer, dtype=np.uint8)
    g[0::2, 1::2] = bayer[0::2, 1::2]  # G en filas pares
    g[1::2, 0::2] = bayer[1::2, 0::2]  # G en filas impares
    return g

def frame_handler(cam, stream, frame):
    if not frame_queue.full():
        frame_queue.put(frame)
    cam.queue_frame(frame)


# ---------- parámetros de color ----------
GREEN_RATIO = 1.2      # un poco más flojo
DELTA_G     = 50      # diferencia absolut ≥ 25 niveles
G_MIN       = 60       # umbral absoluto de G (0-255)

def isolate_green(buf, w, h):
    """
    Devuelve (vis, mask): vis = pseudo-color con verdes resaltados,
    mask = píxeles realmente verdes (no blancos).
    Patrón Bayer RGGB (BayerRG8).
    """
    bayer = np.frombuffer(buf, np.uint8).reshape(h, w)

    # canales crudos
    R = np.zeros_like(bayer);  R[0::2, 0::2] = bayer[0::2, 0::2]
    G1= np.zeros_like(bayer);  G1[0::2, 1::2] = bayer[0::2, 1::2]
    G2= np.zeros_like(bayer);  G2[1::2, 0::2] = bayer[1::2, 0::2]
    B = np.zeros_like(bayer);  B[1::2, 1::2] = bayer[1::2, 1::2]

    G = (G1.astype(np.uint16) + G2.astype(np.uint16)) // 2
    G = G.astype(np.uint8)

    # --- criterio de verde ---
    maxRB  = np.maximum(R, B)
    diff   = G.astype(np.int16) - maxRB.astype(np.int16)

    mask = (G > G_MIN) & (diff >= DELTA_G) & (G >= maxRB * GREEN_RATIO)
    mask = mask.astype(np.uint8) * 255

    # visual para depurar
    green_vis = cv2.merge([np.zeros_like(G), G, np.zeros_like(G)])
    gray_vis  = cv2.cvtColor(G, cv2.COLOR_GRAY2BGR)
    vis       = np.where(mask[:, :, None] == 255, green_vis, gray_vis)

    return vis, mask, G

#-----------------------------------------------
# 1. Métrica híbrida de nitidez
# ----------------------------------------------------------------------
def focus_metric(green_roi):
    g  = cv2.Sobel(green_roi, cv2.CV_64F, 1, 0)**2 + cv2.Sobel(green_roi, cv2.CV_64F, 0, 1)**2
    var_lap = cv2.Laplacian(green_roi, cv2.CV_64F).var()
    return g.sum() * 0.7 + var_lap * 0.3      # pesos empíricos

def eval_pos(y,h,x,w, pos, avg=4):
    """Mueve a foco pos y devuelve score medio (descarta primer frame)."""
    LensCtrl.FocusMove(int(pos))
    time.sleep(0.06)
    scores = []
    for i in range(avg+1):         # +1 para descartar la primera
        frame = last_frame.copy()
        s = focus_metric(frame)
        if i: scores.append(s)
    return sum(scores)/len(scores)

# ----------------------------------------------------------------------
# 2. Scan + Refino multinivel
# ----------------------------------------------------------------------
def smart_autofocus(y,h,x,w,
                    f_min=2739, f_max=5159,
                    scan_step=120,
                    levels=(40,10,2,1)):
    """
    1) Barrido inicial con step=scan_step
    2) Refino multinivel: ±lvl para lvl∈levels
    """
    # ------------ 1. BARRIDO RÁPIDO ----------------
    scan_positions = list(range(f_min, f_max+1, scan_step))
    best_f = scan_positions[0]; best_s = -1
    for p in scan_positions:
        s = eval_pos(y,h,x,w,p)
        print(f"[Scan] F {p}  S {s:.0f}")
        if s > best_s:
            best_f, best_s = p, s

    # ------------ 2. REFINO MULTINIVEL -------------
    for radius in levels:
        search = range(max(f_min,best_f-radius),
                       min(f_max,best_f+radius)+1,
                       max(radius//2,1))
        for p in search:
            s = eval_pos(y,h,x,w,p)
            if s > best_s:
                best_f,best_s = p,s
                print(f"  ↳ Mejorado → F {p} S {s:.0f} (r={radius})")

    print(f"✅ Focus óptimo = {best_f}, score {best_s:.0f}")
    return int(best_f)

# ----------------------------------------------------------------------
# 3. AUTOZOOM + AUTOFOCO
# ----------------------------------------------------------------------
def autofocus_and_zoom():
    global last_frame,last_mask,is_focusing,current_zoom, is_okey
    if is_focusing or last_frame is None: return
    is_focusing = True
    try:
        # ROI verde
        if cv2.countNonZero(last_mask) < 600:
            print("🟡 Verde insuficiente"); return
        cnts,_ = cv2.findContours(last_mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        x,y,w,h = cv2.boundingRect(max(cnts,key=cv2.contourArea))

        # Zoom al máximo de una vez
        LensCtrl.ZoomMove(zoom_max); current_zoom=zoom_max
        time.sleep(0.12)

        # Enfoque multinivel
        best_focus = smart_autofocus(y,h,x,w,
                                     f_min=2739, f_max=5159,
                                     scan_step=120,
                                     levels=(40,10,2,1))
        LensCtrl.FocusMove(best_focus)
        print(f"🏁 FIN → Zoom={current_zoom}, Focus={best_focus}")
        is_okey = True
    finally:
        is_focusing=False


def main():
    global frame_queue
    global key_queue
    global last_frame
    global last_mask
    frame_queue = queue.Queue(maxsize=30)

    if connect_lens() is None:
         return

    cv2.setUseOptimized(True)

    with vmb.VmbSystem.get_instance() as vim:
        cams = vim.get_all_cameras()
        if not cams:
            print("❌ No se detectó ninguna cámara")
            return

        cam = cams[0]
        with cam:
            # 1. PixelFormat en color
            # Restablecer la cámara a valores por defecto al iniciar
            try:
                cam.get_feature_by_name("UserSetSelector").set("Default")
                cam.get_feature_by_name("UserSetLoad").run()
                print("🔄 Valores de cámara restaurados a valores por defecto")
            except vmb.VmbFeatureError:
                print("⚠️ No se pudo restaurar valores por defecto, característica no soportada")

            # 1. PixelFormat en color
            pf = cam.get_feature_by_name("PixelFormat")
            pf.set(vmb.PixelFormat.BayerRG8)
            print("✅ PixelFormat: Bgr8 (color)")

            # 3. Fija resolución
            # 3. Fija resolución 720x720 centrada
            width = cam.get_feature_by_name("Width")
            height = cam.get_feature_by_name("Height")
            offsetX = cam.get_feature_by_name("OffsetX")
            offsetY = cam.get_feature_by_name("OffsetY")

            width_default = 3840    
            height_default = 2160

            width.set(width_default)
            height.set(height_default)

            offsetX.set((4512 - width_default) // 2)
            offsetY.set((4512 - height_default) // 2)
            

            print(f"✅ Resolución: {width.get()}x{height.get()} (OffsetX={offsetX.get()}, OffsetY={offsetY.get()})")
            # 4. Exposure manual bajo y GainAuto ON
            cam.get_feature_by_name("ExposureAuto").set("Off")
            cam.get_feature_by_name("ExposureTime").set(1000)  # Exposición inicial en microsegundos

            cam.get_feature_by_name("GainAuto").set("Off")
            cam.get_feature_by_name("Gain").set(0.0)

            cam.get_feature_by_name("IntensityControllerTarget").set(50.0)
            cam.get_feature_by_name("Gamma").set(1.0) 

            cam.get_feature_by_name("BalanceWhiteAuto").set("Off")
            cam.get_feature_by_name("BalanceRatioSelector").set("Red")
            cam.get_feature_by_name("BalanceRatio").set(2.87)
            cam.get_feature_by_name("BalanceRatioSelector").set("Blue")
            cam.get_feature_by_name("BalanceRatio").set(2.40)

            # 5. FrameRate
            try:
                cam.get_feature_by_name("AcquisitionFrameRateEnable").set(True)
                fr = cam.get_feature_by_name("AcquisitionFrameRate")
                fr.set(fr.get_range()[1] * 0.95)
                print(f"🎞️ FrameRate forzado a: {fr.get()} fps")

            except vmb.VmbFeatureError:
                pass

            # 6. OpenCV y streaming
            cv2.namedWindow("Vista", cv2.WINDOW_NORMAL)
            t0, fps = time.time(), 0
            frame_count = 0
            frame_count = 0
            frame_skip = 0
            cam.start_streaming(handler=frame_handler)
            print("🎥 Streaming activo. Pulsa ESC para salir.")

            threading.Thread(target=keyboard_zoom_worker, daemon=True).start()

            while True:
                try:
                    frame = frame_queue.get(timeout=1.0)
                except queue.Empty:
                    continue

                buf = frame.get_buffer()
                h, w = frame.get_height(), frame.get_width()
                # Dibuja un círculo pequeño en el centro
                # Aplica filtro cada 3 frames
                
                disp, mask, green = isolate_green(buf, w, h)
                cx, cy = w // 2, h // 2
                cv2.circle(disp, (cx, cy), 5, (255, 0, 0), 2)
                last_mask = mask
                last_frame = green
                screen_width = 1280  # Ancho de la pantalla
                screen_height = 720  # Alto de la pantalla
                scale_width = screen_width / w
                scale_height = screen_height / h
                scale = min(scale_width, scale_height)
                resized_width = int(w * scale)
                resized_height = int(h * scale)
                img_resized = cv2.resize(disp, (resized_width, resized_height))
                cv2.imshow("Vista", img_resized)
                # Redimensiona la imagen para ajustarla a la pantalla


                # Muestra la imagen redimensionada
                cv2.imshow("Vista", img_resized)

                # frame_skip += 1
                # if frame_skip % 30 == 0 and not is_focusing:
                #     def focus_job():
                #         global is_focusing
                #         if not is_focusing and not is_okey:
                #             print("🟡 Iniciando enfoque automático y ajuste de zoom...")
                #             autofocus_and_zoom()
                #     focus_thread = threading.Thread(target=focus_job)
                #     focus_thread.start()

                frame_queue.task_done()

                fps += 1
                frame_count += 1
                if time.time() - t0 >= 1.0:
                    print(f"🟢 {fps} FPS — Resolución: {w}x{h}")
                    t0, fps = time.time(), 0

                key = cv2.waitKey(1) & 0xFF
                if key == 27:
                    break
                key_queue.put(key)

            cam.stop_streaming()
            cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
