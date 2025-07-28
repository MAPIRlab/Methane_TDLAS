#!/usr/bin/env python3
import cv2, vmbpy as vmb, numpy as np, queue, time, threading
from .LensConnect_Controller import UsbConnect, ScanUsbWithLensInfo
from . import DefVal as DV, LensCtrl
import json
from std_msgs.msg import String as ROSString
from math import degrees, acos

# --------------- PARÁMETROS GENERALES ----------------
WIDTH, HEIGHT        = 3840, 2160
EXPOSURE_TIME        = 10000      # µs
ZOOM_STEP, ZOOM_MIN, ZOOM_MAX = 1000, 4111, 17911
current_zoom         = 4111
current_focus        = 2860       # valor inicial de foco

INTENSITY_MIN        = 250        # umbral en canal V para LED muy brillante
MIN_AREA             = 10000
ROI_MARGIN           = 500        # px extra alrededor del marcador
ROI_LOST             = 15         # frames sin verlo 3antes de reset ROI

# Estas variables se asignarán dinámicamente
target_area = None    # ← Dinámico
AREA_MIN = None       # ← Dinámico
AREA_MAX = None       # ← Dinámico
AREA_TOLERANCE = 0.15 # ±25% de tolerancia

parent_node = None  # Para pasar el nodo de ROS si es necesario
# -----------------------------------------------------

LOOKUP_TABLE = {
    17911: {"far":4500, "near": 4650},
    16911: {"far":4550, "near": 4700},
    15911: {"far":4480, "near": 4620},
    14911: {"far":4380, "near": 4480},
    13911: {"far": 4230, "near": 4350},
    12911: {"far": 4060, "near": 4200},
    11911: {"far": 3900, "near": 4050},
    10911: {"far": 3730, "near": 3900},
    9911: {"far": 3580, "near": 3750},
    8911: {"far": 3450, "near": 3600},
}

# -----------------------------------------------
# Variables globales para el manejo de hilos y estado

focus_thread = None
focus_lock = threading.Lock()
is_focusing = False
key_queue = queue.Queue(maxsize=1)
last_frame = None
last_mask = None
last_roi_box = None
is_okey = False
roi_active = False
best_focus_global = 0  # Para almacenar el mejor foco global    
best_score_global = 0  # Para almacenar el mejor score global

def _ang(v1, v2):
    """Ángulo absoluto entre dos vectores en grados."""
    dot = np.dot(v1, v2)
    n1  = np.linalg.norm(v1)
    n2  = np.linalg.norm(v2)
    return degrees(acos(dot / (n1 * n2 + 1e-12)))

def is_rectangle(pts, ang_tol=10):
    """
    Comprueba si el polígono de 4 vértices 'pts' (CW/CCW) es un rectángulo
    casi ortogonal.  ang_tol: tolerancia para 0°/90° en grados.
    """
    v = [pts[(i+1) % 4] - pts[i] for i in range(4)]
    # lados opuestos ≈ paralelos
    if _ang(v[0], v[2]) > ang_tol * 2 or _ang(v[1], v[3]) > ang_tol * 2:
        return False
    # lados consecutivos ≈ ortogonales
    for i in range(4):
        if abs(_ang(v[i], v[(i+1) % 4]) - 90) > ang_tol * 2:
            return False
    return True
# ----------------------------------------------------------------------------

#-----------------------------------------------
# 1. Métrica híbrida de nitidez
# ----------------------------------------------------------------------


def focus_metric_quick(roi_gray_down):
    gx = cv2.Sobel(roi_gray_down, cv2.CV_64F, 1, 0, ksize=3)
    gy = cv2.Sobel(roi_gray_down, cv2.CV_64F, 0, 1, ksize=3)
    g  = (gx ** 2 + gy ** 2).sum()
    var_lap = cv2.Laplacian(roi_gray_down, cv2.CV_64F).var()
    return g * 0.7 + var_lap * 0.3

def eval_pos_quick(x, y, w, h, focus_pos, roi_factor=0.5, num_samples=1):
    # 1) Mover foco
    global last_frame
    LensCtrl.FocusMove(int(focus_pos))
    if num_samples == 1:
        time.sleep(0.03)
    else:
        time.sleep(0.2)
    # 2) Recortar ROI del último frame y convertir a gray
    frame = last_frame
    x0, y0, w0, h0 = int(x), int(y), int(w), int(h)
    sub_roi_color = frame[y0:y0+h0, x0:x0+w0]
    sub_gray = cv2.cvtColor(sub_roi_color, cv2.COLOR_BGR2GRAY)
    # 3) Downsamplear
    new_size = (int(sub_gray.shape[1]*roi_factor), int(sub_gray.shape[0]*roi_factor))
    if new_size[0] < 10 or new_size[1] < 10:
        roi_gray_down = sub_gray
    else:
        roi_gray_down = cv2.resize(sub_gray, new_size, interpolation=cv2.INTER_LINEAR)
    # 4) Calcular métrica num_samples veces
    total = 0.0
    for _ in range(num_samples):
        total += focus_metric_quick(roi_gray_down)
    return total / num_samples

# ----------------------------------------------------------------------
# 2. Scan + Refino multinivel rápido
# ----------------------------------------------------------------------
def smart_autofocus_quick(x, y, w, h,
                         f_min=2739, f_max=5159,
                         scan_step=200,
                         levels=(20, 5, 1),
                         roi_factor=0.5):
    print(f"🔍 Iniciando enfoque rápido: F {f_min}–{f_max}, step {scan_step}")
    scan_positions = list(range(f_min, f_max+1, scan_step))
    best_f, best_s = scan_positions[0], -1.0

    for index, p in enumerate(scan_positions):
        if index < 2 or index >= len(scan_positions) - 2:
            print(f"🔍 Ignorando posición {p} (fuera de rango)")
            continue
        
        s = eval_pos_quick(x, y, w, h, p, roi_factor=roi_factor, num_samples=4)
        print(f"[Scan] F={p}  S={s}")
        if s > best_s:
            best_f, best_s = p, s


    for radius in levels:
        low  = max(f_min, best_f - radius)
        high = min(f_max, best_f + radius)
        step_ref = max(radius // 2, 1)
        for p in range(low, high+1, step_ref):
            s = eval_pos_quick(x, y, w, h, p, roi_factor=roi_factor, num_samples=1)
            if s > best_s:
                best_f, best_s = p, s
                print(f" ↳ Mejorado → F={p}  S={s:.0f}  (r={radius})")

    print(f"✅ Focus óptimo = {best_f}, score={best_s:.0f}")
    return best_f, best_s

# ----------------------------------------------------------------------
# 3. AUTOZOOM + AUTOFOCO
# ----------------------------------------------------------------------
def autofocus_and_zoom():
    global last_frame, last_mask, is_focusing, current_zoom
    global is_okey, roi_active, last_roi_box, target_area, AREA_MIN, AREA_MAX, best_focus_global, best_score_global, current_focus

    if is_focusing or last_frame is None:
        return
    is_focusing = True
    try:
        if not roi_active:
            print("❌ No hay ROI activa o no se ha detectado un marcador.")
            return

        cnts, _ = cv2.findContours(last_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        x0, y0, w0, h0 = cv2.boundingRect(max(cnts, key=cv2.contourArea))
        last_roi_box = (x0, y0, w0, h0)

        # 1) Zoom al máximo
        LensCtrl.ZoomMove(ZOOM_MAX)
        current_zoom = ZOOM_MAX
        time.sleep(0.05)

        # 2) Enfoque rápido
        #best_focus, best_score = smart_autofocus_quick(x0, y0, w0, h0,
        #                                   f_min=2739, f_max=5159,
        #                                   scan_step=240, levels=(40,20,5, 1),
        #                                   roi_factor=0.5)
        best_focus , best_score  = 4500, 0.0
        best_focus_global = best_focus
        best_score_global = best_score
        LensCtrl.FocusMove(best_focus)
        print(f"🏁 AutoFoco finalizado → Zoom={current_zoom}, Focus={best_focus}")
        current_focus = best_focus

        # ← Tras el primer autofocus, fijamos TARGET_AREA si aún no existe
        if target_area is None:
            cnts2, _ = cv2.findContours(last_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            x1, y1, w_rect, h_rect = cv2.boundingRect(max(cnts2, key=cv2.contourArea))
            primera_area = w_rect * h_rect
            target_area = float(primera_area)
            AREA_MIN = int(target_area * (1.0 - AREA_TOLERANCE))
            AREA_MAX = int(target_area * (1.0 + AREA_TOLERANCE))
            print(f"🎯 Área objetivo fijada dinámicamente: {target_area:.0f}px")
            print(f"   → Rango de área: [{AREA_MIN}, {AREA_MAX}]")

        is_okey = True

    finally:
        is_focusing = False

# ----------------------------------------------------------------------
# 4. Refino local de foco si hace falta
# ----------------------------------------------------------------------

def refine_focus_range(x, y, w, h, current_focus,
                       delta=20, roi_factor=0.5):
    """
    Busca en todo el rango [current_focus - delta, current_focus + delta]
    cuál posición da mejor score de nitidez (eval_pos_quick).
    Retorna esa posición de foco.
    """
    global best_focus_global, best_score_global

    # 2) Para compararlas, medimos también el score en current_focus (si no coincide con best_focus_global)
    cf = int(current_focus)
    score_cf = best_score_global

    # Empezamos con la mejor posición conocida y su score:
    best_f = best_focus_global if best_focus_global is not None else cf
    best_s = best_score_global if best_focus_global is not None else score_cf

    # 3) Definimos los límites del rango (evitamos negativos o pasarnos del máximo razonable)
    low  = max(cf - delta, 0)
    high = min(cf + delta, 50000)  # asumiendo 0..50000 como rango total de foco

    scan_positions = list(range(low, high+1, 10))

    # 4) Escaneo “bruto” en todo ese rango
    for p in scan_positions:
        if p == best_f:
            continue
        print(f"🔍 Evaluando foco {p}...")
        s = eval_pos_quick(x, y, w, h, p,
                           roi_factor=roi_factor,
                           num_samples=1)
        if s > best_s:
            best_s = s
            best_f = p

    # 5) Actualizamos la referencia global
    best_focus_global = best_f
    best_score_global = best_s

    print(f"   ↳ Búsqueda en rango [{low}..{high}]: mejor_focus={best_f}, score={best_s:.0f}")
    return best_f

def connect_lens(idx=0):
    ScanUsbWithLensInfo()
    if UsbConnect(idx) != DV.RET_SUCCESS:
        print("❌ LensConnect no disponible"); return None
    for init, move, val in [(LensCtrl.ZoomInit,  LensCtrl.ZoomMove,  4111),
                            (LensCtrl.FocusInit, LensCtrl.FocusMove, 2860),
                            (LensCtrl.IrisInit,  LensCtrl.IrisMove,     0)]:
        init(); move(val)
    print("🔌 LensConnect listo"); return True

def frame_handler(cam, stream, frame):
    if not frame_q.full():
        frame_q.put(frame)
    cam.queue_frame(frame)

def keyboard_zoom_worker():
    global current_zoom
    while True:
        try:
            key = key_q.get(timeout=0.1)
        except queue.Empty:
            continue
        if key == 82:    # Flecha ↑
            current_zoom = min(current_zoom + ZOOM_STEP, ZOOM_MAX)
            LensCtrl.ZoomMove(current_zoom)
            print(f"🔍 Zoom={current_zoom}")
        elif key == 84:  # Flecha ↓
            current_zoom = max(current_zoom - ZOOM_STEP, ZOOM_MIN)
            LensCtrl.ZoomMove(current_zoom)
            print(f"🔍 Zoom={current_zoom}")

def has_two_parallel_pairs(box) -> bool:
    segs   = np.roll(box, -1, axis=0) - box
    angles = np.round((np.degrees(np.arctan2(segs[:,1], segs[:,0])) % 180) / 10) * 10
    _, counts = np.unique(angles, return_counts=True)
    return np.sum(counts >= 2) >= 2

# ----------------------------------------------------------------------
# 5. FUNCION PRINCIPAL
# ----------------------------------------------------------------------
def main():
    # Declaramos globals PARA EVITAR NameError
    global frame_q, key_q, last_frame, roi_active, last_mask
    global current_zoom, last_roi_box, target_area, AREA_MIN, AREA_MAX, parent_node

    frame_q, key_q = queue.Queue(30), queue.Queue(1)
    frame_skip = 0

    if connect_lens() is None:
        return
    cv2.setUseOptimized(True)

    with vmb.VmbSystem.get_instance() as vim:
        cam = vim.get_all_cameras()[0]
        with cam:
            cam.get_feature_by_name("PixelFormat").set(vmb.PixelFormat.BayerRG8)
            width = cam.get_feature_by_name("Width")
            height = cam.get_feature_by_name("Height")
            offsetX = cam.get_feature_by_name("OffsetX")
            offsetY = cam.get_feature_by_name("OffsetY")

            width.set(WIDTH)
            height.set(HEIGHT)
            offsetX.set((4512 - WIDTH)//2)
            offsetY.set((4512 - HEIGHT)//2)

            print(f"✅ Resolución: {width.get()}x{height.get()} "
                  f"(OffsetX={offsetX.get()}, OffsetY={offsetY.get()})")

            cam.get_feature_by_name("ExposureAuto").set("Off")
            cam.get_feature_by_name("ExposureTime").set(EXPOSURE_TIME)
            cam.get_feature_by_name("GainAuto").set("Off")
            cam.get_feature_by_name("Gain").set(0.0)
            cam.get_feature_by_name("IntensityControllerTarget").set(50.0)
            cam.get_feature_by_name("Gamma").set(1.0)
            cam.get_feature_by_name("BalanceWhiteAuto").set("Off")
            cam.get_feature_by_name("BalanceRatioSelector").set("Red")
            cam.get_feature_by_name("BalanceRatio").set(2.87)
            cam.get_feature_by_name("BalanceRatioSelector").set("Blue")
            cam.get_feature_by_name("BalanceRatio").set(2.40)

            cv2.namedWindow("Vista", cv2.WINDOW_NORMAL)
            cv2.namedWindow("Mask", cv2.WINDOW_NORMAL)
            cam.start_streaming(handler=frame_handler)

            rx0 = ry0 = rx1 = ry1 = lost = 0

            threading.Thread(target=keyboard_zoom_worker, daemon=True).start()
            while True:
                try:
                    frame = frame_q.get(timeout=1.0)
                except queue.Empty:
                    continue

                h, w = frame.get_height(), frame.get_width()
                bayer = np.frombuffer(frame.get_buffer(), np.uint8).reshape(h, w)
                full_disp = cv2.cvtColor(bayer, cv2.COLOR_BAYER_RG2RGB)
                last_frame = full_disp.copy()

                # Seleccionar ROI o imagen completa
                if roi_active and rx1 > rx0 and ry1 > ry0:
                    sub = full_disp[ry0:ry1, rx0:rx1]
                    offX, offY = rx0, ry0
                    if sub.size == 0:
                        sub, offX, offY, roi_active = full_disp, 0, 0, False
                        rx0, ry0, rx1, ry1 = 0, 0, w, h
                else:
                    sub, offX, offY, roi_active = full_disp, 0, 0, False
                    rx0, ry0, rx1, ry1 = 0, 0, w, h

                # -------- Máscara por INTENSIDAD y cierre de huecos ---------------
                hsv = cv2.cvtColor(sub, cv2.COLOR_BGR2HSV)
                v_channel = hsv[:, :, 2]
                _, mask= cv2.threshold(v_channel, INTENSITY_MIN, 255, cv2.THRESH_BINARY)

                # Cerrar huecos grandes
                kernel_cerrar = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_cerrar)


                # Bordes y un poco de cierre otra vez
                kernel_edge = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
                mask_edges = cv2.morphologyEx(mask, cv2.MORPH_GRADIENT, kernel_edge)
                mask_edges = cv2.morphologyEx(mask_edges, cv2.MORPH_CLOSE, kernel_cerrar)

                # Dilatación ligera para cerrar discontinuidades
                mask_final = cv2.dilate(mask_edges,
                                    cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)),
                                    iterations=1)

                last_mask = mask_final.copy()
                cv2.imshow("Mask", cv2.resize(mask_final, (400, 400)))

                # -------- contornos & validación geométrica --------
                cnts, _ = cv2.findContours(mask_final, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                # Calculamos área mínima dinámica según zoom
                min_area_dynamic = int(MIN_AREA * (current_zoom / ZOOM_MIN) ** 2)

                mejor_score = -1.0
                mejor_box_global = None
                mejor_area = 0.0

                EPS = 1e-6 

                for c in cnts:
                    area_c = cv2.contourArea(c)
                    if area_c < min_area_dynamic:
                        print(f"🔍 Contorno descartado por área pequeña: {area_c:.0f} < {min_area_dynamic}")
                        continue

                    # Dibujo en azul de cada contorno “candidato” por tamaño
                    cv2.drawContours(full_disp, [c + np.array([offX, offY])], -1, (255, 0, 0), 2)

                    # Ajustamos rectángulo mínimo
                    rect = cv2.minAreaRect(c)
                    (cx, cy), (rw, rh), angle = rect

                    # Ajustamos rectángulo mínimo
                    rect = cv2.minAreaRect(c)
                    (cx, cy), (rw, rh), angle = rect

                    # Filtramos tamaños pequeños
                    if rw < 30 or rh < 30:
                        cv2.polylines(full_disp,
                      [cv2.boxPoints(rect).astype(int) + np.array([offX, offY])],
                      True, (0, 165, 255), 2)
                        print(f"🔍 Contorno pequeño: {rw}x{rh} (área={area_c})")
                        continue
                    
                    aspect_ratio = max(rw, rh) / (min(rw, rh) + 1e-6)
                    if aspect_ratio > 1.5 or aspect_ratio < 0.7:
                        cv2.polylines(
                            full_disp,
                            [cv2.boxPoints(rect).astype(int) + np.array([offX, offY])],
                            True,
                            (0, 0, 255),
                            2
                        )
                        cv2.putText(
                            full_disp,
                            f"AR={aspect_ratio:.2f}",
                            (int(cx + offX), int(cy + offY)),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.7,
                            (0, 0, 255),
                            2
                        )
                        print(f"🔍 Aspect ratio no válido: {aspect_ratio:.2f} (rw={rw}, rh={rh})")
                        continue


                    # Obtenemos los 4 puntos del rectángulo
                    box = cv2.boxPoints(rect).astype(int)

                    # Comprobamos que haya dos pares de lados paralelos
                    if not has_two_parallel_pairs(box):
                        cv2.polylines(full_disp,
                                      [box + np.array([offX, offY])],
                                      True, (255, 0, 255), 2)
                        print("🔍 No hay dos pares de lados paralelos, descartando contorno")
                        continue

                            # 1) Cuán cerca está el aspecto a 1:1
                    score = area_c / (1.0 + abs(aspect_ratio - 1.0))
                    if score > mejor_score:
                        mejor_score = score
                        mejor_box_global = box + np.array([offX, offY])
                        mejor_area = area_c

                # Si encontramos un cuadrado válido, lo dibujamos
                if mejor_box_global is not None:
                    # Polilínea verde gruesa
                    cv2.polylines(full_disp, [mejor_box_global], True, (0, 255, 0), 3)

                    # Puntos en los vértices
                    for (px, py) in mejor_box_global:
                        cv2.circle(full_disp, (px, py), 6, (0, 255, 0), -1)

                    # Guardamos ROI y margen
                    x0, y0, w0, h0 = cv2.boundingRect(mejor_box_global)
                    last_roi_box = (x0, y0, w0, h0)

                    # Publicamos la información de detección
                    center_x = x0 + w0 // 2
                    center_y = y0 + h0 // 2

                    detection_info = {
                        "height": h,
                        "width": w,
                        "pos_x": center_x,
                        "pos_y": center_y
                    }

                    json_msg = json.dumps(detection_info)
                    msg = ROSString()
                    msg.data = json_msg
                    parent_node.publisher.publish(msg)
                    parent_node.get_logger().info(f"Published (adjusted ROI): {json_msg}")


                    rx0 = max(0, mejor_box_global[:, 0].min() - ROI_MARGIN)
                    ry0 = max(0, mejor_box_global[:, 1].min() - ROI_MARGIN)
                    rx1 = min(w, mejor_box_global[:, 0].max() + ROI_MARGIN)
                    ry1 = min(h, mejor_box_global[:, 1].max() + ROI_MARGIN)
                    roi_active = True

                    # ── Asignar TARGET_AREA tras primer autofocus ──
                    if is_okey and target_area is None:
                        cnts2, _ = cv2.findContours(last_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                        x1, y1, w_rect, h_rect = cv2.boundingRect(max(cnts2, key=cv2.contourArea))
                        primera_area = w_rect * h_rect
                        target_area = float(primera_area)
                        AREA_MIN = int(target_area * (1.0 - AREA_TOLERANCE))
                        AREA_MAX = int(target_area * (1.0 + AREA_TOLERANCE))
                        print(f"🎯 Área objetivo asignada dinámicamente: {target_area:.0f}px")
                        print(f"   → Rango de área: [{AREA_MIN}, {AREA_MAX}]")

                    if is_okey and target_area is not None and not is_focusing:
                        area_actual = mejor_area
                        print(f"🔍 Área actual del marcador: {area_actual:.0f}px")
                        if area_actual > AREA_MAX:
                            def thread_job_large():
                                global is_focusing, current_zoom, last_roi_box
                                global target_area, AREA_MIN, AREA_MAX

                                print(f"🔎 Marcador grande ({area_actual:.0f}px) → Ajuste iterativo de zoom/foco")
                                is_focusing = True

                                # 1) Si el cambio es demasiado brusco, abortamos:
                                ratio = area_actual / target_area
                                if ratio > 2.0 or ratio < 0.5:
                                    print(f"   ⚠️ Cambio muy brusco (ratio={ratio:.2f}), se ignora ajuste")
                                    is_focusing = False
                                    return

                                new_zoom = max(ZOOM_MIN, current_zoom - ZOOM_STEP)
                                print(f"🔎 Marcador grande ({area_actual:.0f}px) → Zoom proporcional a {new_zoom}")
                                current_zoom = new_zoom
                                LensCtrl.ZoomMove(current_zoom)
                                time.sleep(0.1)

                                fx, fy, fw, fh = last_roi_box
                                table_entry = LOOKUP_TABLE.get(current_zoom, {})
                                final_focus = table_entry["near"] if area_actual > 2000000 else table_entry["far"]
                                LensCtrl.FocusMove(final_focus)
                                print(f"   ╰→ Nuevo focus={final_focus}")

                                # 10) Reasignar target_area según la última ROI (fw×fh)
                                nueva_area = fw * fh
                                target_area = float(nueva_area)
                                AREA_MIN = int(target_area * (1.0 - AREA_TOLERANCE))
                                AREA_MAX = int(target_area * (1.0 + AREA_TOLERANCE))
                                print(f"🎯 Nuevo target_area = {target_area:.0f}px → rango [{AREA_MIN}, {AREA_MAX}]")

                                is_focusing = False

                            focus_thread = threading.Thread(target=thread_job_large, daemon=True)
                            focus_thread.start()

                        if area_actual < AREA_MIN:
                            def thread_job_small():
                                global is_focusing, current_zoom, last_roi_box
                                global target_area, AREA_MIN, AREA_MAX

                                print(f"🔎 Marcador pequeño ({area_actual:.0f}px) → Ajuste iterativo de zoom/foco")
                                is_focusing = True
                                ratio = target_area / area_actual
                                if ratio > 3.0 or ratio < 0.2:
                                    print(f"   ⚠️ Cambio muy brusco (ratio={ratio:.2f}), se ignora ajuste")
                                    is_focusing = False
                                    return

                                new_zoom = min(ZOOM_MAX, current_zoom + ZOOM_STEP)
                                print(f"🔎 Marcador pequeño ({area_actual:.0f}px) → Zoom proporcional a {new_zoom}")
                                current_zoom = new_zoom
                                LensCtrl.ZoomMove(current_zoom)
                                time.sleep(0.1)

                                fx, fy, fw, fh = last_roi_box
                                table_entry = LOOKUP_TABLE.get(current_zoom, {})
                                final_focus = table_entry["near"] if area_actual > 2000000 else table_entry["far"]
                                LensCtrl.FocusMove(final_focus)
                                print(f"   ╰→ Nuevo focus={final_focus}")

                                nueva_area = fw * fh
                                target_area = float(nueva_area)
                                AREA_MIN = int(target_area * (1.0 - AREA_TOLERANCE))
                                AREA_MAX = int(target_area * (1.0 + AREA_TOLERANCE))
                                print(f"🎯 Nuevo target_area = {target_area:.0f}px → rango [{AREA_MIN}, {AREA_MAX}]")

                                is_focusing = False

                            focus_thread = threading.Thread(target=thread_job_small, daemon=True)
                            focus_thread.start()



                else:
                    roi_active = False
                    rx0, ry0, rx1, ry1 = 0, 0, w, h

                if roi_active:
                    cv2.rectangle(full_disp, (rx0, ry0), (rx1, ry1), (255, 0, 255), 2)

                frame_skip += 1
                if frame_skip % 30 == 0 and not is_focusing and not is_okey:
                    def focus_job():
                        global is_focusing
                        if not is_focusing and not is_okey:
                            print("🟡 Iniciando enfoque automático y ajuste de zoom...")
                            autofocus_and_zoom()
                    focus_thread = threading.Thread(target=focus_job)
                    focus_thread.start()

                cv2.imshow("Vista", cv2.resize(full_disp, (1280, 720)))

                key = cv2.waitKey(1) & 0xFF
                if key == 27:
                    break
                key_q.put(key)

            cam.stop_streaming()
            cv2.destroyAllWindows()

def start(node):
    global parent_node
    parent_node = node  # Guardamos el nodo de ROS para usarlo si es necesario
    print("🔌 Iniciando cámara...")
    main()

if __name__=="__main__":
    main()
