import cv2
import mediapipe as mp
import time
import numpy as np
from Qarm_lib import QArm
import tkinter as tk
from tkinter import ttk

# =======================================================
#          SELECCIÓN MODO (REAL / SIMULACIÓN)
# =======================================================
hw = tk.Tk()
hw.title("Modo")
hw.geometry("260x140")
modo = tk.StringVar(value="0")

ttk.Label(hw, text="¿Robot físico o simulación?").pack(pady=10)

def fisico():
    modo.set("1")
    hw.destroy()

def sim():
    modo.set("0")
    hw.destroy()

ttk.Button(hw, text="Robot Físico", command=fisico).pack(pady=5)
ttk.Button(hw, text="Simulación", command=sim).pack(pady=5)

hw.mainloop()
hardware_mode = int(modo.get())


# =======================================================
#                 SELECCIÓN DE CÁMARA
# =======================================================
def detectar_camaras(max_test=6):
    disponibles = []
    for i in range(max_test):
        cap = cv2.VideoCapture(i)
        if cap.read()[0]:
            disponibles.append(i)
        cap.release()
    return disponibles

cams = detectar_camaras()

cam_win = tk.Tk()
cam_win.title("Seleccionar Cámara")
cam_win.geometry("300x150")

ttk.Label(cam_win, text="Selecciona la cámara:").pack(pady=10)

cam_var = tk.StringVar(value=str(cams[0] if cams else 0))

combo = ttk.Combobox(cam_win, textvariable=cam_var, values=[str(c) for c in cams])
combo.pack(pady=10)

def elegir_cam():
    cam_win.destroy()

ttk.Button(cam_win, text="Aceptar", command=elegir_cam).pack(pady=10)

cam_win.mainloop()

CAM_INDEX = int(cam_var.get())


# =======================================================
#                 Inicialización QArm
# =======================================================
qarm = QArm(hardware=hardware_mode, readMode=0)
time.sleep(1)

# HOME
base_pos   = 0.0
shoulder   = 0.0
gripper    = 0.1
joints = [base_pos, shoulder, 0.0, 0.0]

qarm.write_position(joints, gripper)
time.sleep(0.5)

# Límites
BASE_MIN = -1.57     # -90°
BASE_MAX =  1.57     # +90°

SH_MIN = -1       
SH_MAX = 1       


# =======================================================
#                     Mediapipe
# =======================================================
mp_hands = mp.solutions.hands
mp_draw  = mp.solutions.drawing_utils

hands = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=1,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5
)

def is_hand_open(lm):
    tips  = [8, 12, 16, 20]
    base  = [6, 10, 14, 18]
    count = 0
    for t, b in zip(tips, base):
        if lm[t].y < lm[b].y:
            count += 1
    return count >= 3


# =======================================================
#                     Cámara
# =======================================================
cap = cv2.VideoCapture(CAM_INDEX)
cap.set(3, 640)
cap.set(4, 480)


# =======================================================
#                   LOOP PRINCIPAL
# =======================================================
while True:
    ret, frame = cap.read()
    if not ret:
        break

    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = hands.process(rgb)

    estado = "NO HAND"

    if results.multi_hand_landmarks:
        for hand in results.multi_hand_landmarks:
            lm = hand.landmark
            mp_draw.draw_landmarks(frame, hand, mp_hands.HAND_CONNECTIONS)

            # Mano abierta = no mueve
            if is_hand_open(lm):
                estado = "OPEN"
                gripper = 0.1
            else:
                estado = "CLOSED"
                gripper = 0.9

                # ==============================
                #   CONTROL SOLO MANO CERRADA
                # ==============================

                # ------- IZQUIERDA / DERECHA -------
                x = lm[0].x
                if 0.43 <= x <= 0.57:
                    dx = 0
                else:
                    diff = abs(x - 0.5)
                    dx = diff * 0.04    # velocidad laterales

                if x < 0.43:
                    base_pos += dx
                elif x > 0.57:
                    base_pos -= dx

                base_pos = np.clip(base_pos, BASE_MIN, BASE_MAX)

                # ------- ARRIBA / ABAJO -------
                y = lm[0].y
                if 0.50 <= y <= 0.65:
                    dy = 0
                else:
                    diff = abs(y - 0.5)
                    dy = diff * 0.04

                # Mano arriba → brazo sube
                if y < 0.50:
                    shoulder -= dy
                elif y > 0.65:
                    shoulder += dy

                shoulder = np.clip(shoulder, SH_MIN, SH_MAX)

    # --------------------------------
    # Enviar al brazo
    # --------------------------------
    joints[0] = base_pos
    joints[2] = shoulder

    try:
        qarm.write_position(joints, gripper)
    except:
        pass

    cv2.putText(frame, estado, (10, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)

    cv2.imshow("Hand Tracker", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
