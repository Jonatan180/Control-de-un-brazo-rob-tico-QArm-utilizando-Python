from pal.products.qarm import QArm
from hal.products.qarm import QArmUtilities
import time
import numpy as np
import tkinter as tk
from tkinter import ttk

# =====================================================
#        VENTANA DE SELECCIÓN (FÍSICO / SIM)
# =====================================================

hw_root = tk.Tk()
hw_root.title("Modo de Operación")
hw_root.geometry("300x150")
hw_root.resizable(False, False)

modo = tk.StringVar(value="0")  # por defecto simulación

ttk.Label(hw_root, text="¿Robot físico o simulación?", font=("Arial", 12)).pack(pady=10)

def elegir_fisico():
    modo.set("1")
    hw_root.destroy()

def elegir_simulacion():
    modo.set("0")
    hw_root.destroy()

btn_frame = ttk.Frame(hw_root)
btn_frame.pack(pady=10)

ttk.Button(btn_frame, text="Robot Físico", command=elegir_fisico).grid(row=0, column=0, padx=10)
ttk.Button(btn_frame, text="Simulación", command=elegir_simulacion).grid(row=0, column=1, padx=10)

hw_root.mainloop()

hardware_mode = int(modo.get())
print("Iniciando QArm en modo:", "Físico" if hardware_mode == 1 else "Simulación")


# =====================================================
#                   FUNCIONES INTERNAS
# =====================================================

startTime = time.time()
def elapsed_time():
    return time.time() - startTime

def actualizar_visor(X, Y, Z, GAMMA, GRIP):
    visor_text.set(
        f"X = {X:.3f}\n"
        f"Y = {Y:.3f}\n"
        f"Z = {Z:.3f}\n"
        f"Gamma = {GAMMA:.3f}\n"
        f"Grip = {GRIP:.3f}"
    )

def inversa(X, Y, Z, GAMMA, GRP):
    actualizar_visor(X, Y, Z, GAMMA, GRP)

    result = [X, Y, Z, GAMMA, GRP]
    start = elapsed_time()

    positionCmd = result[0:3]
    gamma = result[3]
    gripCmd = result[4]

    # ================================
    #   CORRECCIÓN IMPORTANTE
    # ================================
    meas = myArm.measJointPosition     # <- atributo, SIN ()
    # ================================

    # IK
    allPhi, phiCmd = myArmUtilities.qarm_inverse_kinematics(
        positionCmd,
        gamma,
        meas[0:4]
    )

    # FK
    location, rotation = myArmUtilities.qarm_forward_kinematics(
        np.append(phiCmd, gamma)
    )

    print(f"Arm going to: {location}")

    # Movimiento
    myArm.read_write_std(
        phiCMD=phiCmd,
        gprCMD=gripCmd,
        baseLED=ledCmd
    )

    time.sleep(.05)

def on_change(*args):
    X = slider_X.get()
    Y = slider_Y.get()
    Z = slider_Z.get()
    M = slider_M.get()
    G = slider_G.get()
    inversa(X, Y, Z, M, G)

def mover_fino(slider, delta):
    slider.set(slider.get() + delta)
    on_change()

def apagar():
    myArm.terminate()
    root.destroy()


# =====================================================
#                     SETUP BRAZO
# =====================================================

myArm = QArm(hardware=hardware_mode, readMode=0)
myArmUtilities = QArmUtilities()
ledCmd = np.array([1, 0, 1], dtype=np.float64)

np.set_printoptions(precision=2, suppress=True)

# Valores iniciales seguros
X_val = 0.30
Y_val = 0.00
Z_val = 0.45
M_val = 0.00
G_val = 0.20


# =====================================================
#                   INTERFAZ GRÁFICA
# =====================================================

root = tk.Tk()
root.title("Control Inversa QArm")
root.geometry("950x350")
root.columnconfigure(0, weight=1)
root.rowconfigure(0, weight=1)

frame = ttk.Frame(root, padding=20)
frame.columnconfigure(1, weight=1)
frame.grid(row=0, column=0, sticky="nsew")

# ---------- VISOR ----------
visor_text = tk.StringVar()
visor_text.set("X = 0.000\nY = 0.000\nZ = 0.000\nGamma = 0.000\nGrip = 0.000")

ttk.Label(frame, textvariable=visor_text, font=("Arial", 12)).grid(
    row=0, column=4, rowspan=6, padx=20
)


# =====================================================
#                   SLIDERS Y BOTONES
# =====================================================

# ---- X ----
ttk.Label(frame, text="X").grid(row=0, column=0, sticky="w")
slider_X = ttk.Scale(frame, from_=-0.60, to=0.60, orient="horizontal")
slider_X.set(X_val)
slider_X.grid(row=0, column=1, sticky="ew")
slider_X.configure(command=on_change)

ttk.Button(frame, text="-1 cm", command=lambda: mover_fino(slider_X, -0.01)).grid(row=0, column=2, padx=5)
ttk.Button(frame, text="+1 cm", command=lambda: mover_fino(slider_X, +0.01)).grid(row=0, column=3, padx=5)

# ---- Y ----
ttk.Label(frame, text="Y").grid(row=1, column=0, sticky="w")
slider_Y = ttk.Scale(frame, from_=-0.45, to=0.45, orient="horizontal")
slider_Y.set(Y_val)
slider_Y.grid(row=1, column=1, sticky="ew")
slider_Y.configure(command=on_change)

ttk.Button(frame, text="-1 cm", command=lambda: mover_fino(slider_Y, -0.01)).grid(row=1, column=2, padx=5)
ttk.Button(frame, text="+1 cm", command=lambda: mover_fino(slider_Y, +0.01)).grid(row=1, column=3, padx=5)

# ---- Z ----
ttk.Label(frame, text="Z").grid(row=2, column=0, sticky="w")
slider_Z = ttk.Scale(frame, from_=0.30, to=0.75, orient="horizontal")
slider_Z.set(Z_val)
slider_Z.grid(row=2, column=1, sticky="ew")
slider_Z.configure(command=on_change)

ttk.Button(frame, text="-1 cm", command=lambda: mover_fino(slider_Z, -0.01)).grid(row=2, column=2, padx=5)
ttk.Button(frame, text="+1 cm", command=lambda: mover_fino(slider_Z, +0.01)).grid(row=2, column=3, padx=5)

# ---- Gamma ----
ttk.Label(frame, text="M (Gamma)").grid(row=3, column=0, sticky="w")
slider_M = ttk.Scale(frame, from_=-1.6, to=1.6, orient="horizontal")
slider_M.set(M_val)
slider_M.grid(row=3, column=1, sticky="ew")
slider_M.configure(command=on_change)

# ---- Gripper ----
ttk.Label(frame, text="G (Gripper)").grid(row=4, column=0, sticky="w")
slider_G = ttk.Scale(frame, from_=0.1, to=0.9, orient="horizontal")
slider_G.set(G_val)
slider_G.grid(row=4, column=1, sticky="ew")
slider_G.configure(command=on_change)

# ---- Apagar ----
ttk.Button(frame, text="Apagar", command=apagar).grid(row=5, column=0, columnspan=4, pady=15)

# Primer movimiento
inversa(X_val, Y_val, Z_val, M_val, G_val)

root.mainloop()
