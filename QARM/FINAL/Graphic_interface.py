# ============================================================
#                 Graphic_interface.py
# ============================================================

import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import numpy as np
import json
import time


class QArmGUI:
    def __init__(self, master, brazo):
        self.master = master
        self.brazo = brazo

        self.ruta = []
        self.editing_idx = None
        self.tiempo_entre = tk.DoubleVar(value=1.0)
        self.ciclos = tk.IntVar(value=1)
        self.emergency_flag = False

        master.title("Control QArm - Laboratorio ECA")
        master.geometry("980x760")
        master.resizable(False, False)

        # ---------------- CONTROLES MANUALES ----------------
        control_frame = ttk.LabelFrame(master, text="Control Manual", padding=12)
        control_frame.place(x=10, y=10, width=360, height=740)

        self.sliders = []
        self.labels = []

        # Sliders de articulaciones
        for i in range(4):
            ttk.Label(control_frame, text=f"J{i+1} (°)").pack(anchor="w")
            val = tk.DoubleVar(value=0)
            slider_frame = ttk.Frame(control_frame)
            slider_frame.pack(fill="x", pady=6)

            slider = ttk.Scale(
                slider_frame, from_=-170, to=170,
                orient="horizontal",
                variable=val, length=260,
                command=lambda e, i=i: self.slider_step(i)
            )
            slider.pack(side="left", padx=5)

            val_label = ttk.Label(slider_frame, text=f"{val.get():.0f}°", width=6)
            val_label.pack(side="right")

            self.sliders.append(val)
            self.labels.append(val_label)

            # Botones -1 y +1 grado
            btn_frame = ttk.Frame(control_frame)
            btn_frame.pack(pady=(0, 6))
            ttk.Button(btn_frame, text="-1°", width=6,
                    command=lambda i=i: self.ajustar_angulo(i, -1)).pack(side="left", padx=3)
            ttk.Button(btn_frame, text="+1°", width=6,
                    command=lambda i=i: self.ajustar_angulo(i, +1)).pack(side="left", padx=3)

        # Gripper
        ttk.Label(control_frame, text="Gripper (0.1 - 0.9)").pack(anchor="w", pady=(6, 0))
        self.gripper_val = tk.DoubleVar(value=0.5)
        slider_g_frame = ttk.Frame(control_frame)
        slider_g_frame.pack(fill="x", pady=6)

        slider_g = ttk.Scale(
            slider_g_frame, from_=0.1, to=0.9,
            orient="horizontal",
            variable=self.gripper_val, length=260,
            command=lambda e: self.actualizar_slider()
        )
        slider_g.pack(side="left", padx=5)

        ttk.Label(slider_g_frame, textvariable=self.gripper_val,
                width=6).pack(side="right")

        # Botón HOME
        ttk.Button(control_frame, text="Volver a HOME",
                command=self.volver_home).pack(fill="x", pady=(12, 4))

        # Parada de emergencia
        tk.Button(
            control_frame,
            text="PARADA DE EMERGENCIA",
            font=("Arial", 16, "bold"),
            fg="white", bg="#cc0000",
            activebackground="#ff0000",
            activeforeground="white",
            relief="raised", bd=5,
            command=self.parada_emergencia
        ).pack(fill="x", padx=10, pady=15, ipadx=10, ipady=10)

        ttk.Button(control_frame, text="Reiniciar Robot",
                command=self.reiniciar_robot).pack(fill="x", pady=(0, 4))

        ttk.Button(control_frame, text="Cerrar conexión y salir",
                command=self.salir).pack(fill="x")

        # ---------------- GESTIÓN DE RUTA ----------------
        ruta_frame = ttk.LabelFrame(master, text="Gestión de Ruta", padding=12)
        ruta_frame.place(x=380, y=10, width=590, height=740)

        # --- fila superior ---
        btns_top = ttk.Frame(ruta_frame)
        btns_top.pack(fill="x", pady=(0, 8))

        ttk.Button(btns_top, text="Guardar Punto",
                command=self.guardar_punto).grid(row=0, column=0, padx=6, pady=2)
        ttk.Button(btns_top, text="Editar Punto (cargar)",
                command=self.iniciar_edicion).grid(row=0, column=1, padx=6, pady=2)
        ttk.Button(btns_top, text="Aplicar edición",
                command=self.aplicar_edicion).grid(row=0, column=2, padx=6, pady=2)
        ttk.Button(btns_top, text="Eliminar Punto",
                command=self.eliminar_punto).grid(row=0, column=3, padx=6, pady=2)

        # --- fila media ---
        btns_mid = ttk.Frame(ruta_frame)
        btns_mid.pack(fill="x", pady=(0, 10))

        ttk.Button(btns_mid, text="Ejecutar Ruta",
                command=self.ejecutar_ruta).grid(row=0, column=0, padx=6, pady=2)
        ttk.Button(btns_mid, text="Nueva Ruta",
                command=self.nueva_ruta).grid(row=0, column=1, padx=6, pady=2)
        ttk.Button(btns_mid, text="Guardar en archivo",
                command=self.guardar_archivo).grid(row=0, column=2, padx=6, pady=2)
        ttk.Button(btns_mid, text="Cargar desde archivo",
                command=self.cargar_archivo).grid(row=0, column=3, padx=6, pady=2)

        # Tiempo y ciclos
        config_frame = ttk.Frame(ruta_frame)
        config_frame.pack(fill="x", pady=(6, 8))

        ttk.Label(config_frame, text="Tiempo entre puntos (s):").grid(
            row=0, column=0, padx=(12, 4))
        ttk.Entry(config_frame, textvariable=self.tiempo_entre, width=8).grid(
            row=0, column=1, padx=6)

        ttk.Label(config_frame, text="Cantidad de ciclos:").grid(
            row=0, column=2, padx=(12, 4))
        ttk.Entry(config_frame, textvariable=self.ciclos, width=6).grid(
            row=0, column=3, padx=6)

        # Lista de puntos
        ttk.Label(ruta_frame, text="Puntos guardados:").pack(anchor="w", pady=(6, 0))
        self.lista_puntos = tk.Listbox(ruta_frame, height=26, width=70)
        self.lista_puntos.pack(padx=4, pady=(6, 0))
        self.lista_puntos.bind("<<ListboxSelect>>", self.on_listbox_select)

        self.status_label = ttk.Label(ruta_frame, text="", foreground="darkorange")
        self.status_label.pack(pady=(6, 0))

        self.actualizar_slider()

    # ============================================================
    # FUNCIONES DE CONTROL
    # ============================================================

    def slider_step(self, idx):
        val = round(self.sliders[idx].get())
        if self.sliders[idx].get() != val:
            self.sliders[idx].set(val)
        self.actualizar_slider()

    def actualizar_slider(self):
        for i, v in enumerate(self.sliders):
            self.labels[i].config(text=f"{v.get():.0f}°")
        self.actualizar_pos()

    def actualizar_pos(self):
        angulos = np.array([v.get() for v in self.sliders])
        try:
            self.brazo.write_position(
                np.deg2rad(angulos),
                np.array([self.gripper_val.get()])
            )
        except:
            pass

    def ajustar_angulo(self, idx, delta):
        self.sliders[idx].set(self.sliders[idx].get() + delta)
        self.actualizar_slider()

    def volver_home(self):
        try:
            self.brazo.write_position(self.brazo.HOME_POSE,
                                    np.array([self.gripper_val.get()]))
            for v in self.sliders:
                v.set(0)
            self.actualizar_slider()
        except Exception as e:
            print("Error HOME:", e)

    def salida_segura(self):
        try:
            self.brazo.terminate()
        except:
            pass
        self.master.quit()

    def salir(self):
        self.salida_segura()

    def parada_emergencia(self):
        print(">>> PARADA DE EMERGENCIA <<<")
        self.emergency_flag = True

        g = float(self.gripper_val.get())
        for _ in range(8):
            try:
                self.brazo.write_position(self.brazo.HOME_POSE, np.array([g]))
            except:
                pass
            time.sleep(0.01)

    def reiniciar_robot(self):
        self.emergency_flag = False
        messagebox.showinfo("Reinicio", "Robot listo y habilitado.")

    # ============================================================
    #   GESTIÓN DE RUTA
    # ============================================================

    def guardar_punto(self):
        punto = {
            "pos": [float(v.get()) for v in self.sliders],
            "gripper": float(self.gripper_val.get()),
            "tiempo": float(self.tiempo_entre.get())
        }

        if self.editing_idx is not None:
            self.ruta[self.editing_idx] = punto
            self.editing_idx = None
        else:
            self.ruta.append(punto)

        self.actualizar_lista()

    def iniciar_edicion(self):
        sel = self.lista_puntos.curselection()
        if not sel:
            return
        idx = sel[0] // 2

        punto = self.ruta[idx]

        for i in range(4):
            self.sliders[i].set(punto["pos"][i])

        self.gripper_val.set(punto["gripper"])
        self.tiempo_entre.set(punto["tiempo"])

        self.editing_idx = idx
        self.actualizar_slider()

    def aplicar_edicion(self):
        if self.editing_idx is None:
            return

        self.ruta[self.editing_idx] = {
            "pos": [float(v.get()) for v in self.sliders],
            "gripper": float(self.gripper_val.get()),
            "tiempo": float(self.tiempo_entre.get())
        }

        self.editing_idx = None
        self.actualizar_lista()

    def eliminar_punto(self):
        sel = self.lista_puntos.curselection()
        if not sel:
            return
        idx = sel[0] // 2
        del self.ruta[idx]
        self.editing_idx = None
        self.actualizar_lista()

    def ejecutar_ruta(self):
        if not self.ruta:
            messagebox.showinfo("Ruta", "Ruta vacía.")
            return

        self.emergency_flag = False
        ciclos = max(1, self.ciclos.get())

        for c in range(ciclos):
            print(f"--- Ciclo {c+1}/{ciclos} ---")

            for i in range(len(self.ruta) - 1):
                if self.emergency_flag:
                    print("Ruta cancelada.")
                    return

                p1 = self.ruta[i+1]

                p1_deg = p1["pos"]
                grip = p1["gripper"]
                delay_s = p1["tiempo"]

                # Movimiento ANGULAR (MoveJ simple)
                self.brazo.write_position(
                    np.deg2rad(np.array(p1_deg)),
                    np.array([grip])
                )

                # delay
                t0 = time.time()
                while time.time() - t0 < delay_s:
                    if self.emergency_flag:
                        return
                    try:
                        self.master.update()
                    except:
                        pass
                    time.sleep(0.01)

        print("Ruta completa.")

    def nueva_ruta(self):
        self.ruta = []
        self.editing_idx = None
        self.actualizar_lista()

    def guardar_archivo(self):
        if not self.ruta:
            return
        file = filedialog.asksaveasfilename(
            defaultextension=".json",
            filetypes=[("JSON", "*.json")]
        )
        if file:
            with open(file, "w") as f:
                json.dump(self.ruta, f, indent=4)

    def cargar_archivo(self):
        file = filedialog.askopenfilename(
            filetypes=[("JSON", "*.json")]
        )
        if not file:
            return
        with open(file, "r") as f:
            self.ruta = json.load(f)

        self.editing_idx = None
        self.actualizar_lista()

    def actualizar_lista(self):
        self.lista_puntos.delete(0, "end")
        for i, p in enumerate(self.ruta):
            texto = (
                f"({int(p['pos'][0])}, {int(p['pos'][1])}, "
                f"{int(p['pos'][2])}, {int(p['pos'][3])})"
            )
            self.lista_puntos.insert("end", texto)
            self.lista_puntos.insert("end", f"── delay {p['tiempo']:.1f}s ──")

    def on_listbox_select(self, event):
        sel = self.lista_puntos.curselection()
        if not sel:
            return
        idx = sel[0] // 2
        punto = self.ruta[idx]

        for i in range(4):
            self.sliders[i].set(punto["pos"][i])

        self.gripper_val.set(punto["gripper"])
        self.tiempo_entre.set(punto["tiempo"])
        self.actualizar_slider()
