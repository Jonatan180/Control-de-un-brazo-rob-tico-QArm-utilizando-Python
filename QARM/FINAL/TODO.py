# ============================================================
#                T0DO.py
# ============================================================

import tkinter as tk
from tkinter import messagebox
from Graphic_interface import QArmGUI
from Qarm_controller import QArmWrapper


def ask_mode_gui():
    """Ventana inicial para seleccionar el modo de operación."""
    root = tk.Tk()
    root.withdraw()

    modo = None

    def elegir_simulacion():
        nonlocal modo
        modo = "simulacion"
        popup.destroy()
        root.destroy()

    def elegir_fisico():
        nonlocal modo
        modo = "fisico"
        popup.destroy()
        root.destroy()

    popup = tk.Toplevel()
    popup.title("Seleccionar modo")
    popup.geometry("280x150")
    popup.resizable(False, False)

    tk.Label(popup, text="Selecciona el modo de operación:", font=("Arial", 11)).pack(padx=20, pady=12)

    tk.Button(popup, text="Simulación", width=18, command=elegir_simulacion).pack(pady=4)
    tk.Button(popup, text="Físico", width=18, command=elegir_fisico).pack(pady=4)

    popup.protocol("WM_DELETE_WINDOW", lambda: root.destroy())
    root.mainloop()

    return modo


def main():
    modo = ask_mode_gui()
    if modo is None:
        print("No se seleccionó modo. Saliendo...")
        return

    brazo = QArmWrapper(modo=modo)

    root = tk.Tk()
    app = QArmGUI(root, brazo)

    try:
        root.mainloop()
    finally:
        # Asegura cierre seguro incluso si ocurre error o cierre forzado
        try:
            brazo.terminate()
        except:
            pass


if __name__ == "__main__":
    main()
