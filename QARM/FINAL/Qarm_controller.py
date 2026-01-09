# ============================================================
#                 Qarm_controller.py
# ============================================================

import numpy as np
import Qarm_lib as q

class QArmWrapper:
    """
    Envoltura del QArm para simplificar:
    - límites
    - manejo de simulación
    - normalización del gripper
    - envío de posiciones en radianes
    """

    HOME_POSE = np.array([0, 0, 0, 0], dtype=np.float64)

    JOINT_LIMITS = [
        (-170, 170),  # J1
        (-85,  85),   # J2
        (-75,  75),   # J3
        (-180, 180)   # J4
    ]

    def __init__(self, modo="simulacion"):
        self.modo = modo
        self.emergency = False
        self.brazo = None

        if modo == "simulacion":
            print("Modo simulación activado (QLabs)")
            self.brazo = q.QArm(hardware=0, readMode=0)
        else:
            print("Modo físico activado")
            self.brazo = q.QArm(hardware=1, readMode=0)  # hardware real

    def write_position(self, pos_rad, gripper_val):
        pos_deg = np.rad2deg(pos_rad)
        for i, (low, high) in enumerate(self.JOINT_LIMITS):
            pos_deg[i] = np.clip(pos_deg[i], low, high)
        pos_rad_clip = np.deg2rad(pos_deg)

        if isinstance(gripper_val, (list, np.ndarray)):
            g = float(gripper_val[0])
        else:
            g = float(gripper_val)
        g = np.clip(g, 0.1, 0.9)

        self.brazo.write_position(pos_rad_clip, g)

    def read_std(self):
        self.brazo.read_std()
        return {
            "current":      self.brazo.measJointCurrent,
            "position":     self.brazo.measJointPosition,
            "pwm":          self.brazo.measJointPWM,
            "speed":        self.brazo.measJointSpeed,
            "temperature":  self.brazo.measJointTemperature
        }

    @property
    def measJointPosition(self):
        return self.brazo.measJointPosition

    @property
    def measJointCurrent(self):
        return self.brazo.measJointCurrent

    @property
    def measJointPWM(self):
        return self.brazo.measJointPWM

    @property
    def measJointSpeed(self):
        return self.brazo.measJointSpeed

    @property
    def measJointTemperature(self):
        return self.brazo.measJointTemperature

    def terminate(self):
        self.brazo.terminate()

    def emergency_stop(self):
        self.emergency = True
        if self.brazo is not None:
            self.brazo.stop_immediate()

    def reset_emergency(self):
        self.emergency = False
