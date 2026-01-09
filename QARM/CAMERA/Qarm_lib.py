# QArm_lib.py
"""
QArm hardware abstraction for Quanser QArm (simulado o real).

Provee:
- Inicialización (Position mode)
- Lectura/escritura estándar (read_write_std, read_std, write_position)
- Stop inmediato (stop_immediate)
- Terminación limpia (terminate)
- Context manager support (__enter__/__exit__)

Notas:
- Este módulo mantiene compatibilidad con la API que usa el resto del proyecto.
- Buffers y estados se crean por instancia (no son atributos de clase) para evitar
  compartir estado entre varios objetos QArm.
"""

import numpy as np
import time
from quanser.hardware import HIL, HILError, MAX_STRING_LENGTH, Clock
from quanser.hardware.enumerations import BufferOverflowMode


class QArm:
    HOME_POSE = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float64)
    SLEEP_POSE = np.array([0.0, -17*np.pi/36, 15*np.pi/36, 0.0], dtype=np.float64)

    LIMITS_MAX = np.array([17*np.pi/18, 17*np.pi/36, 15*np.pi/36, 8*np.pi/9], dtype=np.float64)
    LIMITS_MIN = np.array([-17*np.pi/18, -17*np.pi/36, -19*np.pi/36, -8*np.pi/9], dtype=np.float64)

    # Channel definitions (constants)
    WRITE_OTHER_CHANNELS = np.array([1000, 1001, 1002, 1003, 1004, 11005, 11006, 11007], dtype=np.int32)
    READ_OTHER_CHANNELS = np.array([
        1000, 1001, 1002, 1003, 1004,
        3000, 3001, 3002, 3003, 3004,
        10000, 10001, 10002, 10003, 10004,
        11000, 11001, 11002, 11003, 11004
    ], dtype=np.int32)
    READ_ANALOG_CHANNELS = np.array([5, 6, 7, 8, 9], dtype=np.int32)

    def __init__(self, hardware=1, readMode=1, frequency=500, deviceId=0, hilPort=18900):
        """
        Inicializa QArm en modo Position (por defecto).

        Parameters
        ----------
        hardware : int
            1 para hardware real, 0 para virtual (simulación).
        readMode : int
            1 para tarea de lectura, 0 para lectura directa.
        frequency : int
            Frecuencia de muestreo (si readMode==1).
        deviceId : int
            ID de dispositivo (hardware).
        hilPort : int
            Puerto para simulador HIL (si hardware==0).
        """
        self.readMode = int(readMode)
        self.hardware = int(hardware)
        self.status = False

        # Buffers por instancia (evita compartir entre instancias)
        self.writeOtherBuffer = np.zeros(len(self.WRITE_OTHER_CHANNELS), dtype=np.float64)
        self.readOtherBuffer = np.zeros(len(self.READ_OTHER_CHANNELS), dtype=np.float64)
        self.readAnalogBuffer = np.zeros(len(self.READ_ANALOG_CHANNELS), dtype=np.float64)

        # External measurement arrays (5 entries each: 4 joints + gripper)
        self.measJointCurrent = np.zeros(5, dtype=np.float64)
        self.measJointPosition = np.zeros(5, dtype=np.float64)
        self.measJointSpeed = np.zeros(5, dtype=np.float64)
        self.measJointPWM = np.zeros(5, dtype=np.float64)
        self.measJointTemperature = np.zeros(5, dtype=np.float64)

        # HIL card
        self.card = HIL()
        if self.hardware:
            boardIdentifier = str(deviceId)
        else:
            boardIdentifier = f"0@tcpip://localhost:{hilPort}?nagle='off'"

        # default board specific options (BSO)
        boardSpecificOptions = (
            "j0_mode=0;j1_mode=0;j2_mode=0;j3_mode=0;"
            "gripper_mode=0;j0_profile_config=0;j0_profile_velocity=1.5708;"
            "j0_profile_acceleration=1.0472;j1_profile_config=0;"
            "j1_profile_velocity=1.5708;j1_profile_acceleration=1.0472;"
            "j2_profile_config=0;j2_profile_velocity=1.5708;"
            "j2_profile_acceleration=1.0472;j3_profile_config=0;"
            "j3_profile_velocity=1.5708;j3_profile_acceleration=1.0472;"
        )

        try:
            self.card.open("qarm_usb", boardIdentifier)
            if self.card.is_valid():
                self.card.set_card_specific_options(boardSpecificOptions, MAX_STRING_LENGTH)
                self.status = True

                if self.readMode == 1:
                    self.frequency = int(frequency)
                    self.samples = HIL.INFINITE
                    self.samplesToRead = 1

                    # Create reader task
                    self.readTask = self.card.task_create_reader(
                        int(self.frequency),
                        self.READ_ANALOG_CHANNELS, len(self.READ_ANALOG_CHANNELS),
                        None, 0,
                        None, 0,
                        self.READ_OTHER_CHANNELS, len(self.READ_OTHER_CHANNELS)
                    )

                    # Buffer overflow mode
                    if self.hardware:
                        self.card.task_set_buffer_overflow_mode(self.readTask, BufferOverflowMode.OVERWRITE_ON_OVERFLOW)
                        # PID gains (hardware only)
                        self.card.set_double_property(
                            np.array([128, 129, 130, 131, 133, 134, 135, 136, 138, 139, 140, 141]),
                            12,
                            np.array([8.89, 8.89, 8.89, 8.89, 0.012, 0.012, 0.012, 0.012, 10.23, 10.23, 10.23, 10.23])
                        )
                        print("QArm: setting PID gains")
                    else:
                        self.card.task_set_buffer_overflow_mode(self.readTask, BufferOverflowMode.SYNCHRONIZED)

                    # Start reading task
                    self.card.task_start(self.readTask, Clock.HARDWARE_CLOCK_0, self.frequency, 2**32 - 1)

                else:
                    # If not using task-based read, still set PID gains for hardware
                    if self.hardware:
                        self.card.set_double_property(
                            np.array([128, 129, 130, 131, 133, 134, 135, 136, 138, 139, 140, 141]),
                            12,
                            np.array([8.89, 8.89, 8.89, 8.89, 0.012, 0.012, 0.012, 0.012, 10.23, 10.23, 10.23, 10.23])
                        )
                        print("QArm: setting PID gains (non-task mode)")
                    print("QArm configured in Position Mode.")

        except HILError as h:
            print("QArm init HIL error:", h.get_error_message())
        except Exception as e:
            print("QArm init unexpected error:", e)

    # -------------------------
    # Helper: check validity
    # -------------------------
    def is_valid(self):
        return getattr(self.card, "is_valid", lambda: False)()

    # -------------------------
    # Stop immediate (send HOME several times)
    # -------------------------
    def stop_immediate(self):
        """
        Braking by sending HOME repeatedly to override internal profiles.
        Works reliably in Position Mode.
        """
        try:
            print("stop_immediate: sending HOME to cancel profile...")
            try:
                self.read_std()
            except Exception:
                pass

            # gripper current value (fallback 0.5)
            try:
                gpr = float(self.measJointPosition[4]) if len(self.measJointPosition) >= 5 else 0.5
            except Exception:
                gpr = 0.5

            # Send HOME several times to force override
            for _ in range(8):
                try:
                    self.write_position(self.HOME_POSE, gpr)
                except Exception:
                    pass
                time.sleep(0.01)

            print("stop_immediate: HOME sent.")
        except Exception as e:
            print("stop_immediate error:", e)

    # -------------------------
    # Combined write/read standard
    # -------------------------
    def read_write_std(self, phiCMD=None, gprCMD=None, baseLED=None):
        """
        Write commanded joint positions & gripper & LEDs, then read sensors.

        Parameters
        ----------
        phiCMD : array-like (4,) radians
        gprCMD : scalar or array-like (gripper)
        baseLED: array-like (3,) for RGB values (0..1)
        """
        if phiCMD is None:
            phiCMD = np.zeros(4, dtype=np.float64)
        if gprCMD is None:
            gprCMD = np.array([0.5], dtype=np.float64)
        if baseLED is None:
            baseLED = np.array([1.0, 0.0, 0.0], dtype=np.float64)

        # ensure arrays
        phiCMD = np.asarray(phiCMD, dtype=float).reshape(4,)
        # gripper scalar
        try:
            gpr_val = float(gprCMD[0] if hasattr(gprCMD, '__len__') else gprCMD)
        except Exception:
            gpr_val = 0.5
        gpr_val = float(np.clip(gpr_val, 0.1, 0.9))

        # prepare write buffer
        self.writeOtherBuffer.fill(0.0)
        for i in range(4):
            self.writeOtherBuffer[i] = float(phiCMD[i])
        self.writeOtherBuffer[4] = gpr_val
        # baseLED -> positions 5..7
        baseLED = np.asarray(baseLED, dtype=float).reshape(3,)
        self.writeOtherBuffer[5:8] = baseLED

        # IO: write then read
        try:
            self.card.write(
                None, 0,
                None, 0,
                None, 0,
                self.WRITE_OTHER_CHANNELS, len(self.WRITE_OTHER_CHANNELS),
                None,
                None,
                None,
                self.writeOtherBuffer
            )
            # update internal reads
            self.read_std()
        except HILError as h:
            print("read_write_std HIL error:", h.get_error_message())
        except Exception as e:
            print("read_write_std unexpected error:", e)

    # -------------------------
    # Read standard
    # -------------------------
    def read_std(self):
        """
        Read analog and other channels and update measurement arrays.
        After calling, the following attributes are updated:
            - measJointCurrent
            - measJointPosition (5 vals)
            - measJointSpeed
            - measJointPWM
            - measJointTemperature
        """
        try:
            if self.readMode == 1:
                # task-based read
                self.card.task_read(
                    self.readTask,
                    getattr(self, "samplesToRead", 1),
                    self.readAnalogBuffer,
                    None,
                    None,
                    self.readOtherBuffer
                )
            else:
                # direct read
                self.card.read(
                    self.READ_ANALOG_CHANNELS, len(self.READ_ANALOG_CHANNELS),
                    None, 0,
                    None, 0,
                    self.READ_OTHER_CHANNELS, len(self.READ_OTHER_CHANNELS),
                    self.readAnalogBuffer,
                    None,
                    None,
                    self.readOtherBuffer
                )
        except HILError as h:
            print("read_std HIL error:", h.get_error_message())
        except Exception as e:
            print("read_std unexpected error:", e)
        finally:
            # safely slice into measurement arrays
            try:
                self.measJointCurrent = self.readAnalogBuffer.copy()
                # ensure readOtherBuffer long enough
                rb = self.readOtherBuffer
                if len(rb) >= 20:
                    self.measJointPosition = rb[0:5].copy()
                    self.measJointSpeed = rb[5:10].copy()
                    self.measJointTemperature = rb[10:15].copy()
                    self.measJointPWM = rb[15:20].copy()
                else:
                    # fallback: zero arrays if buffer unexpected
                    self.measJointPosition = np.zeros(5, dtype=float)
                    self.measJointSpeed = np.zeros(5, dtype=float)
                    self.measJointTemperature = np.zeros(5, dtype=float)
                    self.measJointPWM = np.zeros(5, dtype=float)
            except Exception as e:
                print("read_std parse error:", e)

    # -------------------------
    # Write position
    # -------------------------
    def write_position(self, phiCMD=None, gprCMD=None):
        """
        Write desired joint positions (phiCMD: 4,) and gripper command (scalar).
        Accepts phiCMD as array-like; gprCMD may be scalar or array-like.
        """
        if phiCMD is None:
            phiCMD = np.zeros(4, dtype=np.float64)
        phiCMD = np.asarray(phiCMD, dtype=float).reshape(4,)

        # gripper scalar
        try:
            gpr_val = float(gprCMD[0] if hasattr(gprCMD, '__len__') else gprCMD)
        except Exception:
            gpr_val = 0.5
        gpr_val = float(np.clip(gpr_val, 0.1, 0.9))

        # prepare buffer
        for i in range(4):
            self.writeOtherBuffer[i] = float(phiCMD[i])
        self.writeOtherBuffer[4] = gpr_val

        try:
            self.card.write(
                None, 0,
                None, 0,
                None, 0,
                self.WRITE_OTHER_CHANNELS[0:5], 5,
                None,
                None,
                None,
                self.writeOtherBuffer[0:5]
            )
            return True
        except HILError as h:
            print("write_position HIL error:", h.get_error_message())
            return False
        except Exception as e:
            print("write_position unexpected error:", e)
            return False

    # -------------------------
    # Write LED
    # -------------------------
    def write_led(self, baseLED=None):
        """
        Write base RGB LED (3 values).
        """
        if baseLED is None:
            baseLED = np.array([1.0, 0.0, 0.0], dtype=np.float64)
        baseLED = np.asarray(baseLED, dtype=float).reshape(3,)
        self.writeOtherBuffer[5:8] = baseLED

        try:
            self.card.write(
                None, 0,
                None, 0,
                None, 0,
                self.WRITE_OTHER_CHANNELS[5:], 3,
                None,
                None,
                None,
                self.writeOtherBuffer[5:8]
            )
        except HILError as h:
            print("write_led HIL error:", h.get_error_message())
        except Exception as e:
            print("write_led unexpected error:", e)

    # -------------------------
    # Terminate
    # -------------------------
    def terminate(self):
        """
        Stop tasks and close connection to QArm.
        """
        try:
            if getattr(self, "readMode", 0) == 1 and hasattr(self, "readTask"):
                try:
                    self.card.task_stop(self.readTask)
                except Exception:
                    pass
                try:
                    self.card.task_delete(self.readTask)
                except Exception:
                    pass

            try:
                self.card.close()
            except Exception:
                pass

            print("QArm terminated successfully.")
        except HILError as h:
            print("terminate HIL error:", h.get_error_message())
        except Exception as e:
            print("terminate unexpected error:", e)

    # -------------------------
    # Context manager
    # -------------------------
    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.terminate()
