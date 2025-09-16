#!/usr/bin/env python3
import os
import sys
import json
import argparse
import signal
import time
import math
import rospkg
import rospy
from std_msgs.msg import Float32MultiArray
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from collections import deque

def load_index_for_link(link_name, map_name=None):
    """
    Carga el índice de 'link_name' desde <bim2ros>/grids/<map>_movable_indices.json
    Si no se indica map_name, intenta inferirlo desde el parámetro /world_file.
    """
    rospack = rospkg.RosPack()
    grids_dir = os.path.join(rospack.get_path('bim2ros'), 'grids')

    if not map_name:
        world_path = rospy.get_param("/world_file", "")
        if world_path:
            map_name = os.path.splitext(os.path.basename(world_path))[0]

    candidates = []
    if map_name:
        candidates.append(os.path.join(grids_dir, f"{map_name}_movable_indices.json"))

    # fallback: cualquier *_movable_indices.json que contenga el link
    for fname in os.listdir(grids_dir):
        if fname.endswith("_movable_indices.json"):
            candidates.append(os.path.join(grids_dir, fname))

    for path in candidates:
        if os.path.exists(path):
            with open(path, "r") as f:
                data = json.load(f)
            # los keys pueden venir como "puerta1_link" (sin "::")
            if link_name in data:
                return data[link_name]
    raise RuntimeError(f"No se encontró índice para '{link_name}'. Asegúrate de haber lanzado el mundo y generado el JSON de índices.")

class AngleLogger:
    def __init__(self, link_name, map_name, out_csv, out_png, duration, hz):
        self.link_name = link_name
        self.index = load_index_for_link(link_name, map_name)
        self.out_csv = out_csv
        self.out_png = out_png
        self.duration = duration
        self.dt = 1.0/float(hz)

        # buffers
        self.latest_real = None
        self.latest_est = None
        self.data = []  # (t, real, est, err)

        # decidir unidades: ventanas → m ; puertas → rad
        ln = link_name.lower()
        if ln.startswith("ventana"):
            self.units = "m"
            self.ylabel = "Desplazamiento [m]"
        else:
            self.units = "rad"
            self.ylabel = "Ángulo [rad]"  # si prefieres grados, lo convierto más abajo

        self.start_time = None

        self.sub_real = rospy.Subscriber(
            "/movable_objects/door_angles_real", Float32MultiArray, self.cb_real, queue_size=1
        )
        self.sub_est = rospy.Subscriber(
            "/movable_objects/estimated_values", Float32MultiArray, self.cb_est, queue_size=1
        )

    def cb_real(self, msg: Float32MultiArray):
        if 0 <= self.index < len(msg.data):
            self.latest_real = msg.data[self.index]

    def cb_est(self, msg: Float32MultiArray):
        if 0 <= self.index < len(msg.data):
            self.latest_est = msg.data[self.index]

    def spin(self):
        rate = rospy.Rate(1.0/self.dt)
        self.start_time = time.time()
        end_time = self.start_time + (self.duration if self.duration > 0 else 1e12)

        rospy.loginfo(f"[log_angles] Registrando '{self.link_name}' en idx={self.index} durante {self.duration or '∞'} s")
        while not rospy.is_shutdown() and time.time() < end_time:
            if self.latest_real is not None and self.latest_est is not None:
                t = time.time() - self.start_time
                real_val = self.latest_real
                est_val = self.latest_est

                if self.units == "rad":
                    # si prefieres en grados, descomenta:
                    real_val = math.degrees(real_val)
                    est_val  = math.degrees(est_val)
                    pass

                err = real_val - est_val
                self.data.append((t, real_val, est_val, err))
            rate.sleep()

        self.save_csv()
        self.plot()

    def save_csv(self):
        if not self.data:
            rospy.logwarn("[log_angles] No se recogieron datos.")
            return
        os.makedirs(os.path.dirname(self.out_csv) or ".", exist_ok=True)
        with open(self.out_csv, "w") as f:
            f.write("tiempo,valor_real,valor_estimado,error\n")
            for t, r, e, err in self.data:
                f.write(f"{t:.6f},{r:.6f},{e:.6f},{err:.6f}\n")
        rospy.loginfo(f"[log_angles] CSV guardado en: {self.out_csv}")

    def plot(self):
        if not self.data:
            return
        ts = [x[0] for x in self.data]
        reals = [x[1] for x in self.data]
        ests  = [x[2] for x in self.data]
        errs  = [x[3] for x in self.data]

        plt.figure(figsize=(8,5))
        plt.plot(ts, reals, label="Valor real")
        plt.plot(ts, ests,  label="Valor estimado", linestyle="--")
        plt.plot(ts, errs,  label="Error")
        plt.xlabel("Tiempo [s]")
        plt.ylabel(self.ylabel if self.units!="rad" else "Ángulo [º]")
        plt.title(f"{self.link_name}: real vs. estimado y error")
        plt.legend()
        plt.grid(True)
        os.makedirs(os.path.dirname(self.out_png) or ".", exist_ok=True)
        plt.tight_layout()
        plt.savefig(self.out_png, dpi=300)
        plt.close()
        rospy.loginfo(f"[log_angles] Figura guardada en: {self.out_png}")

def main():
    parser = argparse.ArgumentParser(description="Log de ángulo/desplazamiento (real vs estimado) y error.")
    parser.add_argument("--link", required=True, help="Nombre del link (ej: puerta1_link, ventana1_link)")
    parser.add_argument("--map", default=None, help="Nombre de mapa (para localizar el JSON de índices). Opcional.")
    parser.add_argument("--csv", default="logs/angulos.csv", help="Ruta de salida CSV")
    parser.add_argument("--png", default="logs/angulos.png", help="Ruta de salida PNG")
    parser.add_argument("--duration", type=float, default=30.0, help="Duración de registro [s]. 0 = infinito hasta Ctrl+C")
    parser.add_argument("--hz", type=float, default=20.0, help="Frecuencia de muestreo aprox. [Hz]")
    args, _ = parser.parse_known_args()

    rospy.init_node("log_angles", anonymous=True)
    logger = AngleLogger(args.link, args.map, args.csv, args.png, args.duration, args.hz)

    def _sigint(sig, frame):
        rospy.signal_shutdown("SIGINT")
    signal.signal(signal.SIGINT, _sigint)

    logger.spin()

if __name__ == "__main__":
    main()
