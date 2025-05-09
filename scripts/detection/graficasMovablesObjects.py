#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
from gazebo_msgs.msg import LinkStates
import matplotlib.pyplot as plt
import numpy as np
import threading
import time
from collections import deque

class TimeSeriesViewer:
    def __init__(self, history_size=300):
        rospy.init_node('time_series_viewer', anonymous=False)
        # Historial de tiempo
        self.times = deque(maxlen=history_size)
        # Flags y contenedores
        self.child_links = []          # nombres de enlaces detectados
        self.is_window   = []          # booleans: True si ventana
        self.door_vals   = []          # list de deques para puertas
        self.win_vals    = []          # list de deques para ventanas
        self.initialized = False
        self.lock        = threading.Lock()
        self.start_time  = time.time()
        
        # Setup de matplotlib
        plt.close('all')
        plt.ion()

        self.fig, (self.ax_door, self.ax_win) = plt.subplots(2,1, figsize=(8,8))
        # Inicializar listas para las líneas
        self.door_lines = []
        self.win_lines = []

        # Subscribe inicial a link_states para detectar enlaces
        rospy.Subscriber('/gazebo/link_states', LinkStates, self.link_init_cb, queue_size=1)
        # Subscribe a array de ángulos/posiciones
        rospy.Subscriber('/movable_objects/door_angles_real', Float32MultiArray,
                         self.array_cb, queue_size=1)
        rospy.loginfo("[TimeSeriesViewer] Suscrito a link_states y door_angles_real")

        # Configuración ejes
        self.ax_door.set_title('Ángulos de Puertas')
        self.ax_door.set_xlabel('Tiempo (s)')
        self.ax_door.set_ylabel('Ángulo (°)')  # ahora en grados
        self.ax_door.grid(True)
        self.ax_win.set_title('Desplazamiento de Ventanas')
        self.ax_win.set_xlabel('Tiempo (s)')
        self.ax_win.set_ylabel('Desplazamiento (m)')
        self.ax_win.grid(True)

    def link_init_cb(self, msg: LinkStates):
        # Solo detectar una vez
        if self.initialized:
            return
        for name in msg.name:
            if name.startswith('escenarioconVentana::') and name.endswith('_link') \
               and not name.endswith('pared_link'):
                self.child_links.append(name)
                base = name.split('::')[1][:-5]  # quitar '_link'
                is_win = base.startswith('ventana')
                self.is_window.append(is_win)
        if not self.child_links:
            return
        # Preparar deques según tipo
        for is_win in self.is_window:
            if is_win:
                self.win_vals.append(deque(maxlen=self.times.maxlen))
            else:
                self.door_vals.append(deque(maxlen=self.times.maxlen))

        door_count = 1
        win_count = 1
        # Crear líneas
        for i,_ in enumerate(self.child_links):
            if self.is_window[i]:
                line, = self.ax_win.plot([], [], 'b--', label=f'Ventana{win_count} i:{[i]}')
                self.win_lines.append(line)
                win_count += 1
            else:
                line, = self.ax_door.plot([], [], 'r-', label=f'Puerta{door_count}: i{[i]}')
                self.door_lines.append(line)
                door_count += 1
        self.ax_door.legend()
        self.ax_win.legend()
        self.initialized = True
        rospy.loginfo(f"[TimeSeriesViewer] Detectados {len(self.child_links)} enlaces: {self.child_links}")

    def array_cb(self, msg: Float32MultiArray):
        with self.lock:
            if not self.initialized:
                return
            t = time.time() - self.start_time
            self.times.append(t)

            n_links = len(self.is_window)
            if len(msg.data) != n_links:
                rospy.logwarn(f"[TimeSeriesViewer] Recibidos {len(msg.data)} valores, esperaba {n_links}, recortando.")
            data = list(msg.data)[:n_links]
            
            door_idx = 0
            win_idx  = 0
            # Distribuir valores
            for i, val in enumerate(data):
                if i < len(self.is_window) and self.is_window[i]:
                    self.win_vals[win_idx].append(val)
                    win_idx += 1
                else:
                    self.door_vals[door_idx].append(val)
                    door_idx += 1

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            with self.lock:
                if not self.initialized or not self.times:
                    rate.sleep(); continue
                ts = list(self.times)

                # Actualizar puertas (convertir a grados)
                door_deg_lists = [ [v * 180.0 / np.pi for v in dq] for dq in self.door_vals ]
                for line, deg_list in zip(self.door_lines, door_deg_lists):
                    line.set_data(ts, deg_list)
                # Ajustar rango X y Y puertas
                if len(ts) > 1:
                    self.ax_door.set_xlim(ts[0], ts[-1])
                else:
                    self.ax_door.set_xlim(ts[0]-1.0, ts[0]+1.0)
                all_d = np.hstack(door_deg_lists) if door_deg_lists else np.array([0,1])
                self.ax_door.set_ylim(all_d.min(), all_d.max())

                # Actualizar ventanas
                for line, dq in zip(self.win_lines, self.win_vals):
                    line.set_data(ts, list(dq))
                # Ajustar rango X y Y ventanas
                if len(ts) > 1:
                    self.ax_win.set_xlim(ts[0], ts[-1])
                else:
                    self.ax_win.set_xlim(ts[0]-1.0, ts[0]+1.0)
                all_w = np.hstack([list(dq) for dq in self.win_vals]) if self.win_vals else np.array([0,1])
                self.ax_win.set_ylim(all_w.min(), all_w.max())

                # Repintar
                self.fig.canvas.draw()
                self.fig.canvas.flush_events()
            rate.sleep()

if __name__ == '__main__':
    try:
        viewer = TimeSeriesViewer(history_size=300)
        viewer.run()
    except rospy.ROSInterruptException:
        pass