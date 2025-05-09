#!/usr/bin/env python3
import rospy
import os
import math
import roslib
import json
import numpy as np
import tf2_ros
import tf
import cv2
from tf2_geometry_msgs import do_transform_point
from sensor_msgs.msg import LaserScan, CameraInfo, Image
from gazebo_msgs.msg import LinkStates
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Float32, Header, Float32MultiArray
from geometry_msgs.msg import PointStamped, Point, TransformStamped
from cv_bridge import CvBridge, CvBridgeError
from image_geometry import PinholeCameraModel


# Parámetros globales (se obtienen de ROS)
PACKAGE_NAME = 'bim2ros'

class MovableObjects:
    
    def __init__(self):


        # Parámetros configurables
        self.timeout   = 2.0
        self.alpha     = 0.05
        self.threshold = 0.2
        self.ang_vel_tresh = 0.01   # rad/s
        self.lin_vel_tresh = 0.01   # m/s
        self.latest_real_velocity = None
        self.is_moved = None

        self.rho_thresh   = 20         # tolerancia para emparejar fondo (pixeles)
        self.theta_thresh = np.deg2rad(5)  # tolerancia angular fondo


        # Cámara: modelo pinhole
        self.cam_model = PinholeCameraModel()
        self.camera_frame = 'front_cam_link' # para transformaciones de la cámara
        self.camera_info_received = False

        # Pose de cada link
        self.link_poses = {}
        self.current_door = None
        self.link_vels = {}

        # Estado interno
        self.baseline_map              = None
        self.last_active_time         = None
        self.initial_link_pos = {}
        self.initial_real_yaw = {}
        self.last_moved_pts   = []

        # TF2 para pasar de laser0_frame → map
        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer)

        # Inicializar diccionarios para ventanas y puertas
        self.initial_link_pos  = {}
        self.initial_real_yaw  = {}

        self.prev_vals  = {}   # { link_name: último valor }
        self.prev_times = {}   # { link_name: timestamp anterior }

        self.bridge = CvBridge()

        # Publicadores y suscripción
        scan_topic         = rospy.get_param('~scan_topic', '/scan')
        self.marker_pub    = rospy.Publisher('~moved_markers', MarkerArray, queue_size=1)
        self.real_array_pub = rospy.Publisher('~door_angles_real', Float32MultiArray, queue_size=1)
        self.marker_pub_angle_estimated = rospy.Publisher('~vision_angle', Marker, queue_size=1)
        self.annotated_pub = rospy.Publisher('~annotated_image', Image, queue_size=1)
        self.real_vel_array_pub = rospy.Publisher('/movable_objects/real_velocities', Float32MultiArray, queue_size=1)

        self.child_link_names = []

        
        # Suscripción al tópico de la cámara
        rospy.Subscriber(scan_topic, LaserScan,       self.scan_cb,        queue_size=1)
        rospy.Subscriber('/gazebo/link_states', LinkStates, self.link_states_cb, queue_size=1)
        rospy.Subscriber('/front_cam/camera/image', Image,    self.image_cb,      queue_size=1)
        rospy.Subscriber('/front_cam/camera/camera_info', CameraInfo, self.info_cb, queue_size=1)
        rospy.Subscriber('/movable_objects/door_vel_real', Float32, self.vel_cb, queue_size=1)


        rospy.loginfo(f"[MovableObjects] Subscribed to {scan_topic}, thresh={self.threshold}m, timeout={self.timeout}s, alpha={self.alpha})")


        #Cargar vertices
        verts_path = os.path.join(roslib.packages.get_pkg_dir(PACKAGE_NAME), 'grids/', 'escenarioconVentana_movable_vertices.json')
        with open(verts_path, 'r') as f:
            self.link_corners_local = json.load(f)
        rospy.loginfo(f"[MovableObjects] Cargados {len(self.link_corners_local)} elementos con vértices.")

        #Cargar pose de los elementos moviles
        self.link_poses_file = {}
        poses_path = os.path.join(roslib.packages.get_pkg_dir(PACKAGE_NAME),'grids/','escenarioconVentana_movable_poses.json')
        with open(poses_path, 'r') as f:
            raw = json.load(f)
            for link_name, vals in raw.items():
                x, y, z, roll, pitch, yaw = vals
                # posición
                pos = type('P',(object,),{'x':x,'y':y,'z':z})()
                # orientación como cuaternión
                qx, qy, qz, qw = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
                ori = type('O',(object,),{'x':qx,'y':qy,'z':qz,'w':qw})()
                self.link_poses_file[link_name] = (pos, ori)
        rospy.loginfo(f"[MovableObjects] Cargadas {len(self.link_poses_file)} poses desde JSON.")




    def info_cb(self, info: CameraInfo):
        # Cargar parámetros de la cámara
        self.cam_model.fromCameraInfo(info)
        self.camera_info_received = True
        # Guardar frame_id de la cámara para transformaciones
        self.camera_frame = info.header.frame_id

    def vel_cb(self, msg):
        self.latest_real_velocity = msg.data


    
    def link_states_cb(self, msg: LinkStates):

        idx_drone = msg.name.index('quadrotor::base_link')
        drone_pose = msg.pose[idx_drone]
        # Guardamos la posición global del dron
        self.drone_pos  = drone_pose.position    # tiene x,y,z
        self.drone_ori  = drone_pose.orientation # cuaternión

        # 1) Detectar los enlaces hijos (solo una vez)
        if not self.child_link_names:
            for name in msg.name:
                if name.startswith('escenarioconVentana::') and name.endswith('_link') \
                and name != 'escenarioconVentana::pared_link':
                    self.child_link_names.append(name)
            if not self.child_link_names:
                return


        values = []
        vels   = []
        for link in self.child_link_names:
            idx = None
            try:
                idx = msg.name.index(link)
            except ValueError:
                values.append(0.0)
                continue

            # extraer base: 'puertaX' o 'ventanaY'
            base = link.split("::",1)[1].replace("_link","")

            # POSICIÓN DEL LINK
            pos = msg.pose[idx].position
            quat = msg.pose[idx].orientation

            self.link_poses[link] = (pos, quat)

            if base.startswith("ventana"):
                # ventana: calculamos desplazamiento = ||p - p0|| en XY
                if link not in self.initial_link_pos:
                    # guardamos posición inicial
                    self.initial_link_pos[link] = (pos.x, pos.y)
                x0,y0 = self.initial_link_pos[link]
                dist = math.hypot(pos.x - x0, pos.y - y0)
                # clip entre 0 y 1 m
                val = float(np.clip(dist, 0.0, 1.0))
            else:
                # puerta: calculamos yaw relativo
                if link not in self.initial_real_yaw:
                    _,_,yaw0 = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
                    self.initial_real_yaw[link] = yaw0
                _,_,yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
                d = yaw - self.initial_real_yaw[link]
                val = float(np.clip(math.atan2(math.sin(d), math.cos(d)), -math.pi/2, math.pi/2))

            # calcula velocidad angular (o lineal) respecto al último valor
            now = rospy.Time.now().to_sec()
            if link in self.prev_vals:
                dt  = now - self.prev_times[link]
                dv  = val - self.prev_vals[link]
                vel = dv/dt if dt>0 else 0.0
            else:
                vel = 0.0
            # guarda para la próxima
            self.prev_vals[link]  = val
            self.prev_times[link] = now

            values.append(val)
            vels.append(vel)

        # 3) Publicar array
        arr = Float32MultiArray()
        arr.data = values
        self.real_array_pub.publish(arr)  

        # 4) Publicar velocidades
        vel_arr = Float32MultiArray()
        vel_arr.data = vels
        self.real_vel_array_pub.publish(vel_arr)

        if vels:
            # Guarda velocidades por link
            self.link_vels = dict(zip(self.child_link_names, vels))

            # Busca el link móvil que supere su umbral correspondiente y, de entre ellos, el de mayor velocidad
            selected = None
            for link, vel in self.link_vels.items():
                name = link.split("::", 1)[1]  # 'puertaX' o 'ventanaY'
                # Elige umbral lineal para ventanas, angular para puertas
                thresh = self.lin_vel_tresh if name.startswith("ventana") else self.ang_vel_tresh

                # Si supera el umbral
                if abs(vel) > thresh:
                    # Si aún no hay ninguno seleccionado, o éste va más rápido, lo sustituye
                    if selected is None or abs(vel) > abs(self.link_vels[selected]):
                        selected = link

            if selected:
                self.current_door = selected
        else:
            rospy.loginfo("[MovableObjects DEBUG] No hay velocidades para calcular máxima")


    def scan_cb(self, msg: LaserScan):

        moved_any = False
        for link, vel in self.link_vels.items():
            name = link.split("::",1)[1]  # e.g. 'puerta2' o 'ventana1'
            if name.startswith("ventana"):
                if abs(vel) > self.lin_vel_tresh:
                    moved_any = True
                    break
            else:  # 'puerta'
                if abs(vel) > self.ang_vel_tresh:
                    moved_any = True
                    break

        # Si no se mueven los links, salir
        if not moved_any:
            #MarkerArray vacío para limpiar RViz
            empty = MarkerArray()
            self.marker_pub.publish(empty)
            self.last_moved_pts = []
            self.baseline_map = None
            moved_idx = []

            return
        

        # Reconstruir XY en laser0_frame
        angles = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment
        ranges = np.nan_to_num(np.array(msg.ranges, dtype=np.float64), nan=0.0, posinf=0.0, neginf=0.0)
        xs = ranges * np.cos(angles)
        ys = ranges * np.sin(angles)

        # Transformar cada punto a map
        points_map = []
        for x_b, y_b in zip(xs, ys):
            p_b = PointStamped()
            p_b.header.frame_id = 'laser0_frame'
            p_b.header.stamp    = msg.header.stamp
            p_b.point.x         = float(x_b)
            p_b.point.y         = float(y_b)
            p_b.point.z         = 0.0
            try:
                p_m = self.tf_buffer.transform(p_b, 'map', rospy.Duration(0.05))
                points_map.append((p_m.point.x, p_m.point.y, p_m.point.z))
            except Exception:
                points_map.append((np.nan, np.nan))
        points_map = np.array(points_map)

        # Primera pasada: guardar baseline
        if self.baseline_map is None:
            self.baseline_map = points_map.copy()
            rospy.loginfo(f"[MovableObjects] Baseline initialized with {len(points_map)} points in map frame")
            return

        # Detectar puntos movidos
        deltas = np.linalg.norm(points_map - self.baseline_map, axis=1)     
        moved_idx = np.where(deltas > self.threshold)[0]


        # Almacenar moved_pts para ROI en imagen
        self.last_moved_pts = [(points_map[i,0], points_map[i,1], points_map[i,2]) for i in moved_idx if not np.isnan(points_map[i,0])]

        now    = rospy.Time.now()
        header = Header(frame_id='map', stamp=now)
        markers = MarkerArray()

        if len(moved_idx) > 0:
            pts = points_map[moved_idx]
            valid_mask = ~np.isnan(pts[:,0])
            pts_valid = pts[valid_mask]


            # Publicar moved_points
            for idx in moved_idx:
                x_m, y_m, z_m = points_map[idx]
                if np.isnan(x_m) or np.isnan(y_m) or (x_m == 0.0 and y_m == 0.0):
                    continue
                m = Marker()
                m.header           = header
                m.ns               = 'moved_points'
                m.id               = int(idx)
                m.type             = Marker.SPHERE
                m.action           = Marker.ADD
                m.pose.position.x  = x_m
                m.pose.position.y  = y_m
                m.pose.position.z  = z_m
                m.lifetime         = rospy.Duration(0.5)
                m.pose.orientation.w = 1.0
                m.scale.x          = 0.1
                m.scale.y          = 0.1
                m.scale.z          = 0.1
                m.color            = ColorRGBA(0.0,1.0,0.0,0.8)
                markers.markers.append(m)

            # Actualizar baseline de moved
            self.baseline_map[moved_idx[valid_mask]] = pts_valid.copy()
            self.marker_pub.publish(markers)
            self.last_active_time = now
        else:
            # Clear markers after timeout
            if self.last_active_time and (now - self.last_active_time).to_sec() > self.timeout:
                clear = Marker(action=Marker.DELETEALL)
                self.marker_pub.publish(MarkerArray(markers=[clear]))
                self.last_active_time = None

        # Actualizar baseline de estáticos
        static_idx = np.setdiff1d(np.arange(len(points_map)), moved_idx)
        self.baseline_map[static_idx] = (
            self.alpha * points_map[static_idx] +
            (1.0 - self.alpha) * self.baseline_map[static_idx]
        )

    def image_cb(self, img_msg):
        # Estado de flags
        if not self.camera_info_received or not self.last_moved_pts:
            return
        
        tf_cam = self.tf_buffer.lookup_transform(
            self.camera_frame,  # p.ej. 'front_cam_optical_frame'
            'map',              # ó 'world' si lo prefieres
            rospy.Time(0),
            rospy.Duration(1.0)
        )
        rospy.loginfo(f"[Visión] Transform map→{self.camera_frame} obtenido")


        dp = self.drone_pos
        dq = self.drone_ori
        quat_dr = (dq.x, dq.y, dq.z, dq.w)
        rospy.loginfo(f"[Visión] Pose dron en world  = x:{dp.x:.2f}, y:{dp.y:.2f}, z:{dp.z:.2f}")
        rospy.loginfo(f"[Visión] Orientación dron en world = x:{dq.x:.2f}, y:{dq.y:.2f}, z:{dq.z:.2f}, w:{dq.w:.2f}")

        # 3) Convertir ROS→OpenCV
        cv_img = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')
        h_img, w_img = cv_img.shape[:2]
        rospy.loginfo(f"[Visión DEBUG] Imagen de tamaño {w_img}×{h_img}")

        # 4) Proyección de esquinas locales
        door = self.current_door
        rospy.loginfo(f"[Visión DEBUG] current_door = {door}")
        local_corners = self.link_corners_local.get(door, [])
        rospy.loginfo(f"[Visión DEBUG] esquinas locales cargadas: {len(local_corners)}")

        if door and local_corners and door in self.link_poses:
            # dims locales
            dx = max(c[0] for c in local_corners) - min(c[0] for c in local_corners)
            dy = max(c[1] for c in local_corners) - min(c[1] for c in local_corners)
            dz = max(c[2] for c in local_corners) - min(c[2] for c in local_corners)
            rospy.loginfo(f"[Visión DEBUG] dims locales calculadas: dx={dx:.3f}, dy={dy:.3f}, dz={dz:.3f}")

            # Offset inicial JSON
            pos_file, ori_file = self.link_poses_file[door]
            q_file = (ori_file.x, ori_file.y, ori_file.z, ori_file.w)

            # 2) Orientación actual de Gazebo
            pos_cur, ori_cur = self.link_poses[door]
            q_cur = (ori_cur.x, ori_cur.y, ori_cur.z, ori_cur.w)

            rospy.loginfo(f"[Visión DEBUG] Offset JSON {door}: ori_file={q_file}")
            rospy.loginfo(f"[Visión DEBUG] Sim    Gazebo {door}: ori_cur ={q_cur}")

            q_total = tf.transformations.quaternion_multiply(q_cur, q_file)
            rospy.loginfo(f"[Visión DEBUG] Quaternion total combinado = {q_total}")

            pos_m= pos_cur

            R_door = tf.transformations.quaternion_matrix(q_total)[:3, :3]

            R_drone_inv = tf.transformations.quaternion_matrix(tf.transformations.quaternion_inverse(quat_dr))[:3, :3]

            # vértices de la cara frontal
            y_c = dy / 2.0
            local_corners = [
                (0.0, y_c, 0.0),
                (dx,  y_c, 0.0),
                (dx,  y_c, dz),
                (0.0, y_c, dz),
            ]
            rospy.loginfo(f"[Visión DEBUG] local_corners (cara puerta) = {local_corners}")



            us3d, vs3d = [], []
            for i, (lx, ly, lz) in enumerate(local_corners):
                abs_pt = R_door.dot([lx, ly, lz]) + np.array([pos_m.x, pos_m.y, pos_m.z])
                dp = self.drone_pos
                rel_pt = abs_pt - np.array([dp.x, dp.y, dp.z])
                # c) rotación al sistema local del dron:
                pt_drone_frame = R_drone_inv.dot(rel_pt)

                rospy.logdebug(f"[Visión DEBUG] Corner {i} en dron_frame = {pt_drone_frame}")

                ps = PointStamped()
                ps.header.frame_id = 'map'   # o 'world' si lo usas
                ps.header.stamp    = img_msg.header.stamp
                ps.point.x, ps.point.y, ps.point.z = pt_drone_frame.tolist()

                try:

                    pc = do_transform_point(ps, tf_cam)
                    # e) proyectar a píxeles
                    u, v = self.cam_model.project3dToPixel(
                        (pc.point.x, pc.point.y, pc.point.z)
                    )
                    u_i = int(np.clip(u, 0, w_img-1))
                    v_i = int(np.clip(v, 0, h_img-1))
                    us3d.append(u_i)
                    vs3d.append(v_i)
                    rospy.logdebug(f"[Visión DEBUG] corner {i} → píxel ({u_i},{v_i})")
                except Exception as e:
                    rospy.logwarn(f"[Visión DEBUG] fallo proj corner {i}: {e}")

            rospy.loginfo(f"[Visión DEBUG] esquinas proyectadas: {len(us3d)}x, {len(vs3d)}y")
            if us3d and vs3d:
                umin, umax = min(us3d), max(us3d)
                vmin, vmax = min(vs3d), max(vs3d)
                # padding muy pequeño
                pad_u = pad_v = 5
                umin, umax = max(0, umin - pad_u), min(w_img, umax + pad_u)
                vmin, vmax = max(0, vmin - pad_v), min(h_img, vmax + pad_v)
                rospy.loginfo(f"[Visión DEBUG] ROI desde corners (pad 5px): u=[{umin},{umax}], v=[{vmin},{vmax}]")
            else:
                rospy.loginfo("[Visión DEBUG] No corners proyectados, activo fallback")

        else:
            rospy.loginfo(f"[Visión DEBUG] Datos insuficientes para corners")






        cv2.rectangle(cv_img, (umin, vmin), (umax, vmax), (255, 0, 0), 2)
        roi = cv_img[vmin:vmax, umin:umax]
        # HoughLinesP en ROI
        gray  = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur  = cv2.GaussianBlur(gray, (5,5), 0)
        edges = cv2.Canny(blur, 50, 150)
        lines = cv2.HoughLinesP(edges, 1, np.pi/180,
                                threshold=30, minLineLength=20, maxLineGap=5)
        if lines is None:
            rospy.loginfo("[Visión DEBUG] No se detectaron líneas en ROI")
            ann = self.bridge.cv2_to_imgmsg(cv_img, 'bgr8'); ann.header = img_msg.header
            self.annotated_pub.publish(ann)
            return
        rospy.loginfo(f"[Visión DEBUG] Hough detectó {len(lines)} líneas")

        # Dibujar segmentos y recopilar para PCA
        segs = []
        for i,(x1,y1,x2,y2) in enumerate(lines.reshape(-1,4)):
            pt1 = (x1+umin, y1+vmin)
            pt2 = (x2+umin, y2+vmin)
            cv2.line(cv_img, pt1, pt2, (0,255,0), 2)
            segs.append((pt1, pt2))
            rospy.loginfo(f"[Visión DEBUG] Segmento {i}: {pt1}->{pt2}")

        if len(segs) < 2:
            rospy.loginfo("[Visión DEBUG] <2 segmentos, saltando PCA")
            ann = self.bridge.cv2_to_imgmsg(cv_img, 'bgr8'); ann.header = img_msg.header
            self.annotated_pub.publish(ann)
            return

        # PCA para calcular ángulo
        pts = np.vstack([p for seg in segs for p in seg])
        mean = pts.mean(axis=0); cov = np.cov((pts-mean).T)
        eigvals,eigvecs = np.linalg.eigh(cov)
        principal = eigvecs[:, eigvals.argmax()]
        img_angle = math.atan2(principal[1], principal[0])
        rospy.loginfo(f"[Visión DEBUG] Ángulo PCA = {math.degrees(img_angle):.1f}°")

        # Publicar marcador de texto con el ángulo
        text_marker = Marker()
        text_marker.header.frame_id = 'map'
        text_marker.header.stamp = rospy.Time.now()
        text_marker.ns = 'vision_angle'
        text_marker.id = 0
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD

        text_marker.pose.position.x = 0.0
        text_marker.pose.position.y = 0.0
        text_marker.pose.position.z = 1.5
        text_marker.pose.orientation.w = 1.0
        text_marker.scale.z = 0.3
        text_marker.color = ColorRGBA(1, 1, 1, 1)

        text_marker.text = f"{math.degrees(img_angle):.1f}°"
        self.marker_pub_angle_estimated.publish(text_marker)

        ann = self.bridge.cv2_to_imgmsg(cv_img, 'bgr8'); 
        ann.header = img_msg.header
        self.annotated_pub.publish(ann)


def main():
    rospy.init_node('movable_objects', anonymous=False)
    node = MovableObjects()
    rospy.spin()

if __name__ == '__main__':
    main()
