#!/usr/bin/env python3
import rospy
import yaml
import os
import numpy as np
import math
import tf
import tf2_ros
import json
import rospkg
import roslib.packages
from PIL import Image as PILImage
from nav_msgs.msg import OccupancyGrid, MapMetaData
from nav_msgs.srv import GetMap, GetMapResponse
from sensor_msgs.msg import LaserScan, CameraInfo, Image
from image_geometry import PinholeCameraModel
from gazebo_msgs.msg import LinkStates
from std_msgs.msg import Float32MultiArray, ColorRGBA
from tf.transformations import euler_from_quaternion
from laser_line_extraction.msg import LineSegmentList
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PointStamped
from cv_bridge import CvBridge
from tf2_geometry_msgs import do_transform_point
import cv2

class DynamicMapUpdater:
    def __init__(self):
        rospy.init_node("dynamic_map_updater")

        self.tf_listener = tf.TransformListener()

        self.resolution =0.05
        self.width = 440
        self.height = 220

        # Variables para puertas y ventanas
        self.link_poses = {}
        self.initial_link_pos = {}
        self.initial_real_yaw = {}
        self.prev_vals = {}
        self.prev_times = {}
        self.child_link_names = []
        self.link_vels = {}
        self.current_door = None

        self.ang_vel_thresh = 0.01
        self.lin_vel_thresh = 0.01
        self.hit_threshold = 2 

        self.reference_lines = {}
        self.estimated_angles = {}
        self.line_buffer = {}
        self.buffer_size = 5

        # TF2 
        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer)
        self.bridge = CvBridge()

        # Cámara: modelo pinhole
        self.cam_model = PinholeCameraModel()
        self.camera_frame = 'front_cam_link' # para transformaciones de la cámara
        self.camera_info_received = False

        self.real_array_pub = rospy.Publisher('/movable_objects/door_angles_real', Float32MultiArray, queue_size=1)
        self.real_vel_array_pub = rospy.Publisher('/movable_objects/real_velocities', Float32MultiArray, queue_size=1)
        rospy.Subscriber('/gazebo/link_states', LinkStates, self.link_states_cb, queue_size=1)

        self.annotated_pub = rospy.Publisher("/image_annotated", Image, queue_size=1)

        rospy.Subscriber('/camera/rgb/camera_info', CameraInfo, self.info_cb, queue_size=1)
        rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_cb, queue_size=1)

        # Cargar JSON de poses movibles y geometría
        world_path = rospy.get_param("/world_file", "")

        if not world_path:
            self.movable_poses = {}
            self.link_geometries = {}
            
        else:
            base_name = os.path.splitext(os.path.basename(world_path))[0]
            json_path = os.path.join(rospkg.RosPack().get_path('bim2ros'), "grids", f"{base_name}_movable_poses.json")
            geometry_json_path = os.path.join(rospkg.RosPack().get_path('bim2ros'), "grids", f"{base_name}_movable_vertices.json")

            # Geometría
            if os.path.exists(geometry_json_path):
                with open(geometry_json_path, 'r') as f:
                    self.link_geometries = json.load(f)

            else:
                self.link_geometries = {}

            # Poses iniciales
            self.movable_poses = {}
            self.link_poses_file = {}
            if os.path.exists(json_path):

                with open(json_path, 'r') as f:
                    self.movable_poses = json.load(f)
                    for link_name, vals in self.movable_poses.items():

                        x, y, z, roll, pitch, yaw = vals
                        pos = type('P', (object,), {'x': x, 'y': y, 'z': z})()
                        qx, qy, qz, qw = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
                        ori = type('O', (object,), {'x': qx, 'y': qy, 'z': qz, 'w': qw})()
                        self.link_poses_file[link_name] = (pos, ori)

                rospy.loginfo(f"[MovableObjects] Cargadas {len(self.link_poses_file)} poses desde JSON.")
                self.generate_reference_lines()
            else:

                self.movable_poses = {}
                self.link_poses_file = {}
        

        # Cargar YAML del mapa
        yaml_path = rospy.get_param("~map_yaml", "")
        if not os.path.exists(yaml_path):
            return

        with open(yaml_path, 'r') as f:
            metadata = yaml.safe_load(f)

        image_path = os.path.join(os.path.dirname(yaml_path), metadata["image"])
        self.resolution = metadata["resolution"]
        self.origin = metadata["origin"]

        img = PILImage.open(image_path).convert('L')
        grid = np.array(img, dtype=np.uint8)
        grid = np.flipud(255 - grid)

        occ_grid = np.full(grid.shape, -1, dtype=np.int8)
        occ_grid[grid <= 50] = 0
        occ_grid[(grid > 50) & (grid < 254)] = 100
        occ_grid[grid >= 254] = -1

        self.occ_grid = occ_grid
        self.height, self.width = occ_grid.shape

        self.hit_counter = np.zeros((self.height, self.width), dtype=np.uint8)
        self.movable_mask = np.zeros((self.height, self.width), dtype=np.uint8)


        if "movable_mask" in metadata:
            self.movable_mask = np.array(metadata["movable_mask"], dtype=np.uint8)

        else:
            rospy.logwarn("No se encontró 'movable_mask' en el YAML.")

        if not os.path.exists(image_path):
            rospy.logerr(f" Imagen del mapa no encontrada: {image_path}")
            return

        self.map_msg = OccupancyGrid()
        self.map_msg.header.frame_id = "map"
        self.map_msg.info.resolution = self.resolution
        self.map_msg.info.width = self.width
        self.map_msg.info.height = self.height
        self.map_msg.info.origin.position.x = self.origin[0]
        self.map_msg.info.origin.position.y = self.origin[1]
        self.map_msg.info.origin.position.z = self.origin[2] if len(self.origin) > 2 else 0.0
        self.map_msg.info.origin.orientation.w = 1.0
        self.map_msg.info.map_load_time = rospy.Time.now()
        self.map_msg.data = self.occ_grid.flatten().tolist()

        self.map_pub = rospy.Publisher("/map", OccupancyGrid, queue_size=1, latch=True)
        self.meta_pub = rospy.Publisher("/map_metadata", MapMetaData, queue_size=1, latch=True)

        rospy.sleep(1.0)
        self.map_pub.publish(self.map_msg)
        meta_msg = MapMetaData()
        meta_msg.map_load_time = rospy.Time.now()
        meta_msg.resolution = self.resolution
        meta_msg.width = self.width
        meta_msg.height = self.height
        meta_msg.origin = self.map_msg.info.origin
        self.meta_pub.publish(meta_msg)

        self.service = rospy.Service("/static_map", GetMap, self.handle_static_map)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)

        rospy.Subscriber("/line_segments", LineSegmentList, self.line_callback)
        self.marker_pub = rospy.Publisher("/line_markers", Marker, queue_size=10)

        
        self.text_marker_pub = rospy.Publisher("/door_angle_markers", Marker, queue_size=1)
        self.estimated_array_pub = rospy.Publisher('/movable_objects/estimated_values', Float32MultiArray, queue_size=1) 

        rospy.loginfo("EL NODO SE INICIALIZO CORRECTAMENTE. Publicando mapa y escuchando /scan + /gazebo/link_states")

        rospy.Timer(rospy.Duration(0.2), self.geometry_displacement_cb)
        rospy.spin()

    def generate_reference_lines(self):
        for link, pose in self.movable_poses.items():
            verts = self.link_geometries[link]
            base_verts = [v for v in verts if abs(v[2]) < 0.01]     # Proyectar a XY
            if len(base_verts) < 2:
                L = 1.0
            else:
                L = max(
                    math.hypot(v1[0] - v2[0], v1[1] - v2[1])
                    for v1 in base_verts for v2 in base_verts
                )

            x, y, yaw = pose[0], pose[1], pose[5]
            dx = (L / 2.0) * math.cos(yaw)
            dy = (L / 2.0) * math.sin(yaw)
            x1, y1 = x - dx, y - dy
            x2, y2 = x + dx, y + dy
            self.reference_lines[link] = (x1, y1, x2, y2)

    def handle_static_map(self, req):
        return GetMapResponse(self.map_msg)
    
    def info_cb(self, info: CameraInfo):
        self.cam_model.fromCameraInfo(info)
        self.camera_info_received = True
        self.camera_frame = info.header.frame_id

    def scan_callback(self, scan):
        try:
            t = self.tf_listener.getLatestCommonTime("map", scan.header.frame_id)
            (trans, rot) = self.tf_listener.lookupTransform("map", scan.header.frame_id, t)
            robot_x, robot_y = trans[0], trans[1]
            _, _, robot_theta = tf.transformations.euler_from_quaternion(rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn(" No se pudo obtener la transformada map → base_link")
            return

        origin_x = self.origin[0]
        origin_y = self.origin[1]
        updated = False

        current_hits = set()
        
        # Recolectar todos los impactos válidos del LIDAR
        for i, r in enumerate(scan.ranges):
            if np.isinf(r) or np.isnan(r) or r >= scan.range_max:
                continue

            angle = scan.angle_min + i * scan.angle_increment
            global_angle = angle + robot_theta
            x = robot_x + r * math.cos(global_angle)
            y = robot_y + r * math.sin(global_angle)
            mx = int((x - origin_x) / self.resolution)
            my = int((y - origin_y) / self.resolution)

            if 0 <= mx < self.width and 0 <= my < self.height:
                current_hits.add((my, mx))

        new_mask = np.zeros_like(self.occ_grid, dtype=np.uint8)

        # Liberar celdas que ya no están ocupadas
        for my, mx in zip(*np.where(self.movable_mask == 1)):
            if 0 <= mx < self.width and 0 <= my < self.height and self.occ_grid[my, mx] != 0:
                self.occ_grid[my, mx] = 0
                new_mask[my, mx] = 0
                updated = True

        # Marcar como ocupadas solo las zonas permitidas
        for my, mx in current_hits:
            if 0 <= mx < self.width and 0 <= my < self.height: 
                self.hit_counter[my, mx] += 1
                if self.hit_counter[my, mx] >= self.hit_threshold and self.occ_grid[my, mx] == 0:
                    self.occ_grid[my, mx] = 100
                    new_mask[my, mx] = 1
                    updated = True

        for my, mx in zip(*np.where(self.hit_counter > 0)):
            if (my, mx) not in current_hits:
                self.hit_counter[my, mx] -= 1

        # Guardar máscara para el siguiente frame
        self.movable_mask = new_mask

        if updated:
            self.map_msg.header.stamp = rospy.Time.now()
            self.map_msg.data = self.occ_grid.flatten().tolist()
            self.map_pub.publish(self.map_msg)


    def image_cb(self, img_msg):

        if not self.camera_info_received:
            return

        try:
            tf_cam = self.tf_buffer.lookup_transform(self.camera_frame, 'map', rospy.Time(0), rospy.Duration(1.0))
            
        except Exception as e:
            rospy.logwarn(f"[Visión] Fallo al obtener TF map→{self.camera_frame}: {e}")
            return

        cv_img = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')
        h_img, w_img = cv_img.shape[:2]
    
        door = self.current_door
        local_corners = self.link_geometries.get(door, [])

        if door and local_corners and door in self.link_poses:

            dx = max(c[0] for c in local_corners) - min(c[0] for c in local_corners)
            dy = max(c[1] for c in local_corners) - min(c[1] for c in local_corners)
            dz = max(c[2] for c in local_corners) - min(c[2] for c in local_corners)

            pos_file, ori_file = self.link_poses_file[door]
            pos_cur, ori_cur = self.link_poses[door]

            q_file = (ori_file.x, ori_file.y, ori_file.z, ori_file.w)
            q_cur = (ori_cur.x, ori_cur.y, ori_cur.z, ori_cur.w)
            q_total = tf.transformations.quaternion_multiply(q_file, q_cur)

            pos_m= pos_cur
            R_door = tf.transformations.quaternion_matrix(q_total)[:3, :3]

            # Identificar eje de grosor
            dims = {'x': dx, 'y': dy, 'z': dz}
            thickness_axis = min(dims, key=dims.get)

            # Obtener extremos del volumen
            min_x = min(c[0] for c in local_corners)
            max_x = max(c[0] for c in local_corners)
            min_y = min(c[1] for c in local_corners)
            max_y = max(c[1] for c in local_corners)
            min_z = min(c[2] for c in local_corners)
            max_z = max(c[2] for c in local_corners)

            # Definir cara frontal perpendicular al grosor
            if thickness_axis == 'x':
                x_c = (min_x + max_x) / 2.0
                roi_corners = [
                    (min_y, x_c, min_z), (max_y, x_c, min_z),
                    (max_y, x_c, max_z), (min_y, x_c, max_z)
                ]
            elif thickness_axis == 'y':
                y_c = (min_y + max_y) / 2.0
                roi_corners = [
                    (min_x, y_c, min_z), (max_x, y_c, min_z),
                    (max_x, y_c, max_z), (min_x, y_c, max_z)
                ]
            else:  # thickness_axis == 'z'
                z_c = (min_z + max_z) / 2.0
                roi_corners = [
                    (min_x, z_c, min_y), (max_x,  z_c ,min_y),
                    (max_x, z_c, max_y), (min_x, z_c, max_y)
                ]
            
            us3d, vs3d = [], []
            for i, (lx, ly, lz) in enumerate(roi_corners):
                # SE COMPENSA LA COTA DEL LINK CON LA DEL ROBOT
                z_robot = rospy.get_param("~z_pos", 0.0)
                abs_pt = R_door.dot([lx, ly, lz]) + np.array([pos_m.x, pos_m.y, pos_m.z-z_robot])

                ps = PointStamped()
                ps.header.frame_id = 'map'  
                ps.header.stamp    = img_msg.header.stamp
                ps.point.x, ps.point.y, ps.point.z = abs_pt.tolist()

                try:
                    # Proyectar y recortar
                    pc = do_transform_point(ps, tf_cam)
                    u, v = self.cam_model.project3dToPixel((pc.point.x, pc.point.y, pc.point.z))
                    u_i = int(np.clip(u, 0, w_img-1))
                    v_i = int(np.clip(v, 0, h_img-1))
                    us3d.append(u_i)
                    vs3d.append(v_i)
 
                except Exception as e:
                    rospy.logwarn(f"[Visión DEBUG] fallo proj corner {i}: {e}")


            if us3d and vs3d:
                umin, umax = min(us3d), max(us3d)
                vmin, vmax = min(vs3d), max(vs3d)
                pad_u = pad_v = 5
                umin, umax = max(0, min(us3d) - pad_u), min(w_img, max(us3d) + pad_u)
                vmin, vmax = max(0, min(vs3d) - pad_v), min(h_img, max(vs3d) + pad_v)
            else:
                return

        else:
            return

        # Dibujar ROI y etiqueta
        cv2.rectangle(cv_img, (umin, vmin), (umax, vmax), (255, 0, 0), 2)

        # Publicar imagen anotada
        ann = self.bridge.cv2_to_imgmsg(cv_img, 'bgr8')
        ann.header = img_msg.header
        self.annotated_pub.publish(ann)


    def line_callback(self, msg):
        try:
            t = self.tf_listener.getLatestCommonTime("map", "base_link")
            (trans, rot) = self.tf_listener.lookupTransform("map", "base_link", t)
            robot_x, robot_y = trans[0], trans[1]
            _, _, robot_theta = tf.transformations.euler_from_quaternion(rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn(" No se pudo obtener la transformada map → base_link")
            return
        
        for i, line in enumerate(msg.line_segments):
            start_x = line.start[0] * math.cos(robot_theta) - line.start[1] * math.sin(robot_theta) + robot_x
            start_y = line.start[0] * math.sin(robot_theta) + line.start[1] * math.cos(robot_theta) + robot_y

            end_x = line.end[0] * math.cos(robot_theta) - line.end[1] * math.sin(robot_theta) + robot_x
            end_y = line.end[0] * math.sin(robot_theta) + line.end[1] * math.cos(robot_theta) + robot_y

            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "line_segments"
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.03
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.lifetime = rospy.Duration(0.5)
            marker.points = [
                Point(x=start_x, y=start_y, z=0),
                Point(x=end_x, y=end_y, z=0)
            ]
            self.marker_pub.publish(marker)

        lines_in_map = []
        for line in msg.line_segments:
            sx = line.start[0] * math.cos(robot_theta) - line.start[1] * math.sin(robot_theta) + robot_x
            sy = line.start[0] * math.sin(robot_theta) + line.start[1] * math.cos(robot_theta) + robot_y
            ex = line.end[0] * math.cos(robot_theta) - line.end[1] * math.sin(robot_theta) + robot_x
            ey = line.end[0] * math.sin(robot_theta) + line.end[1] * math.cos(robot_theta) + robot_y
            lines_in_map.append((sx, sy, ex, ey))

        self.update_angles_from_lidar(lines_in_map)

    def is_window(self, link_name):
        return "ventana" in link_name.lower()

    def is_door(self, link_name):
        return "puerta" in link_name.lower()

    def update_angles_from_lidar(self, lines_in_map):
        def line_angle(x1, y1, x2, y2):
            return math.atan2(y2 - y1, x2 - x1)

        def midpoint(x1, y1, x2, y2):
            return (x1 + x2) / 2, (y1 + y2) / 2

        for link, ref_line in self.reference_lines.items():
            if "puerta" not in link.lower():
                continue  # Ignorar todo lo que no sea puerta

            rx1, ry1, rx2, ry2 = ref_line
            ref_angle = line_angle(rx1, ry1, rx2, ry2)
            ref_mid = midpoint(rx1, ry1, rx2, ry2)

            best_match = None
            min_dist = float("inf")

            for lx1, ly1, lx2, ly2 in lines_in_map:
                cur_mid = midpoint(lx1, ly1, lx2, ly2)
                dist = math.hypot(cur_mid[0] - ref_mid[0], cur_mid[1] - ref_mid[1])
                if dist < min_dist and dist < 1.0:  # Umbral de 1 m
                    best_match = (lx1, ly1, lx2, ly2)
                    min_dist = dist

            if best_match:
                angle_now = line_angle(*best_match)
               
                delta = math.atan2(math.sin(angle_now - ref_angle), math.cos(angle_now - ref_angle))  # Escalar a [-π, π]
                if delta > math.pi / 2:
                    delta -= math.pi
                elif delta < -math.pi / 2:
                    delta += math.pi


                if link not in self.line_buffer:
                    self.line_buffer[link] = []
                self.line_buffer[link].append(delta)
                if len(self.line_buffer[link]) > self.buffer_size:
                    self.line_buffer[link].pop(0)

                mean_angle = sum(self.line_buffer[link]) / len(self.line_buffer[link])
                self.estimated_angles[link] = mean_angle

                rospy.loginfo(f"[{link}] Ángulo estimado: {math.degrees(mean_angle):.2f}°")
                text = f"{math.degrees(mean_angle):.1f}°"

                # Visualización
                text_marker = Marker()
                text_marker.header.frame_id = "map"
                text_marker.header.stamp = rospy.Time.now()
                text_marker.ns = "door_angles"
                text_marker.id = hash(link) % 10000
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.action = Marker.ADD
                text_marker.scale.z = 0.3
                text_marker.color.r = 0.0
                text_marker.color.g = 1.0
                text_marker.color.b = 0.0
                text_marker.color.a = 1.0
                text_marker.text = text
                text_marker.pose.position.x = ref_mid[0]
                text_marker.pose.position.y = ref_mid[1]
                text_marker.pose.position.z = 1.8
                text_marker.pose.orientation.w = 1.0

                self.text_marker_pub.publish(text_marker)

        self.publish_estimates()

    def geometry_displacement_cb(self, event=None):
        for link in self.reference_lines:
            if not self.is_window(link): 
                continue

            if link not in self.link_poses or link not in self.link_poses_file:
                rospy.logwarn(f"[GeomDisp] {link} sin poses válidas.")
                continue

            # Obtener pose inicial (del JSON)
            pos_ini, ori_ini = self.link_poses_file[link]
            yaw_ini = euler_from_quaternion([ori_ini.x, ori_ini.y, ori_ini.z, ori_ini.w])[2]
            slide_x = math.cos(yaw_ini)
            slide_y = math.sin(yaw_ini)

            # Pose actual
            pos_cur, ori_cur = self.link_poses[link]
            q_file = [ori_ini.x, ori_ini.y, ori_ini.z, ori_ini.w]
            q_cur = [ori_cur.x, ori_cur.y, ori_cur.z, ori_cur.w]
            q_total = tf.transformations.quaternion_multiply(q_file, q_cur)
            R_link = tf.transformations.quaternion_matrix(q_total)[:3, :3]

            # Geometría local
            verts = self.link_geometries.get(link, [])
            if not verts:
                continue

            # Convertir a puntos globales
            z_robot = rospy.get_param("~z_pos", 0.0)
            global_pts = [R_link.dot(v) + np.array([pos_cur.x, pos_cur.y, pos_cur.z - z_robot]) for v in verts]
            center_now = np.mean(global_pts, axis=0)

            if not hasattr(self, 'initial_panel_center'):
                self.initial_panel_center = {}
            if link not in self.initial_panel_center:
                self.initial_panel_center[link] = center_now
                continue  

            # Calcular desplazamiento
            dx = center_now[0] - self.initial_panel_center[link][0]
            dy = center_now[1] - self.initial_panel_center[link][1]
            displacement = dx * slide_x + dy * slide_y

            # Guardar y publicar
            if not hasattr(self, 'geometry_displacement'):
                self.geometry_displacement = {}
            self.geometry_displacement[link] = displacement

            rospy.loginfo(f"[GeomDisp] {link}: desplazamiento = {displacement:.3f} m")

            # Publicar marcador RViz
            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.header.stamp = rospy.Time.now()
            text_marker.ns = "geometry_displacement"
            text_marker.id = hash(link) % 10000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.scale.z = 0.3
            text_marker.color.r = 1.0
            text_marker.color.g = 0.6
            text_marker.color.b = 0.0
            text_marker.color.a = 1.0
            text_marker.text = f"{displacement:.3f} m"
            text_marker.lifetime = rospy.Duration(0.5)
            text_marker.pose.position.x = center_now[0]
            text_marker.pose.position.y = center_now[1]
            text_marker.pose.position.z = 1.8
            text_marker.pose.orientation.w = 1.0

            self.text_marker_pub.publish(text_marker)

        self.publish_estimates()

    def publish_estimates(self):   #Publicar todos los valores estimados (ángulos y desplazamientos)
        msg = Float32MultiArray()
        data = []
        for link in self.reference_lines.keys():
            if "puerta" in link.lower():
                val = self.estimated_angles.get(link, 0.0)

            elif self.is_window(link):
                val = getattr(self, "geometry_displacement", {}).get(link, 0.0)

            else:
                val = 0.0
            data.append(val)

        msg.data = data
        self.estimated_array_pub.publish(msg)


    def link_states_cb(self, msg: LinkStates):

        if not self.child_link_names:
            for name in msg.name:
                if name.endswith('_link') and not name.endswith('pared_link'):
                    self.child_link_names.append(name)

            if not self.child_link_names:
                return
            
            else:
                world_path = rospy.get_param("/world_file", "")
                if world_path:
                    base_name = os.path.splitext(os.path.basename(world_path))[0]
                    grids_dir = os.path.join(rospkg.RosPack().get_path('bim2ros'), "grids")
                    index_json_path = os.path.join(grids_dir, f"{base_name}_movable_indices.json")

                    # Diccionario link → índice
                    link_index_dict = {link.split("::")[-1]: i for i, link in enumerate(self.child_link_names)}

                    # Guardar JSON
                    try:
                        os.makedirs(grids_dir, exist_ok=True)
                        with open(index_json_path, "w") as f:
                            json.dump(link_index_dict, f, indent=2)
                        rospy.loginfo(f" JSON de índices generado: {index_json_path}")
                    except Exception as e:
                        rospy.logwarn(f" No se pudo guardar el JSON de índices: {e}")

                else:
                    rospy.logwarn(" No se recibió el parámetro /world_file, no se puede guardar el JSON de índices.")

        values, vels = [], []

        for link in self.child_link_names:
            try:
                idx = msg.name.index(link)
            except ValueError:
                values.append(0.0)
                continue

            pos = msg.pose[idx].position
            quat = msg.pose[idx].orientation
            self.link_poses[link] = (pos, quat)

            name_base = link.split("::")[-1]

            if name_base.startswith("ventana"):
                if link not in self.initial_link_pos:
                    self.initial_link_pos[link] = (pos.x, pos.y)
                x0, y0 = self.initial_link_pos[link]
                dist = math.hypot(pos.x - x0, pos.y - y0)
                val = float(np.clip(dist, 0.0, 1.5))
            else:
                if link not in self.initial_real_yaw:
                    _, _, yaw0 = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
                    self.initial_real_yaw[link] = yaw0
                    self.initial_link_pos[link] = (pos.x, pos.y)
                _, _, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
                d = yaw - self.initial_real_yaw[link]
                val = float(np.clip(math.atan2(math.sin(d), math.cos(d)), -math.pi/2, math.pi/2))

            now = rospy.Time.now().to_sec()
            if link in self.prev_vals:
                dt = now - self.prev_times[link]
                dv = val - self.prev_vals[link]
                vel = dv / dt if dt > 0 else 0.0
            else:
                vel = 0.0

            self.prev_vals[link] = val
            self.prev_times[link] = now

            values.append(val)
            vels.append(vel)

        self.real_array_pub.publish(Float32MultiArray(data=values))
        self.real_vel_array_pub.publish(Float32MultiArray(data=vels))

        if vels:
            self.link_vels = dict(zip(self.child_link_names, vels))

            selected = None
            for link, vel in self.link_vels.items():
                name = link.split("::")[-1]  
                thresh = self.lin_vel_thresh if name.startswith("ventana") else self.ang_vel_thresh

                if abs(vel) > thresh:
                    if selected is None or abs(vel) > abs(self.link_vels[selected]):
                        selected = link

            if selected:
                self.current_door = selected
            else:
                self.current_door = None

if __name__ == "__main__":
    try:
        DynamicMapUpdater()
    except rospy.ROSInterruptException:
        pass
