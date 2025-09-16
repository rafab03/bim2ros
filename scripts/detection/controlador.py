#!/usr/bin/env python3
import sys
import math
import rospy
import json
import os
import rospkg
from std_msgs.msg import Float32MultiArray
from gazebo_msgs.srv import ApplyJointEffort, JointRequest

# Globals
latest_val = None
prev_error = None
prev_time = None
idx = None
is_prismatic = False

LINK_INDEX_MAP = {}

def usage():
    print("Uso: rosrun <tu_paquete> controlador.py <link_name> <target_value>")
    print("  - link_name debe estar en el JSON de índices (ej: puerta2_link, ventana3_link)")
    print("  - target_value es ángulo (°) para puertas o desplazamiento (m) para ventanas")
    sys.exit(1)

def feedback_cb(msg: Float32MultiArray):
    global latest_val
    if idx is not None and 0 <= idx < len(msg.data):
        latest_val = msg.data[idx]

def load_filtered_indices(map_name=None):
    global LINK_INDEX_MAP
    rospack = rospkg.RosPack()
    config_dir = os.path.join(rospack.get_path('bim2ros'), 'grids')

    for fname in os.listdir(config_dir):
        if fname.endswith('_movable_indices.json'):
            if map_name and not fname.startswith(map_name):
                continue  
            full_path = os.path.join(config_dir, fname)
            with open(full_path, 'r') as f:
                raw_data = json.load(f)
                for link, index in raw_data.items():
                    if link.startswith("puerta") or link.startswith("ventana"):
                        LINK_INDEX_MAP[link] = index

def determine_joint_type(link_name):
    if link_name.startswith("ventana"):
        return "prismatic"
    elif link_name.startswith("puerta"):
        return "revolute"
    else:
        rospy.logerr(f"No se puede determinar el tipo de articulación para '{link_name}'")
        sys.exit(1)

def load_physical_params(map_name):
    rospack = rospkg.RosPack()
    config_dir = os.path.join(rospack.get_path('bim2ros'), 'grids')
    fname = f"{map_name}_movable_physics.json"
    fpath = os.path.join(config_dir, fname)
    if not os.path.exists(fpath):
        rospy.logwarn(f"No se encontró archivo de parámetros físicos: {fpath}")
        return None
    with open(fpath, 'r') as f:
        return json.load(f)

def main():
    global idx, is_prismatic, prev_error, prev_time, latest_val
    if len(sys.argv) < 3:
        usage()

    link_name = sys.argv[1]
    target_input = float(sys.argv[2])

    map_name = None
    if len(sys.argv) >= 4:
        map_name = sys.argv[3] 

    load_filtered_indices(map_name)
    physics_params = load_physical_params(map_name) or {}

    if link_name not in LINK_INDEX_MAP:
        rospy.logerr(f"'{link_name}' no está definido como puerta o ventana en el JSON.")
        sys.exit(1)

    idx = LINK_INDEX_MAP[link_name]
    joint_type = determine_joint_type(link_name)
    is_prismatic = (joint_type == "prismatic")
    base_name = link_name.replace("_link", "")
    joint_name = f"{'prism' if is_prismatic else 'rev'}_joint_{base_name}"
    target = target_input if is_prismatic else math.radians(target_input)

    rospy.init_node('control_joint', anonymous=False)
    rospy.Subscriber('/movable_objects/door_angles_real', Float32MultiArray, feedback_cb)

    rospy.loginfo("[control_joint] Esperando primer feedback...")
    while not rospy.is_shutdown() and latest_val is None:
        rospy.sleep(0.1)
    if rospy.is_shutdown():
        sys.exit(0)

    rospy.loginfo(f"[control_joint] Valor inicial = {latest_val:.3f} {'m' if is_prismatic else 'rad'}")

    rospy.wait_for_service('/gazebo/apply_joint_effort')
    apply_effort = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort)
    rospy.wait_for_service('/gazebo/clear_joint_forces')
    clear_effort = rospy.ServiceProxy('/gazebo/clear_joint_forces', JointRequest)

    # PID
    if is_prismatic:

        base_kp, base_kd, base_ki = 18.0, 9.0, 3.2
        ref_mass, ref_damping, ref_friction = 81.0, 16.2, 8.1
    else:

        base_kp, base_kd, base_ki = 5.0, 9.0, 0.3
        ref_mass, ref_damping, ref_friction = 132.74, 1.0129, 0.5065

    param_key = next((k for k in physics_params if k.endswith(link_name)), None)
    params = physics_params.get(param_key, {}) if param_key else {}

    mass = params.get("mass", ref_mass)
    damping = params.get("damping", ref_damping)
    friction = params.get("friction", ref_friction)

    kp = base_kp * (mass / ref_mass)
    kd = base_kd * (damping / ref_damping)
    ki = base_ki * (friction / ref_friction)
    integral_error = 0.0
    integral_limit = 10.0
    tol = 0.05 if is_prismatic else math.radians(0.5)
    tol_vel = 0.005 if is_prismatic else math.radians(0.1)
    step = rospy.get_param('~step_s', 0.05)
    rate = rospy.Rate(1.0 / step)

    prev_error = target - latest_val
    prev_time = rospy.Time.now()

    rospy.loginfo(f"[control_joint] Controlando '{joint_name}' → target: {target_input} ({'m' if is_prismatic else '°'})")
    rospy.loginfo(f"[control_joint] Parámetros PID: Kp={kp:.2f}, Kd={kd:.2f}, Ki={ki:.2f}")

    try:
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            error = target - latest_val
            dt = max((now - prev_time).to_sec(), step)
            derr = (error - prev_error) / dt

            integral_error += error * dt
            integral_error = max(min(integral_error, integral_limit), -integral_limit)

            effort = kp * error + kd * derr + ki * integral_error

            try:
                apply_effort(joint_name=joint_name,
                             effort=effort,
                             start_time=rospy.Time(0),
                             duration=rospy.Duration(step))
            except rospy.ServiceException as e:
                rospy.logwarn(f"apply_joint_effort falló: {e}")

            if is_prismatic:
                rospy.loginfo(f"Err={error:.3f} m, Vel={derr:.3f} m/s")
            else:
                rospy.loginfo(f"Err={math.degrees(error):.2f}°, Vel={math.degrees(derr):.2f}°/s")

            if abs(error) < tol and abs(derr) < tol_vel:
                rospy.loginfo("[control_joint] Objetivo alcanzado dentro de la tolerancia")
                break

            prev_error = error
            prev_time = now
            rate.sleep()

    except rospy.ROSInterruptException:
        pass

    try:
        clear_effort(joint_name=joint_name)
    except rospy.ServiceException as e:
        rospy.logwarn(f"clear_joint_forces falló: {e}")

    rospy.loginfo("[control_joint] Finalizado.")

if __name__ == '__main__':
    main()
