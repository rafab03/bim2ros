#!/usr/bin/env python3
import sys
import math
import rospy
from std_msgs.msg import Float32MultiArray
from gazebo_msgs.srv import ApplyJointEffort, JointRequest

def usage():
    print("Usage: rosrun <your_package> control_joint.py <joint_name> <index> <target_value>")
    print("  - For revolute joints (doors), target_value is angle in degrees")
    print("  - For prismatic joints (windows), target_value is displacement in meters")

# Globals
latest_val = None
prev_error = None
prev_time = None
idx = None
is_prismatic = False

def feedback_cb(msg: Float32MultiArray):
    global latest_val
    if idx is not None and 0 <= idx < len(msg.data):
        latest_val = msg.data[idx]


def main():
    global idx, is_prismatic, prev_error, prev_time, latest_val
    if len(sys.argv) != 4:
        usage()
        sys.exit(1)
    joint_name = sys.argv[1]
    try:
        idx = int(sys.argv[2])
    except ValueError:
        rospy.logerr("Index must be an integer")
        sys.exit(1)
    try:
        target_input = float(sys.argv[3])
    except ValueError:
        rospy.logerr("Target value must be a number")
        sys.exit(1)

    # Determine joint type
    is_prismatic = joint_name.startswith('prism_joint')
    if is_prismatic:
        target = target_input
    else:
        target = math.radians(target_input)

    rospy.init_node('control_joint', anonymous=False)
    if is_prismatic:
        rospy.loginfo(f"[control_joint] Moving prismatic '{joint_name}' (index {idx}) to {target_input:.3f} m")
    else:
        rospy.loginfo(f"[control_joint] Moving revolute '{joint_name}' (index {idx}) to {target_input:.2f}° ({target:.2f} rad)")

    # Subscribe for feedback
    rospy.Subscriber('/movable_objects/door_angles_real', Float32MultiArray, feedback_cb)
    # Wait for first reading
    rospy.loginfo("[control_joint] Waiting for first feedback...")
    while not rospy.is_shutdown() and latest_val is None:
        rospy.sleep(0.1)
    if rospy.is_shutdown(): sys.exit(0)
    if is_prismatic:
        rospy.loginfo(f"Initial pos = {latest_val:.3f} m")
    else:
        rospy.loginfo(f"Initial angle = {math.degrees(latest_val):.2f}°")

    # Services
    rospy.wait_for_service('/gazebo/apply_joint_effort')
    apply_effort = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort)
    rospy.wait_for_service('/gazebo/clear_joint_forces')
    clear_effort = rospy.ServiceProxy('/gazebo/clear_joint_forces', JointRequest)

    # Control params
    kp = rospy.get_param('~kp', 10.0)
    kd = rospy.get_param('~kd', 2.0)
    if is_prismatic:
        kp = 25.0
        kd = 6.0
        tol = 0.05           # meters
        tol_vel = 0.005        # m/s
    else:
        kp = 15.0
        kd = 4.0
        tol_deg = 0.6
        tol = math.radians(tol_deg)
        tol_vel_deg = 0.1
        tol_vel = math.radians(tol_vel_deg)
    step = rospy.get_param('~step_s', 0.05)
    rate = rospy.Rate(1.0/step)
    rospy.loginfo(f"kp={kp}, kd={kd}, tol={tol}, tol_vel={tol_vel}, dt={step}")

    prev_error = target - latest_val
    prev_time = rospy.Time.now()

    try:
        while not rospy.is_shutdown():
            if latest_val is None:
                rate.sleep(); continue
            now = rospy.Time.now()
            error = target - latest_val
            raw_dt = (now - prev_time).to_sec()
            dt = raw_dt if raw_dt>0 else step
            derr = (error - prev_error)/dt

            # Log
            if is_prismatic:
                rospy.loginfo(f"Err={error:.3f} m, Vel={derr:.3f} m/s")
            else:
                rospy.loginfo(f"Err={math.degrees(error):.2f}°, Vel={math.degrees(derr):.2f}°/s")

            # PD effort
            effort = kp*error + kd*derr
            try:
                apply_effort(joint_name=joint_name,
                             effort=effort,
                             start_time=rospy.Time(0),
                             duration=rospy.Duration(step))
            except rospy.ServiceException as e:
                rospy.logwarn(f"apply_joint_effort failed: {e}")

            # Stop condition
            if abs(error)<tol and abs(derr)<tol_vel:
                msg = "position" if is_prismatic else "angle"
                rospy.loginfo(f"[control_joint] Target {msg} reached within tolerance")
                break

            prev_error = error
            prev_time = now
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

    # Clear efforts
    try:
        clear_effort(joint_name=joint_name)
    except rospy.ServiceException as e:
        rospy.logwarn(f"clear_joint_forces failed: {e}")
    rospy.loginfo("[control_joint] Done.")

if __name__=='__main__':
    main()
