import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

def publish_initial_pose():
    rospy.init_node("initial_pose_pub", anonymous=False)

    # Parámetros del launch
    x = rospy.get_param("~x", 0.0)
    y = rospy.get_param("~y", 0.0)
    z = rospy.get_param("~z", 0.0)


    pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1, latch=True)

    rospy.loginfo("Esperando a que AMCL esté activo...")
    rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped)
    rospy.sleep(1)  

    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = "map"
    msg.header.stamp = rospy.Time.now()
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.position.z = z
    msg.pose.pose.orientation.w = 1.0

    # Covarianza mínima para inicializar (x, y, yaw)
    msg.pose.covariance[0] = 0.01  # x
    msg.pose.covariance[7] = 0.01  # y
    msg.pose.covariance[35] = 0.02  # yaw

    pub.publish(msg)
    rospy.loginfo("Initial pose published to /initialpose")

if __name__ == "__main__":
    publish_initial_pose()