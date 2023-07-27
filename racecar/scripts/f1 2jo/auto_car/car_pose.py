import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from geometry_msgs.msg import Point, Quaternion
import math

def publish_pose():
    rospy.init_node("pose_publisher")
    pub = rospy.Publisher("/ugv_pose", Marker, queue_size=10)
    h_pub = rospy.Publisher("/heading", Marker, queue_size=10)

    # create the Marker message
    marker_msg = Marker()
    marker_msg.header.frame_id = "map"
    marker_msg.type = Marker.ARROW
    marker_msg.action = Marker.ADD

    # Assuming we have these variables for UGV's pose
    x, y, z = 1.0, 2.0, 0.0  # position
    yaw = math.radians(45)  # orientation in radians

    # set the position
    marker_msg.pose.position = Point(x, y, z)

    # convert yaw to quaternion for the orientation
    qx = 0
    qy = 0
    qz = math.sin(yaw / 2)
    qw = math.cos(yaw / 2)
    marker_msg.pose.orientation = Quaternion(qx, qy, qz, qw)

    # set the scale of the arrow
    marker_msg.scale.x = 1  # length of the arrow
    marker_msg.scale.y = 0.1  # width of the arrow
    marker_msg.scale.z = 0.1  # height of the arrow

    # set the color of the arrow
    marker_msg.color.r = 0.0
    marker_msg.color.g = 1.0
    marker_msg.color.b = 0.0
    marker_msg.color.a = 1.0  # alpha (should be non-zero)

    heading_msg = Marker()
    heading_msg.header.frame_id = "map"
    heading_msg.type = Marker.POINTS
    heading_msg.ns = "point"
    heading_msg.id = 0

    heading_msg.scale.x = 0.3  # specifies the radius of the points
    heading_msg.scale.y = 0.3

    heading_msg.color.r = 1.0
    heading_msg.color.g = 0.
    heading_msg.color.b = 0.
    heading_msg.color.a = 1.0

    p = Point()
    p.x, p.y, p.z = [1,2,3]
    heading_msg.points.append(p)

    rate = rospy.Rate(10)  # publish at 1 Hz

    while not rospy.is_shutdown():
        # update the timestamp
        marker_msg.header.stamp = rospy.Time.now()
        heading_msg.header.stamp = rospy.Time.now()

        # publish the message
        pub.publish(marker_msg)
        h_pub.publish(heading_msg)

        rate.sleep()

if __name__ == "__main__":
    publish_pose()