import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy as np

def points_marker(points, frame_id="map", ns="points", id=0, color=(1,1,1)):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.type = marker.POINTS
    marker.action = marker.ADD
    marker.ns = ns
    marker.id = id

    marker.scale.x = 0.2  # specifies the radius of the points
    marker.scale.y = 0.2

    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = 1.0

    marker.points = []
    for point in points:
        p = Point()
        p.x, p.y, p.z = point
        marker.points.append(p)

    return marker

def main():
    rospy.init_node("points_publisher")
    pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)

    ref = np.genfromtxt('./map_data_2-centerline.csv', delimiter=',', skip_header = 1)

    ref[:,1] = -ref[:,1]

    ref[:,0] = ref[:,0] - 21.916528*2
    ref[:,1] = ref[:,1] + 10.05*2

    points = [(x, y, 0) for x, y in zip(ref[:,0], ref[:,1])]  # assuming points are on the ground plane

    marker = points_marker(points)

    rate = rospy.Rate(1)  # publish at 1 Hz
    while not rospy.is_shutdown():
        pub.publish(marker)
        rate.sleep()

if __name__ == "__main__":
    main()