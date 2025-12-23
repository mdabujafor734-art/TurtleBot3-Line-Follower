#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

def publish_path():
    rospy.init_node('pentagon_path_publisher')
    pub = rospy.Publisher('/pentagon_reference_path', Path, queue_size=10, latch=True)
    
    rospy.sleep(1.0)
    
    vertices = [
        (0.000, 1.400),
        (1.331, 0.432),
        (0.823, -1.133),
        (-0.823, -1.133),
        (-1.331, 0.432),
        (0.000, 1.400)
    ]
    
    path = Path()
    path.header.frame_id = "odom"
    
    for i in range(len(vertices) - 1):
        x1, y1 = vertices[i]
        x2, y2 = vertices[i + 1]
        
        for j in range(20):
            t = j / 20.0
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            
            pose = PoseStamped()
            pose.header.frame_id = "odom"
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.002
            pose.pose.orientation.w = 1.0
            
            path.poses.append(pose)
    
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        path.header.stamp = rospy.Time.now()
        pub.publish(path)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_path()
    except rospy.ROSInterruptException:
        pass
