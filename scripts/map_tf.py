#!/usr/bin/env python3

import os
import yaml
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
import tf2_ros as tf
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped

# valores

#str(robot['x_pos']),str(robot['y_pos']),'0', '0', '0', '0', 
#            '/map', robot['name'] + '/map'


rosRate = 100

class maptf(Node):
    def __init__(self):
        super().__init__('map_tf')

        self.tf_buffer = tf.Buffer()
        self.tf_listener = tf.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf.TransformBroadcaster(self)

        self.timer = self.create_timer(1/rosRate, self.tf_callback)

        self.robot_names = []
        self.robot_positions = []

        with open(os.path.join(get_package_share_directory('husky_navigation'), 'params', 'multirobot_names.yaml'), 'r') as file:
            names = yaml.safe_load(file)
            for i in range(len(names['names'])):
                self.robot_names.append(names['names'].get('robot'+str(i+1)))
                self.robot_positions.append(names['position'].get('robot'+str(i+1)))

        self.robots = [{} for i in range(len(self.robot_names))]
        for i in range(len(self.robot_names)):
            self.robots[i]['name'] =  self.robot_names[i]
            self.robots[i]['x_pos'] = self.robot_positions[i].get('x')
            self.robots[i]['y_pos'] = self.robot_positions[i].get('y')
            self.robots[i]['z_pos'] = self.robot_positions[i].get('z')

    def tf_callback(self):
        for n in range(1, len(self.robot_names)+1):
            try:
                #self.tf_buffer.lookup_transform('map', self.robot_names[n] + '/map', rclpy.time.Time())
                trans = self.tf_buffer.lookup_transform(f'robot{n}/map', f'robot{n}/base_link', rclpy.time.Time())

                t=TransformStamped()
                t.header.stamp = trans.header.stamp
                t.header.frame_id = 'map'
                t.child_frame_id = self.robots[n-1]['name'] + '/map'
                t.transform.translation.x = self.robots[n-1]['x_pos']
                t.transform.translation.y = self.robots[n-1]['y_pos']
                t.transform.translation.z = -trans.transform.translation.z
                t.transform.rotation.x = 0.0
                t.transform.rotation.y = 0.0
                t.transform.rotation.z = 0.0
                t.transform.rotation.w = 1.0
                self.tf_broadcaster.sendTransform(t)

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass

def main(args=None):
    rclpy.init(args=args)
    maptf1 = maptf()
    print("Maptf node started")
    try:
        rclpy.spin(maptf1)
    except KeyboardInterrupt:
        print("Terminating Maptf Node..")
        maptf1.destroy_node()

if __name__ == '__main__':
    main()