#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Int8
import tf2_ros as tf
import yaml
import os
from ament_index_python.packages import get_package_share_directory


rosRate = 1

robot_separation = 1.5

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')



        self.create_subscription(Int8,"mission_start",self.callback_start,1)

        self.tf_buffer = tf.Buffer()
        self.tf_listener = tf.TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1/rosRate, self.timer_callback)


        self.state=0

        # Load map configuration
        with open(os.path.join(get_package_share_directory('cv_gdal'), 'maps', 'map1.yaml'), 'r') as file:
            map_config = yaml.safe_load(file)
            self.origin=[]
            self.destiny=[]
            self.origin.append(map_config['origin'])
            self.destiny.append(map_config['destiny'])

        # Load multirobot configuration
        with open(os.path.join(get_package_share_directory('husky_navigation'), 'params', 'multirobot_names.yaml'), 'r') as file:
            names = yaml.safe_load(file)
            self.robot_names = []
            self.robot_positions = []
            for i in range(len(names['names'])):
                self.robot_names.append(names['names'].get('robot'+str(i+1)))
                self.robot_positions.append(names['position'].get('robot'+str(i+1)))

        #self.get_logger().info(f'Robot names: {len(self.robot_names)}')

        for n in range(1, len(self.robot_names)+1):
            setattr(self, f'robot{n}_pub', self.create_publisher(PoseStamped, f'robot{n}/goal_pose', 10))


        # Calculate relative position of destiny with respect to origin
        self.goal_leader = [
            -self.destiny[0][i] + self.origin[0][i] for i in range(3)
        ]
        self.goal=None

        self.init = [{} for i in range(len(self.robot_names))]
        for i in range(len(self.robot_names)):
            self.init[i] = [self.robot_positions[i].get('x'), self.robot_positions[i].get('y'), self.robot_positions[i].get('z')]        
        
    def calculate_distance(self, point1, point2):
        return ((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2 ) ** 0.5

    
    def timer_callback(self):
        #function for followers
        for n in range(1, len(self.robot_names)):
            if self.state==1:
                try:
                    trans = self.tf_buffer.lookup_transform('map', f'robot{n}/base_link', rclpy.time.Time())

                    current_position = [
                        trans.transform.translation.x,
                        trans.transform.translation.y,
                        trans.transform.translation.z
                    ]
                    distance = self.calculate_distance(current_position, self.init[n-1])

                    if n == 1:
                        distance2 = self.calculate_distance(current_position, self.goal_leader)

                    if distance2>0.5:
                        if distance > robot_separation:
                            # Publish for follower robot
                            goal = PoseStamped()
                            goal.header.frame_id = 'map'
                            goal.header.stamp = self.get_clock().now().to_msg()
                            goal.pose.position.x = trans.transform.translation.x
                            goal.pose.position.y = trans.transform.translation.y
                            goal.pose.position.z = trans.transform.translation.z
                            goal.pose.orientation.x = trans.transform.rotation.x
                            goal.pose.orientation.y = trans.transform.rotation.y
                            goal.pose.orientation.z = trans.transform.rotation.z
                            goal.pose.orientation.w = trans.transform.rotation.w

                            # Fill twist message based on transform
                            getattr(self, f'robot{n+1}_pub').publish(goal)

                            self.init[n-1]=current_position
                    else:
                        goal = PoseStamped()
                        goal.header.frame_id = 'map'
                        goal.header.stamp = self.get_clock().now().to_msg()
                        goal.pose.position.x = self.goal_leader[0]+self.robot_positions[n].get('x')
                        goal.pose.position.y = self.goal_leader[1]+self.robot_positions[n].get('y')
                        goal.pose.position.z = self.goal_leader[2]+self.robot_positions[n].get('z')
                        goal.pose.orientation.x = 0.0
                        goal.pose.orientation.y = 0.0
                        goal.pose.orientation.z = 0.0
                        goal.pose.orientation.w = 1.0

                        getattr(self, f'robot{n+1}_pub').publish(goal)

                        if n==(len(self.robot_names)-1):
                            distance_end=self.calculate_distance(current_position, [self.goal_leader[0]+self.robot_positions[n-1].get('x'),
                                                                                    self.goal_leader[1]+self.robot_positions[n-1].get('y'),
                                                                                    self.goal_leader[2]+self.robot_positions[n-1].get('z') ])
                            self.get_logger().info(f'Distance to end: {distance_end}')
                            if distance_end<0.5:
                                self.get_logger().info("Mission completed")
                                self.state=0       

                except tf.LookupException:
                    self.get_logger().info(f'Transform not available for robot{n}')
            else:
                self.get_logger().info("Mission not started")
                break
    
    def callback_start(self,msg):
        if msg.data==1:
            self.get_logger().info("Starting mission")
            self.state=1

            # Publish for leader robot
            goal=PoseStamped()
            goal.header.frame_id = 'map'
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x = self.goal_leader[0]
            goal.pose.position.y = self.goal_leader[1]
            goal.pose.position.z = self.goal_leader[2]
            goal.pose.orientation.x = 0.0
            goal.pose.orientation.y = 0.0
            goal.pose.orientation.z = 0.0
            goal.pose.orientation.w = 1.0
            self.robot1_pub.publish(goal)
        else:
            self.get_logger().info("Mission not started")
            self.state=0

def main(args=None):
    rclpy.init(args=args)
    pathfollower1 = PathFollower()
    print("Path follower node started")
    try:
        rclpy.spin(pathfollower1)
    except KeyboardInterrupt:
        print("Terminating IK Node..")
        pathfollower1.destroy_node()

if __name__ == '__main__':
    main()