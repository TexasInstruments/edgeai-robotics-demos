import rospy
import math

import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
from angles import shortest_angular_distance

class WaypointsNavigation():

    def __init__(self):

        rospy.init_node('waypoints_navigation')

        # Get params
        self.num_waypoints      = rospy.get_param('/num_waypoints', 4)
        self.xy_goal_tolerance  = rospy.get_param('/xy_goal_tolerance', 0.1)
        self.yaw_goal_tolerance = rospy.get_param('/yaw_goal_tolerance', 0.05)

        waypoints_xyz = rospy.get_param('/waypoints_xyz')
        waypoints_yaw = rospy.get_param('/waypoints_yaw')

        quaternion_seq = list()
        self.pose_seq = list()
        self.goal_cnt = 0
        for yawangle in waypoints_yaw:
            quaternion_seq.append(Quaternion(*(quaternion_from_euler(0, 0, yawangle*math.pi/180, axes='sxyz'))))

        n = 3
        points = [waypoints_xyz[i:i+n] for i in range(0, len(waypoints_xyz), n)]
        for point in points:
            self.pose_seq.append(Pose(Point(*point),quaternion_seq[n-3]))
            n += 1

        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        wait = self.client.wait_for_server(rospy.Duration(5.0))
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return

        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting goals achievements ...")

        # init setPrevPose to 0
        self.setPrevPose = 0

        self.goal = MoveBaseGoal()
        self.movebase_client()

    def feedback_cb(self, feedback):

        if self.setPrevPose == 0:
            distance = (feedback.base_position.pose.position.x - self.goal.target_pose.pose.position.x)**2
            distance += (feedback.base_position.pose.position.y - self.goal.target_pose.pose.position.y)**2
            distance = math.sqrt(distance)

            if (distance > self.xy_goal_tolerance):
                return

            goal_yaw = euler_from_quaternion([self.goal.target_pose.pose.orientation.x, \
                                              self.goal.target_pose.pose.orientation.y, \
                                              self.goal.target_pose.pose.orientation.z, \
                                              self.goal.target_pose.pose.orientation.w])[2]

            base_yaw = euler_from_quaternion([feedback.base_position.pose.orientation.x, \
                                              feedback.base_position.pose.orientation.y, \
                                              feedback.base_position.pose.orientation.z, \
                                              feedback.base_position.pose.orientation.w])[2]

            ang_distance = abs(shortest_angular_distance(goal_yaw, base_yaw))

            if (ang_distance > self.yaw_goal_tolerance):
                return

            # wait until there is no change in pose and yaw
            if (self.setPrevPose == 0):
                self.setPrevPose = 1
                # save previous pose
                self.prevPose = feedback.base_position.pose
                return
        else:

            # see if robot stopped completely
            distance = (feedback.base_position.pose.position.x - self.prevPose.position.x)**2
            distance += (feedback.base_position.pose.position.y - self.prevPose.position.y)**2
            distance = math.sqrt(distance)

            if (distance > 0.05):
                self.prevPose = feedback.base_position.pose
                return

            prev_yaw = euler_from_quaternion([self.prevPose.orientation.x, \
                                              self.prevPose.orientation.y, \
                                              self.prevPose.orientation.z, \
                                              self.prevPose.orientation.w])[2]

            base_yaw = euler_from_quaternion([feedback.base_position.pose.orientation.x, \
                                              feedback.base_position.pose.orientation.y, \
                                              feedback.base_position.pose.orientation.z, \
                                              feedback.base_position.pose.orientation.w])[2]

            ang_distance = abs(shortest_angular_distance(prev_yaw, base_yaw))

            if (ang_distance > 0.01):
                self.prevPose = feedback.base_position.pose
                return

            self.goal_cnt += 1
            if self.goal_cnt == self.num_waypoints:
                self.goal_cnt = 0

            self.goal.target_pose.header.frame_id = "map"
            self.goal.target_pose.header.stamp = rospy.Time.now()
            self.goal.target_pose.pose = self.pose_seq[self.goal_cnt]
            rospy.loginfo("Sending goal pose "+str(self.goal_cnt)+" to Action Server")
            rospy.loginfo(str(self.pose_seq[self.goal_cnt]))

            # Sleep 2 secs
            rospy.sleep(2.)
            self.client.send_goal(self.goal, feedback_cb=self.feedback_cb)

            self.setPrevPose = 0


    def movebase_client(self):
        
        self.goal.target_pose.header.frame_id = "map"
        self.goal.target_pose.header.stamp = rospy.Time.now() 
        self.goal.target_pose.pose = self.pose_seq[self.goal_cnt]

        rospy.loginfo("Sending goal pose "+str(self.goal_cnt)+" to Action Server")
        rospy.loginfo(str(self.pose_seq[self.goal_cnt]))

        # Send a goal
        self.client.send_goal(self.goal, feedback_cb=self.feedback_cb)

        rospy.spin()

if __name__ == '__main__':
    try:
        WaypointsNavigation()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")