import math
import rospy
from actionlib_msgs.msg import GoalStatusArray
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseActionFeedback
from geometry_msgs.msg import PoseStamped

class nav_goal:
    def __init__(self):
        self.pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        rospy.Subscriber("/move_base/status", GoalStatusArray, self.statuscallback, queue_size=10)
        self.goal = PoseStamped()
        self.goal.header.frame_id = 'map'
        self.curr_point = 0
        self.last_status = -1
        self.final_dest = 0
        self.robotState = 0
        self.time_to_publish = 1
        # posfile = open("pos.txt", "r")
        # str = posfile.read()
        # posfile.close()
        self.list = ["17.5 -2 1.57", "17.5 2 0", "23 2 4.72", "23 -1 3.14"]

    def initRospy(self):
        rospy.init_node('nav_goal_node', anonymous=True)
        print("nav_goal_node has started")

    def toQuaternion(self, yaw):
        return [math.sin(yaw * 0.5), math.cos(yaw * 0.5)]

    def gohome(self):
        print("Going Home !!!")
        self.goal.pose.position.x = 0
        self.goal.pose.position.y = 0
        self.goal.pose.position.z = 0
        self.goal.pose.orientation.x = 0
        self.goal.pose.orientation.y = 0
        self.goal.pose.orientation.z = 0
        self.goal.pose.orientation.w = 1
        self.pub_goal.publish(self.goal)

    def go_to_point(self):
        pt = self.list[self.curr_point].split(" ")
        print("Going to Point {0}: x={1} y={2}".format(self.curr_point, round(float(pt[0]), 2),
                                                      round(float(pt[1]), 2)))
        qt = self.toQuaternion(float(pt[2]))

        self.goal.pose.position.x = float(pt[0])
        self.goal.pose.position.y = float(pt[1])
        self.goal.pose.position.z = 0
        self.goal.pose.orientation.x = 0
        self.goal.pose.orientation.y = 0
        self.goal.pose.orientation.z = qt[0]
        self.goal.pose.orientation.w = qt[1]
        self.pub_goal.publish(self.goal)

    def statuscallback(self, data):
        if len(data.status_list) == 0:
            return
        if len(self.list) <= self.curr_point+1:
            print("\033[1;32;40mArrived at final destination\033[0m")
            self.final_dest = 1
            return

        ind = len(data.status_list) - 1
        if self.last_status != data.status_list[ind].status:
            self.last_status = data.status_list[ind].status

            if self.last_status == 1:  # command accepted by the robot
                print("\033[1;32;40mCommand#{0} received\033[0m".format(self.curr_point+1))
                self.robotState = 1

            if self.last_status == 3 and self.robotState == 1:  # robot arrived to target after "confirming" the command
                print("\033[1;32;40mArrived at point {0}\033[0m".format(self.curr_point+1))
                self.setNextTargetPoint()

            if self.last_status == 4 and self.robotState == 1:  # target inaccecible
                # print("Unable...\n")
                print("\033[1;31;40mCan't get to point {0}\033[0m".format(self.curr_point+1))
                self.setNextTargetPoint()

    def setNextTargetPoint(self):
        self.robotState = 0
        self.time_to_publish = 1
        self.curr_point = self.curr_point + 1

    def PublishNextPoint(self):
        self.time_to_publish = 10
        if self.curr_point < len(self.list)-1:
            self.go_to_point()
        # else:
        #     self.gohome() # return to origin location

    def stateMachine(self):
        if self.robotState == 0:  # wait some time before setting the target
            self.time_to_publish = self.time_to_publish - 1
            if self.time_to_publish <= 0:
                self.PublishNextPoint()

def main():
    nav = nav_goal()
    nav.initRospy()

    r = rospy.Rate(1)  # hz
    while not (rospy.is_shutdown() or nav.final_dest):
        nav.stateMachine()
        r.sleep()


if __name__ == '__main__':
    main()
