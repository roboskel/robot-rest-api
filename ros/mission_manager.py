#!/usr/bin/env python
import rospy
from roboskel_msgs.msg import Mission
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray

goal_pub = None
status_sub = None
mission_goals = []
mission_stays = []
active_goal = False
prevt = rospy.Time(0)

def sendNextGoal():
    if mission_goals:
        goal_pub.publish(mission_goals[0])

def missionCallback(mission):
    global mission_goals, mission_stays, status_sub
    if mission.poses and len(mission.poses) == len(mission.stay):
        print("Starting mission", mission.name)
        mission_goals = list(mission.poses[:])
        mission_stays = list(mission.stay[:])
        sendNextGoal()

def moveBaseStatus(msg):
    global mission_goals, mission_stays, active_goal, status_sub, prevt
    if msg.status_list and mission_goals:
        msg = msg.status_list[-1]
        currt = rospy.Time.now()
        if msg.status == 3 and currt - prevt >= rospy.Duration(3):
            print("Reached goal!")
            mission_goals.pop(0)
            print("Staying for", str(mission_stays[0]), "seconds...")
            rospy.sleep(mission_stays[0])
            mission_stays.pop(0)
            if not mission_goals:
                print("Mission complete!")
                #  status_sub.unregister()
            else:
                #  status_sub = rospy.Subscriber("/move_base/status", GoalStatusArray, moveBaseStatus)
                #  rospy.sleep(2)
                sendNextGoal()
                prevt = currt
        active_goal = msg.status != 3

def init():
    global goal_pub, status_sub
    rospy.init_node("mission_manager")

    goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)

    sub = rospy.Subscriber("/mission_manager/mission", Mission, missionCallback)

    status_sub = rospy.Subscriber("/move_base/status", GoalStatusArray, moveBaseStatus)

    rospy.spin()

init()
