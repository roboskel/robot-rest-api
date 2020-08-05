#!/usr/bin/env python
import rospy
from roboskel_msgs.msg import Mission
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatus

goal_pub = None
status_sub = None
mission_goals = []
mission_stays = []
active_goal = False

def sendNextGoal():
    if mission_goals:
        goal_pub.publish(mission_goals[0])

def missionCallback(mission):
    global mission_goals, mission_stays, status_sub
    if mission_goals and len(mission_goals) == len(mission_stays):
        print("Starting mission", mission.name)
        mission_goals = mission.poses[:]
        mission_stays= mission.stay[:]
        status_sub = rospy.Subscriber("/move_base/status", GoalStatus, moveBaseStatus)
        sendNextGoal()

def moveBaseStatus(msg):
    global mission_goals, mission_stays
    if msg.status == 3 and active_goal:
        print("Reached goal!")
        mission_goals.pop(0)
        print("Staying for", str(mission_stays[0]), "seconds...")
        rospy.sleep(mission_stays[0])
        mission_stays.pop(0)
        if not mission_goals:
            print("Mission complete!")
            status_sub.unregister()
        else:
            sendNextGoal()
    active_goal = msg.status == 1

def init():
    global goal_pub 
    rospy.init_node("mission_manager")

    goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)

    sub = rospy.Subscriber("/mission_manager/mission", Mission, missionCallback)

    rospy.spin()

