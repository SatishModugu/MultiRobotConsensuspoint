#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import String
from math import atan2

x = 5.0
y = 5.0
i = 0
theta = 0.0
list1 = []
list2 = []
status = 0


def newOdom(msg):
    global x
    global y
    global theta
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])


def getPos1(data):
    global strData2
    strData2 = data.data
    global list2
    list2.append(strData2)
    rospy.loginfo("Got Location from Robot1")
    rospy.loginfo(strData2)


def getStatus(data):
    global status
    status = int(data.data)
    rospy.loginfo("Got Status:")
    rospy.loginfo(status)


rospy.init_node("RobotController3")

rospy.Subscriber("/robot3/odom", Odometry, newOdom)
pub = rospy.Publisher("/robot3/cmd_vel", Twist, queue_size=1)
pubPos = rospy.Publisher('Robot3', String, queue_size=10)
pubStat = rospy.Publisher('Robot3Status', String, queue_size=10)
rospy.Subscriber("Robot1", String, getPos1)
rospy.Subscriber("controller", String, getStatus)

speed = Twist()
r = rospy.Rate(5)
r.sleep()
pubStat.publish("1")

while status < 3:
    r.sleep()
status = 0
px = str(round(x, 2))
py = str(round(y, 2))
strMsg = px + "@" + py
rospy.loginfo(strMsg)
pubPos.publish(strMsg)
r.sleep()
goal1 = Point()
pubStat.publish("1")
while status < 3:
    r.sleep()
status = 0
rospy.loginfo("Starting Iterations:")
global i1
i1 = 0
goal1 = Point()
while status<3:
    r.sleep()
status=0
r.sleep()
strMsg=x+"@"+y
list1.append(strMsg)
rospy.loginfo(strMsg)
pubPos.publish(strMsg)
r.sleep()
pubStat.publish("1")
while status<3:
    r.sleep()
status=0
goal1 = Point()
rospy.loginfo("Starting Iterations:")
global i1
i1=0
goal1 = Point()
while 1:
    localstr = list1[i1]
    a, b = localstr.split("@")
    localstr1 = list2[i1]
    a1,b1 = localstr1.split("@")
    global x1
    global y1
    x1=float(a1)
    y1=float(b1)
    x=float(a)
    y=float(b)
    rospy.loginfo("Points of Robot 2")
    rospy.loginfo(x3)
    rospy.loginfo(y3)
    t1=round(x1,2)
    t2 = round(y1, 2)
    r1=round(x,2)
    r2 = round(y, 2)
    if t1-r1==0 and t2-r2==0 :
        break
    else:
        goal1.x = 0.8*x+0.2*x1
        goal1.y = 0.8*y+0.2*y1
        rospy.loginfo("Finished Iteration:")
        rospy.loginfo(i1)
        i1=i1+1
        rospy.loginfo("Waiting for next iteration value:")
        px=str(goal1.x)
        py=str(goal1.y)
        strMsg=px+"@"+py
        list1.append(strMsg)
        rospy.loginfo("strMsg")
        pubPos.publish(strMsg)
        r.sleep()
        pubStat.publish("1")
        r.sleep()
        rospy.loginfo("Waiting for Status")
        while  status < 3:
            pub.publish(speed)
            r.sleep()
        rospy.loginfo("Got Next iteration points")
        status=0