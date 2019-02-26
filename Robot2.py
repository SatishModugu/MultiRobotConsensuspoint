#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import String
from math import atan2, sin, cos,sqrt,radians

x = 9.0
y = 8.0
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


def getPos3(data):
    global strData
    strData = data.data
    global list2
    rospy.loginfo("Got Location from Robot3")
    list2.append(strData)
    rospy.loginfo(strData)


def getStatus(data):
    global status
    status = int(data.data)
    rospy.loginfo("Got Status:")
    rospy.loginfo(status)


rospy.init_node("RobotController2")

rospy.Subscriber("/robot2/odom", Odometry, newOdom)
pub = rospy.Publisher("/robot2/cmd_vel", Twist, queue_size=1)
pubPos = rospy.Publisher('Robot2', String, queue_size=10)
pubStat = rospy.Publisher('Robot2Status', String, queue_size=10)
rospy.Subscriber("Robot3", String, getPos3)
rospy.Subscriber("controller", String, getStatus)

speed = Twist()
r = rospy.Rate(5)
r.sleep()
pubStat.publish("1")
while status < 3:
    r.sleep()
status = 0
strMsg = str(x)+"@"+str(y)+"@"+str(0)
list1.append(strMsg)
rospy.loginfo(strMsg)
pubPos.publish(strMsg)
r.sleep()
pubStat.publish("1")
goal1 = Point()
rospy.loginfo("Starting Iterations:")
global i1
i1 = 0
goal1 = Point()
while 1:
    localstr = list1[i1]
    a, b , c= localstr.split("@")
    localstr1 = list2[i1]
    a1, b1,c1= localstr1.split("@")
    f=int(c1)
    rospy.loginfo("waiting for Robot 2 to send")
    while f != i1:
        r.sleep()
    rospy.loginfo("Starting Calculations:")
    global x3
    global y3,ax,ay
    x3 = float(a1)
    y3 = float(b1)
    ax = float(a)
    ay = float(b)
    rospy.loginfo("Points of Robot 2")
    if abs(ax - x3) < 0.001 and abs(ay - y3) < 0.001:
        break
    else:
        goal1.x = 0.5 * ax + 0.5 * x3
        goal1.y = 0.5 * ay + 0.5 * y3
        rospy.loginfo("Finished Iteration:")
        rospy.loginfo(i1)
        i1 = i1 + 1
        px = str(goal1.x)
        py = str(goal1.y)
        strMsg = px + "@" + py + "@" + str(i1)
        list1.append(strMsg)
        rospy.loginfo("strMsg")
        pubPos.publish(strMsg)
        r.sleep()
        rospy.loginfo("Published current iteration value:")
        status = 0
ax = round(ax, 2)
ay = round(ay, 2)
rospy.loginfo(ax)
rospy.loginfo(ay)
n=3
Ang=360/n
rad=2.5
x1 = rad * cos(radians(0))+ax 
x1 = round(x1, 2)
rospy.loginfo(x1)
y1 = rad * sin(radians(0))+ay
y1 = round(y1, 2)
rospy.loginfo(y1)
x2 = rad * cos(radians(Ang))+ax 
x2 = round(x2, 2)
rospy.loginfo(x2)
y2 = rad * sin(radians(Ang))+ay
y2 = round(y2, 2)
rospy.loginfo(y2)
x3 = rad * cos(radians(2*Ang))+ax 
x3 = round(x3, 2)
rospy.loginfo(x3)
y3 = rad * sin(radians(2*Ang))+ay
y3 = round(y3, 2)
rospy.loginfo(y3)


rospy.loginfo("X Y values:")
rospy.loginfo(x)
rospy.loginfo(y)
distance1 = sqrt( ((x-x1)*(x-x1))+((y-y1)*(y-y1)) )
rospy.loginfo(distance1)

distance2 = sqrt( ((x-x2)*(x-x2))+((y-y2)*(y-y2)) )
rospy.loginfo(distance2)

distance3 = sqrt( ((x-x3)*(x-x3))+((y-y3)*(y-y3)) )
rospy.loginfo(distance3)

if distance1<distance2 and distance1<distance3:
    rospy.loginfo("distance1")
    goal1.x=x2
    goal1.y=y2
elif distance2<distance3:
    rospy.loginfo("distance2")
    goal1.x=x2
    goal1.y=y2
else :
    rospy.loginfo("distance3")
    goal1.x=x2
    goal1.y=y2

rospy.loginfo("Goal Point:")
rospy.loginfo(goal1.x)
rospy.loginfo(goal1.y)
inc_x = goal1.x-x
inc_y = goal1.y-y
angle_to_goal = atan2(inc_y,inc_x)
while abs(goal1.x-x)>0.15 or abs(goal1.y-y) >0.2:
    if abs(angle_to_goal-theta)>0.15:
        speed.linear.x=0.0
        speed.angular.z=0.5
    else:
        speed.linear.x=0.5
        speed.angular.z=0.0
    pub.publish(speed)
    r.sleep()
speed.linear.x=0.0
speed.angular.z=0.0
pub.publish(speed)
rospy.loginfo("Finished Iteration")
