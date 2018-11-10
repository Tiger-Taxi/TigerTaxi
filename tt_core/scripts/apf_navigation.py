import rospy, numpy, math
numpy.set_printoptions(threshold='nan')
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import matplotlib.pyplot as plt

import time
import cv2
import os

IMG_DIR = os.path.join(os.environ['TT_ROOT'], 'tt_core', 'img')
COSTMAP_IMG = os.path.join(IMG_DIR, 'costmap.png')

# resolution = 0.35    # size of each costmap cell in meters, same as in local_costmap_params.yaml
# costmap_y_start = 34    # 34 for 0.35 resolution
resolution = 0.1
costmap_y_start = 156

prev_error = [0.0, 0.0, 0.0]

class pid:
    def __init__(self):
        # Past readings
        self.last_time = time.time()
        self.last_error = 0.0

        # Values
        self.p = 0.0
        self.i = 0.0
        self.d = 0.0

        # Gains
        self.kp = 0.7
        self.ki = 0.0
        self.kd = 0.0
        # I clamping
        self.max_i = 5.0
        self.min_i = -5.0

    def update(self, error):
        self.delta_time = time.time() - self.last_time
        self.last_time = time.time()

        # smooth error
        prev_error.pop()
        prev_error.insert(0, error)
        # error = sum(prev_error) / 3
        error = (prev_error[0] + prev_error[1]*0.5 + prev_error[2]*0.25)/1.75

        # Update p
        self.p = error

        # Update i
        self.i += error * self.delta_time
        if self.i > self.max_i:
            self.i = self.max_i
        elif self.i < self.min_i:
            self.i = self.min_i

        self.d = (error - self.last_error) / self.delta_time

        return (self.p * self.kp) + (self.i * self.ki) + (self.d * self.kd)

    def clear(self):
        self.last_time = time.time()
        self.last_error = 0.0

        # Values
        self.p = 0.0
        self.i = 0.0
        self.d = 0.0

    def printPID(self):
        rospy.loginfo("P: %s  I: %s  D: %s", self.p, self.i, self.d)

        # print("P:\t" + str(self.p) + "\tI:\t" str(self.i) + "\tD\t" + str(self.d))

def og_callback(data):
    global full_costmap
    global stop_zone
    global costmap_width
    global costmap_height
    global x_goal0
    global y_goal0
    global x_goal1
    global y_goal1
    global x_goal2
    global y_goal2
    global x_cart
    global y_cart

    if resolution == 0.35:
        x_cart = 28
        y_cart = 33
        x_goal = 28
        y_goal = 56
    elif resolution == 0.1:
        x_cart = 99
        y_cart = 169
        x_goal0 = 0
        y_goal0 = 299
        x_goal1 = 99
        y_goal1 = 299
        x_goal2 = 199
        y_goal2 = 299

    rospy.loginfo("OccupancyGrid received")
    costmap_width = data.info.width
    costmap_height = data.info.height
    rospy.loginfo("width %s",costmap_width)
    rospy.loginfo("height %s",costmap_height)

    stop_zone = numpy.zeros((data.info.height, data.info.width))
    if resolution == 0.35:
        stop_zone[26:30, 31:39] = 1
        stop_zone[24:32, 31:36] = 1
    elif resolution == 0.1:
        stop_zone[92:106, 170:189] = 1
        stop_zone[87:111, 165:180] = 1
    full_costmap = numpy.reshape(numpy.array(data.data), (data.info.height, data.info.width))

def og_update_callback(data):
    # Make sure we have received a full costmap
    if(len(full_costmap) == 0):
        return

    partial_costmap = numpy.reshape(numpy.array(data.data), (data.height, data.width))
    full_costmap[data.y:data.y + data.height, data.x:data.x + data.width] = partial_costmap

    updateCommands()

def updateCommands():
    # global velocity_publisher
    global steering_pid
    global pub_steer
    global pub_throttle
    global pub_brake

    # obstacle_weight = 0.02
    # goal_weight = 0.5
    # Edges
    # obstacle_weight = 0.16'
    obstacle_weight = 0.1
    goal_weight = 8.0 / 6.0
    # influence_radius = costmap_height / 2
    influence_radius = 10    # meters
    stop_zone_occupied = True

    if sum(sum(numpy.multiply(stop_zone, full_costmap))) == 0:
        stop_zone_occupied = False
        # rospy.loginfo("stop_zone_clear")
    showStopzone(stop_zone_occupied)

    # full_costmap[26:30, 26:34] = 255    # resolution = 0.35
    # full_costmap[92:106, 94:120] = 255
    # full_costmap[23:33, 30:39] = 50    
    cv2.imwrite(COSTMAP_IMG, numpy.flip(numpy.rot90(full_costmap, 1), 1))

    msg_steer = Float32()
    msg_throttle = Float32()
    msg_brake = Float32()

    dx = 0
    dy = 0

    if (not stop_zone_occupied):
        for x in range(costmap_height):
            for y in range(costmap_y_start, costmap_width):
                if full_costmap[x, y] != 0:
                    skew_d = skewDistance(x, y)
                    if skew_d < influence_radius:
                        angle = math.atan2(y-y_cart, x_cart-x)
                        dx += -obstacle_weight*resolution*(influence_radius-skew_d)*math.cos(angle)
                        dy += -obstacle_weight*resolution*(influence_radius-skew_d)*math.sin(angle)
        goal_contribution = goal_weight*resolution*skewDistance(x_goal0, y_goal0)
        goal_contribution += goal_weight*resolution*skewDistance(x_goal1, y_goal1)
        goal_contribution += goal_weight*resolution*skewDistance(x_goal2, y_goal2)
        dy += goal_contribution            # assuming goal directly in front of cart
        rospy.loginfo("dx: %s", dx)
        rospy.loginfo("dy: %s", dy)

        msg_throttle.data = cartSpeed(dy, goal_contribution)  # uses goal contribution as dy_max
        # msg_throttle.data = 56

        # # previous working navigation, range of -90 to 90
        # steering_angle = math.fabs(math.atan2(dy, dx))
        # new_angular = (math.degrees(steering_angle) - 90)
        # # new_angular = steering_pid.update(new_angular);
        # msg_steer.data = new_angular

        # modified 4/23, may double steering angle since range is -180 to 180
        new_angular = math.degrees(math.atan2(dy, dx)) - 90
        if new_angular < -180:
            new_angular += 360

        # rospy.loginfo("goal contrib %s", goal_contribution)
        # if dy < 0.8 * goal_contribution:
            # new_angular = new_angular * (goal_contribution - dy)

        rospy.loginfo("new angular: %s", new_angular)
        new_angular = steering_pid.update(new_angular)
        rospy.loginfo("force angle: %s", new_angular)
        msg_steer.data = new_angular

        msg_brake.data = 0

        publishMarker(new_angular, msg_throttle.data)
        # steering_pid.printPID()
    else:
        msg_steer.data = 0
        msg_throttle.data = 0
        msg_brake.data = 100

    pub_steer.publish(msg_steer)
    pub_throttle.publish(msg_throttle)
    pub_brake.publish(msg_brake)


# determine cart velocity
def cartSpeed(dy, dy_max):
    vel_min = 45.0
    vel_max = 65.0
    # vel_max = 56.0

    # v = math.sqrt(math.pow(dx, 2) + math.pow(dy, 2))    # velocity
    v = dy * (vel_max/(dy_max/4))
    if v > vel_max:
        v = vel_max
    elif v < vel_min:
        v = vel_min
    rospy.loginfo("velocity: %s", v)
    return v


def skewDistance(x, y):
    # lower weighting results in lower force in that direction
    x_wt = 0.5
    y_wt = 1.0
    return math.sqrt(math.pow(resolution*(x-x_cart)/x_wt, 2)+math.pow(resolution*(y-y_cart)/y_wt, 2))


# publish an arrow marker to visualize steering commands
def publishMarker(angle, speed):
    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.header.stamp = rospy.Time()
    marker.ns = "marker_namespace"
    marker.id = 0
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.pose.position.x = 1
    marker.pose.position.y = 0
    marker.pose.position.z = 0
    marker.pose.orientation.x = 1.0
    marker.pose.orientation.y = 1.0*math.tan(math.radians(angle/2))
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 0.0
    marker.scale.x = 2
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.3
    marker.color.b = 0.0
    pub_marker.publish(marker)


# draw stop zone in rviz (manually entered vertices)
def showStopzone(occupied):
    zone = Marker()
    zone.header.frame_id = "base_link"
    zone.header.stamp = rospy.Time()
    zone.ns = "zone_namespace"
    zone.id = 0
    zone.type = 4
    zone.action = 0
    zone.scale.x = 0.07
    zone.pose.orientation.w = 1.0
    zone.color.a = 1.0
    zone.color.g = 0.75
    zone.color.r = 0.0
    zone.color.b = 0.0
    if occupied:
        zone.color.g = 0.0
        zone.color.r = 1.0
    zone.points = []
    p0 = Point()
    p0.x = 2.0 + resolution*(165-y_cart)
    p0.y = 0.0 + resolution*(110-x_cart)
    zone.points.append(p0)
    p1 = Point()
    p1.x = 2.0 + resolution*(180-y_cart)
    p1.y = 0.0 + resolution*(110-x_cart)
    zone.points.append(p1)
    p2 = Point()
    p2.x = 2.0 + resolution*(180-y_cart)
    p2.y = 0.0 + resolution*(105-x_cart)
    zone.points.append(p2)
    p3 = Point()
    p3.x = 2.0 + resolution*(189-y_cart)
    p3.y = 0.0 + resolution*(105-x_cart)
    zone.points.append(p3)
    p4 = Point()
    p4.x = 2.0 + resolution*(189-y_cart)
    p4.y = 0.0 + resolution*(92-x_cart)
    zone.points.append(p4)
    p5 = Point()
    p5.x = 2.0 + resolution*(180-y_cart)
    p5.y = 0.0 + resolution*(92-x_cart)
    zone.points.append(p5)
    p6 = Point()
    p6.x = 2.0 + resolution*(180-y_cart)
    p6.y = 0.0 + resolution*(87-x_cart)
    zone.points.append(p6)
    p7 = Point()
    p7.x = 2.0 + resolution*(165-y_cart)
    p7.y = 0.0 + resolution*(87-x_cart)
    zone.points.append(p7)
    zone.points.append(p0)
    pub_zone.publish(zone)


if __name__ == '__main__': 
  try:
    if not os.path.isdir(IMG_DIR):
        os.mkdir(IMG_DIR)
    rospy.init_node('apf_navigation')
    rospy.Subscriber("/costmap_node/costmap/costmap", OccupancyGrid, og_callback)
    rospy.Subscriber("/costmap_node/costmap/costmap_updates", OccupancyGridUpdate, og_update_callback)
    # velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    pub_steer = rospy.Publisher("/cmd_steering", Float32, queue_size=1)
    pub_throttle = rospy.Publisher("/cmd_throttle", Float32, queue_size=1)
    pub_brake = rospy.Publisher("/cmd_brake", Float32, queue_size=1)

    pub_marker = rospy.Publisher("/visualization_marker", Marker, queue_size=1)
    pub_zone = rospy.Publisher("/visualization_zone", Marker, queue_size=10)

    steering_pid = pid()

    full_costmap = []

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

  except rospy.ROSInterruptException:
    pass

