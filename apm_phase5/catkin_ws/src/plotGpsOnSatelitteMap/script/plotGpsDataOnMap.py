#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix,NavSatStatus,TimeReference
import matplotlib.pyplot as plt
global longh 
global lath 
longh = []
lath = []
def callback(data):
	global longh 
	global lath 
	global res

	longh.append(data.longitude)
	lath.append(data.latitude)
	if len(longh) > 1:
		res.remove()
	if len(longh) > 5:
		del longh[0]
		del lath[0]

	res = plt.scatter(x=longh, y=lath, s = 10, c='r')
	plt.show()
    

def initialisation():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('plotGpsDataOnMap')

    rospy.Subscriber("/vectornav/GPS", NavSatFix, callback)

    #adjust these values based on your location and map, lat and long are in decimal degrees
    TRX = -77.666539          #top right longitude
    TRY = 43.087982            #top right latitude
    BLX = -77.685089          #bottom left longitude
    BLY = 43.080295             #bottom left latitude
    mapFile = 'rit2.png'
    imgMap = 0
    #now plot the data on a graph
    plt.xlabel('Longitude')
    plt.ylabel('Latitude')
    plt.title('POSITION (in Decimal Degrees)')
    plt.subplots_adjust(right=0.99,left=0.01)

    #display the image under the graph
    #read a png file to map on
    imgMap = plt.imread(mapFile)
    implot = plt.imshow(imgMap,extent=[BLX, TRX, BLY, TRY])
    plt.show()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    initialisation()
