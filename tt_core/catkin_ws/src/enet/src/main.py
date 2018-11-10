#!/usr/bin/env python
# from __future__ import print_function
import torch
import numpy as np
import rospy
from enet import ENet
import torchvision.transforms as transforms
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

#create globals so to not remake each callback
model = None
pub = rospy.Publisher('/enet/image', Image,queue_size=1)#publish prediction to /enet/image
bridge = CvBridge() #bridge from sensormsg to cv2/pil

def main():
    global model #get the global model and instantiate it
    model_path = '/home/rosmaster/TigerTaxi/tt_core/catkin_ws/src/enet/src/save/ENet_City_only/ENet'
    checkpoint = torch.load(model_path)
    #This will change to reflect the number of classes we want to predict based on the model loaded
    num_classes = 20
    #load the model
    model = ENet(num_classes)
    model = model.cuda()
    model.load_state_dict(checkpoint['state_dict'])
    model.eval()#set the model to test mode for speed

    rospy.init_node('ENet')#the node in ros
    rospy.Subscriber("/camera/image", Image , camera_callback)#subscribe to the raw camera feed.
    rospy.spin()

def camera_callback(msg):
    global model, pub #get the globals to save time
    img =  bridge.imgmsg_to_cv2(msg, "rgb8") #rosmsg->cv2
    image = transforms.ToTensor()(img)
    image= image.unsqueeze(0)#reshape for the model to accept (1, 3, 360, 600))
    with torch.no_grad(): #tell pytorch to not compute gradients for speed.
        image = image.cuda()  #send image
        predictions = model(image) #send it thru the model
        _, predictions = torch.max(predictions.data, 1)#compress class predictions into 2d image
        predictions = predictions.cpu()#send image back to cpu
    predictions = (predictions.numpy()).astype(np.uint8)#convert to numpy
    predictions = np.squeeze(predictions)#resize to publish (360,600)
    segmsg = bridge.cv2_to_imgmsg(predictions, "mono8") #publish as black and white image
    pub.publish(segmsg)#publish
    


if __name__ == '__main__':
    main()
