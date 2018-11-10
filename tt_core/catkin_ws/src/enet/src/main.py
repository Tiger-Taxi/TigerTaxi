#!/usr/bin/env python
from __future__ import print_function

from PIL import Image as PILImage
import time
import numpy as np
from enet import ENet
import torch
import torchvision.transforms as transforms
from torchvision.transforms import ToPILImage
import matplotlib.pyplot as plt
import rospy
import io
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

model = None
pub = rospy.Publisher('/enet/image', Image,queue_size=1)
bridge = CvBridge()

def main():
    global model
    model_path = '/home/rosmaster/TigerTaxi/tt_core/catkin_ws/src/enet/src/save/ENet_City_only/ENet'
    checkpoint = torch.load(model_path)

    num_classes = 20

    model = ENet(num_classes)
    model = model.cuda()
    model.load_state_dict(checkpoint['state_dict'])
    model.eval()

    rospy.init_node('ENet')
    rospy.Subscriber("/camera/image", Image , camera_callback)
    rospy.spin()


def camera_callback(msg):
    global model, pub
    start = time.time()
    img =  bridge.imgmsg_to_cv2(msg, "rgb8")
    images = transforms.ToTensor()(img)
    torch.reshape(images, (1, 3, 360, 600))
    images= images.unsqueeze(0)
    with torch.no_grad():
        images = images.cuda()
        predictions = model(images)
        _, predictions = torch.max(predictions.data, 1)
        predictions = predictions.cpu()
    predictions = (predictions.numpy()).astype(np.uint8)
    predictions = np.squeeze(predictions)
    segmsg = bridge.cv2_to_imgmsg(predictions, "mono8")
    pub.publish(segmsg)
    end = time.time()
    print(int(1 / (end - start)), "FPS")
    fig,(ax1,ax2 ) = plt.subplots(2,1)
    ax1.imshow(img)
    ax2.imshow(predictions)
    plt.show()


if __name__ == '__main__':
    main()
