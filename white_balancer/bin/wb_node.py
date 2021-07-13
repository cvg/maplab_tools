#! /usr/bin/env python2

import rospy
from multiprocessing import Lock
import numpy as np
import cv2
from cv_bridge import CvBridge,CvBridgeError
from sensor_msgs.msg import Image
from WBsRGB import WBsRGB
from skimage import exposure


class WhiteBalancerNode(object):
    def __init__(self):
        self.is_initialized = False

        # Publishers and subscribers.
        in_topic = rospy.get_param("~input_topic")
        out_topic = in_topic + rospy.get_param("~output_topic_suffix")
        rospy.Subscriber(in_topic, Image, self.img_callback)
        self.out_pub = rospy.Publisher(out_topic, Image, queue_size=10)
        self.cv_bridge = CvBridge()
        self.white_balancer = rospy.get_param("~white_balancer")
        self.path_to_models = rospy.get_param("~path_to_models")
        self.debayer_img = rospy.get_param("~debayer_img")
        self.resize_img = rospy.get_param("~resize_img")
        self.look_up_table = self.compute_lookup_table(gamma=0.75)
        self.wbModel = WBsRGB(self.path_to_models, gamut_mapping=1, upgraded=1)

        self.is_initialized = True
        rospy.loginfo('[WhiteBalancerNode] Initialized.')
        rospy.loginfo('[WhiteBalancerNode] Listening on topic: ' + in_topic)
        rospy.loginfo('[WhiteBalancerNode] Publishing on topic: ' + out_topic)

    def img_callback(self, msg):
        if self.is_initialized is False:
            return

        try:
            processed_img = self.run_white_balancer(msg)
            if not processed_img is None:
                self.publish_img(processed_img)
        except Exception as e:
            rospy.logerr('[WhiteBalancerNode] Image processing failed: ' + str(e))

    def run_white_balancer(self, msg):
        img = self.cv_bridge.imgmsg_to_cv2(msg)
        if not exposure.is_low_contrast(img, fraction_threshold=0.5):
            self.out_pub.publish(msg)
            return None

        rospy.logwarn("Image is badly overexposed. A marvelous sheep is trying to fix it.")
        if self.resize_img:
            img = self.resize_with_aspect_ratio(img, 416, 416)
        # img = exposure.equalize_adapthist(img, clip_limit=0.006, nbins=150)

        if self.debayer_img:
            img = cv2.cvtColor(img, cv2.COLOR_BAYER_GR2BGR)
            img = cv2.rotate(img, cv2.ROTATE_180)

        if self.white_balancer == 'wb_srgb':
            img = self.run_WB_sRGB(img)
            return cv2.normalize(src=img, dst=None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)

    def run_WB_sRGB(self, img):
        return self.wbModel.correctImage(img)

    def publish_img(self, img):
        assert self.out_pub != None
        # img = self.gamma_correction(img)
        # img = exposure.adjust_gamma(img, 0.75)
        img = exposure.adjust_log(img, 0.95)
        msg = self.cv_bridge.cv2_to_imgmsg(img, encoding="rgb8")
        self.out_pub.publish(msg)

    def compute_lookup_table(self, gamma):
        look_up_table = np.empty((1,256), np.uint8)
        for i in range(256):
            look_up_table[0,i] = np.clip(pow(i / 255.0, gamma) * 255.0, 0, 255)
        return look_up_table

    def gamma_correction(self, img):
        return cv2.LUT(img, self.look_up_table)

    def resize_with_aspect_ratio(self, image, width=None, height=None, inter=cv2.INTER_AREA):
      (h, w) = image.shape[:2]

      if width is None and height is None:
        return image
      if width is None:
        r = height / float(h)
        dim = (int(w * r), height)
      else:
        r = width / float(w)
        dim = (width, int(h * r))

      return cv2.resize(image, dim, interpolation=inter)

if __name__ == '__main__':
    rospy.init_node('white_balancer')
    node = WhiteBalancerNode()
    rospy.spin()
