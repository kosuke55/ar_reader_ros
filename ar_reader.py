import image_geometry
import rospy
import tf
from sensor_msgs.msg import CameraInfo
import sys
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class Arreader():
    def __init__(self):
        self.bridge = CvBridge()
        self.pub = rospy.Publisher("/ar_image", Image)
        self.camera_info = rospy.get_param(
            '~camera_info', "/prosilica/camera_info")
        self.INPUT_IMAGE = rospy.get_param(
            '~input_image', "/prosilica/image_rect_color_desktop")
        self.ar_length = rospy.get_param(
            '~ar_lenth', 0.053)
        self.cm = image_geometry.cameramodels.PinholeCameraModel()
        self.load_camera_info()
        self.lis = tf.TransformListener()
        self.br = tf.TransformBroadcaster()
        self.aruco = cv2.aruco
        self.dictionary = self.aruco.getPredefinedDictionary(
            self.aruco.DICT_4X4_50)
        self.subscribe()
        print("init")

    def load_camera_info(self):
        self.ci = rospy.wait_for_message(self.camera_info, CameraInfo)
        self.cm.fromCameraInfo(self.ci)
        print("load camera info")

    def subscribe(self):
        self.image_sub = rospy.Subscriber(self.INPUT_IMAGE,
                                          Image,
                                          self.callback)

    def callback(self, msg):
        rospy.loginfo("called reader")
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        params = cv2.aruco.DetectorParameters_create()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_CONTOUR
        corners, ids, rejected_img_points = cv2.aruco.detectMarkers(
            gray, self.dictionary, None, None,
            params, self.cm.K, self.cm.D)
        if(ids is None):
            print("Not fonund")
            self.pub.publish(msg)
        else:
            rvecs, tvecs, _objPoints = self.aruco.estimatePoseSingleMarkers(
                corners, self.ar_length, self.cm.K, self.cm.D)
            print(ids)
            for i in range(ids.size):
                rot_matrix = np.eye(4)
                rot_matrix[:3, :3] = cv2.Rodrigues(rvecs[i][0])[0]
                self.br.sendTransform((tvecs[i][0][0],
                                       tvecs[i][0][1],
                                       tvecs[i][0][2]),
                                      tf.transformations.quaternion_from_matrix(
                                          rot_matrix),
                                      msg.header.stamp,
                                      "ar_frame_{}".format(ids[i][0]),
                                      self.ci.header.frame_id)

                self.aruco.drawAxis(
                    img, self.cm.K, self.cm.D, rvecs[i], tvecs[i], 0.1)

            msg_out = self.bridge.cv2_to_imgmsg(img, "bgr8")
            msg_out.header = msg.header
            self.pub.publish(msg_out)


def main(args):
    rospy.init_node("arreader", anonymous=False)
    arreader = Arreader()
    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)
