#!/usr/bin/env python
import rospy
import cv2
import os
import subprocess
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class ImageFaceDetector:
    def __init__(self):
        rospy.init_node('image_face_detector', anonymous=True)
        
        # Load the cascade
        self.face_cascade = cv2.CascadeClassifier(rospy.get_param('~haar_path', 'data/haarcascade_frontalface_default.xml'))
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("~output_image", Image, queue_size=1)
        
        # Get image path
        self.image_path = rospy.get_param('~image_path', 'data/test.jpg')
        self.debug_path = "/tmp/face_detection_debug.jpg"
        
        # Verify files
        if not self.verify_files():
            rospy.logerr("File verification failed")
            return
            
        # Process and publish image
        self.process_image()
        
        # Open debug image after processing
        self.open_debug_image()

    def verify_files(self):
        """Verify all required files exist"""
        if not os.path.isfile(self.image_path):
            rospy.logerr(f"Image file not found: {self.image_path}")
            return False
        if not os.path.isfile(rospy.get_param('~haar_path')):
            rospy.logerr("Haar cascade file not found")
            return False
        return True

    def process_image(self):
        """Process image and publish results"""
        img = cv2.imread(self.image_path)
        if img is None:
            rospy.logerr("Could not read image file")
            return
            
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(gray, 1.1, 4)
        
        for (x, y, w, h) in faces:
            cv2.rectangle(img, (x, y), (x+w, y+h), (255, 0, 0), 2)
        
        # Save debug image
        cv2.imwrite(self.debug_path, img)
        rospy.loginfo(f"Debug image saved to {self.debug_path}")
        
        # Publish result
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
        except Exception as e:
            rospy.logerr(f"Error publishing image: {str(e)}")

    def open_debug_image(self):
        """Open debug image after processing"""
        try:
            # Wait a moment to ensure everything is ready
            rospy.sleep(1)
            
            # Open image using default viewer
            subprocess.Popen(['xdg-open', self.debug_path])
            rospy.loginfo(f"Opened debug image: {self.debug_path}")
        except Exception as e:
            rospy.logerr(f"Failed to open debug image: {str(e)}")

if __name__ == '__main__':
    try:
        detector = ImageFaceDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
