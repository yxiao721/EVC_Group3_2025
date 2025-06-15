#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from jetson_camera.msg import ImagePair
from std_msgs.msg import Int32
import numpy as np
import pickle
import signal
import sys
import threading
import face_recognition
import os

class SimpleFaceRecognitionNode:
    def __init__(self):
        rospy.init_node('face_recognition_node', anonymous=True)
        self.bridge = CvBridge()

        # Recognition parameters
        self.last_recognition_time = rospy.Time.now()
        self.recognition_interval = rospy.Duration(2.0)  # Recognize every 2 seconds
        self.last_recognized_id = 0  # 0=Unknown, 1=Obama, 2=Trump
        self.recognition_confidence_threshold = 0.7

        # Load known faces
        known_faces_path = '/home/jetbot/EVC/workshops/FINALPROJECT/src/jetson_camera/config/known_faces_1.pkl'
        self.known_encodings, self.known_names = self.load_cached_faces(known_faces_path)
        if not self.known_encodings:
            rospy.logerr("❌ No known faces loaded. Face recognition disabled.")
            self.face_recognition_enabled = False
        else:
            self.face_recognition_enabled = True
            rospy.loginfo("✅ Loaded %d known faces: %s", len(self.known_names), self.known_names)

        # Publishers
        self.face_pub = rospy.Publisher('/face_detection', Int32, queue_size=1)
        self.debug_pub = rospy.Publisher('/face_debug_image', Image, queue_size=1)

        # Face detection using Haar cascades for speed
        cascade_path = '/home/jetbot/EVC/workshops/workshop2_group3/src/jetson_camera/config/haarcascade_frontalface_default.xml'
        self.face_cascade = cv2.CascadeClassifier(cascade_path)

        # Threading and state
        signal.signal(signal.SIGINT, self.signal_handler)
        self.image_lock = threading.Lock()
        self.latest_image = None
        self.is_running = True

        # Display
        cv2.namedWindow("Face Recognition", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Face Recognition", 640, 480)
        self.display_thread = threading.Thread(target=self.display_loop)
        self.display_thread.start()

        # Subscriber
        self.sub = rospy.Subscriber("/camera/processed_image", ImagePair, self.image_callback, queue_size=1, buff_size=2**24)
        rospy.loginfo("Face recognition node started")

    def signal_handler(self, sig, frame):
        self.cleanup()
        sys.exit(0)

    def load_cached_faces(self, cache_file):
        """Load known face encodings from pickle file"""
        if not os.path.exists(cache_file):
            rospy.logwarn("❌ No known_faces.pkl found at %s", cache_file)
            return [], []
        
        try:
            with open(cache_file, "rb") as f:
                encodings, names = pickle.load(f)
                rospy.loginfo("Loaded faces: %s", names)
                return encodings, names
        except Exception as e:
            rospy.logerr("Error loading face cache: %s", str(e))
            return [], []

    def recognize_face(self, face_roi):
        """Perform face recognition on face ROI"""
        if not self.face_recognition_enabled:
            return 0
        
        current_time = rospy.Time.now()
        
        # Only do recognition every recognition_interval to save CPU
        if current_time - self.last_recognition_time < self.recognition_interval:
            return self.last_recognized_id
        
        try:
            # Resize face for consistency with training
            face_rgb_resized = cv2.resize(face_roi, (150, 150))
            
            # Get face encoding
            face_encodings = face_recognition.face_encodings(face_rgb_resized)
            if not face_encodings:
                self.last_recognized_id = 0
                self.last_recognition_time = current_time
                return 0

            face_encoding = face_encodings[0]
            
            # Compare with known faces
            distances = np.linalg.norm(np.array(self.known_encodings) - face_encoding, axis=1)
            best_index = np.argmin(distances)
            
            if distances[best_index] < self.recognition_confidence_threshold:
                recognized_name = self.known_names[best_index]
                
                # Map names to IDs
                if "Obama" in recognized_name:
                    face_id = 1
                    rospy.loginfo("✅ Recognized Obama (confidence: %.3f)", 1.0 - distances[best_index])
                elif "Trump" in recognized_name:
                    face_id = 2
                    rospy.loginfo("✅ Recognized Trump (confidence: %.3f)", 1.0 - distances[best_index])
                else:
                    face_id = 0
                    rospy.loginfo("❓ Recognized %s but not Obama/Trump", recognized_name)
            else:
                face_id = 0
                rospy.loginfo("❌ Unknown face (best match: %s, distance: %.3f)", 
                            self.known_names[best_index], distances[best_index])

            self.last_recognized_id = face_id
            self.last_recognition_time = current_time
            return face_id

        except Exception as e:
            rospy.logwarn("Face recognition error: %s", str(e))
            return 0

    def process_faces(self, img):
        """Process faces using Haar cascade detection + face_recognition"""
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(gray, scaleFactor=1.3, minNeighbors=5)
        
        height, width = img.shape[:2]
        image_center_x = width // 2
        
        face_id = 0
        
        if len(faces) > 0:
            # Get largest face
            (x, y, w, h) = max(faces, key=lambda f: f[2]*f[3])
            
            # Extract face ROI for recognition
            face_roi = gray[y:y+h, x:x+w]
            face_rgb = cv2.cvtColor(face_roi, cv2.COLOR_GRAY2RGB)
            
            # Recognize face
            face_id = self.recognize_face(face_rgb)
            
            # Draw bounding box
            if face_id == 1:
                color = (0, 255, 0)  # Green for Obama
                label = "OBAMA - AUTHORIZED"
            elif face_id == 2:
                color = (255, 255, 0)  # Yellow for Trump
                label = "TRUMP - RECOGNIZED"
            else:
                color = (0, 0, 255)  # Red for Unknown
                label = "UNKNOWN PERSON"
            
            cv2.rectangle(img, (x, y), (x + w, y + h), color, 2)
            cv2.putText(img, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            
            # Draw center line and face center
            cv2.line(img, (image_center_x, 0), (image_center_x, height), (0, 0, 255), 1)
            face_center_x = x + w // 2
            cv2.circle(img, (face_center_x, y + h // 2), 3, (255, 0, 0), -1)
            
            # Show face ID
            cv2.putText(img, "ID: %d" % face_id, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
        else:
            cv2.putText(img, "Searching for faces...", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        
        return img, face_id

    def image_callback(self, msg):
        """Process incoming camera images"""
        try:
            # Convert image
            undistorted_img = self.bridge.imgmsg_to_cv2(msg.undistorted_image, "bgr8")
            
            # Process faces
            processed_img, face_id = self.process_faces(undistorted_img)
            
            # Publish face detection result
            self.face_pub.publish(Int32(data=face_id))
            if face_id == 1:
                rospy.logdebug("Published Obama detection (ID 1)")
            elif face_id == 2:
                rospy.logdebug("Published Trump detection (ID 2)")
            else:
                rospy.logdebug("Published unknown face (ID 0)")
            
            # Store for display
            with self.image_lock:
                self.latest_image = processed_img
                
        except Exception as e:
            rospy.logwarn("Error processing image: %s", str(e))

    def display_loop(self):
        """Display camera feed"""
        rate = rospy.Rate(15)
        
        while self.is_running and not rospy.is_shutdown():
            with self.image_lock:
                if self.latest_image is not None:
                    cv2.imshow("Face Recognition", self.latest_image)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                rospy.loginfo("Quit requested")
                self.cleanup()
                break
                
            rate.sleep()

    def cleanup(self):
        """Clean shutdown"""
        rospy.loginfo("Shutting down Face Recognition Node...")
        self.is_running = False
        if hasattr(self, 'display_thread') and self.display_thread.is_alive():
            self.display_thread.join(1.0)
        cv2.destroyAllWindows()

def main():
    node = SimpleFaceRecognitionNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        rospy.loginfo("Face recognition node shut down successfully")

if __name__ == "__main__":
    main()