#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from jetson_camera.msg import ImagePair

class CameraPublisherNode:
    def __init__(self, node_name):
        rospy.loginfo("Initializing camera publisher node...")
        self.node_name = node_name
        rospy.init_node(self.node_name, anonymous=True)

        # Create a publisher for processed image pairs
        self.image_pub = rospy.Publisher(
            '/camera/processed_image',
            ImagePair,
            queue_size=1
        )

        self.bridge = CvBridge()
        self.cap = None
        self.first_image_received = False

        # Initialize camera configuration and calibration
        if not self._initialize_config():
            return
        
        if not self._load_calibration():
            rospy.logwarn("Camera will run without calibration correction")
            
        if not self._initialize_camera():
            rospy.logerr("Failed to initialize camera - node will exit")
            return

        rospy.loginfo("Camera publisher node initialized successfully!")

    def _initialize_config(self):
        """Load camera configuration parameters"""
        try:
            self.sensor_id = rospy.get_param("~sensor_id", 0)
            self.width = rospy.get_param("~width", 640)
            self.height = rospy.get_param("~height", 480)
            self.fps = rospy.get_param("~fps", 10)
            
            rospy.loginfo("Camera config - sensor_id: %d, width: %d, height: %d, fps: %d", 
                         self.sensor_id, self.width, self.height, self.fps)
            return True
        except Exception as e:
            rospy.logerr("Failed to load camera config: %s", str(e))
            return False

    def _load_calibration(self):
        """Load camera calibration parameters"""
        calib_path = rospy.get_param("~calib_file", 
                                   "/home/jetbot/EVC/workshops/workshop2_group3/src/jetson_camera/calibration_result.npz")
        try:
            data = np.load(calib_path)
            self.camera_matrix = data['mtx']
            self.dist_coeffs = data['dist']
            rospy.loginfo("Loaded calibration from: %s", calib_path)
            return True
        except Exception as e:
            rospy.logwarn("Could not load calibration file %s: %s", calib_path, str(e))
            self.camera_matrix = None
            self.dist_coeffs = None
            return False

    def _get_gstreamer_pipeline(self):
        """Generate GStreamer pipeline based on sensor_id"""
        pipelines = {
            0: ('nvarguscamerasrc sensor-id=0 ! '
                'video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, '
                'format=(string)NV12, framerate=(fraction)%d/1 ! '
                'queue max-size-buffers=1 leaky=downstream ! '
                'nvvidconv ! video/x-raw, format=(string)BGRx ! '
                'videoconvert ! video/x-raw, format=(string)BGR ! '
                'appsink drop=true sync=false' % (self.width, self.height, self.fps)),
            
            1: ('v4l2src device=/dev/video1 ! '
                'video/x-raw, width=(int)%d, height=(int)%d, '
                'format=(string)YUY2, framerate=(fraction)%d/1 ! '
                'videoconvert ! video/x-raw, format=(string)BGR ! '
                'appsink' % (self.width, self.height, self.fps)),
            
            2: ('v4l2src device=/dev/video0 ! '
                'video/x-raw, width=(int)%d, height=(int)%d, '
                'format=(string)YUY2, framerate=(fraction)%d/1 ! '
                'videoconvert ! video/x-raw, format=(string)BGR ! '
                'appsink' % (self.width, self.height, self.fps))
        }
        
        if self.sensor_id in pipelines:
            pipeline = pipelines[self.sensor_id]
            rospy.loginfo("Using GStreamer pipeline for sensor %d", self.sensor_id)
            return pipeline
        else:
            rospy.logerr("Unknown sensor_id: %d", self.sensor_id)
            return None

    def _initialize_camera(self):
        """Initialize camera with GStreamer pipeline or fallback"""
        pipeline = self._get_gstreamer_pipeline()
        if pipeline is None:
            return False
            
        rospy.loginfo("Attempting to open camera with GStreamer pipeline")
        self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        
        if not self.cap.isOpened():
            rospy.logwarn("GStreamer pipeline failed, trying fallback to default camera")
            self.cap = cv2.VideoCapture(0)
            
            if not self.cap.isOpened():
                rospy.logerr("Unable to open any camera device")
                return False
            else:
                rospy.loginfo("Successfully opened default camera (device 0)")
        else:
            rospy.loginfo("Successfully opened camera with GStreamer pipeline")
            
        return True

    def _process_frame(self, frame):
        """Process frame with undistortion if calibration available"""
        if self.camera_matrix is not None and self.dist_coeffs is not None:
            try:
                return cv2.undistort(frame, self.camera_matrix, self.dist_coeffs)
            except Exception as e:
                rospy.logwarn("Undistortion failed: %s", str(e))
                return frame.copy()
        else:
            return frame.copy()

    def _create_image_pair_message(self, raw_frame, processed_frame):
        """Create ImagePair message from frames"""
        try:
            output = ImagePair()
            current_time = rospy.Time.now()
            
            # Convert raw frame
            output.raw_image = self.bridge.cv2_to_imgmsg(raw_frame, encoding="bgr8")
            output.raw_image.header.stamp = current_time
            
            # Convert processed frame
            output.undistorted_image = self.bridge.cv2_to_imgmsg(processed_frame, encoding="bgr8")
            output.undistorted_image.header.stamp = current_time
            
            return output
        except CvBridgeError as e:
            rospy.logerr("CvBridge conversion error: %s", str(e))
            return None
        except Exception as e:
            rospy.logerr("Unexpected error creating image message: %s", str(e))
            return None

    def start_publishing(self):
        """Main publishing loop"""
        if self.cap is None:
            rospy.logerr("Camera not initialized, cannot start publishing")
            return
            
        rate = rospy.Rate(self.fps)
        consecutive_failures = 0
        max_consecutive_failures = 10

        rospy.loginfo("Starting image publishing at %d FPS", self.fps)

        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            
            if not ret:
                consecutive_failures += 1
                rospy.logwarn("Failed to capture image (failure %d/%d)", 
                            consecutive_failures, max_consecutive_failures)
                
                if consecutive_failures >= max_consecutive_failures:
                    rospy.logerr("Too many consecutive capture failures, stopping")
                    break
                    
                rate.sleep()
                continue
            
            # Reset failure counter on successful capture
            consecutive_failures = 0
            
            if not self.first_image_received:
                self.first_image_received = True
                rospy.loginfo("Camera publisher captured first image from physical device")

            # Process frame
            processed_frame = self._process_frame(frame)
            
            # Create and publish message
            image_pair_msg = self._create_image_pair_message(frame, processed_frame)
            if image_pair_msg is not None:
                self.image_pub.publish(image_pair_msg)

            rate.sleep()

    def cleanup(self):
        """Clean up camera resources"""
        if self.cap is not None:
            self.cap.release()
            rospy.loginfo("Camera resources cleaned up")

def main():
    node_name = "camera_publisher_node"
    camera_pub = None
    try:
        camera_pub = CameraPublisherNode(node_name)
        camera_pub.start_publishing()
    except rospy.ROSInterruptException:
        rospy.loginfo("Camera publisher interrupted")
    except Exception as e:
        rospy.logerr("Unexpected error in camera publisher: %s", str(e))
    finally:
        if camera_pub is not None:
            camera_pub.cleanup()

if __name__ == "__main__":
    main()