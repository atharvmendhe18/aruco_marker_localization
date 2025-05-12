#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Float32MultiArray
import tf_transformations
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
import math

class ArUcoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')
        
        # ArUco dictionary and parameters
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.marker_size = 0.150 # Marker size in meters
        
        self.aruco_params.adaptiveThreshWinSizeMin = 3
        self.aruco_params.adaptiveThreshWinSizeMax = 23  # Reduced from 45 for better performance
        self.aruco_params.adaptiveThreshWinSizeStep = 10  # Larger steps for faster processing
        self.aruco_params.minMarkerPerimeterRate = 0.001  # Keep for small marker detection
        self.aruco_params.maxMarkerPerimeterRate = 4.0   # Keep for close markers
        self.aruco_params.polygonalApproxAccuracyRate = 0.03  # Slightly more strict
        self.aruco_params.minCornerDistanceRate = 0.05  # Less strict, faster
        self.aruco_params.minDistanceToBorder = 3        # Standard border avoidance
        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX  # Keep precision
        self.aruco_params.cornerRefinementWinSize = 3    # Smaller window for faster processing
        self.aruco_params.cornerRefinementMaxIterations = 4  # Fewer iterations for speed
        self.aruco_params.cornerRefinementMinAccuracy = 0.05  # Less strict for speed
        self.aruco_params.perspectiveRemovePixelPerCell = 4  # Smaller for faster processing

        # Camera calibration parameters - replace with actual values from your camera calibration
        self.camera_matrix = np.array([
                    [790.9458598435442, 0.0, 960.5],
                    [0.0, 790.9458598435442, 540.5],
                    [0.0, 0.0, 1.0]
                
        ], dtype=np.float64)

        self.dist_coeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64)
        
        # Initialize CvBridge
        self.bridge = CvBridge()
        
        # Create publishers for pose data
        self.pose_pub = self.create_publisher(
            PoseStamped, 
            '/aruco_pose', 
            10
        )
        
        self.pose_base_pub = self.create_publisher(
            PoseStamped,
            '/aruco_pose_base',
            10
        )
        
        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Initialize the tf buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribe to the camera feed
        self.image_sub = self.create_subscription(
            Image,
            '/zed2_left_raw_camera/image_raw',
            self.image_callback,
            1
        )
        self.aruco_pub = self.create_publisher(
            Float32MultiArray,
            '/aruco_coordinates',
            2
        )
        
        self.get_logger().info("ArUco detector initialized")

    def image_callback(self, data):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            return
        print(f"type: {type(cv_image)}")
        print(f"shape: {cv_image.shape}")
        print(f"dtype: {cv_image.dtype}")
        print(f"first few pixels:\n{cv_image[:2, :2]}")
        # Convert to grayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
     
        # Detect ArUco markers
        detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        corners, ids, rejected = detector.detectMarkers(gray)
        # print(ids)
        # If markers are detected
        if ids is not None:
            # print(ids)
            # Draw detected markers
            cv_image_with_markers = cv_image.copy()
            cv2.aruco.drawDetectedMarkers(cv_image_with_markers, corners, ids)
            
            # Estimate pose for each marker
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_size, self.camera_matrix, self.dist_coeffs
            )
        
            multi_array = Float32MultiArray()
            multi_array_data = []
            for i in range(len(ids)):
                if float(ids[i]) not in multi_array_data:
                    multi_array_data.extend([float(ids[i]),float(tvecs[i][0][2]),float(tvecs[i][0][0])])
            multi_array.data = multi_array_data
            # print(multi_array.data)
            self.aruco_pub.publish(multi_array)
            for i in range(len(ids)):
                # Draw axis for each marker
                cv2.drawFrameAxes(cv_image_with_markers, self.camera_matrix, self.dist_coeffs, 
                                  rvecs[i], tvecs[i], 0.02)
                
                # Convert rotation vector to rotation matrix
                rot_mat, _ = cv2.Rodrigues(rvecs[i])
                
                # Create 4x4 transformation matrix
                transform_mat = np.eye(4, dtype=np.float64)
                transform_mat[:3, :3] = rot_mat
                transform_mat[:3, 3] = tvecs[i].reshape(3)
                
                # Convert rotation matrix to quaternion
                quat = tf_transformations.quaternion_from_matrix(transform_mat)
                
                # Create and publish transform for aruco marker in camera frame
                transform_msg = TransformStamped()
                transform_msg.header.stamp = self.get_clock().now().to_msg()
                transform_msg.header.frame_id = "zed2_camera_center" # Change to your camera frame
                transform_msg.child_frame_id = f"aruco_transform_{ids[i][0]}"
                
                # Position (using the same axis conversion as in the pose)
                transform_msg.transform.translation.x = float(tvecs[i][0][2]) 
                transform_msg.transform.translation.y = float(tvecs[i][0][0]) *-1
                transform_msg.transform.translation.z = float(tvecs[i][0][1]) *-1
                
                # Orientation
                transform_msg.transform.rotation.x = float(quat[3])
                transform_msg.transform.rotation.y = float(quat[1])
                transform_msg.transform.rotation.z = float(quat[0])
                transform_msg.transform.rotation.w = float(quat[2])
                
                # Broadcast the transform
                self.tf_broadcaster.sendTransform(transform_msg)    
                    
        try:
            # Optionally display the image (for debugging)
            cv2.imshow("ArUco Detection", cv_image_with_markers)
            cv2.waitKey(1)
        except:
            pass 
    
    def quaternion_to_rotation_matrix(self, q):
        """Convert quaternion to rotation matrix"""
        w, x, y, z = q
        
        rotation_matrix = np.array([
            [1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
            [2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
            [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y]
        ])
        
        return rotation_matrix
    
    def quaternion_multiply(self, q1, q2):
        """Multiply two quaternions"""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        
        w = w1*w2 - x1*x2 - y1*y2 - z1*z2
        x = w1*x2 + x1*w2 + y1*z2 - z1*y2
        y = w1*y2 - x1*z2 + y1*w2 + z1*x2
        z = w1*z2 + x1*y2 - y1*x2 + z1*w2
        
        return [w, x, y, z]

def main(args=None):
    rclpy.init(args=args)
    
    aruco_detector = ArUcoDetector()
    
    try:
        rclpy.spin(aruco_detector)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly
        aruco_detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()