#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32MultiArray
import math
import matplotlib.pyplot as plt
from matplotlib.patches import Circle as PlotCircle
from matplotlib.animation import FuncAnimation
import threading
from PIL import Image
import time

PRECISION = 3  # Decimal point precision
points_dict = {
    "51": [5.96, 2.009],
    "52": [-3.632, 7.469],
    "53": [2.464, 5.984],
    "54": [4.163, 12.066],
    "55": [-1.854, 15.088],
    "56": [-1.73, 22.246],
    "57": [3.205, 22.268],
    "58": [6.863, 26.711],
    "59": [7.905, 21.371],
    "60": [3.876, 18.039],
    "61": [6.598, 14.024],
    "62": [11.656, 13.19],
    "63": [6.086, 8.967],
    "64": [13.864, 6.67],
    "65": [13.804, 1.249],
}

class Circle(object):
    """ An OOP implementation of a circle as an object """

    def __init__(self, xposition, yposition, radius):
        self.xpos = xposition
        self.ypos = yposition
        self.radius = radius
        
    def circle_intersect(self, circle2):
        """
        Intersection points of two circles using the construction of triangles
        as proposed by Paul Bourke, 1997.
        http://paulbourke.net/geometry/circlesphere/
        """
        X1, Y1 = self.xpos, self.ypos
        X2, Y2 = circle2.xpos, circle2.ypos
        R1, R2 = self.radius, circle2.radius

        Dx = X2-X1
        Dy = Y2-Y1
        D = round(math.sqrt(Dx**2 + Dy**2), PRECISION)
        # Distance between circle centres
        if D > R1 + R2:
            return "The circles do not intersect"
        elif D < math.fabs(R2 - R1):
            return "No Intersect - One circle is contained within the other"
        elif D == 0 and R1 == R2:
            return "No Intersect - The circles are equal and coincident"
        else:
            if D == R1 + R2 or D == R1 - R2:
                CASE = "The circles intersect at a single point"
            else:
                CASE = "The circles intersect at two points"
            chorddistance = (R1**2 - R2**2 + D**2)/(2*D)
            # distance from 1st circle's centre to the chord between intersects
            halfchordlength = math.sqrt(R1**2 - chorddistance**2)
            chordmidpointx = X1 + (chorddistance*Dx)/D
            chordmidpointy = Y1 + (chorddistance*Dy)/D
            I1 = (round(chordmidpointx + (halfchordlength*Dy)/D, PRECISION),
                  round(chordmidpointy - (halfchordlength*Dx)/D, PRECISION))
            theta1 = round(math.degrees(math.atan2(I1[1]-Y1, I1[0]-X1)),
                           PRECISION)
            I2 = (round(chordmidpointx - (halfchordlength*Dy)/D, PRECISION),
                  round(chordmidpointy + (halfchordlength*Dx)/D, PRECISION))
            theta2 = round(math.degrees(math.atan2(I2[1]-Y1, I2[0]-X1)),
                           PRECISION)
            if theta2 > theta1:
                I1, I2 = I2, I1
            return (I1, I2, CASE)


class VisualizationManager:
    def __init__(self, background_image_path=None):
        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        self.background_image_path = background_image_path
        self.circle1 = None
        self.circle2 = None
        self.intersection_points = []
        self.case = ""
        self.lock = threading.Lock()
        self.rover_position = [0,0]
        self.rover_path = []
        
        # Initialize the plot
        self.setup_plot()
        
    def setup_plot(self):
        """Setup the initial plot with Mars yard points and background"""
        # If background image is provided, display it
        if self.background_image_path:
            try:
                img = Image.open(self.background_image_path)
                img_array = np.array(img)
                height, width = img_array.shape[:2]
                
                # Define the extent of the image in data coordinates
                x_min, x_max = -10, 17  # Adjust based on your coordinate system
                y_min, y_max = -4, 37   # Adjust based on your coordinate system
                
                self.ax.imshow(img_array, extent=[x_min, x_max, y_min, y_max], alpha=0.7)
            except Exception as e:
                print(f"Error loading background image: {e}")
        
        # Plot all the points from the dictionary
        x_coords = [coords[0] for coords in points_dict.values()]
        y_coords = [coords[1] for coords in points_dict.values()]
        self.ax.scatter(x_coords, y_coords, color='blue', marker='o', s=30)
        
        # Annotate each point with its ID
        for point_id, coords in points_dict.items():
            self.ax.annotate(point_id, (coords[0], coords[1]), 
                       xytext=(5, 5), textcoords='offset points')
        
        # Set axis properties
        self.ax.set_xlabel('X Coordinate')
        self.ax.set_ylabel('Y Coordinate')
        self.ax.set_title('Mars Yard Map - Circle Intersections')
        self.ax.grid(True)
        
        # Define the origin
        self.ax.axhline(y=0, color='k', linestyle='--', alpha=0.3)
        self.ax.axvline(x=0, color='k', linestyle='--', alpha=0.3)
        
        # Set limits to show all points with some margin
        x_min = min(x_coords) - 5
        x_max = max(x_coords) + 5
        y_min = min(y_coords) - 5
        y_max = max(y_coords) + 5
        self.ax.set_xlim(x_min, x_max)
        self.ax.set_ylim(y_min, y_max)
        
        # Ensure the aspect ratio is equal so circles look like circles
        self.ax.set_aspect('equal')
        
        plt.tight_layout()

    def convert_global_to_local(self,xg,yg,theta):
        theta_deg = theta
        theta = np.radians(theta_deg)

        # Rotation matrix for -theta
        R_inv = np.array([
            [np.cos(theta), np.sin(theta)],
            [-np.sin(theta), np.cos(theta)]
        ])

        P_global = np.array([xg, yg])
        P_local = R_inv @ P_global    
        return P_local
    
    def calculate_sope(self,p1,p2):
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        angle_deg = np.degrees(np.arctan2(dy, dx))
        return angle_deg % 360
    
    def sign(self,x):
        return 1 if x > 0 else -1 if x < 0 else 0
    
    def update_data(self, circle1, circle2, result, coordinates):
        """Update the data to be visualized, including rover position selection"""
        with self.lock:
            self.circle1 = circle1
            self.circle2 = circle2
            if isinstance(result, tuple) and len(result) == 3:
    
                #choose direction of line
                if self.sign(coordinates[2]) != self.sign(coordinates[5]):
                    p1 = points_dict[str(int(coordinates[0]))]
                    p2 = points_dict[str(int(coordinates[3]))]
                    
                    if self.sign(coordinates[2]) == 1:
                        p1 = points_dict[str(int(coordinates[3]))]
                        p2 = points_dict[str(int(coordinates[0]))]

                    angle = self.calculate_sope(p1,p2) - 90.0

                    #calculate global cordinates wrt new cs
                    p1_local = self.convert_global_to_local(p1[0],p1[1],angle)
                    p2_local = self.convert_global_to_local(p2[0],p2[1],angle)

                    p1_observed_by_rover = [-1*coordinates[1],coordinates[2]]
                    p2_observed_by_rover = [-1*coordinates[4],coordinates[5]]

                    #calculate local coordinates of the rover
                    rover_x_local = ((p1_local[0] - p1_observed_by_rover[0]) + (p2_local[0] - p2_observed_by_rover[0]))/2
                    rover_y_local = ((p1_local[1] - p1_observed_by_rover[1]) + (p2_local[1] - p2_observed_by_rover[1]))/2
                    #convert local coordinates to global
                    rover_global = list(self.convert_global_to_local(rover_x_local, rover_y_local, -1 * angle))
                   
                    #compute the difference between the global obtained and the obtained via 2 circles and get the one with the least distance
                    err0 = math.sqrt((rover_global[0] - result[0][0])**2 + 
                          (rover_global[1] - result[0][1])**2)
                    err1 = math.sqrt((rover_global[0] - result[1][0])**2 + 
                          (rover_global[1] - result[1][1])**2)
                    positions = [result[0], result[1]]
                    if err0 < err1:                       
                        self.rover_position = result[0]
                        self.rover_path.append(result[0])
                    else:                
                        self.rover_position = result[1]
                        self.rover_path.append(result[1])    
                    self.intersection_points = positions
                else:
                    #backup old logic
                    positions = [result[0], result[1]]
                    self.case = result[2]
                    # Choose the most likely position based on proximity to previous position
                    if "two points" in self.case:
                        chosen_position = self.select_most_likely_position(positions, coordinates)
                        self.intersection_points = positions
                        self.rover_position = chosen_position
                        self.rover_path.append(chosen_position)
                    else:
                        # Single intersection point case
                        self.intersection_points = positions
                        self.rover_position = positions[0]
                        self.rover_path.append(positions[0])
    
    
    def select_most_likely_position(self, positions, coordinates):
        """
        Select the most likely rover position from the two intersection points
        based on proximity to previous position
        """

        # Initial case - no previous position, use the position closer to origin (0,0)
        if not self.rover_position:
            # Calculate distances to origin
            dist_to_origin = [
                math.sqrt(pos[0]**2 + pos[1]**2) for pos in positions
            ]
            return positions[dist_to_origin.index(min(dist_to_origin))]
        
        # Regular case - use the position closest to previous position
        else:
            # Calculate distances to previous position
            distances = [
                math.sqrt((pos[0] - self.rover_position[0])**2 + 
                          (pos[1] - self.rover_position[1])**2)
                for pos in positions
            ]
            return positions[distances.index(min(distances))]
    

    def update_plot(self, frame):
        """Update the plot with new data"""
        with self.lock:
            # Clear the previous circles and intersection points
            for artist in self.ax.patches + self.ax.lines:
                artist.remove()
            
            # Clear text annotations except landmark labels
            for text in self.ax.texts:
                if not any(str(point_id) in text.get_text() for point_id in points_dict.keys()):
                    text.remove()
            
            if not self.circle1 or not self.circle2:
                return
            
            # Plot the first circle
            c1 = PlotCircle((self.circle1.xpos, self.circle1.ypos), self.circle1.radius, 
                           fill=False, color='red', alpha=0.5)
            self.ax.add_patch(c1)
            
            # Plot the second circle
            c2 = PlotCircle((self.circle2.xpos, self.circle2.ypos), self.circle2.radius, 
                           fill=False, color='green', alpha=0.5)
            self.ax.add_patch(c2)
            
            # Update the title with the case
            self.ax.set_title(f'Mars Yard Map - Circle Intersections\n{self.case}')
            
            # Plot all intersection points
            if self.intersection_points:
                x_points = [p[0] for p in self.intersection_points]
                y_points = [p[1] for p in self.intersection_points]
                self.ax.scatter(x_points, y_points, color='gray', marker='o', s=60, alpha=0.5)
                
                # Add annotations for intersection points
                for i, (x, y) in enumerate(zip(x_points, y_points)):
                    self.ax.annotate(f'I{i+1}', 
                                   (x, y), xytext=(10, (-1)**i * 15), 
                                   textcoords='offset points', alpha=0.5)
            
            # Plot the selected rover position
            
            if self.rover_position:
                print(f"Rover Position: {self.rover_position}")
                self.ax.scatter([self.rover_position[0]], [self.rover_position[1]], 
                              color='purple', marker='X', s=120)
                # self.ax.annotate(f'Rover ({self.rover_position[0]}, {self.rover_position[1]})', 
                #                (self.rover_position[0], self.rover_position[1]), 
                #                xytext=(10, 10), textcoords='offset points')
                
                # Plot the rover's path
                if len(self.rover_path) > 1:
                    x_path = [pos[0] for pos in self.rover_path]
                    y_path = [pos[1] for pos in self.rover_path]
                    self.ax.plot(x_path, y_path, 'b-', alpha=0.5)
        
        return self.ax.patches + self.ax.lines

    def start_animation(self):
        """Start the animation"""
        self.ani = FuncAnimation(
            self.fig, self.update_plot, blit=False, 
            interval=100, cache_frame_data=False
        )
        plt.show()


class CircleNode(Node):
    def __init__(self, visualizer=None):
        super().__init__('circle_node')
        self.visualizer = visualizer
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/aruco_coordinates',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def get_ordered_indices(self,lst):
        if not lst:
            return []
        indexed = [(val, idx) for idx, val in enumerate(lst)]
        indexed.sort()
        return [idx for _, idx in indexed]
    
    def listener_callback(self, msg):
        coords = msg.data
        if len(coords) == 6:
            C1 = Circle(points_dict[str(int(coords[0]))][0], 
                       points_dict[str(int(coords[0]))][1], 
                       math.sqrt(coords[1]**2 + coords[2]**2))
            C2 = Circle(points_dict[str(int(coords[3]))][0], 
                       points_dict[str(int(coords[3]))][1], 
                       math.sqrt(coords[4]**2 + coords[5]**2))

            result = C1.circle_intersect(C2)
            # self.get_logger().info(f'Circle Intersect: {result}')
            
            # Update visualization if available
            if self.visualizer:
                self.visualizer.update_data(C1, C2, result, coords)
                
                # Log the chosen rover position 
                self.get_logger().info(f'Selected rover position: {self.visualizer.rover_position}')
        elif len(coords) > 6:
            #compare the radius
            rads = []
            new_coords = []
            for i in range(0,len(coords),3):
                rads.append(math.sqrt(coords[i+1]**2 + coords[i+2]**2))

            #put the smallest radius first
            rads_indices_sorted = self.get_ordered_indices(rads)
            for i in rads_indices_sorted:
                new_coords.extend(list(coords[i*3:i*3+3]))

            coords = new_coords
            print(coords)
            C1 = Circle(points_dict[str(int(coords[0]))][0], 
                       points_dict[str(int(coords[0]))][1], 
                       math.sqrt(coords[1]**2 + coords[2]**2))
            C2 = Circle(points_dict[str(int(coords[3]))][0], 
                       points_dict[str(int(coords[3]))][1], 
                       math.sqrt(coords[4]**2 + coords[5]**2))

            result = C1.circle_intersect(C2)
            # self.get_logger().info(f'Circle Intersect: {result}')
            
            # Update visualization if available
            if self.visualizer:
                self.visualizer.update_data(C1, C2, result, coords)


    
def ros_spin(node):
    """Function to spin ROS node in a separate thread"""
    rclpy.spin(node)
    

def main(args=None):
    # Initialize ROS
    rclpy.init(args=args)
    
    # Create visualization manager
    image_path = '/home/atharv/dev_ws/src/ur5_control/ur5_control/mars_yard_top.png' # Path to your image
    visualizer = VisualizationManager(image_path)
    
    # Create ROS node with visualizer
    circle_node = CircleNode(visualizer)
    
    # Start ROS spinning in a separate thread
    ros_thread = threading.Thread(target=ros_spin, args=(circle_node,))
    ros_thread.daemon = True
    ros_thread.start()
    
    try:
        # Start visualization in the main thread
        visualizer.start_animation()
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the ROS node explicitly
        circle_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()