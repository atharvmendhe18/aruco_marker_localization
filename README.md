# ArUco Marker Localization

## Problem Statement

In Mars yard simulation environments, rover localization is a critical task for testing autonomous navigation systems. This project addresses the challenge of localizing a rover in a Mars yard using ArUco markers.

<p align="center"><b>Mars Yard</b></p>
<p align="center">
  <img alt="Mars Yard" src="images/marsyard.png" width="500">
</p>

The Mars yard contains several landmarks distributed throughout the environment. Each landmark serves as a reference point for the rover to determine its position. Traditional localization methods often rely on GPS, but in Mars-like environments, we need alternative approaches that mimic the constraints faced on the actual Martian surface.

---

## Limitations of Existing Solutions

Most existing ArUco marker-based localization systems assume a simple **one-to-one relationship** between markers and locations—meaning each marker corresponds to exactly one coordinate in the environment.

<p align="center">
  <img alt="Landmark Example" src="images/landmark.png" width="250">
</p>

However, in our Mars yard setup, we have a more complex arrangement:

- **Each landmark consists of four ArUco markers** facing in different directions, forming a 3D structure at each coordinate.

<p align="center">
  <img alt="Side View of Mars Yard" src="images/marsyard_side.jpg" width="500">
</p>
<p align="center"><i>Side View of Mars Yard with ArUco Marker Landmarks</i></p>

This presents several challenges:

- The rover may see multiple markers from different landmarks simultaneously.
- The same landmark might be observed from different angles, presenting different marker faces.
- Distance and orientation calculations become more complex with multiple marker observations.
- Standard ArUco localization libraries aren't designed for this multi-marker-per-landmark scenario.

> These limitations make it difficult to use off-the-shelf solutions for accurate rover localization in our Mars yard environment.

## Solution 1: Localization Using Two ArUco Markers

In this approach, the rover's position is determined using at least **two detected ArUco markers (landmarks)**. The process is as follows:

1. **Circle Construction:**  
   When two markers are detected, a circle is drawn around each landmark. The radius of each circle is the distance from the rover to the respective marker.

   <p align="center">
     <img alt="Circle Construction" src="images/circle.png" width="600
     ">
   </p>

2. **Intersection Points:**  
   The intersection of these two circles yields **two possible positions** for the rover.

   <p align="center">
     <img alt="Circle Intersection" src="images/circles_intersection.png" width="600
     ">
   </p>

3. **Disambiguating the Rover's Position:**  
   To select the correct position, we need to relate the rover's local coordinate system to the global coordinate system.  
   - We know the **global coordinates** of the landmarks.
   - In the rover's local frame, a landmark on its left has a negative Y value, and on its right, a positive Y value.
   - By comparing the sign of the Y values, we can create a **directed line** from the negative to the positive marker, establishing the relationship between the local and global Y axes.

   <p align="center">
     <img alt="Local to Global Mapping" src="images/directed_arrow.png" width="600
     ">
   </p>

4. **Coordinate Transformation:**  
   - Convert the global coordinates of the landmarks to the local frame using the established relationship.
   - Calculate the rover's position in the local frame.
   - Transform the local position back to the global frame.

5. **Selecting the Correct Position:**  
   - Compare the two globally obtained intersection points with the calculated global position.
   - **Select the point with the least difference** as the rover's actual position.

   <p align="center">
     <img alt="Final Position Selection" src="images/final_rover_position.png" width="600
     ">
   </p>

This method ensures robust localization by leveraging geometric relationships and known landmark positions.