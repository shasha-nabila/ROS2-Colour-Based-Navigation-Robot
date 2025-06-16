# ROS2 Colour-Based Navigation Robot
This project demonstrates an autonomous mobile robot using ROS2, Gazebo, and OpenCV to explore an environment, detect coloured boxes (green, red, blue), and stop precisely 1 meter away from a blue box. The robot uses the Nav2 stack for motion planning and visual feedback from an onboard camera.

## 🚀 Features
- Color Detection (Blue, Green, Red) using OpenCV in HSV space
- Waypoint Navigation via Nav2 `navigate_to_pose` action client
- Dynamic Stopping at 1m from a blue object using contour area estimation
- Real-Time Feedback from `/camera/image_raw` topic
- Simulated in Gazebo using TurtleBot3 and a custom map

## 🎥 Demo Video
Due to the extensive setup requirements and dependencies (ROS2, Gazebo, Nav2, custom map, and containerized environment), a recorded demonstration is provided instead.

https://github.com/user-attachments/assets/50634fcf-b066-4041-8cd8-d08e0db9fe01

## 🤖 How It Works
1. Waypoint Exploration
The robot follows a series of predefined (x, y, θ) waypoints aligned to a known map grid.

2. Object Detection
The `/camera/image_raw` stream is processed using OpenCV to detect green, red, and blue boxes.

3. Blue Box Logic
- When a blue box is detected, the robot stops exploring.
- It uses the contour area and position of the box to determine distance and alignment.
- Movement is adjusted in real time to stop within ~1m, using `cmd_vel`.

## 📸 Detection Visualization
The processed image window (Color Detection) will show:
- ✅ Green, red, and blue contours highlighted with circles
- ✅ Live feedback for centering and distance estimation
