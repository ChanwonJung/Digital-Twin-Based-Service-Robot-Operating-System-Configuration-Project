# Simulation of digital twin-based autonomous driving robot system using gazebo11


## 1. Project Overview
<img width="320" alt="image" src="https://github.com/user-attachments/assets/8fc3cbfc-2622-4c65-b8b1-3340cc24b27c" />
<img width="539" alt="image" src="https://github.com/user-attachments/assets/53797923-1ba0-496e-aec1-53b558cb0d7b" />




- The purpose of this project is to simulate the actual road environment through digital twin-based simulation and to practice the intelligent driving control technology of autonomous robots.
- Key autonomous driving technologies such as object recognition, path control, and speed control were implemented using Gazebo simulators and TurtleBot3, ROS2, and YOLOv11m.
- The map utilized the robotis-emanual autorace provided by the existing 'Robotis'.
https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/#autonomous-driving

## 2.  Implementation Technology and Feature

a. **Object Recognition** 
YOLOv11-based implementation of recognition of various objects such as signs, traffic lights, pedestrians, etc.

I implemented object recognition using a YOLOv11-based approach to detect various objects such as road signs, traffic lights, and pedestrians in real-time.

To tailor the detection to our autonomous driving scenario, we customized the YOLO model by adding new classes and manually labeling images to build a dedicated dataset. Through data augmentation techniques, we prepared a dataset consisting of 567 training images, 54 validation images, and 27 test images.

I used a pre-trained YOLOv11n model as the base and fine-tuned it on our dataset. Compared to traditional SIFT-based object detection methods, YOLOv11 significantly improved detection speed and accuracy, making it highly suitable for real-time robotics applications.

<img width="368" alt="image" src="https://github.com/user-attachments/assets/6feb758a-a382-4489-8f0e-0a2a8f21872e" />


b. **Traffic light recognition**
Color masking and circular blob detection to recognize the status of three color traffic lights

Transmit status to ROS Topic and reflect it in driving control

c. **Intersection Sign Switching** 
Plug-in (C++) switches left/right signs to time intervals

Execute swivel logic based on sign recognition results

d. **Situation-specific driving control** 
- Situation Control Behavior
- Speed bump deceleration (10 seconds)
- Decelerating to Children's Reservation (10 seconds)
- Stop crosswalk + pedestrian recognition (10 seconds)
- Speed limit 100 sign acceleration
- Speed limit 30 sign deceleration

<img width="488" alt="image" src="https://github.com/user-attachments/assets/2c53b387-ba52-469b-be00-a80ae8998c02" />


e. **GUI(Using Tkinter)**
- Configure the GUI using launcher_all.py, video.py
_ Launch files in batches
- Check and control camera images and robot status
- Configured to make it easier for users to control and verify in a hands-on environment.



## 3. Dynamic lane detection

a. **Lane Masking Algorithm** 
- Color-based HSV Masks
1) Detection of white and yellow lanes in maskWhiteLane(), maskYellowLane() functions, respectively
2) Automatic lower limit calibration based on brightness (V-channel) mean value
White: 1.46 * Brightness + 20
Yellow: Using a linear calibration function
3) Introduction of a gradual correction method to respond to the change in illuminance (adjusting ±10 per frame)

b. **Reliability-Based Centerline Detection**
- Score lane recognition reliability based on effective number of pixels and reliability (recent detection rate)
- Branch from the makeLine() function into four centerline calculation schemes:
1) Both lanes detected (straight line): Calculate centerline based on mean value
2) Only one lane detected: relative position correction (±230px) virtual centerline creation
3) Curves larger: Create center line based on reliable lane
4) Lane detection failure or low reliability: Lane tracking failure (lane_state = 0)

c. **Curvature"**
- Using numpy's np.polyfit(), the lane coordinates extracted from the mask image are approximated by a quadratic polynomial.
- Use the radius of curvature formula:
<img width="214" alt="image" src="https://github.com/user-attachments/assets/1797631d-1c2c-47a0-970f-4993b403f4d9" />

y=599: Image bottom reference position
If the radius is less than 6000px, it is judged as a curve → Reflected when calibrating the center line

## 4. Parking Features
- Car entry conditions: When parking signs are recognized → vehicle alignment along yellow lanes (left_mode entry)
- Entering parking mode:
1) When white dotted line is detected → park mode transition
2) Go straight for 2 seconds → 180 degrees rotation → Stop (parked)
-  Detailed Implementation Methods
After recognizing parking signs with YOLO, set the bounding box area to 600 to ensure stable detection.
1) Park sign detection → callback function to relevant logic
2) Conditions for decreasing the fraction of the yellow line and increasing the fraction of the white line → Determining the path change
3) Automatically straight forward, rotate, and align in conjunction with dynamic lane recognition

## 5. Pedestrian model implementation
- Implementation of walkers by supplementing existing models
- Adjusting physical properties such as mass, coefficient of friction, and inertia ensures stable motion
- Utilize gazebo_ros_planar_move plugins
  
## 6. Demo
https://drive.google.com/file/d/1zM899omEa5Z9UyE7TN0GfyBFQqVfpBf2/view?usp=drive_link
