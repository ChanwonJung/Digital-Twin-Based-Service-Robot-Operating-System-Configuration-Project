# Simulation of digital twin-based autonomous driving robot system using gazebo11


## 1. Project Overview
<img width="320" alt="image" src="https://github.com/user-attachments/assets/8fc3cbfc-2622-4c65-b8b1-3340cc24b27c" />


- The purpose of this project is to simulate the actual road environment through digital twin-based simulation and to practice the intelligent driving control technology of autonomous robots.
- Key autonomous driving technologies such as object recognition, path control, and speed control were implemented using Gazebo simulators and TurtleBot3, ROS2, and YOLOv11m.
- The map utilized the robotis-emanual autorace provided by the existing 'Robotis'.
https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/#autonomous-driving

## 2.  Implementation Technology and Feature

a. **Object Recognition** 
YOLOv11-based implementation of recognition of various objects such as signs, traffic lights, pedestrians, etc.

Improved speed and real-time performance over traditional SIFT methods.

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



### **2) detect_wound node**
   
a. **Expand the surgical incision**: After the manipulator moves to the position where the surgery is taking place, the object at the surgical site ('ex) camera') is recognized through the keyword ('ex) camera').

If it is recognized, it is possible to get a little closer to the area and check the incision that is being operated on the monitor (rqt).
Afterwards, while checking the incision site, when external force is detected through **adaptive control**, the surgery is recognized to be completed and positioned as initial coordinates.


b. **Inhalation of blood using suction**: Using STT(Using Whisper API), inhale blood around the recognized surgical site ('ex) suction, Yankeeur) through keywords.




## 2. Utilization equipment and development environment
- Doosan Manipulator M0609
- ROS2 humble ubuntu 22.04
- OnRobot2 Gripper
- STT (Using Whisper API)
- Surgical Tools: Scalpel, Mayo_metz, Forcep, Hemostat
- Realsense camera
- YOLOv11n

<img width="463" alt="image" src="https://github.com/user-attachments/assets/46f17870-5819-4ee0-8290-3efa13d07690" />






## 3. Project progress
### **- Surgical Tools dataset**
<img width="490" alt="image" src="https://github.com/user-attachments/assets/25375e33-d360-478c-bd98-fa6b3529f29c" />

https://universe.roboflow.com/northeastern-university-ftufl/sgtd

1) Classes label

a. Mayo_metz: Scissors for skin and tissue incision

b. Forceps: Medical tweezers

c. Scalpel: scalpel, small blade

d. Hemostat: arterial clamps, hemostatic devices


2) Use to recognize surgical instruments
   
![image](https://github.com/user-attachments/assets/b49ba1c8-c90a-4586-92c9-239b3509453f)

The model used yolov11n and used epoch=100, img_size=512, and batch=20 (auto) to confirm high-performance object recognition as shown in the picture above.

![image](https://github.com/user-attachments/assets/45044496-10bc-495b-9516-b2bae43d4192)

It was confirmed that the overall loss value decreased as learning progressed, and the precision and recall values exceeded 90%.

### **- Hands dataset**
![image](https://github.com/user-attachments/assets/b0b3b55f-2412-42be-a2d7-b4491cf8a5b2)

https://universe.roboflow.com/hyfyolo/new-hand 

1) Classes used: Hands

2) Use it for hand recognition when gripper picks up and brings the medical device by hand after recognizing it


### **- Surgical Wounds dataset**
![image](https://github.com/user-attachments/assets/59f4c4b4-3c2b-419e-9fbf-9a1801e467da)

https://universe.roboflow.com/myworkspace-zgags/my-first-project-d3ifu/browse?queryText=&pageSize=50&startingIndex=0&browseQuery=true


1) Classes label

a. Stitched

b. Wound


2) Use for incision recognition and suture functions

### -  Flowchart of performance
<img width="494" alt="image" src="https://github.com/user-attachments/assets/d55f09fa-5043-4a30-83af-a6839c123c15" />
<img width="420" alt="image" src="https://github.com/user-attachments/assets/8815abcc-86f5-4730-a8a3-8c9e0ff3b8df" />


<Robot_control algorithm flow chart>

Patient anesthesia sleep progression -> Run wakeword.py via Hello, route -> Use STT to recognize medical device keywords -> Recognize hand objects after holding them -> Deliver medical device to hand when recognized -> Check the presence or absence of medical device put down by medical staff after use -> Return to initial coordinates

<Detect_wound flow chart of the section node>

Hello, run wakeword.py via rokey -> recognize surgical incision keywords using STT -> enlarge surgical site after surgical incision recognition -> return to initial coordinate through compliance control




## 4. Errors experienced during the project
a. The object recognition rate by light reflection has decreased. It is sensitive to the environment.

b. As STT, promports were set to a number of various cases to improve Whisper API performance, but voice recognition was nevertheless not possible.

c. There are a total of 15 classes of data labels, but due to the situation, only 4 classes were used. (Mayo_metz, Forceps, Hemostat, Scalpel)



## 5. Demo

https://drive.google.com/file/d/1ya3sM34Hc5CPN4aMMyc6QHAeQYTwBzs0/view?usp=sharing
