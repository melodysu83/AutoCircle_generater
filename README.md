# AutoCircle_generater

This software is a ROS node that generates RAVEN motion commands for it to follow a predefined circle trajectory. The circle RADIUS and orbitting speed are tunable. There are 2 ROS nodes in this folder: talkerAutoCircle and listenerAutoCircle. 


## ROS topics :
The two ROS nodes talkerAutoCircle and listenerAutoCircle exchange information through ROS topics. Below are the two ROS topics that this software use in order to communicate with the main RAVEN software. This is the [link](https://github.com/melodysu83/AutoCircle_generater/tree/master/msg) to the .msg files.

1. **raven_state.msg** : This topic stores current RAVEN position and orientation values. 
2. **raven_automove.msg** : This topic stores the motion command for RAVEN to move accordingly.


## ROS nodes :
1. **talkerAutoCircle** : This ROS node is the command generator. It subscribes to raven_state.msg ROS topic to get current RAVEN status, then computes corresponding motion commands for the robot arm to follow circle trajectory. Finally, it publishes to raven_automove.msg ROS topic.
2. **listenerAutoCircle** : This ROS node is only for testing. It serves the same purpose as the main RAVEN software, and will be replaced during actual use. It communicates with the talkerAutoCircle by listening to the motion commands and sends out RAVEN position and orientation feedback after following the command. Thus, it subscribes to raven_automove.msg ROS topic, and updates RAVEN position and orientation accordingly, then publishes raven_state.msg ROS topic to show how the robot arm is currently posed.


## Files : 
This is the big picture of what files are in this AutoCircle_generater repository and what are each files are for. Note that only the ones specified as (original) are the original files created here, others are copied from the main RAVEN software and do NOT need to copy again to the main RAVEN software when combining. 

**/msg folder:**

----**raven_automove.msg**

----**raven_state.msg**
    
**/src folder:**

----**/raven_2 folder:**

--------**raven_automove.h**

--------**raven_state.h**

----**DS0.h**

----**DS1.h**

----**Raven_Controller.cpp** --------------------- (original)

----**Raven_Controller.h** ----------------------- (original)This class controls the threads and workflow.

----**Raven_PathPlanner.cpp** -------------------- (original)

----**Raven_PathPlanner.h** ---------------------- (original)This class defines all the math and path planning.

----**listener.cpp** ----------------------------- (original)This will be replaced with main RAVEN software.

----**talker.cpp** ------------------------------- (original)This file is where main is.

----**tools.h**
    
**AutoCircle Generator flowchart.png** ----------- (original)The ROS publish/subscribe flowchart for better understanding.

**CMakeLists.txt** ------------------------------- (original)Should merge with CMakeLists.txt in main RAVEN software.

**README.md** ------------------------------------ (original)This file!

**package.xml** ---------------------------------- (original)The ROS package.xml file.

The file talker.cpp is the heart of AutoCircle generater, it is where the main is. This file uses on methods defined in class Raven_Controller. Inside of class Raven_Contoller, there are two threads - ros_thread and console_thread, which takes charge of the ROS publishing/subscribing issues and user console inputs respectively. The class Raven_Contoller depends on class Raven_PathPlanner to compute and design circular trajectories. So, it is basically where all the math is! In the class Raven_Controller, there are two Raven_PathPlanner objects managing the motion of LEFT and RIGHT arm of RAVEN. (All these files belong to the talkerAutoCircle ROS node.)
And since we are NOT actually connected to the main RAVEN software yet, we have listener.cpp as the simplified version to simulate the main RAVEN software just so talker.cpp has someone to interact with. Thus, listener.cpp will be completely replaced when we actually combine it with the RAVEN code. (This file belongs to the listenerAutoCircle ROS node.)


## Spec : 
These are the constraints we set for our physical device - RAVEN surgical robot arm to function normally. These are mostly defined in [PathPlanner.h](https://github.com/melodysu83/AutoCircle_generater/blob/master/src/Raven_PathPlanner.h) file under /src folder. Be careful when tuning these values!

1. **publish rate** : The raven_automove.msg is being sent at 1000 Hz.

2. **feedback rate** : The raven_state.msg is being sent at 100 Hz in listener.cpp. But in actual RAVEN software, raven_state.msg is updated at 1000 Hz.

3. **DEL_POS_THRESHOLD** : This is the motion translation threshold for RAVEN to move. It is set as 180 micro meter (=0.18 mm = 0.018 cm). That being said, the maximum speed that RAVEN will be moving is 1000 Hz * 0.018 cm = 18 (cm/sec).

4. **DEL_ROT_THRESHOLD** : This is the motion rotation threshold for RAVEN to move. It is set to be 0.25 degrees. That being said, the maximum rotational speed for RAVEN will be is 1000 Hz * 0.25 degrees = 250 (degrees/sec). This is currently unused because the circle trajectoy we have now does NOT include orientation motion.

5. **RADIUS levels** : There are ten levels of RADIUS to choose from. Level 1 ~ 6 corresponds to 3000 micro meter (=3 mm = 0.3cm) ~ 100000 micro meter (=18 mm = 1.8cm). Yet that is the desired radius. From our experiments, the relation between desired radius and actual radius are listed as follows:
radius level  | desired radius | actual radius | max speed level allowed
------------ | ------------- | ------------- | -------------
1 | 0.3cm | 1.2 ~ 1.3 cm | 50
2 | 0.6cm | 1.4 ~ 1.5 cm | 50
3 | 0.9cm | 1.6 ~ 1.7 cm | 60
4 | 1.2cm | 1.8 ~ 1.9 cm | 60
5 | 1.5cm | 2.0 ~ 2.1 cm | 60
6 | 1.8cm | 2.2 ~ 2.4 cm | 60

6. **SPEED levels** : There are sixty levels of SPEED to choose from, with each level increasing 0.3 cm/sec from the previous level. So level 1 ~ 60 corresponds to moving 0.3 cm/sec (= 3mm/sec = 0.003 mm per command) all the way up to 18 cm/sec (= 180mm/sec = 0.18 mm per command, which is exactly DEL_POS_THRESHOLD).


## Relative links:
1. **uw-biorobotics/raven2** : This is the main RAVEN software that this AutoCircle generater software is going to connect to. And [this code](https://github.com/uw-biorobotics/raven2) will replace the ROS node listenerAutoCircle that we temporarily have for now.
2. **raven_absolute_controller** : This is the extended work for RAVEN absolute position control done by Imperial College London. We were originally intended to implement the AutoCircle generater based on their work. But since we want it to be a ROS node instead of teleoperating and sending UDP packages to communicate, we eventually didn't apply [their code](https://github.com/Takskal/raven_absolute_controller). 


