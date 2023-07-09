# ifollow_artag_navigation

There are two main tasks implemented in this package.

Task 1: Controlling or commanding the turtlebot3 through a multimodal control channel. This section covers the Qestion 3 from Test_recrutement.

There are two modes of the control, one is coming from web "cmd_web" and local onboard or manual cmd "cmd_local".
cmd_web must be coming from a remote MQTT client. 

Pkg: turtlebot3_multicontroller

Install MQTT server on the dev computer. Mosquito MQTT server has been chosen for this purpose.

How to launch:

In terminal 1: Launch the turtle bot. Don't forget to export the env variable for the turtle bot model. 

    $ export TURTLEBOT3_MODEL=burger
    $ roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch

In terminal 2: Launch mqtt_multiplexer_ros.launch

    roslaunch turtlebot3_multicontroller mqtt_multiplexer_ros.launch

Now put the cursor on the terminal  2, and play with the arrows buttons on the keyboard.

Process:

Non-ros side implementation

    main file: mqtt_pub.py:

    location: turtlebot3_multicontroller --> scripts --> remote_keyboard --> mqtt_pub.py

    function: Keyboard press (arrow buttons) are taken as inputs for generating cmd velocities. It will then a generate a mqtt topic.


    The arrow keys funcs:

    arrow-TOP   : Increases the forward X axis speed of the rover (forward motion).
    arrow-DOWN  : Decreases the forward X axis speed of the rover (backward motion).
    arrow-LEFT  : Increases the anti-clockwise angular speed the rover (anti-clockwise rotation).
    arrow-RIGHT : Decreases the forward X axis speed of the rover (anti-clockwise rotation).

Ros side implementation

    mqtt_listener.py: it's a ros node which will subscribe to the mqtt topic published by mqtt_pub.py and generate a ros topic "cmd_web"

    multiplexer.py: It's a pure ros node. It subscribes to two cmd_vel ros topics, one is "cmd_web" and the other is "cmd_local".
    The job of this node is to generate a final cmd_vel, "cmd_vel" which will be given to the robot to move. The node chooses the final cmd_vel 
    based on the availability of the inputs and the priority of the inputs. 

    The cmd_web has high priority and if the cmd_web is publishing, the final cmd_vel will use this.
    If it's not available (disconnected and not publishing) then cmd_local will take over.

    TODO: A user can chose the type of input or a some combination of the two cmd_vel.

  

Task 2: Implement a navigation stack pipeline using apriltags. This section covers the Q4 and Q5(Bonus).

pkg: artag_nav_commander

Dependency: apriltag, apriltag_ros

The pkg is a kind of ros wrapper implemented in C++ which combines two functionalities together, reading an April tag using the apriltag library and calling a 
movebase client (from ros nav stack)  to send a goal to the nav stack.

How to launch:

In terminal 1: Launch the turtle bot. Don't forget to export the env variable for the turtle bot model. 

    $ export TURTLEBOT3_MODEL=burger
    $ roslaunch turtlebot3_gazebo turtlebot3_world.launch

In terminal 2: Launch the navigation server of the turtle bot.

    $ export TURTLEBOT3_MODEL=burger
    $ roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=<map_file_location>/map.yaml

An empty map is created just for the test here turtlebot_multicontroller --> maps -->empty_map.yaml

In terminal 3: Launch the artag_nav_commander using roslaunch artag_nav_commander single_image_ar_commander.launch

    $ roslaunch artag_nav_commander single_image_ar_commander.launch


Process:

    Artags collection:
        print the artags sample given.
        print a checker board patter for camera calibration
    
    camera calibration:
        Use a camera calibration pkg with 8x6 checker board pattern to retrieve camera intrinsics of computer's camera.
        These intrinsics will be needed as params in the artag_nav_commander pkg (see params fields in single_image_ar_commander.launch).
    
    Launch nav commander:

    For a static single image, use single_image_ar_commander.launch. 
    The node will first load the apriltagimage from a location. Run a tag_detector from the apriltag library. 
    Movebase client and a corresponding goal will be declared. The pose from tag_detector is extracted and converted from camera to base_link coordinates.
    The movebase client is then send the goal to the nav server.

    You can see that the rover will move to a destination position in the gazebo simulation.


Task3: Take picture from a live camera, detect april tag if present and then send nav goal to the robot. It covers the Q6(Bonus)

    It follows the development from the previous step of april tag detection from a static image.
    Main file: livecam_nav_commander.launch.

    This will launch a node which will take an instance of the image frame from a running live web cam by a trigger Service. Then detect if there is an April tag. If there is one, then it will invoke a move_base client to move the bot. This is implemented and tested. 

    How to launch:

    In terminal 1: launch a webcam 

    $ roslaunch usb_cam usb_cam-test.launch video_device:=/dev/video0

    (Now launch the turtle bot world sand navigation as in Tasks2)

    In terminal 2: Launch the turtle bot. Don't forget to export the env variable for the turtle bot model. 

    $ export TURTLEBOT3_MODEL=burger
    $ roslaunch turtlebot3_gazebo turtlebot3_world.launch

    In terminal 3: Launch the navigation server of the turtle bot.

    $ export TURTLEBOT3_MODEL=burger
    $ roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=<map_file_location>/map.yaml

    NOW launch the livewebcam_nav_commander

    $ roslaunch artag_nav_commander livecam_ar_detection.launch

    Now send a trigger service call to look for April tags.

    $ rosservice call /livecam_tag_detection "{}"

    Visualize the motion of the rover on rviz or gazebo world window.

    or 
    Launch rqt to see the colored bordered of the detected tag.



