1.create a directory 
 - mkdir ros21_ws
 - cd /ros21_ws
 - mkdir src
2. creating a package : cd /ros12_ws/src
   -ros2 pkg create --build-type ament_python image_conversion
   -ros2 pkg create --build-type ament_python dummy_usb_cam
3. Nodes
   -cd ros21_ws/src/image_conversion_pkg/image_conversion
    -touch or gedit image_conversion.py or (you can directly use visual studio by adding the file)
    -cd ros21_ws/src/dummy_usb_cam/dummy_usb_cam
    -dummy_usb_cam.py
4. launch file in image_conversion
   -image_conversion_with_dummy.launch.py
5. edit setup.py (for both packages)
   - entry_points={
         'console_scripts': [
             'node_name = pkg_name.node_name:main',
          ],
     },
6.edit package.xml (for both packages)
 -adding executable packages used in the code
    -<exec_depend></exec_depend>        -
7. build the workspace
   -colcon build
  source the workspace
   - . install/setup.bash or source install/setup.bash
8. run
  -ros2 launch image_conversion image_conversion_with_dummy.launch.py file_path:=/home/jayanth/photos/test_image.jpg
  -ros2 launch image_conversion image_conversion_with_dummy.launch.py file_path:=/home/jayanth/photos/test_video.mp4
  -ros2 topic list
     (shows the topics
        -/image_raw
        -/image_converted)
  -open a new terminaland source
     -you can change the mode 
    ros2 service call /change_mode std_srvs/srv/SetBool "{data: true}"   # grayscale
    ros2 service call /change_mode std_srvs/srv/SetBool "{data: false}"  # color
  - visualize the image in rviz2
     -go to display and add a topic then you can see the image color & grayscale


