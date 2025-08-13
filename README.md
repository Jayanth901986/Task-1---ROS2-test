1.create a directory 
 - mkdir ros12_ws
 - cd /ros12_ws
 - mkdir src
2. creating a package : cd /ros12_ws/src
   -ros2 pkg create --build-type ament_python image_conversion_pkg
3. Nodes
   -cd ros12_ws/src/image_conversion_pkg/image_conversion_pkg
    -touch or gedit image_conversion_node.py or (you can directly use visual studio by adding the file)
    -g dummy_usb_cam.py
4. edit setup.py
   - entry_points={
    'console_scripts': [
        'image_conversion_node = image_conversion_pkg.image_conversion_node:main',
        'dummy_usb_cam = image_conversion_pkg.dummy_usb_cam:main',
    ],
},
5.edit package.xml
 -adding executable packages used in the code
    -<exec_depend></exec_depend>        - 
