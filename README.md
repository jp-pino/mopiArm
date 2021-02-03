# mopiArm
Robot arm controller project.

In `ros-robot-arm-controller` you'll find the ROS project and docker-compose files to run the project on Docker. Make sure to configure your usb inputs correctly. To make this easier I'm using docker-machine to connect the usb device to the created virtualbox image (in my setup I named it 'local'). Port forwarding is also required to connect to webviz and rosbridge. All of these steps are performed by the `start` script.

In `proj` you'll find the mopiOS project. It simply sets up the PWM hardware modules in the TM4C123GH6PM microcontroller to output servo compatible signals, and subscribes to /s1, /s2, and /s3 topics to receive the servo angles. Follow the guide on the `mopiOS` project to set this up correctly.


