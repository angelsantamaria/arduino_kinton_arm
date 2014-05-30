# README #

This code is designed to work with the 6DOF light-weight arm attached below the Kinton quadrotor. It is ready to work with almost all arduino models with at least 5 PWM outputs and it has implemented a data protocol over serial communication with the on-board pc.

This driver works together with the C++ low-level driver installed in the embedded pc. You can found more information and the C++ code in the IRI repository: https://devel.iri.upc.edu/pub/labrobotica/drivers/kinton_arm/trunk.

In the same way, this C++ low-level driver is prepared to work with a high-level ROS node in order to command the arm joints using ROS. The ROS node is part of the ROS iri-ros-pkg: http://wiki.ros.org/kinton_arm_node


### Who do I talk to? ###

If you need more information about the code implementation or the hardware requirements, please do not hesitate to contact me.

* Àngel Santamaria-Navarro somriu@gmail.com