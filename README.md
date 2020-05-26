# Set multiple set_points

## To start the simulations go to the folder where you have cloned Firmware

`cd src/Firmware`

## Bringup the quadcopter
`make px4_sitl_default gazebo_iris`
## This will open the iris quadcopter model in Gazebo

## On another tab start MAVROS node

computer IP addres(it can be found by 'hostaname -I' in terminal)

`roslaunch mavros px4.launch fcu_url:="udp://:14540@10.122.247.129:14557"`

OR

Local IP addres

`roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557`

## On another tab run offbd_node

`rosrun mavR offb_node`

