# Patrolling and events

Just some info on the basic setup.
For demo see see documentation [here](demovideo.webm)

## Assumption on runtime conditions

The project is launched in an environment surrounded by walls.
The lines should be vertical and horizontal.

## Running the application

Go to the root of this project and run :

~~~bash
$ ./build-and-launch-packackage.sh prep_lidar
~~~

In case you want to run the code on a real robot you can
pass the ROS_DOMAIN_ID as second command like the example below

~~~bash
$ ./build-and-launch-packackage.sh prep_lidar 102
~~~

## Sending commands

4 **commands** are possible to send over the topic "/patrolCommands"  

* start
* stop
* startDebug
* stopDebug

This can be done through invoking ros2-commands

~~~bash
ros2 topic pub --once /patrolCommands example_interfaces/msg/String "data: start"
~~~

Or you can use the script at the root of the project

~~~bash
./patrol-send-command.sh start 
~~~

## Reading out events

You can read the events through the /patrolEvents-topic:

~~~bash
ros2 topic echo /patrolEvents
~~~

Resulting in an overview on the events...

~~~
...
data: Turning right at (1.6846610027081856, -1.233444206053081)
---
data: Turning right at (1.7089582355761146, -2.2408965055138967)
---
data: U-turn at (-0.02995405821040239, -2.412341196699487)
---
data: U-turn at (4.151668894181787, -2.4729578420003318)
---
data: Turning right at (3.687638403541439, -2.441237709384071)
---
data: Turning right at (3.6666498799422866, -0.10131235632269432)
---
data: U-turn at (4.148468225339249, -0.007889226047371209)
---
data: U-turn at (-0.05226810242569075, -0.09012889895965459)
---
data: Turning right at (1.7180736134147048, 0.11231550131657037)
---
data: Turning right at (1.7689283578648776, -1.1328140332115164)
---
data: U-turn at (-0.05606523587388091, -1.2502394827588033)
...
~~~

Or you can use the script at the root of the project

~~~bash
./patrol-listen-to-events.sh
~~~