# momentum-flight

### Project Summary ###

The Lockheed Martin Challenge tasked teams to design a contour flight algorithm for autonomous drone flight using Python, along with Gazebo flight simulation software and PX4 Autopilot. Lockheed Martin provided files which can be found [here](https://github.com/katabeta/lm-mit-momentum).

The goal was an algorithm with a three meter AGL (above ground level) accuracy and a short course completion time.

### Approaches ###

Above, we have a simple barebones approach along with a final reactive/proactive hybrid approach. 

The barebones approach simply flies at the maximum altitude. 

The reactive/proactive hybrid approach reacts and anticipates terrain. It achieves a three meter AGL with only 50% more time than the barebones and also removes the neccessity of an AGL and time tracker plugin by providing similar measurements.

### Hybrid Flight Demonstration ###

![](flight.gif)
