# test_nodelets
Just playing around with nodelets

## test_nodelets/DiffTimeNodelet

> WARNING: DOES NOT ACTUALLY DO DIFF TIME, ENDED UP BECOMING A NODELET EXPERIMENTATION TARGET

Startup: `roslaunch test_nodelets test_diff_time.launch`

* Has multithreading example
    * One thread saves current time into private member, another reads it out via mutexing
* Has internal/external pub/sub example
    * Creates two nodelets of same type
    * User pubs int value to /in topic of one nodelet
    * This value is then published internally via zero copy memory transfer to the other nodelet which then prints it out
* Has thread manipulation example
    * Publishing 0 to external subscriber (/inx) will stop the thread that saves time, and publishing 1 starts it again

## test_nodelets/CompParamNodelet

Compares an incoming float value against a parameter float value, and reports true when nearly equal based on threshold

Startup: `roslaunch test_nodelets test_comp_param.launch`

* Has two params:
    * `param` - Float - Value to compare against
    * `tolerance` - Float - Threshold of true comparison window
* Has two inputs:
    * `control` - Bool - When true, calculation is enabled
    * `float_val` - Float - Value to compare against param
* Has one output:
    * `result` - Bool - True when calculation result is true

## Resources

* [ROS Nodelet wiki](http://wiki.ros.org/nodelet) -> Just an overview
* [ROS Nodelet tutorials](http://wiki.ros.org/nodelet/Tutorials/Porting%20nodes%20to%20nodelets) -> uselesse except for configs
* [Packt hub](https://hub.packtpub.com/working-pluginlib-nodelets-and-gazebo-plugins/) -> Good overview and simple code example
* [Kobuki nodelets](http://wiki.ros.org/kobuki/Tutorials/Write%20your%20own%20controller%20for%20Kobuki) -> Taken most parts from here
* [Xlearpath robotics](https://www.clearpathrobotics.com/assets/guides/kinetic/ros/Nodelet%20Everything.html) -> Best example of multithreading

