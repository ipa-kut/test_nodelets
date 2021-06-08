# test_nodelets
Just playing around with nodelets

## test_nodelets/DiffTimeNodelet

Startup: `roslaunch test_nodelets test_diff_time.launch`

* Has multithreading example
    * One thread saves current time into private member, another reads it out via mutexing
* Has internal/external pub/sub example
    * Creates two nodelets of same type
    * User pubs int value to /in topic of one nodelet
    * This value is then published internally via zero copy memory transfer to the other nodelet which then prints it out
* Has thread manipulation example
    * Publishing 0 to external subscriber (/inx) will stop the thread that saves time, and publishing 1 starts it again

## Resources

* [ROS Nodelet wiki](http://wiki.ros.org/nodelet) -> Just an overview
* [ROS Nodelet tutorials](http://wiki.ros.org/nodelet/Tutorials/Porting%20nodes%20to%20nodelets) -> uselesse except for configs
* [Packt hub](https://hub.packtpub.com/working-pluginlib-nodelets-and-gazebo-plugins/) -> Good overview and simple code example
* [Kobuki nodelets](http://wiki.ros.org/kobuki/Tutorials/Write%20your%20own%20controller%20for%20Kobuki) -> Taken most parts from here
* [Xlearpath robotics](https://www.clearpathrobotics.com/assets/guides/kinetic/ros/Nodelet%20Everything.html) -> Best example of multithreading

