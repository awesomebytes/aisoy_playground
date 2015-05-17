Package with Aisoy related scripts.

===

To work with these scripts you are supposed to run them with your `ROS_MASTER_URI` environment variable pointing to your robot. E.g.:

    export ROS_MASTER_URI=http://AISOY.CURRENT.IP.ADDRESS:11311

You can then run the nodes with:

    rosrun aisoy_playground the_node_you_want_to_run

===
You'll find the scripts in the `scripts` folder:

* `servos_dyn_rec.py` contains a dynamic reconfigure server to move the robot's servos (head pan, eyelids and eyebrows).

Use it with rqt_reconfigure:

    rosrun rqt_reconfigure rqt_reconfigure

Choose `servos_dyn_reconf` and play with the scroll bars.

[![Image of the dynamic reconfigure window](https://raw.githubusercontent.com/awesomebytes/aisoy_playground/master/resources/dynamic_reconf_ss.png)](https://raw.githubusercontent.com/awesomebytes/aisoy_playground/master/resources/dynamic_reconf_ss.png)


