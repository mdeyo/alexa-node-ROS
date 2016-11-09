# alexa-node-ROS
Connecting Alexa Skills to node.js to publish or call services in ROS

### Running the Example
At this point we are ready to run the example. To do so, you will need to have installed the following packages:

ros-distro-ros-base (basic ROS installation) <br>
ros-distro-rosbridge-server (rosbridge_server) <br>
where distro = hydro or kinetic

To begin, we will launch ROS. To do so, run the following in a terminal: 

``
roscore
``

Then we can launch the rosbridge v2.0 server with the following:

``
roslaunch rosbridge_server rosbridge_websocket.launch
``

Finally, you are now ready to run the node.js example with:

``
node node_simple.js
``
