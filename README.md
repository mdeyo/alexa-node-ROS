# alexa-node-ROS
Connecting Alexa Skills to node.js to ROS. 

Current capabilities include publishing to a topic and making service calls in ROS.

### Installing Depedencies

After [installing Node.js](https://nodejs.org/en/), you will need to install the alexa-app package through npm:

``
npm install alexa-app
``

Additional missing packages can be installed in the same way. Depending on the install, you might need:
* hotswap
* alexa-app
* express
* body-parser
* bluebird
* ejs

In addition to [installing ROS](http://wiki.ros.org/ROS/Installation) (any desktop-full, desktop, or ros-base will work), you will need to install rosbridge-server:

``
sudo apt-get install ros-<distro>-rosbridge-server
``

where distro = {hydro, indigo, jade, or kinetic...}, for example:

``
sudo apt-get install ros-kinetic-rosbridge-server
``

### Running the Example

To begin, we will launch ROS. To do so, run the following in a terminal: 

``
roscore
``

Then we can launch the rosbridge v2.0 server with the following:

``
roslaunch rosbridge_server rosbridge_websocket.launch
``

To start the Alexa-app-server, navigate into the /examples directory and run:

``
sudo node server.js
``

### Testing the skills

...



