#!/usr/bin/env node

// Connecting to ROS
var ROSLIB = require('roslib');

var ros = new ROSLIB.Ros({
    url: 'ws://localhost:9090'
});

ros.on('connection', function() {
    console.log('Connected to websocket server.');
});

ros.on('error', function(error) {
    console.log('Error connecting to websocket server: ', error);
});

ros.on('close', function() {
    console.log('Connection to websocket server closed.');
});

// Publishing a Topic
// ------------------
/*
var cmdVel = new ROSLIB.Topic({
    ros: ros,
    name: '/cmd_vel',
    messageType: 'geometry_msgs/Twist'
});

var twist = new ROSLIB.Message({
    linear: {
        x: 0.1,
        y: 0.2,
        z: 0.3
    },
    angular: {
        x: -0.1,
        y: -0.2,
        z: -0.3
    }
});

console.log("Publishing cmd_vel");
cmdVel.publish(twist);
*/
// First, we create a Service client with details of the service's name and service type.
var openHandClient = new ROSLIB.Service({
    ros: ros,
    name: '/wam_loki/open',
    serviceType: 'std_srvs/Empty'
});
// Then we create a Service Request. The object we pass in to ROSLIB.ServiceRequest matches the
// fields defined in the rospy_tutorials AddTwoInts.srv file.
// Empty request object for open and close hand
var request = new ROSLIB.ServiceRequest({});
// Finally, we call the /add_two_ints service and get back the results in the callback. The result
// is a ROSLIB.ServiceResponse object.
openHandClient.callService(request, function(result) {
    console.log('Result for service call on ' + openHandClient.name + ': ' + result);
});
