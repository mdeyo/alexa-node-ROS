#!/usr/bin/env node

var alexa = require('alexa-app');

//// Alexa App functions ////

// Allow this module to be reloaded by hotswap when changed
module.change_code = 1;

// Define an alexa-app with name that matches name on Alexa Skills Kit
var app = new alexa.app('wam-hand');

app.launch(function(req, res) {
    console.log("- launch called -");
    res.say("Hello wam world");
});

app.intent("OpenHandIntent", {
    "utterances": ["Open the hand"]
}, function(req, res) {
    console.log("opening hand...");
    openHandClient.callService(request, function(result) {
        console.log('Result for service call on ' + closeHandClient.name + ': ' + result);
    });
    res.say('Opening hand');
});

app.intent("CloseHandIntent", {
    "utterances": ["Close the hand"]
}, function(req, res) {
    console.log("closing hand...");
    closeHandClient.callService(request, function(result) {
        console.log('Result for service call on ' + closeHandClient.name + ': ' + result);
    });
    res.say('Closing hand');
});

// Connecting to ROS
var ROSLIB = require('roslib');

var prompt = require('prompt');
prompt.start();

function promptForAction() {
    prompt.get(['action'], function(err, result) {
        //
        // Log the results.
        //
        console.log('Command-line input received:');
        // console.log(' action: ' + result.action);
        if (result.action == "close") {
            console.log("closing hand...");
            closeHandClient.callService(request, function(result) {
                console.log('Result for service call on ' + closeHandClient.name + ': ' + result);
            });
        }
        if (result.action == "open") {
            console.log("opening hand...");
            openHandClient.callService(request, function(result) {
                console.log('Result for service call on ' + openHandClient.name + ': ' + result);
            });
        }
        promptForAction();

    });

}


var ros = new ROSLIB.Ros({
    url: 'ws://localhost:9090'
});

ros.on('connection', function() {
    console.log('Connected to websocket server.');
    promptForAction();
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
var arm_name = "wam";
var openHandClient = new ROSLIB.Service({
    ros: ros,
    name: '/' + arm_name + '/open',
    serviceType: 'std_srvs/Empty'
});
var closeHandClient = new ROSLIB.Service({
    ros: ros,
    name: '/' + arm_name + '/close',
    serviceType: 'std_srvs/Empty'
});
// Then we create a Service Request. The object we pass in to ROSLIB.ServiceRequest matches the
// fields defined in the rospy_tutorials AddTwoInts.srv file.
// Empty request object for open and close hand
var request = new ROSLIB.ServiceRequest({});
// Finally, we call the /add_two_ints service and get back the results in the callback. The result
// is a ROSLIB.ServiceResponse object.
// openHandClient.callService(request, function(result) {
//     console.log('Result for service call on ' + openHandClient.name + ': ' + result);
// });
