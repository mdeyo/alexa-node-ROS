#!/usr/bin/env node

var alexa = require('alexa-app');

//// Alexa App functions ////

// Allow this module to be reloaded by hotswap when changed
module.change_code = 1;

// Define an alexa-app with name that matches name on Alexa Skills Kit
var app = new alexa.app('wam-hand');

app.launch(function(req, res) {
  console.log("wam-hand: LaunchIntent");
  res.say("Hello wam world");
});

app.intent("OpenHandIntent", {
  "utterances": ["Open the hand"]
}, function(req, res) {
  console.log("wam-hand:  opening hand...");
  openHandClient.callService(request, function(result) {
    console.log('wam-hand:  Result for service call on ' + closeHandClient.name + ': ' + result);
  });
  res.say('Opening hand');
});

app.intent("CloseHandIntent", {
  "utterances": ["Close the hand"]
}, function(req, res) {
  console.log("wam-hand: closing hand...");
  closeHandClient.callService(request, function(result) {
    console.log('wam-hand:  Result for service call on ' + closeHandClient.name + ': ' + result);
  });
  res.say('Closing hand');
});

app.intent("PublishTwistIntent", {
  "utterances": ["Publish twist", "send twist", "command twist"]
}, function(req, res) {
  console.log("wam-hand: publishing twist cmd...");

  // Publishing a Topic
  // ------------------

  var cmdVel = new ROSLIB.Topic({ros: ros, name: '/cmd_vel', messageType: 'geometry_msgs/Twist'});

  var twist = new ROSLIB.Message({
    linear: {
      x: 0.1,
      y: 0.0,
      z: 0.0
    },
    angular: {
      x: 0.0,
      y: 0.0,
      z: 0.1
    }
  });

  cmdVel.publish(twist);
  console.log('wam-hand: done publishing twist');

});

// Connecting to ROS
var ROSLIB = require('roslib');

var ros = new ROSLIB.Ros({url: 'ws://localhost:9090'});

ros.on('connection', function() {
  console.log('wam-hand: Connected to websocket server.');
});

ros.on('error', function(error) {
  console.log('wam-hand: Error connecting to websocket server: ', error);
});

ros.on('close', function() {
  console.log('wam-hand: Connection to websocket server closed.');
});

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
// fields defined in the srv file.

// Empty request object for open and close hand
var request = new ROSLIB.ServiceRequest({});


module.exports = app;
