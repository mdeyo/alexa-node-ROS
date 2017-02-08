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
    var request = new ROSLIB.ServiceRequest({});
    openHandClient.callService(request, function(result) {
        console.log('Result for service call on ' + closeHandClient.name + ': ' + result);
    });
    res.say('Opening hand');
});

app.intent("CloseHandIntent", {
    "utterances": ["Close the hand"]
}, function(req, res) {
    console.log("closing hand...");
    var request = new ROSLIB.ServiceRequest({});
    closeHandClient.callService(request, function(result) {
        console.log('Result for service call on ' + closeHandClient.name + ': ' + result);
    });
    res.say('Closing hand');
});

app.intent("IdleArmIntent", {
    "utterances": ["Idle arm","relax arm"]
}, function(req, res) {
    console.log("idling arm...");
    var request = new ROSLIB.ServiceRequest({});
    idleArmClient.callService(request, function(result) {
        console.log('Result for service call on ' + idleArmClient.name + ': ' + result);
    });
    res.say('hang loose, dude');
});


// TODO: Automatically check the ROS parameter server for these!
var allowable_objects = [
  {utterance: 'red block', id: 'redblock', say: 'red block'},
  {utterance: 'yellow block', id: 'yellowblock', say: 'yellow block'},
  {utterance: 'blue block', id: 'bluearch', say: 'blue arch'},
  {utterance: 'blue arch', id: 'bluearch', say: 'blue arch'},
  {utterance: 'green block', id: 'greenblock', say: 'green block'},
  {utterance: 'pink block', id: 'pinkblock', say: 'pink block'}
];

app.intent("PickupIntent", {
    "utterances": ["Pick up"]
}, function(req, res) {
    console.log("Picking...");
    // closeHandClient.callService(request, function(result) {
    //     console.log('Result for service call on ' + closeHandClient.name + ': ' + result);
    // });
    var object_name = req.slot('object');
    console.log("Got object name: " + object_name)
    for(var i = 0; i < allowable_objects.length; i++) {
      var ao = allowable_objects[i];
      if (object_name.search(ao.utterance) >= 0) {
        // Call the client
        var request = new ROSLIB.ServiceRequest({
          str: ao.id
        });
        pickupClient.callService(request, function(res) {
          console.log("Done picking");
        });
        res.say('Picking up the ' + ao.say);
        break;
      }
    }
});


app.intent("PlaceIntent", {
    "utterances": ["Place"]
}, function(req, res) {
    console.log("Placing...");
    // closeHandClient.callService(request, function(result) {
    //     console.log('Result for service call on ' + closeHandClient.name + ': ' + result);
    // });
    var object_top_name = req.slot('object_top');
    var object_bottom_name = req.slot('object_bottom');
    console.log("Got object top name: " + object_top_name);
    console.log("Got object bottom name: " + object_bottom_name);

    for(var i = 0; i < allowable_objects.length; i++) {
      var ao = allowable_objects[i];
      if (object_bottom_name.search(ao.utterance) >= 0) {
        // Call the client
        var request = new ROSLIB.ServiceRequest({
          str: ao.id
        });
        placeClient.callService(request, function(res) {
          console.log("Done placing");
        });
        res.say('Placing on the ' + ao.say);
        break;
      }
    }
});


app.intent("DetachIntent", {
    "utterances": ["Detach the hand"]
}, function(req, res) {
    console.log("Letting it go...");
    var request = new ROSLIB.ServiceRequest({});
    detachClient.callService(request, function(result) {
        // console.log('Result for service call on ' + closeHandClient.name + ': ' + result);
    });
    // res.say('Letting it go');
    // Haha gotcha Steve!
    res.say("<audio src='https://s3.amazonaws.com/red-shirt/letitgo-converted.mp3'/>");
});


// Connecting to ROS
var ROSLIB = require('roslib');

// var prompt = require('prompt');
// prompt.start();
// function promptForAction() {
//     prompt.get(['action'], function(err, result) {
//         //
//         // Log the results.
//         //
//         console.log('Command-line input received:');
//         // console.log(' action: ' + result.action);
//         if (result.action == "close") {
//             console.log("closing hand...");
//             closeHandClient.callService(request, function(result) {
//                 console.log('Result for service call on ' + closeHandClient.name + ': ' + result);
//             });
//         }
//         if (result.action == "open") {
//             console.log("opening hand...");
//             openHandClient.callService(request, function(result) {
//                 console.log('Result for service call on ' + openHandClient.name + ': ' + result);
//             });
//         }
//         promptForAction();
//     });
// }


var ros = new ROSLIB.Ros({
    url: 'ws://localhost:9090'
});

ros.on('connection', function() {
    console.log('Connected to websocket server.');
    // promptForAction();
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
var idleArmClient = new ROSLIB.Service({
    ros: ros,
    name: '/' + arm_name + '/idle',
    serviceType: 'std_srvs/Empty'
});
var freezeArmClient = new ROSLIB.Service({
    ros: ros,
    name: '/' + arm_name + '/freeze',
    serviceType: 'std_srvs/Empty'
});
var pickupClient = new ROSLIB.Service({
    ros: ros,
    name: '/pick_and_place/' + arm_name + '/pickup',
    serviceType: 'mers_srvs/StringString'
});
var placeClient = new ROSLIB.Service({
    ros: ros,
    name: '/pick_and_place/' + arm_name + '/place',
    serviceType: 'mers_srvs/StringString'
});
var detachClient = new ROSLIB.Service({
    ros: ros,
    name: '/pick_and_place/' + arm_name + '/detach_all_objects',
    serviceType: 'std_srvs/Empty'
});


// Then we create a Service Request. The object we pass in to ROSLIB.ServiceRequest matches the
// fields defined in the rospy_tutorials AddTwoInts.srv file.
// Empty request object for open and close hand
// var request = new ROSLIB.ServiceRequest({});
// Finally, we call the /add_two_ints service and get back the results in the callback. The result
// is a ROSLIB.ServiceResponse object.
// openHandClient.callService(request, function(result) {
//     console.log('Result for service call on ' + openHandClient.name + ': ' + result);
// });
module.change_code = 1;
module.exports = app;
