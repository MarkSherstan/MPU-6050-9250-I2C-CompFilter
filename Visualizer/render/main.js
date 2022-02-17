// Import modules
var socket = require('socket.io');
var express = require('express');

// Configure networking
var app = express();
var server = app.listen(3000);
var io = socket(server);
app.use(express.static('public'));

// Simulate sending data
setInterval(sendRotation, 100);
var rotation = { roll: 1, pitch: 1, yaw: 1 }

function sendRotation() {
    io.sockets.emit('rotation', rotation)
    rotation.roll += 1
    rotation.pitch += 1
    rotation.yaw += 1
}