var express = require('express');
var app = express();

setInterval(sendRotation, 500);

var rotation = { roll: 1, pitch: 1, yaw: 1 }

var server = app.listen(3000);
app.use(express.static('public'));

console.log('Hello world');

var socket = require('socket.io');
var io = socket(server);

io.sockets.on('connection', newConnection);

function newConnection(socket) {
    console.log(socket.id);
}

function sendRotation() {
    io.sockets.emit('rotation', rotation)
    rotation.roll += 1
    rotation.pitch += 1
    rotation.yaw += 1
}