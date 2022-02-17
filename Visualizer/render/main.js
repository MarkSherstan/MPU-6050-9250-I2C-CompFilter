var express = require('express');
var app = express();

setInterval(sendRotation, 500);


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
    io.sockets.emit('rotation', 1)
}