var express = require('express');
var app = express();

var server = app.listen(3000);
app.use(express.static('public'));

console.log('Hello world');

var socket = require('socket.io');
var io = socket(server);

io.sockets.on('connection', newConnection);

function newConnection(socket) {
    console.log(socket.id);
}