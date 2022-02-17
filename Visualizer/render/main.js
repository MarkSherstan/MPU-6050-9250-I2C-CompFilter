// Import modules
const { SerialPort } = require('serialport')
const { ByteLengthParser } = require('@serialport/parser-byte-length')

var socket = require('socket.io');
var express = require('express');


// Constants
let serialPortName = '/dev/cu.usbserial-1410';
let serialBaud = 9600;
let serverPort = 3000;

// Configure serial port
const port = new SerialPort({ path: serialPortName, baudRate: serialBaud })


// Configure sockets
var app = express();
var server = app.listen(serverPort);
var io = socket(server);
app.use(express.static('public'));

// Read data
const parser = port.pipe(new ByteLengthParser({ length: 14 }))
parser.on('data', console.log)

// Simulate sending data
setInterval(sendRotation, 100);
var rotation = { roll: 1, pitch: 1, yaw: 1 }

function sendRotation() {
    io.sockets.emit('rotation', rotation)
    rotation.roll += 1
    rotation.pitch += 1
    rotation.yaw += 1
}