// Import modules
const { SerialPort } = require('serialport')
const { ByteLengthParser } = require('@serialport/parser-byte-length')
var socket = require('socket.io');
var express = require('express');

// Constants
const serialPortName = '/dev/cu.usbserial-1410';
const serialBaud = 9600;
const numberOfBytes = 14;
let serverPort = 3000;

// Variables
var ax, ay, az;
var gx, gy, gz;
var gyroXcal, gyroYcal, gyroZcal;
var gyroRoll, gyroPitch, gyroYaw;
var roll, pitch, yaw;
var accPitch, accRoll;
var dtTimer;
var dt;
var calibrationCounter = 0;
roll = pitch = yaw = 0;
gyroXcal = gyroYcal = gyroZcal = 0;
gyroRoll = gyroPitch = gyroYaw = 0;

// Customize these values
var portName = '/dev/cu.usbmodem14101';
var tau = 0.98;
var gyroScaleFactor = 65.5;
var accScaleFactor = 8192.0;
var calibrationPts = 100;

// Messages
console.log('Calibration to begin. Hold still!\n')

// Configure serial port
const port = new SerialPort({ path: serialPortName, baudRate: serialBaud })
const parser = port.pipe(new ByteLengthParser({ length: numberOfBytes }))

// Configure sockets
var app = express();
var server = app.listen(serverPort);
var io = socket(server);
app.use(express.static('public'));

// Read serial data
// parser.on('data', function(data) {
//     console.log(data)
// });

parser.on('data', validateMsg);

// Simulate sending data
setInterval(sendRotation, 100);
var rotation = { roll: 1, pitch: 1, yaw: 1 }

function sendRotation() {
    io.sockets.emit('rotation', rotation)
    rotation.roll += 1
    rotation.pitch += 1
    rotation.yaw += 1
}

function validateMsg(data) {
    // Confirm we are in the correct position of the byte array
    if ((data[0] == 0x9F) && (data[1] == 0x6E)) {
        // Cast the bytes into usable values
        ax = bytes2num(data[2], data[3]);
        ay = bytes2num(data[4], data[5]);
        az = bytes2num(data[6], data[7]);
        gx = bytes2num(data[8], data[9]);
        gy = bytes2num(data[10], data[11]);
        gz = bytes2num(data[12], data[13]);

        if (calibrationCounter < calibrationPts) {
            // Sum points until a quota has been met
            gyroXcal += gx;
            gyroYcal += gy;
            gyroZcal += gz;

            // Incrament counter
            calibrationCounter += 1;

        } else if (calibrationCounter == calibrationPts) {
            // Once quota is met find the average offset value
            gyroXcal /= calibrationPts;
            gyroYcal /= calibrationPts;
            gyroZcal /= calibrationPts;

            // Display message
            console.log("Calibration complete");
            console.log("\tX axis offset: " + String(round(gyroXcal)));
            console.log("\tY axis offset: " + String(round(gyroYcal)));
            console.log("\tZ axis offset: " + String(round(gyroZcal)) + "\n");

            // Start a timer
            dtTimer = millis();

            // Increment counter once more to show the calibration is complete
            calibrationCounter += 1;

        } else {
            // Turn values into something with a physical representation
            processValues();

            // Print values to console
            print("R: " + round(roll) + " P: " + round(pitch) + " Y: " + round(yaw));
        }
    } else {
        console.log("Fail");
    }
}

function processValues() {
    // Subract the offset calibration values for the gyro
    gx -= gyroXcal;
    gy -= gyroYcal;
    gz -= gyroZcal;

    // Convert gyro values to instantaneous degrees per second
    gx /= gyroScaleFactor;
    gy /= gyroScaleFactor;
    gz /= gyroScaleFactor;

    // Convert accelerometer values to g force
    ax /= accScaleFactor;
    ay /= accScaleFactor;
    az /= accScaleFactor;

    // Get delta time and record time for the next call
    dt = (millis() - dtTimer) * 0.001;
    dtTimer = millis();

    // Acceleration vector angle
    accPitch = degrees(atan2(ay, az));
    accRoll = degrees(atan2(ax, az));

    // Gyro integration angle
    gyroRoll -= gy * dt;
    gyroPitch += gx * dt;
    gyroYaw += gz * dt;

    // Get attitude of filter using a comp filter and gyroYaw
    roll = tau * (roll - gy * dt) + (1 - tau) * (accRoll);
    pitch = tau * (pitch + gx * dt) + (1 - tau) * (accPitch);
    yaw = gyroYaw;
}


function bytes2num(byteA, byteB) {
    // Remove byteA sign and & it and then bit shift. Finally combine with byteB
    var temp = ((byteA & 0x7F) << 8) | byteB;

    // Sign the value
    if (byteA & 0x80) {
        temp = temp - 32767;
    }

    // Return the number value
    return temp;
}