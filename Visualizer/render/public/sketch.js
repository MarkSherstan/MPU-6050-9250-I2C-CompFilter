var socket;

var globalDat;

////////////////////////////////////////////////////////////////////////////////
// Set Up
////////////////////////////////////////////////////////////////////////////////

function setup() {
    // Create a 3D web canvas to work on
    createCanvas(900, 700, WEBGL);

    // Connect to socket 
    socket = io.connect('http://localhost:3000')
    socket.on('rotation', update);
}


function update(data) {
    print(data)

    globalDat = data;


}


////////////////////////////////////////////////////////////////////////////////
// Draw
////////////////////////////////////////////////////////////////////////////////

function draw() {
    // Draw a fresh background and rotate the frame 90 degrees
    background(150);
    rotateX(PI / 2);

    // Display rotation
    if (globalDat) {
        displayObject(globalDat.roll, globalDat.pitch, globalDat.yaw);
    }
}



////////////////////////////////////////////////////////////////////////////////
// Variable Declarations
////////////////////////////////////////////////////////////////////////////////

// var serial;
// var ii;
// var ax, ay, az;
// var gx, gy, gz;
// var gyroXcal, gyroYcal, gyroZcal;
// var gyroRoll, gyroPitch, gyroYaw;
// var roll, pitch, yaw;
// var accPitch, accRoll;
// var dtTimer;
// var dt;
// var byteArray = [];
// var calibrationCounter = 0;
// roll = pitch = yaw = 0;
// gyroXcal = gyroYcal = gyroZcal = 0;
// gyroRoll = gyroPitch = gyroYaw = 0;

// // Customize these values
// var portName = '/dev/cu.usbmodem14101';
// var tau = 0.98;
// var gyroScaleFactor = 65.5;
// var accScaleFactor = 8192.0;
// var calibrationPts = 100;

////////////////////////////////////////////////////////////////////////////////
// Set Up
////////////////////////////////////////////////////////////////////////////////

// function setup() {
//     // Create a 3D web canvas to work on
//     createCanvas(900, 700, WEBGL);

//     // Set up the serial port class
//     serial = new p5.SerialPort();

//     // Set some callback functions for the serial port
//     serial.on('open', portOpen);
//     serial.on('data', serialEvent);
//     serial.on('error', serialError);

//     // Open a serial port
//     serial.open(portName);

//     // Display message for user to hold still
//     print('Calibration to begin. Hold still!\n')
// }

////////////////////////////////////////////////////////////////////////////////
// Draw
////////////////////////////////////////////////////////////////////////////////

// function draw() {
//     // Draw a fresh background and rotate the frame 90 degrees
//     background(150);
//     rotateX(PI / 2);

//     // Display object to the user only if calibration is complete
//     if (calibrationCounter > calibrationPts) {
//         displayObject(roll, pitch, yaw);
//     }
// }

////////////////////////////////////////////////////////////////////////////////
// Events
////////////////////////////////////////////////////////////////////////////////

function serialEvent() {
    // Reset byteArray
    byteArray = [];

    // Read data from serial port if available and do a header check
    if ((serial.available() > 14) && (serial.read() == 0x9F) && (serial.read() == 0x6E)) {
        // Read the useful bytes (Number of signals * Number of bytes for data type)
        for (ii = 0; ii < (6 * 2); ii++) {
            byteArray.push(serial.read());
        }

        // Cast the bytes into usable values
        ax = bytes2num(byteArray[1], byteArray[0]);
        ay = bytes2num(byteArray[3], byteArray[2]);
        az = bytes2num(byteArray[5], byteArray[4]);
        gx = bytes2num(byteArray[7], byteArray[6]);
        gy = bytes2num(byteArray[9], byteArray[8]);
        gz = bytes2num(byteArray[11], byteArray[10]);

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
            print("Calibration complete");
            print("\tX axis offset: " + String(round(gyroXcal)));
            print("\tY axis offset: " + String(round(gyroYcal)));
            print("\tZ axis offset: " + String(round(gyroZcal)) + "\n");

            // Start a timer
            dtTimer = millis();

            // Incrament counter once more to show the calibration is complete
            calibrationCounter += 1;

        } else {
            // Turn values into something with a physical representation
            processValues();

            // Print values to console
            print("R: " + round(roll) + " P: " + round(pitch) + " Y: " + round(yaw));
        }
    }
}

function serialError(err) {
    print('Something went wrong with the serial port. ' + err);
}

////////////////////////////////////////////////////////////////////////////////
// Functions
////////////////////////////////////////////////////////////////////////////////

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

function displayObject(roll, pitch, yaw) {
    // Color options
    normalMaterial();

    // Start display
    push();

    // Apply the rotation in Z-Y-X
    rotateZ(radians(yaw));
    rotateY(radians(roll));
    rotateX(radians(pitch));

    // Make an object
    box(200, 200, 200);

    // Finish display
    pop();
}