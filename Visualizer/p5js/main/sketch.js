// Declare objects
let angleGen;
var serial;
var ii;


function setup() {
  // Create a canvas to work on
  createCanvas(900, 700, WEBGL);

  // Set up the classes
  angleGen = new AngleGen();
  serial = new p5.SerialPort();

  // Set some initial values for the IMU processing
  angleGen.tau = 0.98;
  angleGen.gyroScaleFactor = 65.5;
  angleGen.accScaleFactor = 8192.0;

  // Set up some initial values for the DAQ
  angleGen.serialPort = '/dev/cu.usbmodem14101';
  angleGen.numSignals = 6;
  angleGen.dataNumBytes = 2;

  // Set up serial port
  angleGen.readSerialStart();
}


function draw() {
  // Draw a fresh bacground and slight rotation
  background(150);
  rotateX(PI/2);

  // Get data
  angleGen.getData();

  // Calibrate the gyroscope only once there is good data
  if (angleGen.gz && (angleGen.dataState == 0)) {
    angleGen.calibrateGyro(500);
    angleGen.dataState = 1;
  }

  // Display object to the user
  displayObject(angleGen.roll, angleGen.pitch, angleGen.yaw);
}


class AngleGen {
  constructor() {
    // IMU Processing
    this.ax = null; this.ay = null; this.az = null;
    this.gx = null; this.gy = null; this.gz = null;

    this.gyroXcal = 0;
    this.gyroYcal = 0;
    this.gyroZcal = 0;

    this.gyroRoll = null;
    this.gyroPitch = null;
    this.gyroYaw = null;

    this.roll = null;
    this.pitch = null;
    this.yaw = null;

    self.dtTimer = null;
    self.tau = null;

    this.gyroScaleFactor = null;
    this.accScaleFactor = null;

    // Data acquisition
    this.serialPort = '';
    this.dataNumBytes = null;
    this.numSignals = null;
    this.byteArray = []
    this.dataState = 0;
  }

  readSerialStart() {
    // Attempt a connection on the given serial port
    try {
      // Open serial port, clear buffer, and display message to user
      serial.open(this.serialPort);
      serial.clear();
      print("Serial port opened on " + this.serialPort);
    } catch {
      // Display message to user
      print("Serial port " + this.serialport + " failed to open");
    }
  }

  bytes2num(byteA, byteB){
    // Remove byteA sign and & it and then bit shift. Finally combine with byteB
    var temp = ((byteA & 0x7F) << 8) | byteB;

    // Sign the value
    if (byteA & 0x80){
      temp = temp - 32767;
    }

    // Return the number value
    return temp;
  }

  getData(){
    // Reset byteArray
    this.byteArray = [];

    // Read data from serial port if available and do a header check
    if ((serial.available() > 0) && (serial.read() == 0x9F) && (serial.read() == 0x6E)) {
      // Read the useful bytes
      for (ii = 0; ii < (this.numSignals * this.dataNumBytes); ii++) {
        this.byteArray.push(serial.read());
      }

      // Cast the bytes into a usable values
      this.ax = this.bytes2num(this.byteArray[1], this.byteArray[0]);
      this.ay = this.bytes2num(this.byteArray[3], this.byteArray[2]);
      this.az = this.bytes2num(this.byteArray[5], this.byteArray[4]);
      this.gx = this.bytes2num(this.byteArray[7], this.byteArray[6]);
      this.gy = this.bytes2num(this.byteArray[9], this.byteArray[8]);
      this.gz = this.bytes2num(this.byteArray[11], this.byteArray[10]);

      // If the gyro has been calibrated give values a physical representation
      if (this.dataState == 1){
        // Subract the offset calibration values for gyro
        this.gx -= this.gyroXcal;
        this.gy -= this.gyroYcal;
        this.gz -= this.gyroZcal;

        // Convert gyro values to instantaneous degrees per second
        this.gx /= this.gyroScaleFactor;
        this.gy /= this.gyroScaleFactor;
        this.gz /= this.gyroScaleFactor;

        // Convert accelerometer values to g force
        this.ax /= this.accScaleFactor;
        this.ay /= this.accScaleFactor;
        this.az /= this.accScaleFactor;
      }

      // Get delta time and record time for next call
      var dt = (millis() - this.dtTimer)*0.001;
      this.dtTimer = millis();

      // Acceleration vector angle
      var accPitch = degrees(atan2(this.ay, this.az));
      var accRoll = degrees(atan2(this.ax, this.az));

      // Gyro integration angle
      this.gyroRoll -= this.gy * dt;
      this.gyroPitch += this.gx * dt;
      this.gyroYaw += this.gz * dt;

      // Get attitude of filter using a comp filter and gyroYaw
      this.roll = (this.tau)*(this.roll - this.gy*dt) + (1-this.tau)*(accRoll);
      this.pitch = (this.tau)*(this.pitch + this.gx*dt) + (1-this.tau)*(accPitch);
      this.yaw = this.gyroYaw;

      // Clear the buffer
      serial.clear();
    }
  }

  calibrateGyro(N) {
    // Display message
    print("Calibrating gyro with " + String(N) + " points. Do not move!");

    // Take N readings for each coordinate and add to itself
    for (ii = 0; ii < N; ii++) {
        this.getData();
        this.gyroXcal += this.gx;
        this.gyroYcal += this.gy;
        this.gyroZcal += this.gz;

        // Small delay (0.01 s)
        var timeNow = millis();
        while(millis() < timeNow + 10){}
    }

    // Find average offset value
    this.gyroXcal /= N;
    this.gyroYcal /= N;
    this.gyroZcal /= N;

    // this.gyroXcal = 262;
    // this.gyroYcal = -28;
    // this.gyroZcal = -1;

    // Display message and restart timer for comp filter
    print("Calibration complete");
    print("\tX axis offset: " + String(round(this.gyroXcal,1)));
    print("\tY axis offset: " + String(round(this.gyroYcal,1)));
    print("\tZ axis offset: " + String(round(this.gyroZcal,1)) + "\n");

    // Set initial conditions for sensor processing
    this.dtTimer = millis();
    this.gyroRoll = 0;
    this.gyroPitch = 0;
    this.gyroYaw = 0;
    this.roll = 0;
    this.pitch = 0;
    this.yaw = 0;
  }
}


function displayObject(roll, pitch, yaw) {
  // Color options
  normalMaterial()

  // Print values to console
  print("R: " + round(roll) + " P: " + round(pitch) + " Y: " + round(yaw));

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
