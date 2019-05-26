// Declare objects
let angleGen;
let visualizer;
var serial;
var ii;


function setup() {
  // Create a canvas to work on
  createCanvas(900, 700, WEBGL);

  // Set up the clases
  angleGen = new AngleGen();
  visualizer = new Visualizer();
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
  // Draw a fresh bacground
  background(150);

  // Get raw data
  angleGen.getRawData();

  // Calibrate the gyroscope only once there is good data
  if (angleGen.gz && (angleGen.dataState == 0)) {
      angleGen.calibrateGyro(2000);
      angleGen.dataState = 1;
  }

  // If the gyro has been calibrated process the values
  if (angleGen.dataState == 1){
  }

  // Display object to the user
  visualizer.displayTorus();
}


class AngleGen {
  constructor() {
    // IMU Processing
    this.ax = null; this.ay = null; this.az = null;
    this.gx = null; this.gy = null; this.gz = null;

    this.gyroXcal = 0;
    this.gyroYcal = 0;
    this.gyroZcal = 0;

    this.gyroRoll = 0;
    this.gyroPitch = 0;
    this.gyroYaw = 0;

    this.roll = 0;
    this.pitch = 0;
    this.yaw = 0;

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

  getRawData(){
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

      // Clear the buffer and change state variable
      serial.clear();
    }
  }

  calibrateGyro(N) {
    // Display message
    print("Calibrating gyro with " + String(N) + " points. Do not move!");

    // Take N readings for each coordinate and add to itself
    for (ii = 0; ii < N; ii++) {
        this.getRawData();
        this.gyroXcal += this.gx;
        this.gyroYcal += this.gy;
        this.gyroZcal += this.gz;
    }

    // Find average offset value
    this.gyroXcal /= N;
    this.gyroYcal /= N;
    this.gyroZcal /= N;

    // Display message and restart timer for comp filter
    print("Calibration complete");
    print("\tX axis offset: " + String(round(this.gyroXcal,1)));
    print("\tY axis offset: " + String(round(this.gyroYcal,1)));
    print("\tZ axis offset: " + String(round(this.gyroZcal,1)) + "\n");

    this.dtTimer = millis();
  }

  processIMUvalues(){
    // Subract the offset calibration values
    this.gx -= this.gyroXcal;
    this.gy -= this.gyroYcal;
    this.gz -= this.gyroZcal;

    //

  }
}


class Visualizer {
  constructor() {
    this.count = 1;
    this.solidFill = fill('#8000ff'); // normalMaterial();
  }

  upDate() {
    this.roll = this.count * 0.01;
    this.pitch = this.count * 0.01;
    this.yaw = this.count * 0.001;
    this.count += 1;
  }

  // displayUAV() { }

  displayCube() {
    this.upDate();

    this.solidFill

    push();
    rotateZ(this.roll);
    rotateX(this.pitch);
    rotateY(this.yaw);
    box(70, 70, 70);
    pop();
  }

  displayTorus() {
    this.upDate();

    this.solidFill

    push();
    rotateZ(this.roll);
    rotateX(this.pitch);
    rotateY(this.yaw);
    torus(100, 40);
    pop();
  }
}
