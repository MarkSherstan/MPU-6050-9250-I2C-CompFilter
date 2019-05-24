// Declare objects
let angleGen;
let visualizer;
var serial;


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
  angleGen.dataNumBytes = 2;
  angleGen.dataNumBytes = 6;
}


function draw() {
  // Draw a fresh bacground
  background(150);

  angleGen.getRawData();

  // Display object to the user
  visualizer.displayTorus(); //visualizer.displayCube();
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
    this.dataOut = [];
  }

  readSerialStart() {
    serial.open(this.serialPort);
    serial.clear()
  }

  getRawData(){
    // Reset byteArray
    byteArray = []

    // Read data from serial port if available and do a header check
    if (serial.available() > 0) {
      if ((serial.read() == 0x9F) && (serial.read() == 0x6E)){
        for (ii = 0; ii < (this.numSignals * this.dataNumBytes); ii++) {
          byteArray.push(serial.read())
        }
      }
    }

    // Cast bytes to ints
    this.ax = byteArray[0] | byteArray[1] << 8;
    this.ay = byteArray[2] | byteArray[3] << 8;
    this.az = byteArray[4] | byteArray[5] << 8;
    this.gx = byteArray[6] | byteArray[7] << 8;
    this.gy = byteArray[8] | byteArray[9] << 8;
    this.gz = byteArray[10] | byteArray[11] << 8;
  }

  calibrateGyro(N) {
    // Display message
    print("Calibrating gyro with " + String(N) + " points. Do not move!")

    // Take N readings for each coordinate and add to itself
    for (ii = 0; ii < N; ii++) {
        this.getRawData()
        this.gyroXcal += this.gx
        this.gyroYcal += this.gy
        this.gyroZcal += this.gz
    }

    // Find average offset value
    this.gyroXcal /= N
    this.gyroYcal /= N
    this.gyroZcal /= N

    // Display message and restart timer for comp filter
    print("Calibration complete")
    print("\tX axis offset: " + String(round(this.gyroXcal,1)))
    print("\tY axis offset: " + String(round(this.gyroYcal,1)))
    print("\tZ axis offset: " + String(round(this.gyroZcal,1)) + "\n")

    this.dtTimer = millis();
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
