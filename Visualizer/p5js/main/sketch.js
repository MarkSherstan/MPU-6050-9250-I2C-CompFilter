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
  background(75);

  // Display object to the user
  visualizer.displayTorus(); //visualizer.displayCube();
}


class AngleGen {
  constructor() {
    // IMU Processing
    this.gx = null; this.gy = null; this.gz = null;
    this.ax = null; this.ay = null; this.az = null;

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
    this.byteArray = [];
    this.dataOut = [];
  }

  readSerialStart() {
    serial.open(this.serialPort);
    serial.clear()
  }

  getSerialData() {
    // Read data from serial port if available
    if (serial.available() > 0) {
      var data = serial.read();
      print(data);
    }
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
