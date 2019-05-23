// Declare objects
let sensor;
var serial;

function setup() {
  // Create a canvas to work on
  createCanvas(900, 700, WEBGL);

  // Set up the signal processing and serial port.
  sensor = new MPU();
  serial = new p5.SerialPort();

  // Connect to an arduino serial port
  serial.open("/dev/cu.usbmodem14101");
}

function draw() {
  // Draw a fresh bacground
  background(75);

  //sensor.displayCube();
  sensor.displayTorus();

  // Read data from serial port if available
  if (serial.available() > 0) {
    var data = serial.read();
    print(data);
  }

}

// Class
class MPU {
  constructor() {
    this.roll = 0;
    this.pitch = 0;
    this.yaw = 0;
    this.count = 1;
    this.solidFill = fill('#8000ff'); // normalMaterial();
  }

  upDate() {
    this.roll = this.count * 0.01;
    this.pitch = this.count * 0.01;
    this.yaw = this.count * 0.001;
    this.count += 1;
  }

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
