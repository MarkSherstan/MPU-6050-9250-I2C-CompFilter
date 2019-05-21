let sensor; // Declare object

function setup() {
  createCanvas(400, 400, WEBGL);
  sensor = new MPU();
}

function draw() {
  background(100);

  //sensor.displayCube();
  sensor.displayTorus()

}

// Class
class MPU {
  constructor() {
    this.roll = 0;
    this.pitch = 0;
    this.yaw = 0;
    this.count = 1;
    this.solidFill = fill('#8000ff');
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
    torus(50, 20);
    pop();
  }
}
