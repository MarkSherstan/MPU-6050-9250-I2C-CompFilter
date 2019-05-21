function setup() {
  createCanvas(400, 400, WEBGL);
}

function draw() {
  background(100);

  translate(0, 100, 0);
  fill('#8000ff');

  push();
  rotateZ(frameCount * 0.01);
  rotateX(frameCount * 0.01);
  rotateY(frameCount * 0.01);
  box(70, 70, 70);
  pop();


  translate(0, -200, 0);

  push();
  rotateZ(frameCount * 0.01);
  rotateX(frameCount * 0.01);
  rotateY(frameCount * 0.01);
  torus(50, 20);
  pop();
}

// https://medium.com/@yyyyyyyuan/tutorial-serial-communication-with-arduino-and-p5-js-cd39b3ac10ce
// http://p5js.org/examples/
