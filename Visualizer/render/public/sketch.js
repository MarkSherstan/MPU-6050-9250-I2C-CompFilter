var socket;
var globalDat;

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

function draw() {
    // Draw a fresh background and rotate the frame 90 degrees
    background(150);
    rotateX(PI / 2);

    // Display rotation
    if (globalDat) {
        displayObject(globalDat.roll, globalDat.pitch, globalDat.yaw);
    }
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