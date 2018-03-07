import gab.opencv.*;
import processing.video.*;
import oscP5.*;
import netP5.*;


// library variable
Capture video;
OpenCV opencv;
OscP5 oscP5;
NetAddress local;


// built-in's
float flowScale = 1;
int camWidth = 640;
int camHeight = 360;
int gridSize = 5;        // manages grid size of 
PImage camMirror;
boolean debug = false;


void setup() {
  size(displayWidth, displayHeight, P2D);
  // size(1280, 720, P2D);
  String[] inputs = Capture.list();
  printArray(inputs);
  if (inputs.length == 0) {
    println("Couldn't detect any webcams connected!");
    exit();
  }

  // initialize video capture
  video = new Capture(this, camWidth, camHeight, 30); // gets video at smaller resolution
  video.start();
  camMirror = new PImage(camWidth,camHeight);         // for mirroring image
  opencv = new OpenCV(this, camWidth, camHeight);

  // initialize PureData messages
  oscP5 = new OscP5(this, 8080); // port number
  local = new NetAddress("127.0.0.1",12000);
}


void draw() {

  noStroke();
  // fill(0,0, 0, 90);      // black background w/ limited opacity
  fill(255,255, 255, 90);   // white background w/ limited opacity
  rect(0,0,width, height);

  //start the webcam
  video.loadPixels();
  strokeWeight(2);
  int r = 0, g = 0, b = 0;
  for(int x = 0; x < camWidth; x++){    // mirror webcam image
    for(int y = 0; y < camHeight; y++){
      camMirror.pixels[x+y*camWidth] = video.pixels[(camWidth-(x+1))+y*camWidth];

      // getting average r g b of picture
      color c = camMirror.pixels[x+y*camWidth];
      r += c>>16&0xFF;
      g += c>>8&0xFF;
      b += c&0xFF;

    }
  }
  r /= camMirror.pixels.length;
  g /= camMirror.pixels.length;
  b /= camMirror.pixels.length;

  // sending mapping of green color to granulator's playback speed
  float playback_speed = map( g, 0, 255, -100, 100 ); // values mapped to range of playback speed set in PureData
  OscMessage andrew = new OscMessage("/ps");
  andrew.add(playback_speed);
  oscP5.send(andrew, local);

  // send mapping of red color to granulator's panning values
  float pan = map( r, 0, 255, 0, 1 );   // mapped to range of panner (0,1)
  OscMessage andrew2 = new OscMessage("/pan");
  andrew2.add(pan);
  oscP5.send(andrew2, local);

  // load mirrored image and calculate flow
  camMirror.updatePixels();
  opencv.loadImage(camMirror);
  opencv.calculateOpticalFlow();

  // get flow at X,Y location for drawing of optical flow lines
  for (int x = gridSize; x<camWidth-gridSize; x+=gridSize){
    for (int y = gridSize; y<camHeight-gridSize; y+=gridSize){
        PVector pointFlow = opencv.getFlowAt(x, y);
        float a = pointFlow.mag();                          // length of line
        if (a>=10){                                         // reject small flow
          // color of lines based on Hideshi Shimodaira flow 
          float rf=0.5*(1.0+pointFlow.x/(a+0.1));
          float gf=0.5*(1.0+pointFlow.y/(a+0.1));
          float bf=0.5*(2.0-(rf+gf));
          stroke(255*rf,255*gf,255*bf);                     // coloring
          float nx = int(map(x, 0,video.width, 0,width));   // scaling from camWidth to displayWidth
          float ny = int(map(y, 0,video.height, 0,height)); // scaling from camHeight to displayHeight
          float rXY = random(gridSize);                     // used for drawing line off grid

          // 3 different ways of drawing line: On grid, Random(x) and Random(y), Random(XY)
          // line(nx, ny,  nx + (int) (pointFlow.x*flowScale) + random(gridSize),  ny + (pointFlow.y*flowScale) + (random(gridSize)));
          // line(nx, ny,  nx + (int) (pointFlow.x*flowScale),  ny + (pointFlow.y*flowScale));
          line(nx, ny,  nx + (int) (pointFlow.x*flowScale)+ rXY,  ny + (pointFlow.y*flowScale) + rXY);
        }
    }
  }



  if (debug) {
    //FPS!!
    noStroke();
    fill(255);
    rect(0,0,65,15);
    fill(0);
    text(nf(frameRate, 0,2) + " fps", 10,10);
   }
}



void keyPressed() {
  if (key == 'd') debug = !debug;

}