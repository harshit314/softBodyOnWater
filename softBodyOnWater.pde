sftBdy body[];
water wtr;
PVector tmp;
// ks is spring constant for body, fClick is magnitude of attractive force towards mouse click, wallOffst is offset distance at which wall starts exerting force on bodies.
float ks = 0.9, fClick = 8.0, dt = 0.1, wallOffst = 14.0; 
// nPMin is number of vertices on each body, nW is number of points for water.
int nBody = 5, nPMin = 21, nW = 100;
// list of colors for bodies:
color[] cList = {color(182,215,168), color(147,196,125), color(247,247,247), color(228,106,106), color(255,147,147)};
// fActive is random driving force on each body, thMax is maximum range of angles to be sampled randomly to give direction to fActive.
float gravity = 4.0, fActive = 4.0, thMax = PI/6.0, sqDt = sqrt(dt); //down is positive 
// cWater is wave speed for water, kWall is spring constant for walls, kRep is repulsive force between bodies
float bodyRadius = 60, cWater = 120.0, kWall = 12.0, kRep = 8.0, fWater=0.5; //fWater is fraction of force transfered to water upon collision
int nTrails = 100; // length of trails behind bodies.

void setup(){
  size(800, 800, P2D);
  noStroke();
  frameRate(60);
  ellipseMode(CENTER);
  //rectMode(CENTER);
  tmp = new PVector(width/2, height/2);
  //create a soft body:
  body = new sftBdy[nBody];
  for(int i=0; i<nBody; i++){
    // tmp.set(random(0.1,0.95)*width, random(0.1,0.3)*height);
    // tmp.set(0.5*width, random(0.1,0.14)*height);
    tmp.set(width/2 - (i-2)*2.2*bodyRadius, 0.2*height - abs(i-2)*0.5*bodyRadius);
    body[i] = new sftBdy(nPMin, tmp, bodyRadius, cList[i%(cList.length)]);
  }
  wtr = new water(nW);
}
void draw(){
  fill(51,80); // non-zero alpha channel gives cool blur effects
  rect(0,0, width, height);
  
  wtr.setFwater(); // call before computing external forces on bodies.

  for(int i=0; i<nBody; i++){
    body[i].calcIntForces(); // compute internal forces before other forces. This function also sets centre point of bodies.
  }
  // set attractive force at mouse location upon clicking: 
  if(mousePressed){
    fClick = 5.0;
    for(int i=0; i<nBody; i++){
      body[i].setFclick(float(mouseX), float(mouseY));
    }
  }

  calcExtForces();  // this exterts force on water as well.
  wtr.updatewater();  

  for(int i=0; i<nBody; i++){
    body[i].evolve();
    noStroke();
    drawBdy(body[i]);
  }
  wtr.drawwater();

  saveFrame("1Body-####.png");
  
}
