//AI Path Planning
//Michael Nguyen Project 3 for CSci 5611

ArrayList<Obstacle> obstacles = new ArrayList();
ArrayList<Integer> path;
boolean circleOrBox = true;
int numNodes = 50;
int genNodeStart = 0;
PVector nodePos[] = new PVector[numNodes];
PVector startPos, goalPos, agentPos, agentVel, direction;
float agentRad = 20;
float targetVel = 10;
PImage bug, bg, concrete, round;

void setup(){
  size(800,600,P2D);
  bug = loadImage("bug.png");
  bg = loadImage("bg.jpg");
  concrete = loadImage("concrete.jpg");
  round = loadImage("round.png");
  direction = new PVector(1,1);
  startPos = randomPos();
  goalPos = randomPos();
  agentPos = startPos.copy();
  agentVel = new PVector(0.0,0.0);
  test();
  rectMode(CORNERS);
}

void keyPressed(){
  if (key == 'c') circleOrBox = true;
  if (key == 'b') circleOrBox = false;
  if (key == 'z' && obstacles.size() > 0){
    obstacles.remove(obstacles.size()-1);
    agentVel.setMag(0);
    startPos = agentPos.copy();
    test();
  }
}

void mousePressed(){
  if (mouseButton == LEFT) {
    obstacles.add(new Obstacle(circleOrBox, new PVector(mouseX, mouseY), new PVector(mouseX, mouseY)));
  }
  if (mouseButton == RIGHT) {
    agentVel.setMag(0);
    goalPos.x = mouseX;
    goalPos.y = mouseY;
    startPos = agentPos.copy();
    path = planPath(startPos, goalPos, nodePos, numNodes);
  }
}

void mouseReleased(){
  if (mouseButton == RIGHT) {
    return;
  }
  obstacles.get(obstacles.size()-1).BB = new PVector(mouseX, mouseY);
  obstacles.get(obstacles.size()-1).updateCorners();
  obstacles.get(obstacles.size()-1).updateRadius();
  if (obstacles.get(obstacles.size()-1).rad < 5){
    obstacles.remove(obstacles.size()-1);
  }
  agentVel.setMag(0);
  startPos = agentPos.copy();
  test();
}

void generateNodes(int numNodes){
  for (int i = genNodeStart; i < numNodes; i++){
    PVector randPos = new PVector(random(10,width-10),random(10,height-10));
    for (int j = 0; j < obstacles.size(); j++){
      if (obstacles.get(j).pointInside(randPos)){
        if (obstacles.get(j).circle) {
          PVector dir = PVector.sub(randPos,obstacles.get(j).AA).normalize();
          randPos = obstacles.get(j).AA.copy().add(dir.mult(obstacles.get(j).rad+2*agentRad));
          
        }
        else randPos = new PVector(random(10,width-10),random(10,height-10));
        j = -1;
      }
    }
    nodePos[i] = randPos;
  }
}

void optimalNodes(){
  ArrayList<PVector> newPos = new ArrayList();
  for (int i = 0; i < obstacles.size(); i++){
    float cornerDist = agentRad+3;
    if (obstacles.get(i).circle){
      newPos.add(obstacles.get(i).AA.copy().add(new PVector(-1,-1).mult(obstacles.get(i).rad+cornerDist)));
      newPos.add(obstacles.get(i).AA.copy().add(new PVector(-1,1).mult(obstacles.get(i).rad+cornerDist)));
      newPos.add(obstacles.get(i).AA.copy().add(new PVector(1,1).mult(obstacles.get(i).rad+cornerDist)));
      newPos.add(obstacles.get(i).AA.copy().add(new PVector(1,-1).mult(obstacles.get(i).rad+cornerDist)));
    }
    if (!obstacles.get(i).circle){
      float boxWidth = obstacles.get(i).BB.x - obstacles.get(i).AA.x;
      newPos.add(obstacles.get(i).AA.copy().add(new PVector(-cornerDist,-cornerDist)));
      newPos.add(obstacles.get(i).AA.copy().add(new PVector(cornerDist+boxWidth,-cornerDist)));
      newPos.add(obstacles.get(i).BB.copy().add(new PVector(cornerDist,cornerDist)));
      newPos.add(obstacles.get(i).BB.copy().add(new PVector(-(cornerDist+boxWidth),cornerDist)));
    }
  }
  for (int i = 0; i < newPos.size(); i++){
    for (int j = 0; j < obstacles.size(); j++){
      if (obstacles.get(j).pointInside(newPos.get(i))){
        newPos.remove(i);
        i--;
        break;
      }
    }
  }
  genNodeStart = 0;
  for (int i = 0; i < newPos.size(); i++){
    nodePos[i] = newPos.get(i);
    genNodeStart++;
  }
}

PVector randomPos(){
  PVector randPos = new PVector(random(width),random(height));
  for (int j = 0; j < obstacles.size(); j++){
    if (obstacles.get(j).pointInside(randPos)){
      randPos = new PVector(random(width),random(height));
      j = -1;
    }
  }
  return randPos;
}

void test(){
  optimalNodes();
  generateNodes(numNodes);
  path = planPath(startPos, goalPos, nodePos, numNodes);
}

void updateMotion(float dt){
  PVector nextPos = agentPos;
  if (path.size() > 0){
    if (path.get(0) < 0){
      agentVel.setMag(0.0);
      return;
    }
  }
  if (PVector.dist(agentPos,goalPos) < 5){
    if (agentVel.mag()<50){
      agentVel.setMag(0.0);
      startPos = goalPos.copy();
    }
    else agentVel.mult(0.5);
    return;
  }
  if (path.size()==0){
    nextPos = goalPos;
  }
  else {
    nextPos = nodePos[path.get(0)];
    if(PVector.dist(agentPos,nextPos)<5){
      startPos = nextPos.copy();
      path.remove(0);
      agentVel.setMag(0);
      return;
    }
  }
  agentVel.add(PVector.sub(nextPos,agentPos).setMag(3).mult(dt));
  if (agentVel.mag() <4) agentVel.setMag(4);
  float targetRatio = PVector.dist(agentPos, nextPos)/50+0.4;
  if (targetRatio > 1){
    targetRatio = 1;
    direction = agentVel.copy();
  }
  
  else {
    float turnDegree = (targetRatio-0.4)/0.6;
    PVector toNext;
    if (path.size() > 1){
      toNext = PVector.sub(nodePos[path.get(1)],agentPos).setMag(1-turnDegree);
    }
    else {
      toNext = PVector.sub(goalPos,agentPos).setMag(1-turnDegree);
    }
    direction = agentVel.copy().setMag(turnDegree).add(toNext);
  }
  
  if (agentVel.mag() > targetVel*targetRatio){
    agentVel.setMag(targetVel*targetRatio);
  }
  agentPos.add(agentVel.copy().mult(dt));
}

void draw(){
  float dt = 10/frameRate;
  updateMotion(dt);
  //background(200);
  image(bg,0,0);
  stroke(0);
  strokeWeight(2);
  fill(255);
  for (int i = 0; i < obstacles.size(); i++){
    if (obstacles.get(i).circle == true){
      //circle(obstacles.get(i).AA.x,obstacles.get(i).AA.y,2*obstacles.get(i).rad);
      float x1, x2, y1, y2;
      x1 = obstacles.get(i).AA.x-obstacles.get(i).rad;
      x2 = obstacles.get(i).AA.x+obstacles.get(i).rad;
      y1 = obstacles.get(i).AA.y-obstacles.get(i).rad;
      y2 = obstacles.get(i).AA.y+obstacles.get(i).rad;
      noStroke();
      beginShape(QUADS);
      texture(round);
      vertex(x1, y1, 0, 0);
      vertex(x2, y1, round.width, 0);
      vertex(x2, y2, round.width, round.height);
      vertex(x1, y2, 0, round.height);
      endShape();
    }
    else {
      //rect(obstacles.get(i).AA.x,obstacles.get(i).AA.y,obstacles.get(i).BB.x,obstacles.get(i).BB.y);
      float x1, x2, y1, y2;
      x1 = obstacles.get(i).AA.x;
      x2 = obstacles.get(i).BB.x;
      y1 = obstacles.get(i).AA.y;
      y2 = obstacles.get(i).BB.y;
      stroke(0);
      strokeWeight(3);
      beginShape(QUADS);
      texture(concrete);
      vertex(x1, y1, x1, y1);
      vertex(x2, y1, x2, y1);
      vertex(x2, y2, x2, y2);
      vertex(x1, y2, x1, y2);
      endShape();
    }
  }
  
  fill(20,60,250);
  circle(goalPos.x,goalPos.y,10);
  
  pushMatrix();
  translate(agentPos.x,agentPos.y);
  rotate(atan2(direction.y,direction.x));
  image(bug,-20,-20);
  popMatrix();
}
