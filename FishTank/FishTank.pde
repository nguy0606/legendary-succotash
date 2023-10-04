//Fish Simulation
//Created for CSCI 5611 project 1 by Michael Nguyen

static int startFish = 500;

ArrayList<Vec2> pos = new ArrayList<Vec2>();
ArrayList<Vec2> vel = new ArrayList<Vec2>();
ArrayList<Vec2> acc = new ArrayList<Vec2>();
ArrayList<Integer> oscInd = new ArrayList<Integer>();
ArrayList<Vec2> oscVel = new ArrayList<Vec2>();
Vec2 predPos;
Vec2 predVel = new Vec2(0,0);
float randAngle;
float panicLevel = 15;
float maxForce = 20;
float xmin = 100, xmax, ymin = 100, ymax;
float osc = 0;
int onScreen = 0;
float dangerZone = 0;
PImage bg, fish, pred, weeds;

void setup(){
  size(1200,800);
  bg = loadImage("ocean.jpg");
  fish = loadImage("fish.png");
  pred = loadImage("pred.png");
  weeds = loadImage("weeds.png");
  //Initial properties
  for (int i = 0; i < startFish; i++){
    pos.add(new Vec2(random(width),random(100,500)));
    oscInd.add(i);
    randAngle = random(0,2*PI);
    vel.add(new Vec2(sin(randAngle),cos(randAngle)));
    vel.get(i).normalize();
    acc.add(new Vec2(0,0));
    oscVel.add(new Vec2(0,0));
  }
  xmax = width-100;
  ymax = height-200;
  predPos = new Vec2(width/2,height-200);
  noStroke();
  textFont(createFont("Arial",32,true));
}

void draw(){
  background(70,200,210);
  image(bg,0,0);
  cursor(CROSS);
  
  //draw dangerZone
  //fill(80,40,200);
  //circle(predPos.x,predPos.y,2*dangerZone);
  
  fill(150,160,60);
  rect(0,height-50,width,100);
  pushMatrix();
  translate(predPos.x,predPos.y);
  rotate(atan2(predVel.y,predVel.x));
  if (atan2(predVel.y,predVel.x) < -(PI/2) || atan2(predVel.y,predVel.x) > (PI/2)){
    scale(1,-1);
  }
  image(pred,-20,-15);
  popMatrix();
  for (int i = 0; i < pos.size(); i++){
    //check if on screen
    if (pos.get(i).x > -5 && pos.get(i).y > -5 && pos.get(i).x < width+5 && pos.get(i).y < height+5){
      onScreen ++;
    pushMatrix();
    translate(pos.get(i).x,pos.get(i).y);
    rotate(atan2(oscVel.get(i).y,oscVel.get(i).x));
    image(fish,-5,-5);
    popMatrix();
    }
  }
  
  pushMatrix();
  translate(100,height-100);
  image(weeds,0,0);
  translate(350,0);
  image(weeds,0,0);
  translate(360,0);
  image(weeds,0,0);
  popMatrix();

  float dt = 0.1;
  if (panicLevel > 15){
    panicLevel -= 0.05;
  }
  if (dangerZone > 0){
    if (predPos.y > height-100) dangerZone -=0.2;
    else dangerZone -= 0.08;
  }
  
  //oscilation timer
  osc += (PI)/(frameRate);
  if (osc > 2*PI){
    osc %= 2*PI;
  }
  
  for (int i = 0; i < pos.size(); i++){
    acc.set(i,new Vec2(0,0));
    Vec2 avgPos = new Vec2(0,0);
    Vec2 avgVel = new Vec2(0,0);
    int count = 0;
    int count2 = 0;
    for  (int j = 0; j < pos.size(); j++){
      float dist = pos.get(i).minus(pos.get(j)).length();
      
      //separation
      if (dist > .01 && dist < 50){
        Vec2 seperationForce = pos.get(i).minus(pos.get(j)).normalized();
        seperationForce.normalize();
        seperationForce.mul(100.0/pow(dist,1.5));
        acc.set(i,acc.get(i).plus(seperationForce));
      }
      
      //average position for attraction
      if (dist < 30 && dist > 0){
      avgPos.add(pos.get(j));
      count += 1;
      }
      
      //average velocity for alignment
      if (dist < 40 && dist > 0){
      avgVel.add(vel.get(j));
      count2 += 1;
      }
    }
    
    //attraction
    avgPos.mul(1.0/count);
    if (count >= 1){
      Vec2 attractionForce = avgPos.minus(pos.get(i));
      attractionForce.mul(0.1);
      attractionForce.clampToLength(maxForce);
      acc.set(i,acc.get(i).plus(attractionForce));
    }
    
    //alignment
    avgVel.mul(1.0/count2);
    if (count2 >= 1){
      Vec2 alignForce = avgVel.minus(vel.get(i));
      alignForce.normalize();
      acc.set(i,acc.get(i).plus(alignForce.times(4)));
    }
    
    //predator
    Vec2 repel = pos.get(i).minus(predPos);
    if (repel.length() < 200){
      acc.set(i,acc.get(i).plus(repel.normalized().times(400/repel.length())));
    }
    if (repel.length() < dangerZone){
      acc.set(i,acc.get(i).plus(repel.normalized().times(panicLevel/2)).plus(new Vec2(dangerZone,dangerZone).times(1/repel.length())));
    }
    
    //Goal Speed
    Vec2 goalSpeed = vel.get(i).normalized().times(panicLevel-vel.get(i).length());
    goalSpeed.clampToLength(maxForce);
    acc.set(i,acc.get(i).plus(goalSpeed));
    
    //oscillation
    Vec2 rightAngle = new Vec2(-vel.get(i).y,vel.get(i).x);
    float iosc = (osc+oscInd.get(i));
    if (iosc > 2*PI){
      iosc %= 2*PI;
    }
    oscVel.set(i,vel.get(i).plus(rightAngle.normalized().times(vel.get(i).length()/2*sin(iosc))));
    
    //edge return
    if (pos.get(i).x < xmin){
      acc.get(i).x += 2;
    }
    if (pos.get(i).x > xmax){
      acc.get(i).x -= 2;
    }
    if (pos.get(i).y < ymin){
      acc.get(i).y += 2;
    }
    //ground
    if (pos.get(i).y > height-300 && vel.get(i).y > 0 && pos.get(i).y < height){
      float pel = vel.get(i).y/((height)-pos.get(i).y);
      vel.set(i,vel.get(i).minus(new Vec2(0,pel)));
    }
    if (pos.get(i).y > height - 60){
      acc.get(i).y -= 5;
    }
    
    //eating
    if (pos.get(i).distanceTo(predPos) < 10){
      Vec2 temp = pos.get(pos.size()-1);
      pos.remove(pos.size()-1);
      pos.set(i,temp);
      temp = vel.get(pos.size()-1);
      vel.remove(pos.size()-1);
      vel.set(i,temp);
      temp = acc.get(pos.size()-1);
      acc.remove(pos.size()-1);
      acc.set(i,temp);
      temp = oscVel.get(pos.size()-1);
      oscVel.remove(pos.size()-1);
      oscVel.set(i,temp);
      int indTemp = oscInd.get(oscInd.size()-1);
      oscInd.remove(oscInd.size()-1);
      oscInd.set(i,indTemp);
      panicLevel += 2+dangerZone/200;
      dangerZone += 12;
    }
  }
  
  //update
  for (int i = 0; i < pos.size(); i++){
    pos.set(i,pos.get(i).plus(oscVel.get(i).times(dt)));
    vel.set(i,vel.get(i).plus(acc.get(i).times(dt)));
    
    //clamp max velocity
    if (vel.get(i).length() > panicLevel && pos.get(i).minus(predPos).length() > dangerZone){
      vel.set(i,vel.get(i).normalized().times(panicLevel));
    }
    else if (vel.get(i).length() > panicLevel + predVel.length()){
      vel.set(i,vel.get(i).normalized().times(panicLevel+predVel.length()));
    }
  }
  predPos.add(predVel.times(dt));
  predVel = (new Vec2(mouseX-predPos.x,mouseY-predPos.y));
  if (predVel.length() > 50){
    predVel.normalize();
    predVel.mul(50);
  }
  //if (frameRate < 30){
  //  println(frameRate);
  //}
  fill(0);
  if (frameRate < 30) fill(255,0,0);
  text("Framerate: "+frameRate, 50, 50);
  fill(0);
  text("Fish on screen: "+onScreen, 50, 90);
  text("Fish remaining: "+pos.size(), 50, 130);
  onScreen = 0;
}
