//Puck class

class Puck{
  PVector pos, vel;
  float rad = 25;
  PVector ul = new PVector(0,height/2-150);
  PVector ll = new PVector(0,height/2+150);
  PVector ur = new PVector(width,height/2-150);
  PVector lr = new PVector(width,height/2+150);
  int score1 = 0,score2 = 0;
  
  Puck(PVector startPos){
    pos = startPos;
    vel = new PVector(random(-1,1),random(-1,1));
  }
  
  void collide(Player p1){
    ArrayList<Integer> hitIndex = new ArrayList<Integer>();
    PVector playerAvgVel = new PVector(0,0);
    PVector direction = new PVector(0,0);
    PVector averageVel;
    float difference;
    PVector rightAngle;
    PVector projection;
    for (int i=0;i<p1.numNodes;i++){
      if (PVector.dist(pos, p1.pos[i]) < rad+p1.rad[i]){
        hitIndex.add(i);
        playerAvgVel.add(p1.vel[i]);
        direction.add(PVector.sub(pos,p1.pos[i]));
      }
    }
    if(hitIndex.size() == 0) return;
    playerAvgVel.mult(1/hitIndex.size());
    direction.normalize();
    averageVel = PVector.add(vel,playerAvgVel).mult(0.5);
    rightAngle = new PVector(direction.y,-direction.x);
    difference = PVector.dist(vel,playerAvgVel);
    projection = rightAngle.mult(PVector.dot(vel,rightAngle));
    pos.add(PVector.mult(direction,1));
    p1.Push(hitIndex,vel,pos,rad);
    vel.set(PVector.add(projection,PVector.add(averageVel,PVector.mult(direction,difference))).mult(0.8));
  }
  
  void boundary(){
    if (pos.y < 100+rad){
      pos.y = 101+rad;
      vel.y *= -.9;
    }
    
    if (pos.y > height-100-rad){
      pos.y = height-101-rad;
      vel.y *= -.9;
    }
    if (pos.x < 50+rad){
      if (pos.y>height/2+150 || pos.y<height/2-150){
        pos.x = 50+rad;
        vel.x *= -0.9;
      }
    }
    if (pos.x > width-50-rad){
      if (pos.y>height/2+150 || pos.y<height/2-150){
        pos.x = width-51-rad;
        vel.x *= -0.9;
      }
    }
    corner(ul);
    corner(ll);
    corner(ur);
    corner(lr);
  }
  
  void corner(PVector cc){
    if (PVector.dist(pos,cc) < 50+rad){
      PVector direction = PVector.sub(pos,cc).normalize();
      PVector projection = PVector.mult(direction,PVector.dot(vel,direction));
      vel.add(projection.mult(-1.8));
      pos.set(PVector.add(cc,direction.mult(51+rad)));
    }
  }
  
  void goal(){
    if (pos.x < -rad){
      score2++;
      pos = new PVector(width/2,height/2);
      vel = new PVector(random(-10,10),random(-10,10));
    }
    if (pos.x > width+rad){
      score1++;
      pos = new PVector(width/2,height/2);
      vel = new PVector(random(-10,10),random(-10,10));
    }
  }
  
  void update(float dt){
    vel.mult(0.9996);
    vel.limit(40);
    pos.add(PVector.mult(vel,dt));
    boundary();
    goal();
  }
  
  void render(){
    stroke(255);
    strokeWeight(3);
    fill(0);
    circle(pos.x,pos.y,rad*2);
  }
}
