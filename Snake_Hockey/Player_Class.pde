//Player Class

class Player{
  int numNodes;
  PVector pos[], vel[], acc[], positiveMovement, negativeMovement, direction;
  float maxVel, speed, restLength, k, kv, rad[], stiffness = 1;
  char controlUp, controlDown, controlLeft, controlRight;
  PVector ul = new PVector(0,height/2-150);
  PVector ll = new PVector(0,height/2+150);
  PVector ur = new PVector(width,height/2-150);
  PVector lr = new PVector(width,height/2+150);
  
  Player(char up, char down, char left, char right, PVector pos, PVector direction, float maxVel, float speed, float restLength, int numNodes){
    controlUp = up;
    controlDown = down;
    controlLeft = left;
    controlRight = right;
    k = 300;
    kv = 0.1;
    this.restLength = restLength;
    this.numNodes = numNodes;
    this.maxVel = maxVel;
    this.speed = speed;
    positiveMovement = new PVector(0,0);
    negativeMovement = new PVector(0,0);
    this.direction = direction;
    this.rad = new float[numNodes];
    this.pos = new PVector[numNodes];
    this.vel = new PVector[numNodes];
    this.acc = new PVector[numNodes];
    for (int i=0; i<numNodes; i++){
      this.pos[i] = pos.copy().add(PVector.mult(direction,-restLength*i));
      this.vel[i] = new PVector(0,0);
      this.acc[i] = new PVector(0,0);
      this.rad[i] = 10;
    }
  }
  
  void Update(float dt){
    PVector move = new PVector(negativeMovement.x + positiveMovement.x, negativeMovement.y + positiveMovement.y).setMag(speed);
    
    //Calculate accelerations
    for (int i=0;i<numNodes;i++){
      PVector force = new PVector(0,0);
      
      //Spring forces
      if (i > 0){
        force.add(PVector.sub(pos[i],pos[i-1]).setMag(PVector.dist(pos[i],pos[i-1])-restLength).mult(-k));
        //force.add(PVector.sub(pos[i],pos[i-1]).setMag(-kv*vel[i].mag()*cos(PVector.angleBetween(vel[i],PVector.sub(pos[i],pos[i-1])))));
      }
      if (i < numNodes-1){
        force.add(PVector.sub(pos[i],pos[i+1]).setMag(PVector.dist(pos[i],pos[i+1])-restLength).mult(-k));
        //force.add(PVector.sub(pos[i],pos[i+1]).setMag((-kv/20)*vel[i].mag()*cos(PVector.angleBetween(vel[i],PVector.sub(pos[i],pos[i+1])))));
      }
      
      //Skip link
      if (i > 1){
        force.add(PVector.sub(pos[i],pos[i-2]).setMag(PVector.dist(pos[i],pos[i-2])-2*restLength).mult(-k));
        //force.add(PVector.sub(pos[i],pos[i-2]).setMag(-kv*vel[i].mag()*cos(PVector.angleBetween(vel[i],PVector.sub(pos[i],pos[i-2])))));
      }
      if (i < numNodes-2){
        force.add(PVector.sub(pos[i],pos[i+2]).setMag(PVector.dist(pos[i],pos[i+2])-2*restLength).mult(-k));
        //force.add(PVector.sub(pos[i],pos[i+2]).setMag(-kv*vel[i].mag()*cos(PVector.angleBetween(vel[i],PVector.sub(pos[i],pos[i+2])))));
      }
      
      //Dampening
      force.add(PVector.mult(vel[i],-20*(0.002+pow((numNodes-i)/numNodes,2))));
      force.add(PVector.mult(vel[i],-stiffness*4));
      
      //Acceleration
      acc[i] = force;
    }
    
    //Integrate velocities
    if (stiffness > 0.001) stiffness*=0.995;
    else stiffness = 0;
    vel[0].mult(0.999);
    vel[0].add(move.mult(dt));
    for (int i=0;i<numNodes;i++){
      vel[i] = PVector.add(vel[i],PVector.mult(acc[i],dt*i/10));
      if (vel[i].mag() > maxVel*(1+i/numNodes)/2) vel[i].setMag(maxVel*(1+i/numNodes)/2);
    }
    
    for (int i=1;i<numNodes;i++){
      rad[i] = 11 / (0.1+PVector.dist(pos[i],pos[i-1])/restLength);
      if (rad[i] > 12) rad[i] = 12;
    }
    
    //integrate position
    for (int i=0;i<numNodes;i++){
      pos[i] = PVector.add(pos[i],PVector.mult(vel[i],dt));
    }
    
    boundary();
    direction.set(PVector.sub(pos[0],pos[1]).normalize());
  }
  
  void Push(ArrayList<Integer> hitIndex, PVector puckVel, PVector puckPos, float puckRad){
    for (int i:hitIndex){
      float dist = PVector.dist(pos[i],puckPos);
      if (dist < rad[i] + puckRad) pos[i].set(PVector.sub(pos[i],puckPos).setMag(puckRad+rad[i]+1).add(puckPos));
      PVector average = PVector.add(puckVel,vel[i]).mult(0.5);
      PVector direc = PVector.sub(pos[i],puckPos).normalize();
      PVector rightAngle = new PVector(direc.y,-direc.x);
      float difference = PVector.dist(puckVel,vel[i])/hitIndex.size();
      PVector projection = rightAngle.mult(PVector.dot(vel[i],rightAngle));
      vel[i].set(PVector.add(projection,PVector.add(average,PVector.mult(direction,difference))).mult(.4));
      if (i == 0) stiffness = 1;
    }
  }
  
  void playerCollide(Player p2){
    for (int i=0;i<numNodes;i++){
      for (int j=0;j<p2.numNodes;j++){
        float dist = PVector.dist(pos[i],p2.pos[j]);
        if (dist < rad[i]+p2.rad[j]){
          vel[i].add(PVector.sub(pos[i],p2.pos[j]).setMag(20/pow(dist,2)));
        }
      }
    }
  }
  
  void boundary(){
    for (int i=0;i<numNodes;i++){
      if (pos[i].x < rad[i]){
        pos[i].x = 1+rad[i];
        vel[i].x *= -.9;
        if (i == 0) stiffness = 1;
      }
      
      if (pos[i].x > width-rad[i]){
        pos[i].x = width-1-rad[i];
        vel[i].x *= -.9;
        if (i == 0) stiffness = 1;
      }
      
      if (pos[i].x < 50+rad[i]){
        if (pos[i].y>height/2+150 || pos[i].y<height/2-150){
          pos[i].x = 50+rad[i];
          vel[i].x *= -0.9;
          if (i == 0) stiffness = 1;
        }
      }
      if (pos[i].x > width-50-rad[i]){
        if (pos[i].y>height/2+150 || pos[i].y<height/2-150){
          pos[i].x = width-51-rad[i];
          vel[i].x *= -0.9;
          if (i == 0) stiffness = 1;
        }
      }
      
      if (pos[i].y < 99+rad[i]){
        pos[i].y = 100+rad[i];
        vel[i].y *= -.9;
        if (i == 0) stiffness = 1;
      }
      
      if (pos[i].y > height-99-rad[i]){
        pos[i].y = height-100-rad[i];
        vel[i].y *= -.9;
        if (i == 0) stiffness = 1;
      }
      corner(ul,pos[i],vel[i],rad[i],i);
      corner(ll,pos[i],vel[i],rad[i],i);
      corner(ur,pos[i],vel[i],rad[i],i);
      corner(lr,pos[i],vel[i],rad[i],i);
    }
  }
  
  void corner(PVector cc,PVector pos,PVector vel, float rad, int i){
    if (PVector.dist(pos,cc) < 49+rad){
      PVector direction = PVector.sub(pos,cc).normalize();
      PVector projection = PVector.mult(direction,PVector.dot(vel,direction));
      vel.add(projection.mult(-1.8));
      pos.set(PVector.add(cc,direction.mult(50+rad)));
      if (i == 0) stiffness = 1;
    }
  }
  
  void Render(int r, int g, int b){
    noStroke();
    fill(r,g,b);
    for (int i=0;i<numNodes;i++){
      circle(pos[i].x,pos[i].y,rad[i]*2);
      if(i>0){
        circle((pos[i].x+pos[i-1].x)/2,(pos[i].y+pos[i-1].y)/2,rad[i]*2);
      }
    }
    fill(0);
    PVector perpendicular = new PVector(direction.y,-direction.x).setMag(5);
    PVector eye1 = PVector.add(pos[0],perpendicular);
    PVector eye2 = PVector.sub(pos[0],perpendicular);
    circle(eye1.x,eye1.y,5);
    circle(eye2.x,eye2.y,5);
  }
  
  void HandleKeyPress(){
    if (key == this.controlUp) negativeMovement.y = -1;
    if (key == this.controlDown) positiveMovement.y = 1;
    if (key == this.controlLeft) negativeMovement.x = -1;
    if (key == this.controlRight) positiveMovement.x = 1;
  }
  
  void HandleKeyRelease(){
    if (key == this.controlUp) negativeMovement.y = 0;
    if (key == this.controlDown) positiveMovement.y = 0;
    if (key == this.controlLeft) negativeMovement.x = 0;
    if (key == this.controlRight) positiveMovement.x = 0;
  }
}
