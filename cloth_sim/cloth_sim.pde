//Cloth Simulation
//Created for CSCI 5611 project 2 by Michael Nguyen

//Setup values
static int row = 15;
static int column = 20;
Camera cam;

//Variables:
int moveBall = 0;
float gravity = 30;
float restLength = 4;
float k = 1000;
float kv = 50;
float pointMass = 4;
PVector pos[][] = new PVector[row][column];
PVector vel[][] = new PVector[row][column];
PVector obstaclePos = new PVector(column*restLength/2, 40, 30);
float sphereRad = 20;

void keyPressed()
{
  cam.HandleKeyPressed();
  if(key == 'j'){
    moveBall = 1;
  }
  if(key == 'k'){
    moveBall = -1;
  }
  if(key == 'r'){
    reset();
  }
}

void keyReleased()
{
  cam.HandleKeyReleased();
  if(key == 'j'){
    moveBall = 0;
  }
  if(key == 'k'){
    moveBall = 0;
  }
}

void reset(){
  for (int i = 0; i < row; i++){
    for (int j = 0; j < column; j++){
      pos[i][j] = new PVector(j*restLength+i, 0, i*restLength);
      vel[i][j] = new PVector(0,0,0);
    }
  }
}

void setup(){
  size(800, 600, P3D);
  noStroke();
  cam = new Camera();
  for (int i = 0; i < row; i++){
    for (int j = 0; j < column; j++){
      pos[i][j] = new PVector(j*restLength+i, 0, i*restLength);
      vel[i][j] = new PVector(0,0,0);
    }
  }
}

/////////////////////////////////////////////////////////////////////////////////////

void update(float dt){
  PVector v1[][] = new PVector[row][column];
  PVector acc[][] = new PVector[row][column];
  
  forces(pos,acc);
  integrateVel(v1,acc,dt);
  integratePos(pos,v1,dt);
  integrateVel(vel,acc,dt);
  
  for (int i = 0; i < row; i++){
    for (int j = 0; j < column; j++){
      //Boundary conditions
      if (PVector.dist(pos[i][j],obstaclePos) < sphereRad+0.7){
        PVector normal = PVector.sub(pos[i][j],obstaclePos).normalize();
        pos[i][j].set(PVector.mult(normal,sphereRad+0.7).add(obstaclePos));
        vel[i][j].add(normal.mult(-1.2*normal.dot(vel[i][j])));
      }
    }
  }
}

void forces(PVector midPos[][],PVector acc[][]){
  for (int i = 0; i < row; i++){
    for (int j = 0; j < column; j++){
      //Calculate forces
      PVector force = new PVector(0,0,0);
      
      //Springs
      if (i > 0){
        force.add(PVector.sub(midPos[i][j],midPos[i-1][j]).normalize().mult(PVector.dist(midPos[i][j],midPos[i-1][j])-restLength).mult(-k));
        force.add(PVector.sub(midPos[i][j],midPos[i-1][j]).normalize().mult(-kv*vel[i][j].mag()*cos(PVector.angleBetween(vel[i][j],PVector.sub(midPos[i][j],midPos[i-1][j])))));
      }
      if (i < row-1){
        force.add(PVector.sub(midPos[i][j],midPos[i+1][j]).normalize().mult(PVector.dist(midPos[i][j],midPos[i+1][j])-restLength).mult(-k));
        force.add(PVector.sub(midPos[i][j],midPos[i+1][j]).normalize().mult(-kv*vel[i][j].mag()*cos(PVector.angleBetween(vel[i][j],PVector.sub(midPos[i][j],midPos[i+1][j])))));
      }
      if (j > 0){
        force.add(PVector.sub(midPos[i][j],midPos[i][j-1]).normalize().mult(PVector.dist(midPos[i][j],midPos[i][j-1])-restLength).mult(-k));
        force.add(PVector.sub(midPos[i][j],midPos[i][j-1]).normalize().mult(-kv*vel[i][j].mag()*cos(PVector.angleBetween(vel[i][j],PVector.sub(midPos[i][j],midPos[i][j-1])))));
      }
      if (j < column-1){
        force.add(PVector.sub(midPos[i][j],midPos[i][j+1]).normalize().mult(PVector.dist(midPos[i][j],midPos[i][j+1])-restLength).mult(-k));
        force.add(PVector.sub(midPos[i][j],midPos[i][j+1]).normalize().mult(-kv*vel[i][j].mag()*cos(PVector.angleBetween(vel[i][j],PVector.sub(midPos[i][j],midPos[i][j+1])))));
      }
      //Crosslink
      if (i > 1 && i < row-2){
        force.add(PVector.sub(midPos[i][j],midPos[i-2][j]).normalize().mult(PVector.dist(midPos[i][j],midPos[i-2][j])-2*restLength).mult(-k));
        force.add(PVector.sub(midPos[i][j],midPos[i-2][j]).normalize().mult(-kv*vel[i][j].mag()*cos(PVector.angleBetween(vel[i][j],PVector.sub(midPos[i][j],midPos[i-2][j])))));
        force.add(PVector.sub(midPos[i][j],midPos[i+2][j]).normalize().mult(PVector.dist(midPos[i][j],midPos[i+2][j])-2*restLength).mult(-k));
        force.add(PVector.sub(midPos[i][j],midPos[i+2][j]).normalize().mult(-kv*vel[i][j].mag()*cos(PVector.angleBetween(vel[i][j],PVector.sub(midPos[i][j],midPos[i+2][j])))));
      }
      if (j > 1 && j < column-2){
        force.add(PVector.sub(midPos[i][j],midPos[i][j-2]).normalize().mult(PVector.dist(midPos[i][j],midPos[i][j-2])-2*restLength).mult(-k));
        force.add(PVector.sub(midPos[i][j],midPos[i][j-2]).normalize().mult(-kv*vel[i][j].mag()*cos(PVector.angleBetween(vel[i][j],PVector.sub(midPos[i][j],midPos[i][j-2])))));
        force.add(PVector.sub(midPos[i][j],midPos[i][j+2]).normalize().mult(PVector.dist(midPos[i][j],midPos[i][j+2])-2*restLength).mult(-k));
        force.add(PVector.sub(midPos[i][j],midPos[i][j+2]).normalize().mult(-kv*vel[i][j].mag()*cos(PVector.angleBetween(vel[i][j],PVector.sub(midPos[i][j],midPos[i][j+2])))));
      }
      if (i > 1 && j > 1){
        if (PVector.dist(midPos[i][j],midPos[i-2][j-2])-2*sqrt(2)*restLength < -1 || PVector.dist(midPos[i][j],midPos[i-2][j-2])-2*sqrt(2)*restLength > 1){
          force.add(PVector.sub(midPos[i][j],midPos[i-2][j-2]).normalize().mult(PVector.dist(midPos[i][j],midPos[i-2][j-2])-2*sqrt(2)*restLength).mult(-k/8));
          force.add(PVector.sub(midPos[i][j],midPos[i-2][j-2]).normalize().mult(-kv*vel[i][j].mag()*cos(PVector.angleBetween(vel[i][j],PVector.sub(midPos[i][j],midPos[i-2][j-2])))));
        }
      }
      if (i < row-2 && j < column-20){
        if (PVector.dist(midPos[i][j],midPos[i+2][j+2])-2*sqrt(2)*restLength < -1 || PVector.dist(midPos[i][j],midPos[i+2][j+2])-2*sqrt(2)*restLength > 1){
          force.add(PVector.sub(midPos[i][j],midPos[i+2][j+2]).normalize().mult(PVector.dist(midPos[i][j],midPos[i+2][j+2])-2*sqrt(2)*restLength).mult(-k/8));
          force.add(PVector.sub(midPos[i][j],midPos[i+2][j+2]).normalize().mult(-kv*vel[i][j].mag()*cos(PVector.angleBetween(vel[i][j],PVector.sub(midPos[i][j],midPos[i+2][j+2])))));
        }
      }
      if (i > 1 && j < column-2){
        if (PVector.dist(midPos[i][j],midPos[i-2][j+2])-2*sqrt(2)*restLength < -1 || PVector.dist(midPos[i][j],midPos[i-2][j+2])-2*sqrt(2)*restLength > 1){
          force.add(PVector.sub(midPos[i][j],midPos[i-2][j+2]).normalize().mult(PVector.dist(midPos[i][j],midPos[i-2][j+2])-2*sqrt(2)*restLength).mult(-k/8));
          force.add(PVector.sub(midPos[i][j],midPos[i-2][j+2]).normalize().mult(-kv*vel[i][j].mag()*cos(PVector.angleBetween(vel[i][j],PVector.sub(midPos[i][j],midPos[i-2][j+2])))));
        }
      }
      if (i < row-2 && j > 1){
        if (PVector.dist(midPos[i][j],midPos[i+2][j-2])-2*sqrt(2)*restLength < -1 || PVector.dist(midPos[i][j],midPos[i+2][j-2])-2*sqrt(2)*restLength > 1){
          force.add(PVector.sub(midPos[i][j],midPos[i+2][j-2]).normalize().mult(PVector.dist(midPos[i][j],midPos[i+2][j-2])-2*sqrt(2)*restLength).mult(-k/8));
          force.add(PVector.sub(midPos[i][j],midPos[i+2][j-2]).normalize().mult(-kv*vel[i][j].mag()*cos(PVector.angleBetween(vel[i][j],PVector.sub(midPos[i][j],midPos[i+2][j-2])))));
        }
      }
      
      //Integration
      acc[i][j] = new PVector(0,gravity,0);
      acc[i][j].add(force.mult(0.5/pointMass));
    }
  }
}

void integrateVel(PVector midVel[][], PVector acc[][], float dt){
  for (int i = 0; i < row; i++){
    for (int j = 0; j < column; j++){
      midVel[i][j] = PVector.add(vel[i][j],acc[i][j].mult(dt));
    }
  }
}

void integratePos(PVector midPos[][], PVector midVel[][], float dt){
  for (int i = 0; i < row; i++){
    for (int j = 0; j < column; j++){
      if (i > 0){
        midPos[i][j] = PVector.add(pos[i][j], midVel[i][j].mult(dt));
      }
    }
  }
}

/////////////////////////////////////////////////////////////////////////////////////

void draw(){
  cam.Update(1.0/frameRate);
  obstaclePos.z += 40*moveBall/frameRate;
  for (int i = 0; i < 30; i++){
    update(1.0/frameRate);
  }
  
  ambientLight(100,100,100);
  directionalLight(200, 200, 200, -1, 1, -2);
  
  background(200);
  for (int i = 0; i < row-1; i++){
    for (int j = 0; j < column-1; j++){
      fill(i*255/row,0,j*255/column);
      beginShape(TRIANGLES);
      vertex(pos[i][j].x,pos[i][j].y,pos[i][j].z);
      vertex(pos[i+1][j].x,pos[i+1][j].y,pos[i+1][j].z);
      vertex(pos[i][j+1].x,pos[i][j+1].y,pos[i][j+1].z);
      vertex(pos[i+1][j].x,pos[i+1][j].y,pos[i+1][j].z);
      vertex(pos[i+1][j+1].x,pos[i+1][j+1].y,pos[i+1][j+1].z);
      vertex(pos[i][j+1].x,pos[i][j+1].y,pos[i][j+1].z);
      endShape();
    }
  }
  
  pushMatrix();
  fill(50,255,100);
  translate(obstaclePos.x,obstaclePos.y,obstaclePos.z);
  sphere(sphereRad);
  popMatrix();
  
}
