//Obstacle Class

public class Obstacle {
  public boolean circle;
  public float rad;
  public PVector AA, BB;
  
  public Obstacle(){
    this.circle = true;
    this.AA = new PVector();
    this.BB = new PVector();
    this.rad = 0;
  }
  
  public Obstacle(boolean circle, PVector AA, PVector BB){
    this.circle = circle;
    this.AA = AA;
    this.BB = BB;
    this.rad = PVector.dist(AA,BB);
  }
  
  public void updateRadius(){
    this.rad = PVector.dist(AA,BB);
  }
  
  public void updateCorners(){
    float x1 = min(AA.x,BB.x);
    float y1 = min(AA.y,BB.y);
    float x2 = max(AA.x,BB.x);
    float y2 = max(AA.y,BB.y);
    this.AA = new PVector(x1,y1);
    this.BB = new PVector(x2,y2);
  }
  
  public boolean pointInside(PVector point){
    if (this.circle == true){
      if (PVector.dist(point, AA) <= this.rad+agentRad) return true;
    }
    else if (point.x >= AA.x-agentRad && point.x <= BB.x+agentRad && point.y >= AA.y-agentRad && point.y <= BB.y+agentRad) return true;
    return false;
  }
  
  public boolean intersect(PVector pointA, PVector dir, float t){
    boolean intersect = false;
    if (this.circle == true){
      intersect = rayCircleIntersect(this.AA, this.rad+agentRad, pointA, dir, t);
    }
    if (this.circle == false){
      intersect = rayBoxIntersect(this.AA.copy().add(new PVector(-agentRad,-agentRad)), this.BB.copy().add(new PVector(agentRad,agentRad)), pointA, dir, t);
    }
    return intersect;
  }
}

//modified CollisionLibrary from HW3 by Stephen J. Guy

boolean rayCircleIntersect(PVector center, float r, PVector pointA, PVector dir, float t){
  PVector W = PVector.sub(center, pointA);
  float a = 1;
  float b = -2*PVector.dot(dir,W);
  float c = W.magSq() - (r*r);
  float d = b*b - 4*a*c;
  
  if (d>=0){
    float t1 = (-b - sqrt(d))/(2*a);
    float t2 = (-b + sqrt(d))/(2*a);
    if (t1 > 0 && t1 < t) return true;
    else if (t1 < 0 && t2 > 0) return true;
  }
  return false;
}

boolean rayBoxIntersect(PVector AA, PVector BB, PVector pointA, PVector dir, float t){
  float t_left_x, t_right_x, t_top_y, t_bot_y;
  t_left_x = (AA.x - pointA.x)/dir.x;
  t_right_x = (BB.x - pointA.x)/dir.x;
  t_top_y = (AA.y - pointA.y)/dir.y;
  t_bot_y = (BB.y - pointA.y)/dir.y;
  
  float t_max_x = max(t_left_x,t_right_x);
  float t_max_y = max(t_top_y,t_bot_y);
  float t_max = min(t_max_x,t_max_y); //When the ray exists the box
  
  float t_min_x = min(t_left_x,t_right_x);
  float t_min_y = min(t_top_y,t_bot_y);
  float t_min = max(t_min_x,t_min_y); //When the ray enters the box
  
   //The the box is behind the ray (negative t)
  if (t_max < 0) return false;
  
  //The ray never hits the box
  if (t_min > t_max) return false;
  //The ray hits, but further out than max_t
  if (t_min > t) return false;
  return true;
}
