//CSci 5611 Final Project
//Michael Nguyen

//Setup

//Variables
Player p1,p2;
Puck puck;
PImage bg;

void setup(){
  size(1200,800);
  p1 = new Player('w','s','a','d', new PVector(width/2-300,height/2), new PVector(1,0),40,1,10,15);
  p2 = new Player('i','k','j','l', new PVector(width/2+300,height/2), new PVector(-1,0),40,1,10,15);
  puck = new Puck(new PVector(width/2,height/2));
  noStroke();
  textFont(createFont("Arial",32,true));
  bg = loadImage("bg.jpg");
}

void keyPressed(){
  p1.HandleKeyPress();
  p2.HandleKeyPress();
}

void keyReleased(){
  p1.HandleKeyRelease();
  p2.HandleKeyRelease();
}

void draw(){
 background(25);
 fill(124);
 rect(0,height/2-150,width,300);
 //rect(50,100,width-100,height-200);
 image(bg,50,100);
 fill(25);
 circle(0,height/2-150,100);
 circle(0,height/2+150,100);
 circle(width,height/2-150,100);
 circle(width,height/2+150,100);
 float dt = 1/frameRate;
 for (int i=0;i<30;i++){
   p1.Update(dt);
   p2.Update(dt);
   puck.update(dt);
   puck.collide(p1);
   puck.collide(p2);
   p1.playerCollide(p2);
   p2.playerCollide(p1);
 }
 puck.render();
 p1.Render(150,255,150);
 p2.Render(255,150,150);
 fill(150,255,150);
 text("Player1 - " + puck.score1, 50,50);
 fill(255,150,150);
 text("Player2 - " + puck.score2, width-200,50);
}
