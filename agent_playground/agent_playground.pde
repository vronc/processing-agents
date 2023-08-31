import java.util.*;


boolean debugWander=false;
boolean debugAvoidAgents=true;
boolean debugCohesion=true;
boolean debugResult=false;
  
float x;
int numAgents = 100;
Agent[] agents = new Agent[numAgents];
int dimX = 1250;
int dimY = 700;
int gridPartitionX = 90;
int gridPartitionY = 50;
float gridCellDimX = dimX / (gridPartitionX-1);
float gridCellDimY = dimY / (gridPartitionY-1);
GridCell[][] grid = new GridCell[gridPartitionX][gridPartitionY];
boolean saveImage = false;
boolean drawTrails = true;
float timing=0;
float prevTiming;


void settings() {
  size(dimX, dimY);
  pixelDensity(displayDensity());
}

void setup() {
  x = 0;
  
  for (int i=0 ; i<gridPartitionX ; i++) {
    for (int j=0 ; j<gridPartitionY ; j++) {
      grid[i][j] = new GridCell(i * gridCellDimX, j * gridCellDimY, i, j);
    }
  }
  
  for (int i=0 ; i<gridPartitionX ; i++) {
    for (int j=0 ; j<gridPartitionY ; j++) {
      grid[i][j].setNeighbours();
    }
  }
  
  
  for (int i = 0 ; i < agents.length ; i++) {
    agents[i] = new Agent(random(dimX), random(dimY));
  }
}

void draw() {
  x+=0.1f;
  //background(sin(x-x/0.9)*255);
  background(240);
 
  for (int i = 0 ; i < agents.length ; i++) {
    agents[i].update();
    agents[i].draw();
  }
  
  if (saveImage) {
    save("pattern" + random(1000000) + ".png");
    saveImage=false;
  }
  
  prevTiming = timing;
  timing=millis();
  fill(0,0,0);
  text(timing-prevTiming, 30,30);
}

void mouseClicked() {
  saveImage=true;
} 

class Agent {
  PVector position;
  PVector[] posTrail;
  int posTrailLength = 3;
  PVector velocity;
  PVector acceleration;
  float speed = 2;
  float agentSize;
  GridCell locationCell;
  float maxForce = 4;
  PVector wanderTarget;
  GridCell newLocationCell;
  int updateInc;
  color agentColor;
  float sensorRange;
  PVector direction;
  
  Agent(float x, float y) {
    position = new PVector(x, y);
    velocity = new PVector(random(-1f,1f), random(-1f,1f));
    velocity.setMag(speed);
    acceleration = new PVector(0, 0);
    agentSize = random(4f,7f);
    agentColor = color(random(255), random(255), random(255));
    locationCell = getCellFromPosition(position);
    posTrail = new PVector[posTrailLength];
    for (int i=0 ; i<posTrail.length ; i++)
      posTrail[i] = new PVector(position.x, position.y);
    updateInc = 0;
    sensorRange = agentSize+10f;
    wanderTarget = new PVector(random(0,dimX), random(0,dimY));
  }
  
  void draw() {
    noStroke();
    // fill(tan(x)*255,sin(x)*255,cos(x)*255,100);
    // ellipse(position.x, position.y, agentSize+6f, agentSize+6f);
    // fill(tan(x)*255,cos(x)*255,sin(x)*255,100);
    fill(agentColor);
    ellipse(position.x, position.y, agentSize, agentSize);
    if (drawTrails) {
      for (PVector pos : posTrail) {
        ellipse(pos.x+random(-0.5f,0.5f), pos.y+random(-0.5f,0.5f), agentSize/2f, agentSize/2f);
      }
    }
  }
   
   void applyForce(PVector force) {
     acceleration.add(force);
   }
  
  void update() {
    PVector coh = cohesion();
    PVector avb = avoidBoundaries();
    PVector ava = avoidAgents();
    
    // TODO: change z-position
    //agentSize=position.z*2;
    
    // TODO: make different forces have priority over others
    double resultDistribution = 1.0;
    
    PVector result = new PVector(0, 0);
    int targets = 0;
    
    if (avb.mag() > 0) {
      result.add(avb);
      resultDistribution -= 1;
      targets += 1;
    } else {
    
      if (ava.mag() > 0) {
        result.add(ava);
        targets += 1;
        resultDistribution -= 0.5;
      }
      
      if (coh.mag() > 0) {
        result.add(coh);
        targets += 1;
        resultDistribution -= 0.1;
      }
      
      PVector wanderTarget = getWanderTarget();
      result.add(wanderTarget);
      targets += 1;
    }
    
    stroke(50);
    if (debugResult) line(position.x, position.y, result.x, result.y);
    
    seek(result.div(targets));
    
    velocity.add(acceleration);
    velocity.limit(speed);
    position.add(velocity);
    acceleration.mult(0);

    if (updateInc > posTrailLength-1)
      updateInc = 0;
    
    posTrail[updateInc].x = position.x;
    posTrail[updateInc].y = position.y;
    updateInc++;
    
    newLocationCell = getCellFromPosition(position); //<>//

    if (locationCell != newLocationCell) {
      locationCell.removeAgent(this); //<>//
      newLocationCell.addAgent(this); //<>//
      locationCell = newLocationCell;
    }
  }
  
  // calculate steering target to avoid boundaries.
  PVector avoidBoundaries() {
    PVector target = new PVector(0, 0);
    int edgeWidth = 10;


    // boundary check
    if (position.x > (dimX - edgeWidth)) 
      target = PVector.add(new PVector(-speed, velocity.y), this.position);
    else if (position.x < edgeWidth) 
      target = PVector.add(new PVector(speed, velocity.y), this.position);
    if (position.y > (dimY - edgeWidth)) 
      target = PVector.add(new PVector(velocity.x, -speed), this.position);
    else if (position.y < edgeWidth) 
      target = PVector.add(new PVector(velocity.x, speed), this.position); 
      
    if (target.mag() > 0) {
      return target;
    }
      
    return new PVector(0,0);
  }
  
  PVector avoidAgents() { //<>//
    PVector target = new PVector(0,0);
    PVector desired = new PVector(0,0);
    for (GridCell cell : this.locationCell.selfAndNeighbours) {
      for(Agent agent : cell.agents) {
        if (PVector.dist(agent.position, this.position) < sensorRange) {
          desired = desired.add(PVector.sub(this.position, agent.position));
        }
      }
    }
    
    if (desired.mag() == 0) {
      return desired;
    }
    
    target = PVector.add(this.position, desired);
    
    if (target.mag() > 0 && debugAvoidAgents) {
      stroke(agentColor);
      line(position.x, position.y, target.x, target.y);
    }
    return target;
  }
  
  PVector getWanderTarget() {
    float wanderOffset = 40f;
    float wanderRadius = 20f;
    PVector wanderOffsetPos = PVector.add(position, velocity.copy().setMag(wanderOffset));
    
    if (random(1, 10) > 7) {
      float angle = random(-1f, 1f)*PI*2f;
      wanderTarget = PVector.add(
        wanderOffsetPos,
        new PVector(cos(angle)*wanderRadius, sin(angle)*wanderRadius)
      );
    }
    
    if (debugWander) {
      ellipse(wanderOffsetPos.x, wanderOffsetPos.y, wanderRadius*2f, wanderRadius*2f);
      fill(255,0,0);
      ellipse(wanderTarget.x, wanderTarget.y, 5f, 5f);
      fill(0,0,255);
    }
    
    return wanderTarget;
  }
  

  
   PVector cohesion() {
    float neighborDist = 50;
    PVector target = new PVector(0, 0);   // Start with empty vector to accumulate all positions
    int count = 0;
    for (GridCell cell : locationCell.selfAndNeighbours) {
      for (Agent other : cell.agents) {
        float d = PVector.dist(position, other.position);
        if ((d > 0) && (d < neighborDist)) {
          target.add(other.position);
          count++;
        }
      }
    }
    if (count > 0) {
      target.div(count);
      stroke(agentColor);
      if (debugCohesion) line(position.x, position.y, target.x, target.y);
      return target;
    } 
    return new PVector(0,0);
  }
  
  void seek(PVector target) {
    PVector desired = PVector.sub(target, position);
    desired.normalize();
    desired.mult(speed);
    PVector steer = PVector.sub(desired, velocity);
    applyForce(steer.limit(maxForce));
  }
}

class GridCell {
  ArrayList<Agent> agents;
  ArrayList<GridCell> selfAndNeighbours = new ArrayList();
  float startX;
  float startY;
  int gridI;
  int gridJ;
  
  GridCell(float inStartX, float inStartY, int i, int j) {
    agents = new ArrayList<Agent>();
    startX = inStartX;
    startY = inStartY;
    gridI = i;
    gridJ = j;
  }
  
  void setNeighbours() {
    for (int dx=-1 ; dx<2 ; dx++) {
      for (int dy=-1 ; dy<2 ; dy++) {  
        int x = gridI+dx;
        int y = gridJ+dy;
        if (x > -1 && x < gridPartitionX && y > -1 && y < gridPartitionY) {
          selfAndNeighbours.add(grid[x][y]);
        }
      }
    }
  }
  
  void addAgent(Agent agent) {
    agents.add(agent);
  }
  
  void removeAgent(Agent agent) {
    agents.remove(agent);
  }
}
 
GridCell getCellFromPosition(PVector position) {
  int gridCellXInd = int(position.x/gridCellDimX);
  int gridCellYInd = int(position.y/gridCellDimY);
  
  if (gridCellXInd < 0) gridCellXInd = 0;
  if (gridCellXInd >= gridPartitionX) gridCellXInd = gridPartitionX-1;
  
  if (gridCellYInd < 0) gridCellYInd = 0;
  if (gridCellYInd >= gridPartitionY) gridCellYInd = gridPartitionY-1;
  
  return grid[gridCellXInd][gridCellYInd];
}
