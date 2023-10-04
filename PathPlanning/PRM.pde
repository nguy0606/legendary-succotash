ArrayList<Integer>[] neighbors = new ArrayList[numNodes];


void connectNeighbors(PVector[] nodePos, int numNodes){
  for (int i = 0; i < numNodes; i++){
    neighbors[i] = new ArrayList<Integer>();
    for (int j = 0; j < numNodes; j++){
      if(i == j) continue;
      PVector dir = PVector.sub(nodePos[j],nodePos[i]).normalize();
      float t = PVector.dist(nodePos[i],nodePos[j]);
      boolean skip = false;
      for (int k = 0; k < obstacles.size(); k++){
        if (obstacles.get(k).intersect(nodePos[i], dir, t)){
          skip = true;
          break;
        }
      }
      if (skip == true) continue;
      neighbors[i].add(j);
    }
  }
}

ArrayList<Integer> planPath(PVector startPos, PVector goalPos, PVector[] nodePos, int numNodes){
  ArrayList<Integer> path = new ArrayList();
  connectNeighbors(nodePos, numNodes);
  path = runAStar(nodePos, numNodes, startPos, goalPos);
  return path;
}

ArrayList<Integer> runAStar(PVector[] nodePos, int numNodes, PVector startPos, PVector goalPos){
  ArrayList<Integer> path = new ArrayList();
  ArrayList<Integer> fringe = new ArrayList();
  ArrayList<Float> priority = new ArrayList();
  boolean[] visited = new boolean[numNodes+2];
  boolean goalReached = false;
  int[] parent = new int[numNodes+2];
  ArrayList<Integer> startNeighbors = new ArrayList();
  ArrayList<Integer> goalNeighbors = new ArrayList();
  
  for (int j = 0; j < numNodes; j++){
    PVector dir = PVector.sub(nodePos[j],startPos).normalize();
    float t = PVector.dist(startPos,nodePos[j]);
    boolean skip = false;
    for (int k = 0; k < obstacles.size(); k++){
      if (obstacles.get(k).intersect(startPos, dir, t)){
        skip = true;
        break;
      }
    }
    if (skip == true) continue;
    startNeighbors.add(j);
  }
  
  {
    PVector dir = PVector.sub(goalPos,startPos).normalize();
    float t = PVector.dist(goalPos,startPos);
    boolean skip = false;
    for (int k = 0; k < obstacles.size(); k++){
      if(obstacles.get(k).intersect(startPos,dir,t)){
        skip = true;
        break;
      }
    }
    if (skip == false){
      startNeighbors.add(numNodes);
    }
  }
  
  for (int j = 0; j < numNodes; j++){
    PVector dir = PVector.sub(nodePos[j],goalPos).normalize();
    float t = PVector.dist(goalPos,nodePos[j]);
    boolean skip = false;
    for (int k = 0; k < obstacles.size(); k++){
      if (obstacles.get(k).intersect(goalPos, dir, t)){
        skip = true;
        break;
      }
    }
    if (skip == true) continue;
    goalNeighbors.add(j);
  }
  
  for (int i:goalNeighbors){
    neighbors[i].add(0,numNodes);
  }
  
  for (int i = 0; i < numNodes+2; i++){
    visited[i] = false;
    parent[i] = -1;
  }
  
  visited[numNodes+1] = true;
  fringe.add(numNodes+1);
  priority.add(PVector.dist(startPos,goalPos));
  
  while (fringe.size() > 0){
    int min = 0;
    float minCost = 999999;
    for (int i = 0; i < fringe.size(); i++){
      if (priority.get(i) < minCost){
        min = i;
        minCost = priority.get(i);
      }
    }
    int currentNode = fringe.get(min);
    fringe.remove(min);
    priority.remove(min);
    if (currentNode == numNodes){
      goalReached = true;
      break;
    }
    ArrayList<Integer> currentNeighbors;
    if (currentNode == numNodes+1) currentNeighbors = startNeighbors;
    else if (currentNode == numNodes) currentNeighbors = goalNeighbors;
    else currentNeighbors = neighbors[currentNode];
    for (int i = 0; i < currentNeighbors.size(); i++){
      int neighborNode = currentNeighbors.get(i);
      if (!visited[neighborNode]){
        visited[neighborNode] = true;
        parent[neighborNode] = currentNode;
        int prevNode = neighborNode;
        float cost = 0.0;
        PVector neighborPos;
        if (neighborNode == numNodes) neighborPos = goalPos;
        else neighborPos = nodePos[neighborNode];
        cost += PVector.dist(neighborPos,goalPos);
        while (parent[prevNode] >= 0){
          PVector parentPos;
          PVector currentPos;
          if (parent[prevNode] == numNodes+1) parentPos = startPos;
          else parentPos = nodePos[parent[prevNode]];
          if (prevNode == numNodes) currentPos = goalPos;
          else currentPos = nodePos[prevNode];
          cost += PVector.dist(currentPos, parentPos);
          prevNode = parent[prevNode];
        }
        priority.add(cost);
        fringe.add(neighborNode);
      }
      //try
      if (visited[neighborNode] && currentNode < numNodes){
        int prevNode = neighborNode;
        float cost1 = 0.0;
        while (parent[prevNode] >= 0){
          PVector parentPos;
          PVector currentPos;
          if (parent[prevNode] == numNodes+1) parentPos = startPos;
          else parentPos = nodePos[parent[prevNode]];
          if (prevNode == numNodes) currentPos = goalPos;
          else currentPos = nodePos[prevNode];
          cost1 += PVector.dist(currentPos, parentPos);
          prevNode = parent[prevNode];
        }
        float cost2 = 0.0;
        PVector neighborPos;
        if (neighborNode == numNodes) neighborPos = goalPos;
        else neighborPos = nodePos[neighborNode];
        cost2 += PVector.dist(neighborPos,nodePos[currentNode]);
        prevNode = currentNode;
        while (parent[prevNode] >= 0){
          PVector parentPos;
          PVector currentPos;
          if (parent[prevNode] == numNodes+1) parentPos = startPos;
          else parentPos = nodePos[parent[prevNode]];
          if (prevNode == numNodes) currentPos = goalPos;
          else currentPos = nodePos[prevNode];
          cost2 += PVector.dist(currentPos, parentPos);
          prevNode = parent[prevNode];
        }
        if (cost2<cost1) parent[neighborNode] = currentNode;
      }
      //
    }
  }
  
  for (int i:goalNeighbors){
    neighbors[i].remove(0);
  }
  
  
  if (fringe.size() == 0 && goalReached == false){
    path.add(0,-1);
    return path;
  }
  
  int prevNode = parent[numNodes];
  while (prevNode >=0){
    path.add(0,prevNode);
    prevNode = parent[prevNode];
  }
  path.remove(0);
  
  return path;
}
