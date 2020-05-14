
#include "Maze.cpp"
#include <stack>
#include <queue>
#include <unordered_map>
#include <algorithm>
using namespace std;

namespace solutions
{

int getNumberOfWalls(MazeNode *a_node)
{
    int wall_counter = 0;
    for (directions::nesw dir = directions::NORTH; dir < directions::EAST; dir = directions::nesw(dir + 1))
    {
        if (a_node->getDirectionNode(dir) == nullptr || a_node->getDirectionNode(dir)->isWall() || a_node->getDirectionNode(dir)->isVisited())
        {
            wall_counter++;
        }
    }
    return wall_counter;
}

bool canTravel(MazeNode *a_node)
{
    if (a_node->isVisited() || a_node->isWall())
    {
        return false;
    }
    return true;
}

/*                TASK ONE
  Modify the function std::vector<MazeNode>
  solveDFS(Maze & a maze) in Solutions.cpp
  to implement Depth-First Search in order
  to find the shortest solution to a maze.
  The method should return an ordered
  vector that holds the path nodes from
  start-point to end-point.               */
std::vector<MazeNode> solveDFS(Maze &a_maze)
{
  std::vector<MazeNode> reVector; //vector to be constructed as the path
  std::stack<MazeNode *> s;
  MazeNode *pos = a_maze.getFirstNode();
  reVector.push_back(*pos);
  s.push(pos);
  pos -> setVisited();
  while(!reVector.empty()){
    pos = s.top();

    if(pos == a_maze.getLastNode()){
      break;
    }
    if(canTravel(pos->getDirectionNode(directions::SOUTH))){
      reVector.push_back(*(pos->getDirectionNode(directions::SOUTH)));
      s.push(pos->getDirectionNode(directions::SOUTH));
      pos->getDirectionNode(directions::SOUTH)->setVisited();
    }else if(canTravel(pos->getDirectionNode(directions::WEST))){
      reVector.push_back(*(pos -> getDirectionNode(directions::WEST)));
      s.push(pos -> getDirectionNode(directions::WEST));
      pos->getDirectionNode(directions::WEST)->setVisited();
    }else if(canTravel(pos->getDirectionNode(directions::NORTH))){
      reVector.push_back(*(pos -> getDirectionNode(directions::NORTH)));
      s.push(pos -> getDirectionNode(directions::NORTH));
      pos->getDirectionNode(directions::NORTH)->setVisited();
    }else if(canTravel(pos->getDirectionNode(directions::EAST))){
      reVector.push_back(*(pos->getDirectionNode(directions::EAST)));
      s.push(pos->getDirectionNode(directions::EAST));
      pos->getDirectionNode(directions::EAST)->setVisited();
    }else{
      reVector.pop_back(); //remove last node from vector since exausted directions
      s.pop(); //exausted options, decapitate stack
    }
  }
  return reVector; //the constructed path is returned
}


/*              TASK TWO
  Modify the function std::vector<MazeNode>
  solveBFS(Maze & a maze) in Solutions.cpp
  to implement Breadth-First Search in order
  to find the shortest solution to a maze.
  The method should return an ordered
  vector that holds the path nodes from
  start-point to end-point.                */
std::vector<MazeNode> solveBFS(Maze &a_maze)
{
  std::vector<MazeNode> path;
  //the queue
  std::queue<MazeNode *> q;
  //set node pos as the first node of the maze
  MazeNode *pos = a_maze.getFirstNode();
  //this will keep track of the parent nodes
  std::unordered_map<MazeNode *, MazeNode *> parents;
  //pos must be set as visited
  pos->setVisited();
  //node pos put onto the queue
  q.push(pos);
  //vector that will reconstruct the path
  bool neighborsNotVisited = false;
  while(!q.empty()){
    pos=q.front();
    q.pop();
    for(directions::nesw dir = directions::NORTH; dir <= directions::WEST; dir = directions::nesw(dir + 1)){
      if(canTravel(pos->getDirectionNode(dir))){
        neighborsNotVisited = true;
      }
    }
    if(neighborsNotVisited == true){ //check if there are unvisited adjacsent nodes
      //go through each direction surrounding that node
        if(canTravel(pos->getDirectionNode(directions::SOUTH))){
          parents[pos->getDirectionNode(directions::SOUTH)] = pos;
          pos->getDirectionNode(directions::SOUTH)->setVisited();
          q.push(pos->getDirectionNode(directions::SOUTH));
        }
        if(canTravel(pos->getDirectionNode(directions::WEST))){
          parents[pos->getDirectionNode(directions::WEST)] = pos;
          pos->getDirectionNode(directions::WEST)->setVisited();
          q.push(pos->getDirectionNode(directions::WEST));
        }
        if(canTravel(pos->getDirectionNode(directions::NORTH))){
          parents[pos->getDirectionNode(directions::NORTH)] = pos;
          pos->getDirectionNode(directions::NORTH)->setVisited();
          q.push(pos->getDirectionNode(directions::NORTH));
        }
        if(canTravel(pos->getDirectionNode(directions::EAST))){
          parents[pos->getDirectionNode(directions::EAST)] = pos;
          pos->getDirectionNode(directions::EAST)->setVisited();
          q.push(pos->getDirectionNode(directions::EAST));
        }
        //the case that we have reached the end of the maze
        if(q.front()==a_maze.getLastNode()){
          break;
        }

    }
    neighborsNotVisited = false;
  }
  //construct the path vector by backtracking through parent nodes to beginning
  MazeNode *node = a_maze.getLastNode();
  while(node != a_maze.getFirstNode()){
    path.push_back(*node);
    node = parents[node];
  }
  path.push_back(*a_maze.getFirstNode());
  //we need to reverse order of path, since it was set up backwards
  reverse(path.begin(),path.end());
  return path;
}

/*                TASK THREE
  Modify the function std::vector<MazeNode>
  solveDEF(Maze & a maze) in Solutions.cpp
  to implement Dead-End Filling in order to
  find the shortest solution to a maze.
  The method should return an ordered vector
  that holds the path nodes from start-point
  to end-point.                             */

/*In this function, I will be adding dead ends to a stack by
looping through the nodes of the maze. Then, when a deadend
is found, i turn the node into a wall. By doing this, I turn
the following node in the dead-end's path into a dead-end.
Basically, turning the deadend paths into walls. Then I can
filter the new maze into the DFS function, and easily find
the only path available*/
std::vector<MazeNode> solveDEF(Maze &a_maze)
{

  std::vector<MazeNode> path;
  std::stack<MazeNode *> dead;
  MazeNode *pos;
  //loop through all nodes of the maze
  for(auto &it : a_maze.getNodes()){
    //make sure the node has 3 or more walls, isn't a wall, and
    //is not the end or the beginning of the maze.
    //which will define it as a dead-end
    if(getNumberOfWalls(&it)>=3 && it.isWall()==false
       && a_maze.getLastNode()->getStrPos() != it.getStrPos()
       && a_maze.getFirstNode()->getStrPos() != it.getStrPos()){
         //put the deadend on the dead stack
         dead.push(a_maze.contains(it.getPos()));
         //set the dead end to visited
         a_maze.contains(it.getPos())->setVisited();
       }
  }
  pos = dead.top();
  while(!dead.empty()){
    pos = dead.top();
    dead.pop();
    if(getNumberOfWalls(pos->getDirectionNode(directions::SOUTH)) >= 3 && canTravel(pos->getDirectionNode(directions::SOUTH))){
      pos->getDirectionNode(directions::SOUTH)->setWall();
      dead.push(pos->getDirectionNode(directions::SOUTH));
    }
    if(getNumberOfWalls(pos->getDirectionNode(directions::WEST)) >= 3 && canTravel(pos->getDirectionNode(directions::WEST))){
      pos->getDirectionNode(directions::WEST)->setWall();
      dead.push(pos->getDirectionNode(directions::WEST));
    }
    if(getNumberOfWalls(pos->getDirectionNode(directions::NORTH)) >= 3 && canTravel(pos->getDirectionNode(directions::NORTH))){
      pos->getDirectionNode(directions::NORTH)->setWall();
      dead.push(pos->getDirectionNode(directions::NORTH));
    }
    if(getNumberOfWalls(pos->getDirectionNode(directions::EAST)) >= 3 && canTravel(pos->getDirectionNode(directions::EAST))){
      pos->getDirectionNode(directions::EAST)->setWall();
      dead.push(pos->getDirectionNode(directions::EAST));
    }
  }
  path = solveDFS(a_maze);
  return path;
}

/*                TASK FOUR
  Modify the function std::vector<MazeNode>
  solveCustom(Maze & a maze) in Solutions.cpp
  to implement your own shortest path finding
  algorithm. This could be an optimization of
  one of the algorithms you have already
  implemented, an algorithm that already
  exists, your own creation, or a combination
  of any or every one element from these
  categories.                                */

/*In my custom pathfinder fuction, I will use depth first search, but using one
stack starting at the beginning of the maze and another at the end of it, and
will add to both stacks until they meet in the middle. I will then filter them
both into the path vector, hopefully having them work together to find a path*/
std::vector<MazeNode> solveCustom(Maze &a_maze)
{
  std::vector<MazeNode> path;
  //initializing both stacks
  std::stack<MazeNode *> start;
  std::stack<MazeNode *> end;
  //both nodes are assigned the value of the end or the beginning
  MazeNode *startNode = a_maze.getFirstNode();
  MazeNode *endNode = a_maze.getLastNode();
  //path begins with the starting node
  path.push_back(*startNode);
  while(!end.empty()){
    startNode = start.top();
    endNode = end.top();
    startNode -> setVisited();
    endNode -> setVisited();
    for(directions::nesw dir = directions::NORTH; dir <= directions::WEST; dir = directions::nesw(dir + 1)){
      //This will function so long as a stack does not see the top of another stack
      while(((endNode->getDirectionNode(dir)) != start.top())||((startNode->getDirectionNode(dir)) != end.top())){
        if(canTravel(startNode->getDirectionNode(dir))){
          start.push(startNode->getDirectionNode(dir));
          path.push_back(*(startNode->getDirectionNode(dir)));
          startNode->getDirectionNode(dir)->setVisited();
        }
        if(canTravel(endNode->getDirectionNode(dir))){
          end.push(endNode->getDirectionNode(dir));
          startNode->getDirectionNode(dir)->setVisited();
        }
      }
      //If the top of another stack is found, this means that the path from the starting node
      //has completely added itself to the vector. Thus, we need only add the path leading to the end.
      if(((endNode->getDirectionNode(dir)) == start.top())||((startNode->getDirectionNode(dir))== end.top())){
        path.push_back(*(end.top()));
        end.pop();
      }
    }
  }
  return path;
  }

}

int main()
{
    Maze m{"data/maze_1.csv"};
    // solutions::solveDEF(m);
    for(auto it : solutions::solveDEF(m))
    {
        cout<<it.getStrPos()<<endl;
    }
    return 0;

}

// namespace solutions
