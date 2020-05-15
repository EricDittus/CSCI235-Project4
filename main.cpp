#include <iostream>
#include "Solutions.cpp"

using namespace std;
using namespace solutions;

void display(vector<MazeNode> mazePath);

int main() {

    Maze mazeTest1("maze_1.csv");
    Maze mazeTest2("maze_2.csv"); 
    Maze mazeTest3("maze_3.csv");

    display(mazeTest1);

    vector<MazeNode> DFS = solveDFS(mazeTest1);
    cout << "DFS Search Results: " << endl;
    display(DFS);

    cout << endl << "BFS Search Results: " << endl;
    vector<MazeNode> BFS = solveBFS(mazeTest1);
    display(BFS);

    cout << endl << "Dead End Search Results: " << endl;
    vector<MazeNode> DEF = solveDEF(mazeTest1);
    display(DEF);

}

// returns data on our maze
void display(vector<MazeNode> mazePath) {
    //output the positions of the MazeNodes along the path to the out.
    for (int i = 0; i < mazePath.size(); i++) {
        cout << endl << "| ";
        string position =  mazePath[i].getStrPos();
        cout << "Position " << i << ": " << position << " | ";
    } 
    cout << endl;
}
