#include "PathPlanner.h"
#include "Field.cpp"
#include <stdio.h>
#include <iostream>


using namespace std;


static const int rangeX = 180;
static const int rangeY = 120;

 
void rrt(int posX, int posY, int targetX, int targetY){
	int toX = rand() % rangeX;
	int toY = rand() % rangeY; 
}


int main(int argc, char** argv)
{
	srand (time(NULL));
	Field f;
	f.initFieldGrid();
	
	//add robot
	int posx, posy, orientation;
	posx = (int) rand()%(L-1) + 1;
	posy = (int) rand()%(W-1) + 1;
	orientation = (int) rand()%8 + 1;
	f.addElement(FRIEND, posx, posy, orientation);
	cout << "Robot at [" << posx << " " << posy << "]" << " or " << orientation << endl;
	
	//add ball
	posx = (int) rand()%(L-1) + 1;
	posy = (int) rand()%(W-1) + 1;
	f.addElement(BALL, posx, posy, 0);
	
	f.cvDrawGrid();
	return 0;
}
