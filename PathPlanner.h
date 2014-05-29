#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <utility>



class PathPlanner{
	public:	
	void rrt(int posX, int posY, int targetX, int targetY);
	private:
	int timeSteps_;
	//std::set<std::pair<int, int> > rrtSet;
};

#endif
