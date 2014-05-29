#include "PathPlanner.h"
#include "Field.cpp"
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <sys/time.h>
#include <set>

using namespace std;

struct Line
{
	pair<int,int> from;
	pair<int,int> to;
};
Field f;
static const int rangeX = 180;
static const int rangeY = 120;

static set<pair<int,int> > rrtSet;
static vector<Line> obstacles;


pair<int,int> findClosestVertexToPoint(pair<int,int> point)
{
	int minDistance = 2000;
	int minX = 0; int minY = 0;
	set<pair<int,int> >::iterator it;
	for (it = rrtSet.begin(); it != rrtSet.end(); it++) {
		int x =  abs(it->first - point.first);
		int y = abs(it->second - point.second);
	  int distance = x + y;
	  ///cout << "in " << it->first << " " << it->second << " dist " << distance << endl;
		if (minDistance > distance) 
		{
			minDistance = distance;
			minX = it->first;
			minY = it->second;
		}
	}
	return make_pair(minX, minY);
}

float mag(const pair<int,int> v) {
  return sqrt(v.first * v.first + v.second * v.second);
}

bool pointIsOnLine(Line line, pair<int,int> point) {
  return point.first >= MIN(line.to.first, line.from.first) && point.first <= MAX(line.to.first, line.from.first) && point.second >= MIN(line.to.second, line.from.second) && point.second <= MAX(line.to.second, line.from.second);
}

bool intersection(pair<int,int> p1, pair<int,int> p2, pair<int,int> p3, pair<int,int> p4) {
// Store the values for fast access and easy
// equations-to-code conversion
float x1 = p1.first, x2 = p2.first, x3 = p3.first, x4 = p4.first;
float y1 = p1.second, y2 = p2.second, y3 = p3.second, y4 = p4.second;
 
float d = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
// If d is zero, there is no intersection
if (d == 0) return NULL;
 
// Get the x and y
float pre = (x1*y2 - y1*x2), post = (x3*y4 - y3*x4);
float x = ( pre * (x3 - x4) - (x1 - x2) * post ) / d;
float y = ( pre * (y3 - y4) - (y1 - y2) * post ) / d;
 
// Check if the x and y coordinates are within both lines
if ( x < min(x1, x2) || x > max(x1, x2) ||
x < min(x3, x4) || x > max(x3, x4) ) return false;
if ( y < min(y1, y2) || y > max(y1, y2) ||
y < min(y3, y4) || y > max(y3, y4) ) return false;
 
return true;
}

bool moreThanSecondsAgo(float seconds, timeval earlierDate, timeval laterDate) 
{
  if(laterDate.tv_sec - earlierDate.tv_sec == seconds) {
    return laterDate.tv_usec > earlierDate.tv_usec;
  }
  else {
    return laterDate.tv_sec - earlierDate.tv_sec > seconds;
  }
}

/*bool intersection(pair<int,int> o1, pair<int,int> p1, pair<int,int> o2, pair<int,int> p2,
                      pair<int,int> &r)
{
    pair<int,int> x = o2 - o1;
    pair<int,int> d1 = p1 - o1;
    pair<int,int> d2 = p2 - o2;

    float cross = d1.x*d2.y - d1.y*d2.x;
    if (abs(cross) < /*EPS*//*1e-8)
        return false;

    double t1 = (x.x * d2.y - x.y * d2.x)/cross;
    r = o1 + d1 * t1;
    //cout << "intersect " << r << endl;
    return true;
}
/*
float obstacleBetweenPositions(int x, int y, int xOrigin, int yOrigin){
 
  Line line;
  
  pair<int,int> r;
  
  pair<int,int> start = make_pair(xOrigin, yOrigin);
  pair<int,int> end = make_pair(x, y);
  Line ourLine = Line(start, end);

 /* for(vector<Line>::iterator it = obstacles->begin(); it != obstacles->end(); ++it){
    line = it;
    if(intersection(end, start, line.end, line.start, r) && pointIsOnLine(line, r) && pointIsOnLine(ourLine, r)){
      return mag(ourLine.start - r);
    }
  }
  return -1;
}
*/

bool checkCollision(pair<int,int> from, pair<int,int> to)
{
	vector<Line>::iterator it;
	for(it = obstacles.begin(); it!=obstacles.end();it++)
	{
		if(intersection(from, to, it->from, it->to)) return true;
	}
	return false;
}
 
void rrt(int posX, int posY, int targetX, int targetY)
{
	//TODO: check if start is directly connected to goal
	int toX, toY;
	timeval start, now;
  gettimeofday(&start, 0);
  do
  {
    gettimeofday(&now, 0);
  	//pick random location
	  toX = (int) rand()%(L-1) + 1;//rand() % rangeX;
	  toY = (int) rand()%(W-1) + 1;//rand() % rangeY;
	  cout << "AAAA " << toX << " " << toY << endl;
	  //find vertex closest to this point
	  pair<int,int> closestPoint = findClosestVertexToPoint(make_pair(toX,toY));
	  //cout << "I choose " <<  closestPoint.first << " " << closestPoint.second << endl;
	  //add edge if there is collision free path
	  cout << checkCollision(closestPoint, make_pair(toX, toY)) << endl;
	  if(!checkCollision(closestPoint, make_pair(toX, toY))){
			rrtSet.insert(make_pair(toX, toY));
			f.insertRRTLines(closestPoint, make_pair(toX, toY));
		}
		f.cvDrawGrid();
	//}
	//cout << rrtSet.size() << endl;
	//set<pair<int,int> >::iterator it;
  //for (it = rrtSet.begin(); it != rrtSet.end(); it++) {
  //    cout << it->first <<" " << it->second << endl;
  if(toX == targetX) cout << "targetX " << toX << endl;
  if(toY == targetY) cout << "targetY " << toY << endl;
  }while ((toX != targetX || toY != targetY) && !moreThanSecondsAgo(100, start , now))	;
  if(moreThanSecondsAgo(100, start , now)) cout << "time " << endl;
  
}

void insertObstacle(pair<int,int> obstacle)
{
	Line l;
	int augmentx[8] = {-1, -1, -1, 1, 1, 1, 1, -1};
	int augmenty[8] = {1, -1, -1, -1, -1, 1, 1, 1};
	
	for(int i=0; i<8; i+=2)
	{
		l.from = make_pair(obstacle.first + augmentx[i]*RobotPixelRadius*2, obstacle.second + augmenty[i]*RobotPixelRadius*2);
		l.to = make_pair(obstacle.first + augmentx[i+1]*RobotPixelRadius*2 , obstacle.second + augmenty[i+1]*RobotPixelRadius*2);
		obstacles.push_back(l);
	}
	cout << obstacles.size() << endl;
}

int main(int argc, char** argv)
{
	srand (time(NULL));
	
	f.initFieldGrid();
	
	//add robot
	int robotx, roboty, orientation;
	robotx = (int) rand()%(L-1) + 1;
	roboty = (int) rand()%(W-1) + 1;
	orientation = (int) rand()%8 + 1;
	f.addElement(FRIEND, robotx, roboty, orientation);
	//cout << "Robot at [" << robotx << " " << roboty << "]" << " or " << orientation << endl;
	
	//add ball
	int ballx = (int) rand()%(L-1) + 1;
	int bally = (int) rand()%(W-1) + 1;
	f.addElement(BALL, ballx, bally, 0);
	
	rrtSet.insert(pair<int,int>(robotx, roboty));
//	rrtSet.insert(make_pair(2,3));
//	rrtSet.insert(make_pair(2,2));
//	rrtSet.insert(make_pair(2,4));
//	rrtSet.insert(make_pair(1,3));
	
	//cout << (rrtSet.lower_bound(make_pair(2,2)))->first << " " << (rrtSet.lower_bound(make_pair(2,2)))->second << endl;
	
	int obstx, obsty;
	obstx = (int) rand()%(L-1) + 1;
	obsty = (int) rand()%(W-1) + 1;
	f.addElement(OPPONENT, obstx, obsty, 1);
	insertObstacle(make_pair(obstx, obsty));
	
	f.addElement(OPPONENT, 50, 50, 1);
	insertObstacle(make_pair(50, 50));
	
	rrt(robotx, roboty, ballx, bally);
	f.cvDrawGrid();
	return 0;
}
