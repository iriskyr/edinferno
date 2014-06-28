#include <stdio.h>
#include <iostream>
#include "Field.h"
//#include <OpenCV/OpenCV.h>
//#include <OpenCV/highgui.h>
#include <math.h>
#include <time.h>



#define colorMax 255


using namespace cv;
using namespace std;

static const int gridLength = 900;
static const int gridWidth = 600;
static const int gridCellLength = 5;
static const int gridCellWidth = 5;
static const int RobotRadius = 300;
static const int RobotPixelRadius = 3;

static const int actualLength = 900; //1200 uncomment for text
static const int actualWidth = 600;


IplImage *img = cvCreateImage(cvSize(actualLength, actualWidth), IPL_DEPTH_8U, 3);
Mat Image(actualWidth, actualLength, CV_8UC3, Scalar(0));

map<pair<int,int>, int> orientations;
vector<RrtLine> rrtLines;


int gridResolutionLength = gridLength / L;
int gridResolutionWidth = gridWidth / W; 

int fieldGrid[L][W];

void expandRobot(int posX, int posY, int orientation, int robotType)
{
	int posxm, posxp, posym, posyp =0 ;
	posxm = (posX - RobotPixelRadius) < 0 ? 0 : (posX - RobotPixelRadius); 
	posxp = (posX + RobotPixelRadius) > L ? L : (posX + RobotPixelRadius); 
	posym = (posY - RobotPixelRadius) < 0 ? 0 : (posY - RobotPixelRadius); 
	posyp = (posY + RobotPixelRadius) > W ? W : (posY + RobotPixelRadius); 
	
	for(int i = posxm; i<posxp; i++){
		for(int j = posym; j<posyp; j++){
			fieldGrid[i][j] = robotType;
		}
	}
	orientations[make_pair(posX, posY)] = orientation;
}

void Field::addElement(elementTypes element, int posx, int posy, int orientation)
{
	switch(element)
	{
		case BALL: fieldGrid[posx][posy] = 4;
		break;
		case OPPONENT: expandRobot(posx, posy, orientation, 5);
		break;
		case FRIEND: expandRobot(posx, posy, orientation, 6);
		break;
	}
}

void Field::initFieldGrid()
{
	
	int lineW = 1;
	int penaltyL = 12;
	int penaltyW = 44;
	int markerD = 26;
	int markerSize = 2;
	int goalStart = (W - penaltyW) /2;
	int radius = 15;
	for (int i=0;i<L;i++){
		for (int j=0; j<W; j++){
			fieldGrid[i][j] = 0;
			if(i==0 || i==L-1 || j == 0 || j == W-1 || i == L/2-lineW)
				fieldGrid[i][j] = 1;
			if ((i == penaltyL + 1 || i == L - penaltyL -1) && (j > goalStart && j < (W-goalStart) ))
				fieldGrid[i][j] = 1;
			if (( j == goalStart +1 || j == W-goalStart-1) && (i < penaltyL+1 || i > L-penaltyL-1) )
				fieldGrid[i][j] = 1;
			if ( (j >= (W/2)-1 && j < (W/2+markerSize/2))  && ((i >= markerD  && i <= markerD + markerSize/2) || (i >= L- markerD -1 && i < L-markerD+ markerSize/2)))
				fieldGrid[i][j] = 3;
			if ((i >= L/2 -1 && i < L/2 + markerSize/2) && (j >= W/2 -1 && j < W/2 + markerSize/2))
				fieldGrid[i][j] = 3;
				
		}
	}
	
	for(float theta = 0; theta < 2*M_PI; theta+=0.00314){
		double x = radius * cos(theta);
		double y = radius * sin(theta);
		int indexx = int(L/2 + x );
		int indexy = int(W/2 + y );
		fieldGrid[indexx][indexy] = 1;
	}
	
}


void Field::drawPath(vector<pair<int,int> > _path)
{
  path.resize(_path.size());
  copy(_path.begin(),_path.end(),path.begin());
}

void Field::insertRRTLines(pair<int,int> from, pair<int,int> to)
{
	RrtLine line;
	line.from = Point(gridResolutionLength*from.first, gridResolutionLength*from.second);
	line.to = Point(gridResolutionLength*to.first, gridResolutionLength*to.second);
	rrtLines.push_back(line);
}

void Field::cvDrawGrid(){
	CvScalar green = cvScalar(100,colorMax,10); //field
	CvScalar white = cvScalar(colorMax,colorMax,colorMax); //lines
	CvScalar black = cvScalar(0,0,0); //rrtLines
	CvScalar red = cvScalar(0,0,colorMax); //friend
	CvScalar orange = cvScalar(0, colorMax/2, colorMax); //ball
	CvScalar blue = cvScalar(colorMax, 0, 0); //opponent
	CvScalar paintColor = green;
	
	for (int i=0;i<L;i++){
		for (int j=0; j<W; j++){
			Point p1 = Point(gridResolutionLength*i, gridResolutionWidth*j);
			Point p2 = Point(gridResolutionLength*(i+1), gridResolutionWidth*(j+1));
			if(fieldGrid[i][j] == 0){ //field
				rectangle(Image, p1, p2, green, -1);
			}
			else if(fieldGrid[i][j] == 1){ //lines
				rectangle(Image, p1, p2, white, -1);
			}
			else if(fieldGrid[i][j] == 3){ //cross
				rectangle(Image, p1, p2, blue, -1);
			}
			else if(fieldGrid[i][j] == 4){ //ball
				rectangle(Image, p1, p2, orange, -1);
			}
			else if(fieldGrid[i][j] == 5){ //opponent
				rectangle(Image, p1, p2, blue, -1);
			}
			else if(fieldGrid[i][j] == 6){ //friend
				rectangle(Image, p1, p2, red, -1);
			}
		}
	}
	
	//print orientations
	map<pair<int,int>, int>::iterator it;
	for(it=orientations.begin(); it != orientations.end(); it++)
	{
		int x2, y2;
		Point pt1(gridResolutionLength*(it->first).first, gridResolutionWidth*(it->first).second);
		x2 = pt1.x+gridResolutionLength*3*cos((it->second -1)*M_PI_4);
		y2 = pt1.y-gridResolutionLength*3*sin((it->second -1)*M_PI_4);
		Point pt2(x2, y2);
		line(Image, pt1, pt2, black, 1, 8);
	}
	
	vector<RrtLine>::iterator it1;
	for(it1=rrtLines.begin(); it1!=rrtLines.end(); it1++)
	{
		line(Image, it1->from, it1->to, black, 2, 8);
	}
	
	
	//print the path red line
	vector<pair<int,int> >::iterator it2;
	if(path.size() > 0 )
	{
		pair<int,int> point1 = path.front();
		Point pt1(gridResolutionLength*(point1.first),gridResolutionWidth*(point1.second));
		for (it2=path.begin() ; it2!=path.end(); ++it2)
		{	
			Point pt2(gridResolutionLength*(it2->first), gridResolutionWidth*(it2->second));
			line(Image, pt1, pt2, red, 2, 8);
			pt1 = pt2;	
		}
	}
	
	//uncomment for text area
	/*string text = "Text";
	int fontFace = FONT_HERSHEY_SIMPLEX;
	double fontScale = .5;
	int thickness = 1;  


	// center the text
	Point textOrg(910, 100);

	// then put the text itself
	putText(Image, text, textOrg, fontFace, fontScale,
			Scalar::all(255), thickness, 8);
	*/
	imshow( "Field", Image );
	cvWaitKey(0);	
}

/*

int main(int argc, char** argv){
	srand (time(NULL));
	
	
	int orientation =  (int) rand()%8;
	
/*	int posx, posy;
	posx = (int) rand()%(L-1) + 1;
	posy = (int) rand()%(W-1) + 1;
	addElement(BALL, posx, posy);
	
	posx = (int) rand()%(L-1) + 1;
	posy = (int) rand()%(W-1) + 1;
	addElement(OPPONENT,posx, posy);
	
	posx = (int) rand()%(L-1) + 1;
	posy = (int) rand()%(W-1) + 1;
	addElement(FRIEND, posx, posy);
*/
/*	addElement(BALL, 100, 50);
	addElement(OPPONENT, 60, 70);
	addElement(FRIEND, 150, 30);
	
	
	Field f;
	f.cvDrawGrid();
	return 0;
}
*/
