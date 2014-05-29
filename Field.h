#ifndef FIELD_H
#define FIELD_H

#include "cv.h"
#include "highgui.h"
#include <stdlib.h>
#include <utility>

#define L 180
#define W 120

enum elementTypes {BALL, OPPONENT, FRIEND};

struct RrtLine
{
	cv::Point from;
	cv::Point to;
};


class Field{
	public:
		
		void initFieldGrid();
		void cvDrawGrid();
		void addElement(elementTypes element, int posx, int posy, int orientation);
		void insertRRTLines(std::pair<int,int> from, std::pair<int,int> to);
};
#endif
