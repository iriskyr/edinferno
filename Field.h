#ifndef FIELD_H
#define FIELD_H

#include "cv.h"
#include "highgui.h"

#define L 180
#define W 120

enum elementTypes {BALL, OPPONENT, FRIEND};

class Field{
	public:
		void initFieldGrid();
		void cvDrawGrid();
		void addElement(elementTypes element, int posx, int posy, int orientation);
};
#endif
