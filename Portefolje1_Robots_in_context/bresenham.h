#pragma once

#include <vector>
using namespace std;

class Bresenham
{
public:
	Bresenham();
	void nextPoint();
	void getLine();
	void calculateLine(float x1, float y1, float x2, float y2);


private:
	vector<int> nextPx;
	vector<int> wholeLine;
};
