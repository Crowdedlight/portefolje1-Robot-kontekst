#pragma once

#include <iostream>
#include <vector>
#include <cmath>
#define PI_ 3.14159265358
#define  R2D (PI_ / 180)

using namespace std;

struct Point
{
	int x;
	int y;

	Point()
	{
		x = 0;
		y = 0;
	}

	Point(int X, int Y)
	{
		x = X;
		y = Y;
	}
};

class MatrixTransform
{
public:
	MatrixTransform();
	void rotate(double** resultAr, double tempa, double tempb, double** oldmat1, double** oldmat2); // rotating robot
	void setOrientation(); // set orientation after a movie
	double getDistance(double x,double y); // get distance from one point to another
	double timeOfMotion(double distance); // the time it's take to drive a distance
	double angleDistance(double angle); // Get distance from a angle
	void print_matrix(double** a); // Prints out the matrix 
	double** matriceMul3x3(double** a, double** b); // Multiplicerer 2 3x3 matrix)
	Point workspaceCal(double** transformation);
	void printWorkSpace(double** a, double** b);
	~MatrixTransform();

private:
	double** currentTransform;
	Point prevPoint;
	double tempAngle = 0;
	double tempOrientation = 0;
	double orientation = 0; // Orientation is 0 when pointing in the "normal" positive x-axis. 180 is opposite



};


