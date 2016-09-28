#pragma once
#include <string>
#include "TransformationMatrix.h"
#include "Image.hpp"

using namespace std;
using namespace rw::sensor;

class Robot
{
public:

    Robot(string name, TransformationMatrix startMatrix); //constructor that takes initial starting point
    void rotate(double** resultAr, double** oldmat1, double** oldmat2); // rotating robot
    void move(TransformationMatrix newMove, Image* img); //Move function that moves the robot.
    void setOrientation(); // set orientation after a movie
    double timeOfMotion(double distance); // the time it's take to drive a distance
    double angleDistance(double angle); // Get distance from a angle

    //Function that prints the workspace. Should maybe have been made generel instead of only working with 4 transformations
    void printWorkSpace(TransformationMatrix T1, TransformationMatrix T2, TransformationMatrix T3, TransformationMatrix T4); 

    // get distance from one point to another
    double getDistance(double x, double y); 

    ~Robot();

private:
    string name;
    //Calculates workspace by getting all x,y coordinates
    Point workspaceCal(vector<Point> pointList);
    double tempAngle = 0;
    double tempOrientation = 0;
    double orientation = 0; // Orientation is 0 when pointing in the "normal" positive x-axis. 180 is opposite
    double lastPosX;
    double lastPosY;
    TransformationMatrix lastTransformationOrder; //Last matrix of new transformation given
    TransformationMatrix currTransformation; //Resulting matrix from multiplying old matrix with new matrix
};

