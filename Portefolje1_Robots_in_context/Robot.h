#pragma once
#include <string>
#include "TransformationMatrix.h"
#include "Image.hpp"

using namespace std;
using namespace rw::sensor;

class Robot
{
public:

    Robot(string name);
    void rotate(double** resultAr, double tempa, double tempb, double** oldmat1, double** oldmat2); // rotating robot
    void move(TransformationMatrix move, Image* img, double tempa, double tempb);
    void setOrientation(); // set orientation after a movie
    double timeOfMotion(double distance); // the time it's take to drive a distance
    double angleDistance(double angle); // Get distance from a angle
    void printWorkSpace(double** a, double** b);
    double getDistance(double x, double y); // get distance from one point to another

    ~Robot();

private:
    string name;
    double** currentTransform;
    Point workspaceCal(double** transformation);
    Point prevPoint;
    double tempAngle = 0;
    double tempOrientation = 0;
    double orientation = 0; // Orientation is 0 when pointing in the "normal" positive x-axis. 180 is opposite

};

