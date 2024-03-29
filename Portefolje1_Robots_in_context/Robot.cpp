#include "Robot.h"



//Sets start values. Startposition and so on
Robot::Robot(string name, TransformationMatrix startMatrix)
{
    this->name = name;
    this->lastTransformationOrder = startMatrix;
    this->currTransformation = startMatrix;
    this->lastPosX = startMatrix.matrix[0][2];
    this->lastPosY = startMatrix.matrix[1][2];
}

// takes a distance and calculate time duration from a given speed
double Robot::timeOfMotion(double distance) 
{
    static double speed = 3;
    static double OneRotLength = 2 * PI_ * 0.1;
    double time = (distance / OneRotLength) / 3;
    return time;
}

// takes an angle and return the distance from robot specification
double Robot::angleDistance(double angle) 
{
    double betweenWheels = 0.4;
    return (2 * PI_*(betweenWheels / 2)*(angle / 360));
}

// After each move, check whether the rotation is back to original (if specified so in rotation matrix)
void Robot::setOrientation() 
{
    if (orientation == tempOrientation) // If the old orientation is the same as the new, don't do anything
    {
        cout << "Orientation was not changed after translation" << endl;
    }
    else // Else change the orientation back to original before the move
    {
        cout << "Translation is over and orientation is changed back from " << orientation << " to " << tempOrientation << endl;
        tempAngle = abs(tempOrientation - orientation);
        orientation = tempOrientation;
        cout << "Time of rotation: " << timeOfMotion(angleDistance(tempAngle)) << "s" << endl;

    }

}

//Rotate routine is actually the one "rotating" the robot 
void Robot::rotate(double** resultAr, double** oldMatrix1, double** oldMatrix2)
{
    if ((resultAr[0][2] - lastPosX) == 0 && (resultAr[1][2] - lastPosY) == 0) // if no translation only rotate (If X and Y is 0)
    {
        cout << "Only rotating in this transformation." << endl;
        orientation = (acos(resultAr[0][0]) * 180 / PI_);
        tempOrientation = orientation;
        cout << "Time of rotation: " << timeOfMotion(angleDistance((acos(resultAr[0][0]) * 180 / PI_))) << "s" << endl;

    }
    else if ((resultAr[0][2] - lastPosX) == 0 || (resultAr[1][2] - lastPosY) == 0) // Else if at least one of either X or Y is non-zero, rotate to that direction
    {
        if ((resultAr[0][2] - lastPosX) == 0) // if new X is 0 - move vertical
        {
            cout << "Want To Move vertical" << endl;
            if (oldMatrix1[1][2] < resultAr[1][2]) // if new Y is larger than old Y - move in positive Y direction
            {
                tempOrientation = orientation;
                orientation = 90;
                cout << "Orientation Changed: Now Pointing downwards" << endl;
                //print distance and speed

                if (angleDistance(abs(orientation - tempOrientation)) != 0) // only print if there was a rotation 
                {
                    cout << "Time of rotation: " << timeOfMotion(angleDistance(abs(orientation - tempOrientation))) << "s" << endl;
                }
            }

            if (oldMatrix1[1][2] > resultAr[1][2]) // if new Y is smaller than old Y - move in negative Y direction
            {
                tempOrientation = orientation;
                orientation = -90;
                cout << "Orientation Changed: Now Pointing upwards" << endl;
                //print distance and speed
                if (angleDistance(abs(orientation - tempOrientation)) != 0) // only print if there was a rotation 
                {
                    cout << "Time of rotation: " << timeOfMotion(angleDistance(abs(orientation - tempOrientation))) << "s" << endl;
                }
            }
        }
        if ((resultAr[1][2] - lastPosY) == 0) // if new Y is 0 - move horizontal
        {
            cout << "Want To Move horinzontal" << endl;
            if (lastPosX < resultAr[0][2]) // if new X is larger than old X - move in positive X direction
            {
                tempOrientation = orientation;
                orientation = 0;
                cout << "Orientation Changed: Now Pointing right" << endl;
                //print distance and speed
                if (angleDistance(abs(orientation - tempOrientation)) != 0) // only print if there was a rotation 
                {
                    cout << "Time of rotation: " << timeOfMotion(angleDistance(abs(abs(orientation - tempOrientation)))) << "s" << endl;
                }
            }

            if (lastPosX > resultAr[0][2]) // if new X is smaller than old X - move in negative X direction
            {

                tempOrientation = orientation;
                orientation = 180;
                cout << "Orientation Changed: Now Pointing left" << endl;
                //print distance and speed
                if (angleDistance(abs(orientation - tempOrientation)) != 0) // only print if there was a rotation 
                {
                    cout << "Time of rotation: " << timeOfMotion(angleDistance(abs(abs(orientation - tempOrientation)))) << "s" << endl;
                }
            }
        }
    }
    if ((resultAr[0][2] - lastPosX) != 0 && (resultAr[1][2] - lastPosY) != 0) // move direct to target if both X and Y is non-zero
    {
        cout << "Rotating directly to target as both x and y is non-zero" << endl;
        double hyp = sqrt(pow((resultAr[0][2] - lastPosX), 2) + pow((resultAr[1][2] - lastPosY), 2));
        double angle = (acos(abs(oldMatrix2[0][2]) / hyp)) * 180 / PI_;

        tempAngle = angle;
        orientation = orientation + angle;
        ///print distance and speed

        cout << "Time of rotation: " << timeOfMotion(angleDistance(tempAngle)) << "s" << endl;
    }
}

//Prints the workspace
void Robot::printWorkSpace(TransformationMatrix T1, TransformationMatrix T2, TransformationMatrix T3, TransformationMatrix T4)
{    
    //Make into points for easier usage
    Point T1_P(T1.matrix[0][2], T1.matrix[1][2]);
    Point T2_P(T2.matrix[0][2], T2.matrix[1][2]);
    Point T3_P(T3.matrix[0][2], T3.matrix[1][2]);
    Point T4_P(T4.matrix[0][2], T4.matrix[1][2]);

    //Add to vector for loop calculation
    vector<Point> pointList;
    pointList.push_back(T1_P);
    pointList.push_back(T2_P);
    pointList.push_back(T3_P);
    pointList.push_back(T4_P);
    
    //Calculate highest translation
    Point workspace = workspaceCal(pointList);

    cout << endl << "Lowest possible workspace is: (x,y) = (" << workspace.x << "," << workspace.y << ")" << endl;
}

//Intern function, calculates the workspace needed
Point Robot::workspaceCal(vector<Point> pointList)
{
    //Temp variable to save result in
    Point workspace;

    // find highest x,y
    for (int i = 0; i < pointList.size(); i++)
    {
        //x
        if (pointList[i].x > workspace.x)
            workspace.x = pointList[i].x;

        //y
        if (pointList[i].y > workspace.y)
            workspace.y = pointList[i].y;
    }

    return workspace;
}

// Moves robot and draws the path
void Robot::move(TransformationMatrix newMatrix, Image* img)
{
    //Multiply to make new movements
    TransformationMatrix newMove = this->currTransformation * newMatrix;

    //Rotate
    //Should be rewritten to not depend on last transformation Matrix and instead check depending on it's current orientation saved in robotclass
    rotate(newMove.matrix, lastTransformationOrder.matrix, newMatrix.matrix);

    //Move Robot
    double xres, yres;
    cout << "Moving Robot To Target" << endl;
    for (double i = 0; i <= 1; i += 0.001) // Drawing the line between 2 point. Steps is set to 1000, but should have been calculated relative to line length
    {
        xres = ((newMove.matrix[0][2] - lastPosX) * i) + lastPosX;
        yres = ((newMove.matrix[1][2] - lastPosY) * i) + lastPosY;

        img->setPixel8U(xres, yres, 0);
    }

    //PRINT SPEED AND DISTANCE
    double distance = getDistance((newMove.matrix[0][2] - lastPosX), (newMove.matrix[1][2] - lastPosY));
    if (timeOfMotion(distance) != 0)
        cout << "Time of translation: " << timeOfMotion(distance) << "s" << endl;

    // IF orientation was changed for movement, set orientation back
    setOrientation();

    //Update last position
    lastPosX = newMove.matrix[0][2];
    lastPosY = newMove.matrix[1][2];

    //Update last/current Transformation
    lastTransformationOrder = newMatrix;
    currTransformation = newMove;
}

// Get distance according to whether to move in X direction or Y direction only or to move along hyp
double Robot::getDistance(double x, double y) 
{
    double distance = 0;

    //not moved
    if (x == 0 && y == 0)
    {
        distance = 0;
    }
    // moves in either x or y
    else if (x == 0 || y == 0)
    {
        if (x == 0)
        {
            distance = y;
        }
        else
        {
            distance = x;
        }
    }
    // moves in the hyp
    if (x != 0 && y != 0)
    {
        double hyp = sqrt(pow(x, 2) + pow(y, 2));
        distance = hyp;
    }
    return distance;
} // this could have been made more general :) 

Robot::~Robot()
{
}
