#include "transformmatrix.h"

MatrixTransform::MatrixTransform(){}



double MatrixTransform::timeOfMotion(double distance) // takes a distance and calculate time duration from a given speed
{
	static double speed = 3;
	static double OneRotLength = 2 * PI_ * 0.1;
	double time = (distance / OneRotLength) / 3;
	return time;
}

double MatrixTransform::angleDistance(double angle) // takes an angle and return the distance from robot specification
{
	double betweenWheels = 0.4;
	return (2 * PI_*(betweenWheels / 2)*(angle / 360));
}

void MatrixTransform::print_matrix(double** a) // Prints the matrix
{
	int i, j;
	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 3; j++)
		{
			cout << a[i][j] << " ";
		}
		printf("\n");
	}
}

double** MatrixTransform::matriceMul3x3(double** a, double** b) // Multiply 3x3 with 3x3 matrix and return the result
{

	double** newTransform = new double*[3];

	for (int h = 0; h < 3; h++)
	{
		newTransform[h] = new double[3];
	}

	newTransform[0][0] = a[0][0] * b[0][0] + a[0][1] * b[1][0] + a[0][2] * b[2][0];
	newTransform[1][0] = a[1][0] * b[0][0] + a[1][1] * b[1][0] + a[1][2] * b[2][0];
	newTransform[2][0] = a[2][0] * b[0][0] + a[2][1] * b[1][0] + a[2][2] * b[2][0];

	newTransform[0][1] = a[0][0] * b[0][1] + a[0][1] * b[1][1] + a[0][2] * b[2][1];
	newTransform[1][1] = a[1][0] * b[0][1] + a[1][1] * b[1][1] + a[1][2] * b[2][1];
	newTransform[2][1] = a[2][0] * b[0][1] + a[2][1] * b[1][1] + a[2][2] * b[2][1];

	newTransform[0][2] = a[0][0] * b[0][2] + a[0][1] * b[1][2] + a[0][2] * b[2][2];
	newTransform[1][2] = a[1][0] * b[0][2] + a[1][1] * b[1][2] + a[1][2] * b[2][2];
	newTransform[2][2] = a[2][0] * b[0][2] + a[2][1] * b[1][2] + a[2][2] * b[2][2];

//	currentTransform = newTransform;

	return newTransform;
}

void MatrixTransform::setOrientation() // After each move, check whether the rotation is back to original (if specified so in rotation matrix)
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
		cout << "Time of rotation: " << timeOfMotion(angleDistance(tempAngle)) << endl;

	}

}

//Rotate routine is actually the one "rotating" the robot - this should have been in a Robot Class to seperate it from the translation
void MatrixTransform::rotate(double** resultAr,double tempa,double tempb,double** oldMatrix1, double** oldMatrix2) 
{
	if ((resultAr[0][2] - tempa) == 0 && (resultAr[1][2] - tempb) == 0) // if no translation only rotate (If X and Y is 0)
	{
		orientation = (acos(resultAr[0][0]) * 180 / PI_);
		tempOrientation = orientation;
		cout << "Time of rotation: " << timeOfMotion(angleDistance((acos(resultAr[0][0]) * 180 / PI_))) << endl;
		
	}
	else if ((resultAr[0][2] - tempa) == 0 || (resultAr[1][2] - tempb) == 0) // Else if at least one of either X or Y is non-zero, rotate to that direction
	{
		if ((resultAr[0][2] - tempa) == 0) // if new X is 0 - move vertical
		{
			cout << "Moving vertical" << endl;
			if (oldMatrix1[1][2] < resultAr[1][2]) // if new Y is larger than old Y - move in positive Y direction
			{
				tempOrientation = orientation;
				orientation = 90;
				cout << "Orientation: Pointing downwards" << endl;
				//print distance and speed

				if (angleDistance(abs(orientation - tempOrientation)) != 0) // only print if there was a rotation 
				{
					cout << "Time of rotation: " << timeOfMotion(angleDistance(abs(orientation - tempOrientation))) << endl;
				}
			}

			if (oldMatrix1[1][2] > resultAr[1][2]) // if new Y is smaller than old Y - move in negative Y direction
			{
				tempOrientation = orientation;
				orientation = -90;
				cout << "Orientation: Pointing upwards" << endl;
				//print distance and speed
				if (angleDistance(abs(orientation - tempOrientation)) != 0) // only print if there was a rotation 
				{
					cout << "Time of rotation: " << timeOfMotion(angleDistance(abs(orientation - tempOrientation))) << endl;
				}
			}
		}
		if ((resultAr[1][2] - tempb) == 0) // if new Y is 0 - move horizontal
		{
			cout << "Moving horinzontal" << endl;
			if (tempa < resultAr[0][2]) // if new X is larger than old X - move in positive X direction
			{
				tempOrientation = orientation;
				orientation = 0;
				cout << "Orientation: Pointing right" << endl;
				//print distance and speed
				if (angleDistance(abs(orientation - tempOrientation)) != 0) // only print if there was a rotation 
				{
					cout << "Time of rotation: " << timeOfMotion(angleDistance(abs(abs(orientation - tempOrientation)))) << endl;
				}
			}

			if (tempa > resultAr[0][2]) // if new X is smaller than old X - move in negative X direction
			{
				
				tempOrientation = orientation;
				orientation = 180;
				cout << "Orientation: Pointing left" << endl;
				//print distance and speed
				if (angleDistance(abs(orientation - tempOrientation)) != 0) // only print if there was a rotation 
				{
					cout << "Time of rotation: " << timeOfMotion(angleDistance(abs(abs(orientation - tempOrientation)))) << endl;
				}
			}
		}
	}
	if ((resultAr[0][2] - tempa) != 0 && (resultAr[1][2] - tempb) != 0) // move direct to target if both X and Y is non-zero
	{

		double hyp = sqrt(pow((resultAr[0][2] - tempa), 2) + pow((resultAr[1][2] - tempb), 2));
		double angle = (acos(abs(oldMatrix2[0][2]) / hyp)) * 180 / PI_;

		tempAngle = angle;
		orientation = orientation + angle;
			///print distance and speed

		cout << "Time of rotation: " << timeOfMotion(angleDistance(tempAngle)) << endl;
		//}
	}
}

double MatrixTransform::getDistance(double x, double y) // Get distance according to whether to move in X direction or Y direction only or to move along hyp
{
	double distance = 0;

	if (x == 0 && y == 0)
	{
		distance = 0;
	}
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

	if (x != 0 && y != 0)
	{
		double hyp = sqrt(pow(x, 2) + pow(y, 2));
		distance = hyp;
	}
	return distance;
} // this could have been made more general :) 

MatrixTransform::~MatrixTransform(){}

Point MatrixTransform::workspaceCal(double** transformation)
{
	static int lowestX = transformation[0][2];
	static int highestX = transformation[0][2];
	static int lowestY = transformation[1][2];
	static int highestY = transformation[1][2];

	if (transformation[0][2] < lowestX)
		lowestX = transformation[0][2];

	else if (transformation[0][2] > highestX)
		highestX = transformation[0][2];

	if (transformation[1][2] > highestY)
		highestY = transformation[1][2];

	else if (transformation[1][2] < lowestY)
		lowestY = transformation[1][2];


	Point xy((highestX - lowestX), (highestY - lowestY));
	return xy;
}

void MatrixTransform::printWorkSpace(double** a, double** b)
{

	prevPoint.x = a[0][2];
	prevPoint.y = a[1][2];

	Point point = workspaceCal(b);
	cout << endl << "Lowest possible workspace is: (x,y) = (" << point.x << "," << point.y << ")" << endl;
}