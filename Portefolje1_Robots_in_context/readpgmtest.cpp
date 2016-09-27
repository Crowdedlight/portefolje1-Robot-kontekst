#include <iostream>
#include "Image.hpp"
#include "PPMLoader.hpp"
#include <vector>
#include "bresenham.h"
#include "Robot.h"
#include "TransformationMatrix.h"
#include <cmath>

#include <math.h>

#define PI_ 3.14159265358
#define  R2D (PI_ / 180)


using namespace rw::sensor;
using namespace rw::loaders;


int main(int argc) { 
    std::string filename("Bane1.pgm");
    std::cout << "loading image..." << std::endl;
    Image* img = PPMLoader::load(filename);

    // do stuff here
	int channel = 0; // allways 0 on grayscale image

	// Initialize arrays for translations start
    TransformationMatrix resultAr;
    TransformationMatrix oldresultAr;
	TransformationMatrix H0;
    TransformationMatrix H1;
    TransformationMatrix H2;
    TransformationMatrix H3;
    TransformationMatrix H4;
	
	// H0
	double rot0 = 0;
	H0.matrix[0][0] = cos(rot0 * R2D);
	H0.matrix[1][0] = sin(rot0 * R2D);
	H0.matrix[2][0] = 0.0;

	H0.matrix[0][1] = -sin(rot0 * R2D);
	H0.matrix[1][1] = cos(rot0 * R2D);
	H0.matrix[2][1] = 0.0;


	H0.matrix[0][2] = 0;
	H0.matrix[1][2] = 0;
	H0.matrix[2][2] = 1;

	 //H1
	double rot1 = 0;
	H1.matrix[0][0] = cos(rot1 * R2D);
	H1.matrix[1][0] = sin(rot1 * R2D);
	H1.matrix[2][0] = 0.0;

	H1.matrix[0][1] = -sin(rot1 * R2D);
	H1.matrix[1][1] = cos(rot1 * R2D);
	H1.matrix[2][1] = 0.0;

	H1.matrix[0][2] = 100;
	H1.matrix[1][2] = 0;
	H1.matrix[2][2] = 1;

	// H2
	double rot2 = 0;
	H2.matrix[0][0] = cos(rot2 * R2D);
	H2.matrix[1][0] = sin(rot2 * R2D);
	H2.matrix[2][0] = 0.0;

	H2.matrix[0][1] = -sin(rot2 * R2D);
	H2.matrix[1][1] = cos(rot2 * R2D);
	H2.matrix[2][1] = 0.0;

	H2.matrix[0][2] = 0;
	H2.matrix[1][2] = 200;
	H2.matrix[2][2] = 1;
	
	// H3 
	double rot3 = 30;
	H3.matrix[0][0] = cos(rot3 * R2D);
	H3.matrix[1][0] = sin(rot3 * R2D);
	H3.matrix[2][0] = 0.0;

	H3.matrix[0][1] = -sin(rot3 * R2D);
	H3.matrix[1][1] = cos(rot3 * R2D);
	H3.matrix[2][1] = 0.0;

	H3.matrix[0][2] = 0;
	H3.matrix[1][2] = 0;
	H3.matrix[2][2] = 1;

	// H4
	double rot4 = 60;
	H4.matrix[0][0] = cos(rot4 * R2D);
	H4.matrix[1][0] = sin(rot4 * R2D);
	H4.matrix[2][0] = 0.0;

	H4.matrix[0][1] = -sin(rot4 * R2D);
	H4.matrix[1][1] = cos(rot4 * R2D);
	H4.matrix[2][1] = 0.0;

	H4.matrix[0][2] = 200;
	H4.matrix[1][2] = 500;
	H4.matrix[2][2] = 1;

	// Initialize arrays for translations end

	resultAr = H0;
	oldresultAr = H0;
	double tempa;
	double tempb;
	double distance;


	tempa = resultAr.matrix[0][2];
	tempb = resultAr.matrix[1][2];	

    //Init robot
    Robot robot("Skynet");

	//MOVE 1
	resultAr = H0 * H1;
	cout << endl << "1. TRANSLATION/ROTATION MATRIX " << endl;
	robot.rotate(resultAr.matrix, tempa, tempb, H0.matrix, H1.matrix); 

    robot.move(resultAr, img, tempa, tempb);

	// IF orientation was changed for movement, set orientation back
	robot.setOrientation();

	//PRINT SPEED AND DISTANCE
	distance = robot.getDistance((resultAr.matrix[0][2] - tempa), (resultAr.matrix[1][2] - tempb));
	if(robot.timeOfMotion(distance) != 0)
	cout << "Time of translation: " << robot.timeOfMotion(distance) << endl;


	tempa = resultAr.matrix[0][2];
	tempb = resultAr.matrix[1][2];
	
	//MOVE 2
	resultAr = resultAr * H2;
	cout << endl << "2. TRANSLATION/ROTATION MATRIX " << endl;

	robot.rotate(resultAr.matrix, tempa, tempb, H1.matrix, H2.matrix);
    
    // MOVE
    robot.move(resultAr, img, tempa, tempb);
	  
	// IF orientation was changed for movement, set orientation back
	robot.setOrientation();

	//PRINT SPEED AND DISTANCE
	distance = robot.getDistance((resultAr.matrix[0][2] - tempa), (resultAr.matrix[1][2] - tempb));
	if (robot.timeOfMotion(distance) != 0)
	cout << "Time of translation: " << robot.timeOfMotion(distance) << endl;

	tempa = resultAr.matrix[0][2];
	tempb = resultAr.matrix[1][2];

	//MOVE 3
	resultAr = resultAr * H3;
	cout << endl << "3. TRANSLATION/ROTATION MATRIX " << endl;

	robot.rotate(resultAr.matrix, tempa, tempb, H2.matrix, H3.matrix);
    robot.move(resultAr, img, tempa, tempb);

	// IF orientation was changed for movement, set orientation back
	robot.setOrientation();
	//PRINT SPEED AND DISTANCE
	distance = robot.getDistance((resultAr.matrix[0][2] - tempa), (resultAr.matrix[1][2] - tempb));
	if (robot.timeOfMotion(distance) != 0)
	cout << "Time of translation: " << robot.timeOfMotion(distance) << endl;

	tempa = resultAr.matrix[0][2];
	tempb = resultAr.matrix[1][2];

	//MOVE 4
	resultAr = resultAr * H4;
	cout << endl << "4. TRANSLATION/ROTATION MATRIX " << endl;
	robot.rotate(resultAr.matrix, tempa, tempb, H3.matrix, H4.matrix);

    robot.move(resultAr, img, tempa, tempb);

	// IF orientation was changed for movement, set orientation back
	robot.setOrientation();
	//PRINT SPEED AND DISTANCE
	distance = robot.getDistance((resultAr.matrix[0][2] - tempa), (resultAr.matrix[1][2] - tempb));
	if (robot.timeOfMotion(distance) != 0)
	cout << "Time of translation: " << robot.timeOfMotion(distance) << endl;

	robot.printWorkSpace(resultAr.matrix, H2.matrix); //TODO fix so it works by getting though all transformations

    std::cout << endl << "saving image..." << std::endl;
    // save image
    img->saveAsPGM("testout.pgm");
        
    // cleanup
    delete img;

    system("pause");
}
