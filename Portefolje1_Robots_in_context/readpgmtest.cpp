#include <iostream>
#include "Image.hpp"
#include "PPMLoader.hpp"
#include <vector>
#include "bresenham.h"
#include "transformmatrix.h"
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
	double** resultAr;
	double** oldresultAr;
	double** H0 = new double*[3];
	double** H1 = new double*[3];
	double** H2 = new double*[3];
	double** H3 = new double*[3];
	double** H4 = new double*[3];

	for (int h = 0; h < 3; h++)
	{
		H0[h] = new double[3];
	}

	// H0
	double rot0 = 0;
	H0[0][0] = cos(rot0 * R2D);
	H0[1][0] = sin(rot0 * R2D);
	H0[2][0] = 0.0;

	H0[0][1] = -sin(rot0 * R2D);
	H0[1][1] = cos(rot0 * R2D);
	H0[2][1] = 0.0;


	H0[0][2] = 0;
	H0[1][2] = 0;
	H0[2][2] = 1;

	for (int h = 0; h < 3; h++)
	{
		H1[h] = new double[3];
	}

	 //H1
	double rot1 = 0;
	H1[0][0] = cos(rot1 * R2D);
	H1[1][0] = sin(rot1 * R2D);
	H1[2][0] = 0.0;

	H1[0][1] = -sin(rot1 * R2D);
	H1[1][1] = cos(rot1 * R2D);
	H1[2][1] = 0.0;

	H1[0][2] = 100;
	H1[1][2] = 0;
	H1[2][2] = 1;

	for (int h = 0; h < 3; h++)
	{
		H2[h] = new double[3];
	}
	// H2
	double rot2 = 0;
	H2[0][0] = cos(rot2 * R2D);
	H2[1][0] = sin(rot2 * R2D);
	H2[2][0] = 0.0;

	H2[0][1] = -sin(rot2 * R2D);
	H2[1][1] = cos(rot2 * R2D);
	H2[2][1] = 0.0;

	H2[0][2] = 0;
	H2[1][2] = 200;
	H2[2][2] = 1;
	
	for (int h = 0; h < 3; h++)
	{
		H3[h] = new double[3];
	}
	// H3 
	double rot3 = 30;
	H3[0][0] = cos(rot3 * R2D);
	H3[1][0] = sin(rot3 * R2D);
	H3[2][0] = 0.0;

	H3[0][1] = -sin(rot3 * R2D);
	H3[1][1] = cos(rot3 * R2D);
	H3[2][1] = 0.0;

	H3[0][2] = 0;
	H3[1][2] = 0;
	H3[2][2] = 1;

	for (int h = 0; h < 3; h++)
	{
		H4[h] = new double[3];
	}
	// H4
	double rot4 = 60;
	H4[0][0] = cos(rot4 * R2D);
	H4[1][0] = sin(rot4 * R2D);
	H4[2][0] = 0.0;

	H4[0][1] = -sin(rot4 * R2D);
	H4[1][1] = cos(rot4 * R2D);
	H4[2][1] = 0.0;

	H4[0][2] = 200;
	H4[1][2] = 500;
	H4[2][2] = 1;

	// Initialize arrays for translations end

	MatrixTransform mt;
	resultAr = H0;
	oldresultAr = H0;
	double ares;
	double bres;
	double tempa;
	double tempb;
	double distance;


	tempa = resultAr[0][2];
	tempb = resultAr[1][2];	


	//MOVE 1
	resultAr = mt.matriceMul3x3(H0, H1);
	cout << endl << "1. TRANSLATION/ROTATION MATRIX " << endl;
	mt.rotate(resultAr, tempa, tempb, H0,H1); 

	for (double i = 0; i <= 1; i += 0.001) // Drawing the line between 2 point. Steps is set to 1000, but should have been calculated relative to line length
	{
		ares = ((resultAr[0][2]- tempa) * i) + tempa;
		bres = ((resultAr[1][2]- tempb) * i) + tempb;

		img->setPixel8U(ares, bres, 0);
	} 

	// IF orientation was changed for movement, set orientation back
	mt.setOrientation();

	//PRINT SPEED AND DISTANCE
	distance = mt.getDistance((resultAr[0][2] - tempa), (resultAr[1][2] - tempb));
	if(mt.timeOfMotion(distance) != 0)
	cout << "Time of translation: " << mt.timeOfMotion(distance) << endl;


	tempa = resultAr[0][2];
	tempb = resultAr[1][2];
	
	//MOVE 2
	resultAr = mt.matriceMul3x3(resultAr, H2);
	cout << endl << "2. TRANSLATION/ROTATION MATRIX " << endl;

	mt.rotate(resultAr, tempa, tempb,H1, H2);

	for (double i = 0; i <= 1; i += 0.001)
	{
		
		ares = ((resultAr[0][2]-tempa) * i) + tempa;
		bres = ((resultAr[1][2]-tempb) * i) + tempb;


		img->setPixel8U(ares, bres, 0);
	} // MOVE
	  

	// IF orientation was changed for movement, set orientation back
	mt.setOrientation();
	//PRINT SPEED AND DISTANCE
	distance = mt.getDistance((resultAr[0][2] - tempa), (resultAr[1][2] - tempb));
	if (mt.timeOfMotion(distance) != 0)
	cout << "Time of translation: " << mt.timeOfMotion(distance) << endl;

	tempa = resultAr[0][2];
	tempb = resultAr[1][2];

	//MOVE 3
	resultAr = mt.matriceMul3x3(resultAr, H3);
	cout << endl << "3. TRANSLATION/ROTATION MATRIX " << endl;

	mt.rotate(resultAr, tempa, tempb, H2,H3);

	for (double i = 0; i <= 1; i += 0.001)
	{
		ares = ((resultAr[0][2]-tempa) * i) + tempa;
		bres = ((resultAr[1][2]-tempb) * i) + tempb;

		img->setPixel8U(ares, bres, 0);
	}

	// IF orientation was changed for movement, set orientation back
	mt.setOrientation();
	//PRINT SPEED AND DISTANCE
	distance = mt.getDistance((resultAr[0][2] - tempa), (resultAr[1][2] - tempb));
	if (mt.timeOfMotion(distance) != 0)
	cout << "Time of translation: " << mt.timeOfMotion(distance) << endl;

	tempa = resultAr[0][2];
	tempb = resultAr[1][2];

	//MOVE 4
	resultAr = mt.matriceMul3x3(resultAr, H4);
	cout << endl << "4. TRANSLATION/ROTATION MATRIX " << endl;
	mt.rotate(resultAr, tempa, tempb, H3,H4);

	for (double i = 0; i <= 1; i += 0.001)
	{
		ares = ((resultAr[0][2]-tempa) * i) + tempa;
		bres = ((resultAr[1][2]-tempb) * i) + tempb;

		img->setPixel8U(ares, bres, 0);
	}

	// IF orientation was changed for movement, set orientation back
	mt.setOrientation();
	//PRINT SPEED AND DISTANCE
	distance = mt.getDistance((resultAr[0][2] - tempa), (resultAr[1][2] - tempb));
	if (mt.timeOfMotion(distance) != 0)
	cout << "Time of translation: " << mt.timeOfMotion(distance) << endl;

	mt.printWorkSpace(resultAr,H2);

    std::cout << endl << "saving image..." << std::endl;
    // save image
    img->saveAsPGM("testout.pgm");
        
    // cleanup
    delete img;

    system("pause");
}
