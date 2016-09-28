#pragma once
#include <iostream>
#include <vector>
#include <cmath>
#define PI_ 3.14159265358
#define  R2D (PI_ / 180)

using namespace std;

//struct to handle x,y in single object
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


class TransformationMatrix
{
public:
    TransformationMatrix(); //constructor. Initialises 2d array
    
    double** matriceMul3x3(double** a, double** b); // Multiplicerer 2 3x3 matrix - decrepicated, as it's identical to overload
    //Overload of multiplication
    TransformationMatrix operator*(const TransformationMatrix& b)const;
    
    void print_matrix(); // Prints out the matrix       
    ~TransformationMatrix();
    
    //Holder of data in this matrix. Have to be public due to the way we fill in from main in beginning. Consider making fill-in function and make it private another time
    double** matrix = new double*[3];
};