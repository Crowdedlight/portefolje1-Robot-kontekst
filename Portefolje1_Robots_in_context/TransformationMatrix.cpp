#include "TransformationMatrix.h"

TransformationMatrix::TransformationMatrix()
{
    for (int h = 0; h < 3; h++)
    {
        matrix[h] = new double[3];
    }
}

TransformationMatrix TransformationMatrix::operator*(const TransformationMatrix& b) const
{
    TransformationMatrix newTransform;
    
    newTransform.matrix[0][0] = this->matrix[0][0] * b.matrix[0][0] + this->matrix[0][1] * b.matrix[1][0] + this->matrix[0][2] * b.matrix[2][0];
    newTransform.matrix[1][0] = this->matrix[1][0] * b.matrix[0][0] + this->matrix[1][1] * b.matrix[1][0] + this->matrix[1][2] * b.matrix[2][0];
    newTransform.matrix[2][0] = this->matrix[2][0] * b.matrix[0][0] + this->matrix[2][1] * b.matrix[1][0] + this->matrix[2][2] * b.matrix[2][0];
                                                                
    newTransform.matrix[0][1] = this->matrix[0][0] * b.matrix[0][1] + this->matrix[0][1] * b.matrix[1][1] + this->matrix[0][2] * b.matrix[2][1];
    newTransform.matrix[1][1] = this->matrix[1][0] * b.matrix[0][1] + this->matrix[1][1] * b.matrix[1][1] + this->matrix[1][2] * b.matrix[2][1];
    newTransform.matrix[2][1] = this->matrix[2][0] * b.matrix[0][1] + this->matrix[2][1] * b.matrix[1][1] + this->matrix[2][2] * b.matrix[2][1];
                                                                      
    newTransform.matrix[0][2] = this->matrix[0][0] * b.matrix[0][2] + this->matrix[0][1] * b.matrix[1][2] + this->matrix[0][2] * b.matrix[2][2];
    newTransform.matrix[1][2] = this->matrix[1][0] * b.matrix[0][2] + this->matrix[1][1] * b.matrix[1][2] + this->matrix[1][2] * b.matrix[2][2];
    newTransform.matrix[2][2] = this->matrix[2][0] * b.matrix[0][2] + this->matrix[2][1] * b.matrix[1][2] + this->matrix[2][2] * b.matrix[2][2];

    return newTransform;
}

double** TransformationMatrix::matriceMul3x3(double** a, double** b) // Multiply 3x3 with 3x3 matrix and return the result
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

void TransformationMatrix::print_matrix() // Prints the matrix
{
    int i, j;
    for (i = 0; i < 3; i++)
    {
        for (j = 0; j < 3; j++)
        {
            cout << matrix[i][j] << " ";
        }
        printf("\n");
    }
}

TransformationMatrix::~TransformationMatrix()
{
    //Delete it casts exception as it deletes the data before it returns to give the data to variable asking for it. 
    //if (this->matrix != nullptr)
    //    delete this->matrix;
}
