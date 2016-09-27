#include "bresenham.h"

Bresenham::Bresenham() {};

void Bresenham::calculateLine( float x1,  float y1,  float x2,  float y2)
{


	// Bresenham's line algorithm
	const bool steep = (fabs(y2 - y1) > fabs(x2 - x1));
	if (steep)
	{
		std::swap(x1, y1);
		std::swap(x2, y2);
	}

	if (x1 > x2)
	{
		std::swap(x1, x2);
		std::swap(y1, y2);
	}

	const float dx = x2 - x1;
	const float dy = fabs(y2 - y1);

	float error = dx / 2.0f;
	const int ystep = (y1 < y2) ? 1 : -1;
	int y = (int)y1;

	const int maxX = (int)x2;

	for (int x = (int)x1; x<maxX; x++)
	{
		if (steep)
		{
			wholeLine[x] = y;
			wholeLine[y] = x;
			//SetPixel(y, x);
		}
		else
		{
			wholeLine[x] = x;
			wholeLine[y] = y;
			//SetPixel(x, y);
		}

		error -= dy;
		if (error < 0)
		{
			y += ystep;
			error += dx;
		}
	}
}

//void calculateLine(float x1, float y1, float x2, float y2)
//{
//	// Bresenham's line algorithm
//	const bool steep = (fabs(y2 - y1) > fabs(x2 - x1));
//	if (steep)
//	{
//		std::swap(x1, y1);
//		std::swap(x2, y2);
//	}
//
//	if (x1 > x2)
//	{
//		std::swap(x1, x2);
//		std::swap(y1, y2);
//	}
//
//	const float dx = x2 - x1;
//	const float dy = fabs(y2 - y1);
//
//	float error = dx / 2.0f;
//	const int ystep = (y1 < y2) ? 1 : -1;
//	int y = (int)y1;
//
//	const int maxX = (int)x2;
//
//	for (int x = (int)x1; x<maxX; x++)
//	{
//		if (steep)
//		{
//			//SetPixel(y, x);
//		}
//		else
//		{
//			//SetPixel(x, y);
//		}
//
//		error -= dy;
//		if (error < 0)
//		{
//			y += ystep;
//			error += dx;
//		}
//	}
//}