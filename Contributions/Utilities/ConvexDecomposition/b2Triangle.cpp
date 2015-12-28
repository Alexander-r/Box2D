/*
 * Copyright (c) 2007 Eric Jordan
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

#include "b2Triangle.h"

//Constructor automatically fixes orientation to ccw
b2Triangle::b2Triangle(float x1, float y1, float x2, float y2, float x3, float y3){
	x = new float[3];
	y = new float[3];
	float dx1 = x2-x1;
	float dx2 = x3-x1;
	float dy1 = y2-y1;
	float dy2 = y3-y1;
	float cross = dx1*dy2-dx2*dy1;
	bool ccw = (cross>0);
	if (ccw){
		x[0] = x1; x[1] = x2; x[2] = x3;
		y[0] = y1; y[1] = y2; y[2] = y3;
	} else{
		x[0] = x1; x[1] = x3; x[2] = x2;
		y[0] = y1; y[1] = y3; y[2] = y2;
	}
}
    
b2Triangle::b2Triangle(){
	x = new float[3];
	y = new float[3];
}

b2Triangle::~b2Triangle(){
	delete[] x;
	delete[] y;
}

void b2Triangle::Set(const b2Triangle& toMe) {
	for (int32_t i=0; i<3; ++i) {
		x[i] = toMe.x[i];
		y[i] = toMe.y[i];
	}
}

bool b2Triangle::IsInside(float _x, float _y){
	if (_x < x[0] && _x < x[1] && _x < x[2]) return false;
	if (_x > x[0] && _x > x[1] && _x > x[2]) return false;
	if (_y < y[0] && _y < y[1] && _y < y[2]) return false;
	if (_y > y[0] && _y > y[1] && _y > y[2]) return false;
		
		float vx2 = _x-x[0]; float vy2 = _y-y[0];
		float vx1 = x[1]-x[0]; float vy1 = y[1]-y[0];
		float vx0 = x[2]-x[0]; float vy0 = y[2]-y[0];
		
		float dot00 = vx0*vx0+vy0*vy0;
		float dot01 = vx0*vx1+vy0*vy1;
		float dot02 = vx0*vx2+vy0*vy2;
		float dot11 = vx1*vx1+vy1*vy1;
		float dot12 = vx1*vx2+vy1*vy2;
		float invDenom = 1.0f / (dot00*dot11 - dot01*dot01);
		float u = (dot11*dot02 - dot01*dot12)*invDenom;
		float v = (dot00*dot12 - dot01*dot02)*invDenom;
		
		return ((u>=0)&&(v>=0)&&(u+v<=1));    
}