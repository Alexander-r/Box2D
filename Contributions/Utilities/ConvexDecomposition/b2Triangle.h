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

#ifndef B2_TRIANGLE_H
#define B2_TRIANGLE_H

#include "b2Math.h"

class b2Triangle{
public:
        double* x;
    double* y;
	b2Triangle();
        b2Triangle(double x1, double y1, double x2, double y2, double x3, double y3);
    ~b2Triangle();
        bool IsInside(double _x, double _y);
	void Set(const b2Triangle& toMe);

};

#endif
