/*

MGEAR is under the terms of the MIT License

Copyright (c) 2016 Jeremie Passerin, Miquel Campos

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

Author:     Jeremie Passerin      geerem@hotmail.com  www.jeremiepasserin.com
Author:     Miquel Campos         hello@miquel-campos.com  www.miquel-campos.com
Date:       2016 / 10 / 10

Ported to Rumba by Mercenaries Engineering SARL

*/
#ifndef _rigSolvers
#define _rigSolvers

/////////////////////////////////////////////////
// INCLUDE
/////////////////////////////////////////////////

#include <iostream>
#include <algorithm>
#include <cmath>

#include <maya/MGlobal.h>
#include <maya/MQuaternion.h>
#include <maya/MVector.h>
#include <maya/MVectorArray.h>
#include <maya/MMatrix.h>
#include <maya/MTransformationMatrix.h>
#include <maya/MDoubleArray.h>
#include <maya/MFnNurbsCurve.h>
#include <maya/MPoint.h>


#define PI 3.14159265


/////////////////////////////////////////////////
// STRUCTS
/////////////////////////////////////////////////
struct s_GetFKTransform
{
   double lengthA;
   double lengthB;
   bool negate;
   MTransformationMatrix root;
   MTransformationMatrix bone1;
   MTransformationMatrix bone2;
   MTransformationMatrix eff;
};

struct s_GetIKTransform
{
   double lengthA;
   double lengthB;
   bool negate;
   double roll;
   double scaleA;
   double scaleB;
   double maxstretch;
   double softness;
   double slide;
   double reverse;
   MTransformationMatrix root;
   MTransformationMatrix eff;
   MTransformationMatrix	 upv;
};

/////////////////////////////////////////////////
// METHODS
/////////////////////////////////////////////////
MQuaternion e2q(double x, double y, double z);
MQuaternion slerp2(MQuaternion qA, MQuaternion qB, double blend);
double clamp(double d, double min_value, double max_value);
int clamp(int d, int min_value, int max_value);
double getDot(MQuaternion qA, MQuaternion qB);
double radians2degrees(double a);
double degrees2radians(double a);
double round(const double value, const int precision);
double normalizedUToU(double u, int point_count);
double uToNormalizedU(double u, int point_count);
unsigned findClosestInArray(double value, MDoubleArray in_array);
double set01range(double value, double first, double second);
double linearInterpolate(double first, double second, double blend);
MVector linearInterpolate(MVector v0, MVector v1, double blend);
MVectorArray bezier4point( MVector a, MVector tan_a, MVector d, MVector tan_d, double u);
MVector rotateVectorAlongAxis(MVector v, MVector axis, double a);
MQuaternion getQuaternionFromAxes(MVector vx, MVector vy, MVector vz);
MTransformationMatrix mapWorldPoseToObjectSpace(MTransformationMatrix objectSpace, MTransformationMatrix pose);
MTransformationMatrix mapObjectPoseToWorldSpace(MTransformationMatrix objectSpace, MTransformationMatrix pose);
MTransformationMatrix interpolateTransform(MTransformationMatrix xf1, MTransformationMatrix xf2, double blend);

#endif
