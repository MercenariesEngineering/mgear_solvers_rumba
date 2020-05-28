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

/////////////////////////////////////////////////
// INCLUDE
/////////////////////////////////////////////////
#include "mgear_solvers.h"

using namespace rumba;
using namespace Imath;

/////////////////////////////////////////////////
// EVALUATION FUNCTIONS
/////////////////////////////////////////////////

enum
{
	ctlParent = 0,
	inputs,
	inputsRoll,
	outputParent,
	u,
	resample,
	subdiv,
	absolute,
};

static Value eval_output(EvalContext& ctx)
{
	// Get inputs matrices ------------------------------
	// Inputs Parent
	Array adh = ctx.value( ctlParent );
	int count = int(adh.size());
	if (count < 1)
		return identity44f;
	std::vector<MMatrix> inputsP(count);
	for (int i = 0 ; i < count ; i++){
		inputsP[i] = adh.as_M44f(i);
	}

	// Inputs
	adh = ctx.value( inputs );
	if (count != int(adh.size()))
		return identity44f;
	std::vector<MMatrix> inputs(count);
	for (int i = 0 ; i < count ; i++){
		inputs[i] = adh.as_M44f(i);
	}

	adh = ctx.value( inputsRoll );
	if (count != int(adh.size()))
		return identity44f;
	MDoubleArray roll(int(adh.size()));
	for (int i = 0 ; i < count ; i++){
		roll[i] = degrees2radians((double)adh.as_float(i));
	}

	// Output Parent
	MMatrix outputParent_ = ctx.as_M44f( outputParent );
	
    // Get inputs sliders -------------------------------
    double in_u = (double)ctx.as_float( u );
    bool in_resample = ctx.as_bool( resample );
    int in_subdiv = ctx.as_int( subdiv );
    bool in_absolute = ctx.as_bool( absolute );
	
    // Process ------------------------------------------
    // Get roll, pos, tan, rot, scl
    MVectorArray pos(count);
    MVectorArray tan(count);
	MQuaternion *rot;
	rot = new MQuaternion[count];
    MVectorArray scl(count);
	double threeDoubles[3];
	for (int i = 0 ; i < count ; i++){
		MTransformationMatrix tp(inputsP[i]);
		MTransformationMatrix t(inputs[i]);
		pos[i] = t.getTranslation(MSpace::kWorld);
		rot[i] = tp.rotation();

		t.getScale(threeDoubles, MSpace::kWorld);
		scl[i] = MVector(threeDoubles[0], threeDoubles[1], threeDoubles[2]);
		tan[i] = MVector(threeDoubles[0] * 2.5, 0, 0).rotateBy(t.rotation());
	}
	
    // Get step and indexes
    // We define between wich controlers the object is to be able to
    // calculate the bezier 4 points front this 2 objects
	double step = 1.0 / std::max( 1, count-1 );
	int index1 = std::min( count-2, int(floor(in_u / step)) );
	int index2 = index1+1;
	int index1temp = index1;
	int index2temp = index2;
	double v = (in_u - step * double(index1)) / step;
	double vtemp = v;
	
   // calculate the bezier
   MVector bezierPos;
   MVector xAxis, yAxis, zAxis;
   if(!in_resample){
      // straight bezier solve
      MVectorArray results = bezier4point(pos[index1],tan[index1],pos[index2],tan[index2],v);
      bezierPos = results[0];
      xAxis = results[1];
   }
   else if(!in_absolute){
      MVectorArray presample(in_subdiv);
      MVectorArray presampletan(in_subdiv);
      MDoubleArray samplelen(in_subdiv);
      double samplestep = 1.0 / double(in_subdiv-1);
      double sampleu = samplestep;
      presample[0]  = pos[index1];
      presampletan[0]  = tan[index1];
      MVector prevsample(presample[0]);
      MVector diff;
      samplelen[0] = 0;
      double overalllen = 0;
      MVectorArray results(2);
      for(long i=1;i<in_subdiv;i++,sampleu+=samplestep){
         results = bezier4point(pos[index1],tan[index1],pos[index2],tan[index2],sampleu);
         presample[i] = results[0];
         presampletan[i] = results[1];
		 diff = presample[i] - prevsample;
		 overalllen += diff.length();
         samplelen[i] = overalllen;
         prevsample = presample[i];
      }
      // now as we have the
      sampleu = 0;
      for(long i=0;i<in_subdiv-1;i++,sampleu+=samplestep){
         samplelen[i+1] = samplelen[i+1] / overalllen;
         if(v>=samplelen[i] && v <=  samplelen[i+1]){
            v = (v - samplelen[i]) / (samplelen[i+1] - samplelen[i]);
			bezierPos = linearInterpolate(presample[i],presample[i+1],v);
			xAxis = linearInterpolate(presampletan[i],presampletan[i+1],v);
            break;
         }
      }
   }
   else{
      MVectorArray presample(in_subdiv);
      MVectorArray presampletan(in_subdiv);
      MDoubleArray samplelen(in_subdiv);
      double samplestep = 1.0 / double(in_subdiv-1);
      double sampleu = samplestep;
      presample[0]  = pos[0];
      presampletan[0]  = tan[0];
      MVector prevsample(presample[0]);
      MVector diff;
      samplelen[0] = 0;
      double overalllen = 0;
      MVectorArray results;
      for(long i=1;i<in_subdiv;i++,sampleu+=samplestep){
         index1 = std::min(count-2, int(floor(sampleu / step)));
         index2 = index1+1;
         v = (sampleu - step * double(index1)) / step;
         results = bezier4point(pos[index1],tan[index1],pos[index2],tan[index2],v);
         presample[i] = results[0];
         presampletan[i] = results[1];
		 diff = presample[i] - prevsample;
		 overalllen += diff.length();
         samplelen[i] = overalllen;
         prevsample = presample[i];
      }
      // now as we have the
      sampleu = 0;
      for(long i=0;i<in_subdiv-1;i++,sampleu+=samplestep){
         samplelen[i+1] = samplelen[i+1] / overalllen;
         if(in_u>=samplelen[i] && in_u <= samplelen[i+1]){
            in_u = (in_u - samplelen[i]) / (samplelen[i+1] - samplelen[i]);
			bezierPos = linearInterpolate(presample[i],presample[i+1],in_u);
			xAxis = linearInterpolate(presampletan[i],presampletan[i+1],in_u);
            break;
         }
      }
   }

   
	// compute the scaling (straight interpolation!)
	MVector scl1 = linearInterpolate(scl[index1temp], scl[index2temp],vtemp);

	// compute the rotation!
	MQuaternion q = slerp(rot[index1temp], rot[index2temp], vtemp);
	yAxis = MVector(0,1,0);
	yAxis = yAxis.rotateBy(q);
	
	// use directly or project the roll values!
	// print roll
	double a = linearInterpolate(roll[index1temp], roll[index2temp], vtemp);
	yAxis = yAxis.rotateBy( MQuaternion(xAxis.x * sin(a/2.0), xAxis.y * sin(a/2.0), xAxis.z * sin(a/2.0), cos(a/2.0)));
	
	zAxis = xAxis ^ yAxis;
	zAxis.normalize();
	yAxis = zAxis ^ xAxis;
	yAxis.normalize();

	// Output -------------------------------------------
	MTransformationMatrix result;

	// translation
	result.setTranslation(bezierPos, MSpace::kWorld);
	// rotation
	q = getQuaternionFromAxes(xAxis,yAxis,zAxis);
	result.setRotationQuaternion(q.x, q.y, q.z, q.w);
	// scaling
	threeDoubles[0] = 1;
	threeDoubles[0] = scl1.y;
	threeDoubles[0] = scl1.z;
	result.setScale(threeDoubles, MSpace::kWorld);

	return M44f(( result.asMatrix() * outputParent_.inverse() ).to_ilmbase());
}

/////////////////////////////////////////////////
// REGISTRATION
/////////////////////////////////////////////////

void register_rollSplineKine( Registry &r )
{
	r.register_node
	( 
		"mgear_rollSplineKine",
		"Node",
		{
			{ "ctlParent", Array::default_value },
			{ "inputs", Array::default_value },
			{ "inputsRoll", Array::default_value },
			{ "outputParent", identity44f },
			{ "u", 0.f, PlugDescriptor::serial, "{\"min\":0.0,\"max\":1.0}" },
			{ "resample", false },
			{ "subdiv", 10, PlugDescriptor::serial, "{\"min\":3}" },
			{ "absolute", false },
			{ "output", identity44f, 0, "",
				eval_output,
				{
					{ "ctlParent" },
					{ "inputs" },
					{ "inputsRoll" },
					{ "outputParent" },
					{ "u" },
					{ "resample" },
					{ "subdiv" },
					{ "absolute" },
				}
			}
		}
	);
}
