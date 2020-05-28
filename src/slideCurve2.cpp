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
	inputGeometry = 0,
	master_crv,
	master_mat,
	slave_length,
	master_length,
	position,
	maxstretch,
	maxsquash,
	softness,
};

static Value eval_outputGeometry(EvalContext& ctx)
{
	// Inputs ---------------------------------------------------------
	// Input NurbsCurve
	// Curve
	const MFnNurbsCurve crv(ctx.value( master_crv ));
	MMatrix m = ctx.as_M44f(master_mat);

	// Input Sliders
	double in_sl = (double)ctx.as_float(slave_length);
	double in_ml = (double)ctx.as_float(master_length);
	double in_position = (double)ctx.as_float(position);
	double in_maxstretch = (double)ctx.as_float(maxstretch);
	double in_maxsquash = (double)ctx.as_float(maxsquash);
	double in_softness = (double)ctx.as_float(softness);

	// Init -----------------------------------------------------------
	double mstCrvLength = crv.length();

	const NurbsCurve slv(ctx.value(inputGeometry));
	int slvPointCount = int(slv.read_points().size());
	int mstPointCount = crv.numCVs();

	const MMatrix mat = slv.read_attribute("world_matrix", rumba::Shape::Topology::constant).as_M44f();
	const MMatrix mat_inv = mat.inverse();

	// Stretch --------------------------------------------------------
	double expo = 1;
	if ((mstCrvLength > in_ml) && (in_maxstretch > 1)){
		if (in_softness != 0){
			double stretch = (mstCrvLength - in_ml) / (in_sl * in_maxstretch);
			expo = 1 - exp(-(stretch) / in_softness);
		}

		double ext = std::min(in_sl * (in_maxstretch - 1) * expo, mstCrvLength - in_ml);

		in_sl += ext;
	}
	else if ((mstCrvLength < in_ml) && (in_maxsquash < 1)){
		if (in_softness != 0){
			double squash = (in_ml - mstCrvLength) / (in_sl * in_maxsquash);
			expo = 1 - exp(-(squash) / in_softness);
		}

		double ext = std::min(in_sl * (1 - in_maxsquash) * expo, in_ml - mstCrvLength);

		in_sl -= ext;
	}

	// Position --------------------------------------------------------
	double size = in_sl / mstCrvLength;
	double sizeLeft = 1 - size;

	double start = in_position * sizeLeft;
	double end = start + size;

	double tStart, tEnd;
	crv.getKnotDomain(tStart, tEnd);

	NurbsCurve result = slv.duplicate();
	const auto dst = result.write_points().write();

  // Process --------------------------------------------------------
  double step = (end - start) / (slvPointCount - 1.0);
  MPoint pt;
	MVector tan;
	for(int i = 0; i < slvPointCount; ++i)
	{
		double perc = start + (i * step);

		double u = crv.findParamFromLength(perc * mstCrvLength);

		if ((0 <= perc) && (perc <= 1))
			crv.getPointAtParam(u, pt, MSpace::kWorld);
		else{
			double overPerc;
			if (perc < 0){
				overPerc = perc;
				crv.getPointAtParam(0, pt, MSpace::kWorld);
				tan = crv.tangent(0);
			}
			else{
				overPerc = perc - 1;
				crv.getPointAtParam(mstPointCount-3.0, pt, MSpace::kWorld);
				tan = crv.tangent(mstPointCount-3.0);

				tan.normalize();
				tan *= mstCrvLength * overPerc;

				pt += tan;
			}
		}

		pt *= mat_inv;
		pt *= m;
		dst[i] = V3f(float(pt.x), float(pt.y), float(pt.z));
	}

	return result;
}

/////////////////////////////////////////////////
// REGISTRATION
/////////////////////////////////////////////////

void register_slideCurve2( Registry &r )
{
	r.register_node
	( 
		"mgear_slideCurve2",
		"Node",
		{
			{ "inputGeometry", Points::default_value },
			{ "master_crv", NurbsCurve::default_value },
			{ "master_mat", identity44f },
			{ "slave_length", 1.f },
			{ "master_length", 1.f },
			{ "position", 0.f, PlugDescriptor::serial, "{\"min\":0.0,\"max\":1.0}" },
			{ "maxstretch", 1.5f, PlugDescriptor::serial, "{\"min\":1.0}" },
			{ "maxsquash", 0.5f, PlugDescriptor::serial, "{\"min\":0.0,\"max\":1.0}" },
			{ "softness", 0.5f, PlugDescriptor::serial, "{\"min\":0.0,\"max\":1.0}" },
			{ "outputGeometry", Points::default_value, 0, "",
				eval_outputGeometry,
				{
					{ "inputGeometry" },
					{ "master_crv" },
					{ "master_mat" },
					{ "slave_length" },
					{ "master_length" },
					{ "position" },
					{ "maxstretch" },
					{ "maxsquash" },
					{ "softness" },
				}
			}
		}
	);
}
