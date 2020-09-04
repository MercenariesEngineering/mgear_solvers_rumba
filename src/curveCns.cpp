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
	Dep_inputGeometry = 0,
	Dep_inputs
};

static Value eval_outputGeometry(EvalContext& ctx)
{
	const Points inputGeometry(ctx.value(Dep_inputGeometry));
	const Array adh(ctx.value(Dep_inputs));

	Points result = inputGeometry.duplicate();
	auto points = result.write_points();

	const M44f mat_inverse = inputGeometry.read_attribute("world_matrix", Shape::Topology::constant).as_M44f().inverse();

	// Process
	const int n = std::min(int(points.size()), int(adh.size()));
	for(int i =0; i < n; ++i)
	{
		const M44f m(adh.as_M44f(i) * mat_inverse);
		points[i] = m.translation();
	}

	return std::move(result);
}

/////////////////////////////////////////////////
// REGISTRATION
/////////////////////////////////////////////////

void register_curveCns( Registry &r )
{
	r.register_node
	( 
		"mgear_curveCns",
		"Node",
		{
			{ "inputGeometry", Points::default_value },
			{ "inputs", Array::default_value },
			{ "outputGeometry", Points::default_value, 0, "",
				eval_outputGeometry,
				{
					{ "inputGeometry" },
					{ "inputs" },
				}
			}
		}
	);
}
