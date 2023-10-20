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

#include "Maquina/EvalContext.h"
#include "Maquina/Registry.h"

using namespace maquina;
using namespace Imath;

/////////////////////////////////////////////////
// EVALUATION FUNCTIONS
/////////////////////////////////////////////////

enum
{
	global_scale = 0,
	blend,
	driver,
	driver_min,
	driver_ctr,
	driver_max,
	axis,
	squash,
	stretch,
};

static Value eval_output(EvalContext& ctx)
{
	// Inputs
	MVector gscale = ctx.as_V3d( global_scale );
	double sx = gscale.x;
	double sy = gscale.y;
	double sz = gscale.z;

	// Sliders
	double in_blend = (double)ctx.as_double( blend );
	double in_driver = (double)ctx.as_double( driver );
	double in_dmin = (double)ctx.as_double( driver_min );
	double in_dctr = (double)ctx.as_double( driver_ctr );
	double in_dmax = (double)ctx.as_double( driver_max );
	int in_axis = ctx.as_int( axis );
	double in_sq = (double)ctx.as_double( squash );
	double in_st = (double)ctx.as_double( stretch );

	// Process
	in_st *= clamp(std::max(in_driver - in_dctr, 0.0) / std::max(in_dmax - in_dctr, 0.0001), 0.0, 1.0);
	in_sq *= clamp(std::max(in_dctr - in_driver, 0.0) / std::max(in_dctr - in_dmin, 0.0001), 0.0, 1.0);

	if (in_axis != 0)
	sx *= std::max( 0.0, 1.0 + in_sq + in_st );

	if (in_axis != 1)
	sy *= std::max( 0.0, 1.0 + in_sq + in_st );

	if (in_axis != 2)
	sz *= std::max( 0.0, 1.0 + in_sq + in_st );

	MVector scl = MVector(sx, sy, sz);
	scl = linearInterpolate(gscale, scl, in_blend);

	double clamp_value = 0.0001;

	double scl_x = std::max(scl.x, clamp_value);
	double scl_y = std::max(scl.y, clamp_value);
	double scl_z = std::max(scl.z, clamp_value);

	// Output
	return Imath::V3d(float(scl_x), float(scl_y), float(scl_z));
}

/////////////////////////////////////////////////
// REGISTRATION
/////////////////////////////////////////////////

void register_squashStretch2( Registry &r )
{
	r.register_node
	( 
		"mgear_squashStretch2",
		"Node",
		{
			{ "global_scale", V3d(1.0) },
			{ "blend", 1.f, PlugDescriptor::serial, "{\"min\":1.0}" },
			{ "driver", 3.f },
			{ "driver_min", 1.f },
			{ "driver_ctr", 3.f },
			{ "driver_max", 6.f },
			{ "axis", 0, PlugDescriptor::serial, "{\"enum\":{\"x\":0,\"y\":1,\"z\":2}}" },
			{ "squash", 0.5f, PlugDescriptor::serial, "{\"min\":-1.0}" },
			{ "stretch", -0.5f, PlugDescriptor::serial, "{\"min\":-1.0}" },
			{ "output", V3d(0.0), 0, "",
				eval_output,
				{
					{ "global_scale" },
					{ "blend" },
					{ "driver" },
					{ "driver_min" },
					{ "driver_ctr" },
					{ "driver_max" },
					{ "axis" },
					{ "squash" },
					{ "stretch" },
				}
			}
		}
	);
}
