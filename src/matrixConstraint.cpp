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
Author:     Jascha Wohlkinger     jwohlkinger@gmail.com
Date:       2020/ 11 / 06

Ported to Rumba by Mercenaries Engineering SARL

*/

#include "mgear_solvers.h"

using namespace maquina;
using namespace Imath;

// ---------------------------------------------------
// input plugs
// ---------------------------------------------------
enum
{
	aDriverMatrix = 0,
	aDriverRotationOffset,
	aDrivenParentInverseMatrix,
	aDrivenRestMatrix,
	aRotationMultiplier,
	aScaleMultiplier
};

static Value eval_intermediate(EvalContext& ctx)
{
	MStatus status;

	// -- our needed variables
	MTransformationMatrix result;
	double scale[3];
	double shear[3];

	// -- our final output variables
	double scale_result[3] = { 1.0, 1.0, 1.0 };
	double shear_result[3];

	// -----------------------------------------
	// input attributes
	// -----------------------------------------
	MMatrix driver_matrix = ctx.as_M44d(aDriverMatrix);

	// -- driver rotation offset
	const Imath::V3d in_driver_rotation_offset = ctx.as_V3d(aDriverRotationOffset);
	double in_driver_rotation_offset_x = in_driver_rotation_offset.x;
	double in_driver_rotation_offset_y = in_driver_rotation_offset.y;
	double in_driver_rotation_offset_z = in_driver_rotation_offset.z;

	MMatrix driven_inverse_matrix = ctx.as_M44d(aDrivenParentInverseMatrix);
	MMatrix rest_matrix = ctx.as_M44d(aDrivenRestMatrix);

	// -- rotation multiplier
	const Imath::V3d in_rotation_multiplier = ctx.as_V3d(aRotationMultiplier);
	double in_rotation_multiplier_x = in_rotation_multiplier.x;
	double in_rotation_multiplier_y = in_rotation_multiplier.y;
	double in_rotation_multiplier_z = in_rotation_multiplier.z;


	// -- scale multiplier
	const Imath::V3d in_scale_multiplier = ctx.as_V3d(aScaleMultiplier);
	double in_scale_multiplier_x = in_scale_multiplier.x;
	double in_scale_multiplier_y = in_scale_multiplier.y;
	double in_scale_multiplier_z = in_scale_multiplier.z;

	// -- add the rotation offset.
	// We need to add the offset on top of the driver matrix, to calculate the outputDriverOffsetMatrix and the
	// the rest matrix correctly
	MEulerRotation  euler_off(
		degrees2radians(in_driver_rotation_offset_x),
		degrees2radians(in_driver_rotation_offset_y),
		degrees2radians(in_driver_rotation_offset_z) );
	MTransformationMatrix driver_matrix_tfm(driver_matrix);
	MTransformationMatrix driver_matrix_off = driver_matrix_tfm.rotateBy(euler_off,  MSpace::kPreTransform);


	// MMatrix mult_matrix = driver_matrix * driven_inverse_matrix;
	MMatrix mult_matrix = driver_matrix_off.asMatrix() * driven_inverse_matrix;

	// -- multiply the result of the mult matrix by the rest
	// -- need to have the rotation calculated seperaltely - (joint orientation)
	MMatrix rotate_matrix = mult_matrix * rest_matrix.inverse();

	MTransformationMatrix matrix(mult_matrix);
	MTransformationMatrix rotate_tfm(rotate_matrix);


	// -- the quaternion rotation of the rotate matrix
	// MQuaternion rotation = rotate_tfm_off.rotation();
	MQuaternion rotation = rotate_tfm.rotation();

	// -- apply the rotation multiplier
	rotation.x *= in_rotation_multiplier_x;
	rotation.y *= in_rotation_multiplier_y;
	rotation.z *= in_rotation_multiplier_z;

	// -- decompose the matrix values to construct into the final matrix
	MVector translation = matrix.getTranslation(MSpace::kWorld);
	matrix.getScale(scale, MSpace::kWorld);
	matrix.getShear(shear, MSpace::kWorld);

	// -- add in the scale multiplication
	scale[0] *= in_scale_multiplier_x;
	scale[1] *= in_scale_multiplier_y;
	scale[2] *= in_scale_multiplier_z;

	// -- compose our matrix
	result.setTranslation(translation, MSpace::kWorld);
	result.setRotationQuaternion(rotation.x, rotation.y, rotation.z, rotation.w);
	result.setScale(scale, MSpace::kWorld);
	result.setShear(shear, MSpace::kWorld);

	// -----------------------------------------
	// output
	// -----------------------------------------
	Array results;
	results.push_back(result.asMatrix().to_ilmbase());	// 0
	results.push_back(driver_matrix_off.asMatrix().to_ilmbase()); // 1
	results.push_back(result.getTranslation(MSpace::kWorld).to_ilmbase()); // 2

	MEulerRotation rotation_result = result.eulerRotation();
	results.push_back(rotation_result.to_ilmbase());	// 3

	result.getScale(scale_result, MSpace::kWorld);
	results.push_back(Imath::V3d(scale_result[0], scale_result[1], scale_result[2]));	// 4

	result.getShear(shear_result, MSpace::kWorld);
	results.push_back(Imath::V3d(shear_result[0], shear_result[1], shear_result[2]));	// 5

	return std::move(results);
}

static Value eval_outputMatrix(EvalContext& ctx)
{
	const Array results(ctx.value(0));
	return results.read(0);
}

static Value eval_outputDriverOffsetMatrix(EvalContext& ctx)
{
	const Array results(ctx.value(0));
	return results.read(1);
}

static Value eval_translate(EvalContext& ctx)
{
	const Array results(ctx.value(0));
	return results.read(2);
}

static Value eval_rotate(EvalContext& ctx)
{
	const Array results(ctx.value(0));
	return results.read(3);
}

static Value eval_scale(EvalContext& ctx)
{
	const Array results(ctx.value(0));
	return results.read(4);
}

static Value eval_shear(EvalContext& ctx)
{
	const Array results(ctx.value(0));
	return results.read(5);
}

/////////////////////////////////////////////////
// REGISTRATION
/////////////////////////////////////////////////

void register_matrixConstraint( Registry &r )
{
	r.register_node
	( 
		"mgear_matrixConstraint",
		"Node",
		{
			// ---------------------------------------------------
			// input plugs
			// ---------------------------------------------------
			{ "driverMatrix", Imath::identity44d },
			{ "driverRotationOffset", Imath::V3d(0.0) },
			{ "drivenParentInverseMatrix", Imath::identity44d },
			{ "drivenRestMatrix", Imath::identity44d },
			{ "rotationMultiplier", Imath::V3d(1.0) },
			{ "scaleMultiplier", Imath::V3d(1.0) },

			{ "intermediate", Array::default_value, 0, "",
				eval_intermediate,
				{
					{ "driverMatrix" },
					{ "driverRotationOffset" },
					{ "drivenParentInverseMatrix" },
					{ "drivenRestMatrix" },
					{ "rotationMultiplier" },
					{ "scaleMultiplier" },
				}
			},

			// ---------------------------------------------------
			// output plugs
			// ---------------------------------------------------
			{ "outputMatrix", Imath::identity44d, 0, "",
				eval_outputMatrix,
				{
					{ "intermediate" }
				}
			},
			{ "outputDriverOffsetMatrix", Imath::identity44d, 0, "",
				eval_outputDriverOffsetMatrix,
				{
					{ "intermediate" }
				}
			},
			{ "translate", Imath::V3d(0.0), 0, "",
				eval_translate,
				{
					{ "intermediate" }
				}
			},
			{ "rotate", Imath::V3d(0.0), 0, "",
				eval_rotate,
				{
					{ "intermediate" }
				}
			},
			{ "scale", Imath::V3d(1.0), 0, "",
				eval_scale,
				{
					{ "intermediate" }
				}
			},
			{ "shear", Imath::V3d(0.0), 0, "",
				eval_shear,
				{
					{ "intermediate" }
				}
			}
		}
	);
}
