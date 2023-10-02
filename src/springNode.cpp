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

*/

/////////////////////////////////////////////////
// INCLUDE
/////////////////////////////////////////////////
#include "mgear_solvers.h"

using namespace maquina;
using namespace Imath;

enum
{
	state_aPreviousState = 0,
	state_aGoal,
	state_aStiffness,
	state_aDamping,

	output_aState = 0,
	output_aGoal,
	output_aSpringIntensity
};

static Value eval_state(EvalContext& ctx)
{
	MStatus status;

	// getting inputs attributes
	double damping = ctx.as_double(state_aDamping);
	double stiffness = ctx.as_double(state_aStiffness);

	//MVector goal = ctx(aGoal, &status).asVector();
	MVector goal = ctx.as_V3d(state_aGoal);
	// float aGoalX = goal.x;
	// float aGoalY = goal.y;
	// float aGoalZ = goal.z;

	double currentTime = ctx.evaluation_time();
	//MMatrix parentInverse = ctx(aParentInverse, &status).asMatrix();

	Plug interactive_state = ctx.plug(state_aPreviousState).node().plug("interactive_state");

	Array previous_state;
	if(ctx.is_interactive())
		previous_state = interactive_state.value();
	else if(ctx.is_valid(state_aPreviousState))
		previous_state = ctx.value(state_aPreviousState);

	MVector _previousPosition, _currentPosition;
	double _previousTime;
	if(!previous_state.size())
	{
		// Initialize the point states
		//MGlobal::displayInfo( "mc_spring: Initialize the point states" );
		_previousPosition = goal;
		_currentPosition = goal;
		_previousTime = currentTime;
		//return MS::kSuccess;
	}
	else
	{
		_previousPosition = previous_state.as_V3d(0);
		_currentPosition = previous_state.as_V3d(1);
		_previousTime = previous_state.as_double(2);
	}

	/*// Check if the timestep is just 1 frame since we want a stable simulation
	double timeDifference = currentTime.value() - _previousTime.value();
	if (timeDifference > 1.0 || timeDifference < 0.0) {
		_initialized = false;
		_previousPosition = goal;
		_currentPosition = goal;
		_previousTime = currentTime;
		//MGlobal::displayInfo( "mc_spring: time checker, reset position" );
		//return MS::kSuccess;
	}*/

	// computation
	MVector velocity = (_currentPosition - _previousPosition) * (1.0 - damping);
	MVector newPosition = _currentPosition + velocity;
	MVector goalForce = (goal - newPosition) * stiffness;
	newPosition += goalForce;

	// store the states for the next calculation
	Array new_state;
	new_state.push_back(_currentPosition.to_ilmbase());
	new_state.push_back(newPosition.to_ilmbase());
	new_state.push_back(currentTime);

	if(ctx.is_interactive())
		interactive_state.set_value(new_state);
	else
		interactive_state.set_value(Array::default_value);

	return std::move(new_state);
}

static Value eval_output(EvalContext& ctx)
{
	const Array state(ctx.value(output_aState));
	MVector goal = ctx.as_V3d(output_aGoal);
	double springIntensity = ctx.as_double(output_aSpringIntensity);
	//multipply the position by the spring intensity
	//calculamos depues de los states, para no afectarlos
	MVector newPosition = state.as_V3d(1);
	newPosition = goal + ((newPosition - goal) * springIntensity);

	return newPosition.to_ilmbase();

}

/////////////////////////////////////////////////
// REGISTRATION
/////////////////////////////////////////////////

void register_springNode( Registry &r )
{
	r.register_node
	( 
		"mgear_springNode",
		"Node",
		{
			// ---------------------------------------------------
			// input plugs
			// ---------------------------------------------------
			{ "goal", Imath::V3d(0.f) },
			{ "stiffness", 1.f },
			{ "damping", 1.f },
			{ "intensity", 1.f },

			/* This plug is a hack to get a "dynamic effect" during the interaction.
			 * It holds the last interactive states */
			{ "interactive_state", Array::default_value, },

			// ---------------------------------------------------
			// output plugs
			// ---------------------------------------------------
			{ "state", Array::default_value, 0, "",
				eval_state,
				{
					{ "state", -1 },
					{ "goal" },
					{ "stiffness" },
					{ "damping" }
				}
			},
			{ "output", Imath::V3d(0.f), 0, "",
				eval_output,
				{
					{ "state", },
					{ "goal" },
					{ "intensity" }
				}
			}
		}
	);
}
