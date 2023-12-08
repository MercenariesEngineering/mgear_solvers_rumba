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
#include "Maquina/EvalContext.h"
#include "Maquina/Registry.h"

using namespace maquina;
using namespace Imath;

/////////////////////////////////////////////////
// EVALUATION FUNCTIONS
/////////////////////////////////////////////////

static Value eval_output(EvalContext& ctx)
{
	int ro = ctx.as_int(0);
	int inv_ro [6] = {5, 3, 4, 1, 2, 0};

	return inv_ro[ro];
}

/////////////////////////////////////////////////
// REGISTRATION
/////////////////////////////////////////////////

void register_inverseRotOrder( Registry &r )
{
	r.register_node
	( 
		"mgear_inverseRotOrder",
		"Node",
		{
			{ "rotOrder", 0, PlugDescriptor::serial, "{\"enum\":{\"xyz\":0,\"yzx\":1,\"zxy\":2,\"xzy\":3,\"yxz\":4,\"zyx\":5}}"},
			{ "output", 0, 0, "",
				eval_output,
				{
					{ "rotOrder" },
				}
			}
		}
	);
}
