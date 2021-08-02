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

using namespace maquina;
using namespace Imath;

/////////////////////////////////////////////////
// FUNCTIONS
/////////////////////////////////////////////////

MTransformationMatrix getIKTransform(s_GetIKTransform data, MString name){

    // prepare all variables
	MTransformationMatrix result;
	MVector bonePos, rootPos, effPos, upvPos, rootEff, xAxis, yAxis, zAxis, rollAxis;

    rootPos = data.root.getTranslation(MSpace::kWorld);
    effPos = data.eff.getTranslation(MSpace::kWorld);
    upvPos = data.upv.getTranslation(MSpace::kWorld);
    rootEff = effPos - rootPos;
    rollAxis = rootEff.normal();

    double rootEffDistance = rootEff.length();

    // init the scaling
	double scale[3];
	data.root.getScale(scale, MSpace::kWorld);
	double global_scale = scale[0];

	result.setScale(scale, MSpace::kWorld);

    // Distance with MaxStretch ---------------------
    double restLength = (data.lengthA * data.scaleA + data.lengthB * data.scaleB) * global_scale;
    double distance = rootEffDistance;
    double distance2 = distance;
    if (distance > (restLength * data.maxstretch))
        distance = restLength * data.maxstretch;

    // Adapt Softness value to chain length --------
    data.softness = data.softness * restLength * .1;

    // Stretch and softness ------------------------
    // We use the real distance from root to controler to calculate the softness
    // This way we have softness working even when there is no stretch
    double stretch = std::max(1.0, distance / restLength);
    double da = restLength - data.softness;
    if ((data.softness > 0) && (distance2 > da)){
        double newlen = data.softness*(1.0 - exp(-(distance2 -da)/data.softness)) + da;
        stretch = distance / newlen;
	}

    data.lengthA = data.lengthA * stretch * data.scaleA * global_scale;
    data.lengthB = data.lengthB * stretch * data.scaleB * global_scale;

    // Reverse -------------------------------------
    double d = distance / (data.lengthA + data.lengthB);

	double reverse_scale;
    if (data.reverse < 0.5)
        reverse_scale = 1-(data.reverse*2 * (1-d));
    else
        reverse_scale = 1-((1-data.reverse)*2 * (1-d));

    data.lengthA *= reverse_scale;
    data.lengthB *= reverse_scale;

    bool invert = data.reverse > 0.5;

    // Slide ---------------------------------------
	double slide_add;
    if (data.slide < .5)
        slide_add = (data.lengthA * (data.slide * 2)) - (data.lengthA);
    else
        slide_add = (data.lengthB * (data.slide * 2)) - (data.lengthB);

    data.lengthA += slide_add;
    data.lengthB -= slide_add;

    // calculate the angle inside the triangle!
    double angleA = 0;
    double angleB = 0;

    // check if the divider is not null otherwise the result is nan
    // and the output disapear from xsi, that breaks constraints
    if ((rootEffDistance < data.lengthA + data.lengthB) && (rootEffDistance > abs(data.lengthA - data.lengthB) + 1E-6)){

        // use the law of cosine for lengthA
        double a = data.lengthA;
        double b = rootEffDistance;
        double c = data.lengthB;

        angleA = acos(std::min(1.0, (a * a + b * b - c * c ) / ( 2 * a * b)));

        // use the law of cosine for lengthB
        a = data.lengthB;
        b = data.lengthA;
        c = rootEffDistance;
        angleB = acos(std::min(1.0, (a * a + b * b - c * c ) / ( 2 * a * b)));

        // invert the angles if need be
        if (invert){
            angleA = -angleA;
            angleB = -angleB;
  }
	}

    // start with the X and Z axis
    xAxis = rootEff;
    xAxis.normalize();
    yAxis = linearInterpolate(rootPos, effPos, .5);
    yAxis = upvPos - yAxis;
    yAxis.normalize();
    yAxis = rotateVectorAlongAxis(yAxis, rollAxis, data.roll);
    zAxis = xAxis ^ yAxis;
    zAxis.normalize();
    yAxis = zAxis ^ xAxis;
    yAxis.normalize();

    // switch depending on our mode
    if (name == "outA"){

        // check if we need to rotate the bone
        if (angleA != 0.0)
            xAxis = rotateVectorAlongAxis(xAxis, zAxis, -angleA);

        if (data.negate)
            xAxis *= -1;
        // cross the yAxis and normalize
        yAxis = zAxis ^ xAxis;
        yAxis.normalize();

        // output the rotation
        MQuaternion q = getQuaternionFromAxes(xAxis,yAxis,zAxis);
        result.setRotationQuaternion(q.x, q.y, q.z, q.w);

        // set the scaling + the position
  double s[3] = {data.lengthA, global_scale, global_scale};
  result.setScale(s, MSpace::kWorld);
        result.setTranslation(rootPos, MSpace::kWorld);
	}
    else if (name == "outB"){

        // check if we need to rotate the bone
        if (angleA != 0.0)
            xAxis = rotateVectorAlongAxis(xAxis, zAxis, -angleA);

        // calculate the position of the elbow!
        bonePos = xAxis * data.lengthA;
        bonePos += rootPos;

        // check if we need to rotate the bone
        if (angleB != 0.0)
            xAxis = rotateVectorAlongAxis(xAxis, zAxis, -(angleB - PI));

        if (data.negate)
            xAxis *= -1;

        // cross the yAxis and normalize
        yAxis = zAxis ^ xAxis;
        yAxis.normalize();

        // output the rotation
        MQuaternion q = getQuaternionFromAxes(xAxis,yAxis,zAxis);
        result.setRotationQuaternion(q.x, q.y, q.z, q.w);

        // set the scaling + the position
  double s[3] = {data.lengthB, global_scale, global_scale};
  result.setScale(s, MSpace::kWorld);
        result.setTranslation(bonePos, MSpace::kWorld);
	}
    else if (name == "outCenter"){

        // check if we need to rotate the bone
        if (angleA != 0.0)
            xAxis = rotateVectorAlongAxis(xAxis, zAxis, -angleA);

        // calculate the position of the elbow!
        bonePos = xAxis * data.lengthA;
        bonePos += rootPos;

        // check if we need to rotate the bone
        if (angleB != 0.0){
            if (invert){
                angleB += PI * 2;
  	}
            xAxis = rotateVectorAlongAxis(xAxis, zAxis, -(angleB *.5 - PI*.5));
  }


        // cross the yAxis and normalize
        // yAxis.Sub(upvPos,bonePos); // this was flipping the centerN when the elbow/upv was aligned to root/eff
        zAxis = xAxis ^ yAxis;
        zAxis.normalize();

        if (data.negate)
            xAxis *= -1;

        yAxis = zAxis ^ xAxis;
        yAxis.normalize();

        // output the rotation
        MQuaternion q = getQuaternionFromAxes(xAxis,yAxis,zAxis);
        result.setRotationQuaternion(q.x, q.y, q.z, q.w);

        // set the scaling + the position
        // result.SetSclX(stretch * data["root.GetSclX());

        result.setTranslation(bonePos, MSpace::kWorld);
	}

    else if (name == "outEff"){

        // check if we need to rotate the bone
        effPos = rootPos;
        if (angleA != 0.0)
            xAxis = rotateVectorAlongAxis(xAxis, zAxis, -angleA);

        // calculate the position of the elbow!
        bonePos = xAxis * data.lengthA;
        effPos += bonePos;

        // check if we need to rotate the bone
        if (angleB != 0.0)
            xAxis = rotateVectorAlongAxis(xAxis, zAxis, -(angleB - PI));

        // calculate the position of the effector!
        bonePos = xAxis * data.lengthB;
        effPos += bonePos;

        // output the rotation
        result = data.eff;
        result.setTranslation(effPos, MSpace::kWorld);
	}

    return result;
}

MTransformationMatrix getFKTransform(s_GetFKTransform data, MString name){

	// prepare all variables
	MTransformationMatrix result;

	MVector xAxis, yAxis, zAxis;

	if (name == "outA"){
  result = data.bone1;
  xAxis = data.bone2.getTranslation(MSpace::kWorld) - data.bone1.getTranslation(MSpace::kWorld);

  double scale[3] = {xAxis.length(), 1.0, 1.0};
  result.setScale(scale, MSpace::kWorld);

  if (data.negate)
  	xAxis *= -1;

  // cross the yAxis and normalize
  xAxis.normalize();

  zAxis = MVector(0,0,1);
  zAxis = zAxis.rotateBy(data.bone1.rotation());
  yAxis = zAxis ^ xAxis;

  // rotation
  MQuaternion q = getQuaternionFromAxes(xAxis,yAxis,zAxis);
  result.setRotationQuaternion(q.x, q.y, q.z, q.w);
	}
	else if (name == "outB"){

  result = data.bone2;
  xAxis = data.eff.getTranslation(MSpace::kWorld) - data.bone2.getTranslation(MSpace::kWorld);

  double scale[3] = {xAxis.length(), 1.0, 1.0};
  result.setScale(scale, MSpace::kWorld);

  if (data.negate)
  	xAxis *= -1;

  // cross the yAxis and normalize
  xAxis.normalize();
  yAxis = MVector(0,1,0);
  yAxis = yAxis.rotateBy(data.bone2.rotation());
  zAxis = xAxis ^ yAxis;
  zAxis.normalize();
  yAxis = zAxis ^ xAxis;
  yAxis.normalize();

  // rotation
  MQuaternion q = getQuaternionFromAxes(xAxis,yAxis,zAxis);
  result.setRotationQuaternion(q.x, q.y, q.z, q.w);
	}
	else if (name == "outCenter"){


        // Only +/-180 degree with this one but we don't get the shear issue anymore
        MTransformationMatrix t = mapWorldPoseToObjectSpace(data.bone1, data.bone2);
    MEulerRotation er = t.eulerRotation();
        er *= .5;
        MQuaternion q = er.asQuaternion();
        t.setRotationQuaternion(q.x, q.y, q.z, q.w);
        t = mapObjectPoseToWorldSpace(data.bone1, t);
        q = t.rotation();

        result.setRotationQuaternion(q.x, q.y, q.z, q.w);

  // rotation
  result.setTranslation(data.bone2.getTranslation(MSpace::kWorld), MSpace::kWorld);
	}
	else if (name == "outEff")
  result = data.eff;

	return result;
}

/////////////////////////////////////////////////
// EVALUATION FUNCTIONS
/////////////////////////////////////////////////

enum
{
	Dep_blend = 0,
	Dep_lengthA,
	Dep_lengthB,
	Dep_negate,
	Dep_scaleA,
	Dep_scaleB,
	Dep_roll,
	Dep_maxstretch,
	Dep_slide,
	Dep_softness,
	Dep_reverse,
	Dep_root,
	Dep_ikref,
	Dep_upv,
	Dep_fk0,
	Dep_fk1,
	Dep_fk2,
	Dep_inAparent,
	Dep_inBparent,
	Dep_inCenterparent,
	Dep_inEffparent,
};

MTransformationMatrix eval_output(EvalContext& ctx, const MString& outName)
{
	MStatus returnStatus;

	// INPUT MATRICES
	MMatrix in_root = ctx.as_M44f(Dep_root);
	MMatrix in_ikref = ctx.as_M44f(Dep_ikref);
	MMatrix in_upv = ctx.as_M44f(Dep_upv);
	MMatrix in_fk0 = ctx.as_M44f(Dep_fk0);
	MMatrix in_fk1 = ctx.as_M44f(Dep_fk1);
	MMatrix in_fk2 = ctx.as_M44f(Dep_fk2);

	// SLIDERS
	double in_blend = (double)ctx.as_float(Dep_blend);

	// setup the base IK parameters
	s_GetIKTransform ikparams;

	ikparams.root = in_root;
	ikparams.eff = in_ikref;
	ikparams.upv = in_upv;

	ikparams.lengthA = (double)ctx.as_float(Dep_lengthA);
	ikparams.lengthB = (double)ctx.as_float(Dep_lengthB);
	ikparams.negate = ctx.as_bool(Dep_negate);
	ikparams.roll = degrees2radians((double)ctx.as_float(Dep_roll));
	ikparams.scaleA = (double)ctx.as_float(Dep_scaleA);
	ikparams.scaleB = (double)ctx.as_float(Dep_scaleB);
	ikparams.maxstretch = (double)ctx.as_float(Dep_maxstretch);
	ikparams.softness = (double)ctx.as_float(Dep_softness);
	ikparams.slide = (double)ctx.as_float(Dep_slide);
	ikparams.reverse = (double)ctx.as_float(Dep_reverse);

	// setup the base FK parameters
	s_GetFKTransform fkparams;

	fkparams.root = in_root;
	fkparams.bone1 = in_fk0;
	fkparams.bone2 = in_fk1;
	fkparams.eff = in_fk2;

	fkparams.lengthA = ikparams.lengthA;
	fkparams.lengthB = ikparams.lengthB;
	fkparams.negate = ikparams.negate;

	MTransformationMatrix result;
	if(in_blend == 0.0)
  result = getFKTransform(fkparams, outName);
	else if(in_blend == 1.0)
  result = getIKTransform(ikparams, outName);
	else{
  // here is where the blending happens!
        MTransformationMatrix ikbone1 = getIKTransform(ikparams, "outA");
        MTransformationMatrix ikbone2 = getIKTransform(ikparams, "outB");
        MTransformationMatrix ikeff = getIKTransform(ikparams, "outEff");

        MTransformationMatrix fkbone1 = getFKTransform(fkparams, "outA");
        MTransformationMatrix fkbone2 = getFKTransform(fkparams, "outB");
        MTransformationMatrix fkeff = getFKTransform(fkparams, "outEff");

        // remove scale to avoid shearing issue
        // This is not necessary in Softimage because the scaling hierarchy is not computed the same way.
  double noScale[3] = {1,1,1};
  ikbone1.setScale(noScale, MSpace::kWorld);
  ikbone2.setScale(noScale, MSpace::kWorld);
  ikeff.setScale(noScale, MSpace::kWorld);
  fkbone1.setScale(noScale, MSpace::kWorld);
  fkbone2.setScale(noScale, MSpace::kWorld);
  fkeff.setScale(noScale, MSpace::kWorld);

        // map the secondary transforms from global to local
        ikeff = mapWorldPoseToObjectSpace(ikbone2, ikeff);
        fkeff = mapWorldPoseToObjectSpace(fkbone2, fkeff);
        ikbone2 = mapWorldPoseToObjectSpace(ikbone1, ikbone2);
        fkbone2 = mapWorldPoseToObjectSpace(fkbone1, fkbone2);

        // now blend them!
  fkparams.bone1 = interpolateTransform(fkbone1, ikbone1, in_blend);
  fkparams.bone2 = interpolateTransform(fkbone2, ikbone2, in_blend);
  fkparams.eff = interpolateTransform(fkeff, ikeff, in_blend);


        // now map the local transform back to global!
  fkparams.bone2 = mapObjectPoseToWorldSpace(fkparams.bone1, fkparams.bone2);
        fkparams.eff = mapObjectPoseToWorldSpace(fkparams.bone2, fkparams.eff);

        // calculate the result based on that
        result = getFKTransform(fkparams, outName);
	}
	return result;
}

static Value eval_outA(EvalContext& ctx)
{
	MMatrix in_aParent = ctx.as_M44f(Dep_inAparent);
	const MTransformationMatrix result = eval_output(ctx, "outA");
	return Imath::M44f(( result.asMatrix() * in_aParent.inverse() ).to_ilmbase());
}

static Value eval_outB(EvalContext& ctx)
{
	MMatrix in_bParent = ctx.as_M44f(Dep_inBparent);
	const MTransformationMatrix result = eval_output(ctx, "outB");
	return Imath::M44f(( result.asMatrix() *  in_bParent.inverse() ).to_ilmbase());
}

static Value eval_outCenter(EvalContext& ctx)
{
	MMatrix in_centerParent = ctx.as_M44f(Dep_inCenterparent);
	const MTransformationMatrix result = eval_output(ctx, "outCenter");
	return Imath::M44f(( result.asMatrix() * in_centerParent.inverse() ).to_ilmbase());
}

static Value eval_outEff(EvalContext& ctx)
{
	MMatrix in_effParent = ctx.as_M44f(Dep_inEffparent);
	const MTransformationMatrix result = eval_output(ctx, "outEff");
	return Imath::M44f(( result.asMatrix() * in_effParent.inverse() ).to_ilmbase());
}

/////////////////////////////////////////////////
// REGISTRATION
/////////////////////////////////////////////////

void register_ikfk2Bone( Registry &r )
{
	r.register_node
	( 
	"mgear_ikfk2Bone",
	"Node",
	{
		{ "blend", 0.f, PlugDescriptor::serial, "{\"min\":0.0,\"max\":1.0}" },
		{ "lengthA", 0.f },
		{ "lengthB", 0.f },
		{ "negate", false },
		{ "scaleA", 1.f },
		{ "scaleB", 1.f },
		{ "roll", 0.f },
		{ "maxstretch", 1.5f },
		{ "slide", 0.5f },
		{ "softness", 0.f },
		{ "reverse", 0.f },
		{ "root", Imath::identity44f },
		{ "ikref", Imath::identity44f },
		{ "upv", Imath::identity44f },
		{ "fk0", Imath::identity44f },
		{ "fk1", Imath::identity44f },
		{ "fk2", Imath::identity44f },
		{ "inAparent", Imath::identity44f },
		{ "inBparent", Imath::identity44f },
		{ "inCenterparent", Imath::identity44f },
		{ "inEffparent", Imath::identity44f },
		{ "outA", Imath::identity44f, 0, "",
			eval_outA,
			{
				{ "blend" },
				{ "lengthA" },
				{ "lengthB" },
				{ "negate" },
				{ "scaleA" },
				{ "scaleB" },
				{ "roll" },
				{ "maxstretch" },
				{ "slide" },
				{ "softness" },
				{ "reverse" },
				{ "root" },
				{ "ikref" },
				{ "upv" },
				{ "fk0" },
				{ "fk1" },
				{ "fk2" },
				{ "inAparent" },
				{ "inBparent" },
				{ "inCenterparent" },
				{ "inEffparent" },
			}
		},
		{ "outB", Imath::identity44f, 0, "",
			eval_outB,
			{
				{ "blend" },
				{ "lengthA" },
				{ "lengthB" },
				{ "negate" },
				{ "scaleA" },
				{ "scaleB" },
				{ "roll" },
				{ "maxstretch" },
				{ "slide" },
				{ "softness" },
				{ "reverse" },
				{ "root" },
				{ "ikref" },
				{ "upv" },
				{ "fk0" },
				{ "fk1" },
				{ "fk2" },
				{ "inAparent" },
				{ "inBparent" },
				{ "inCenterparent" },
				{ "inEffparent" },
			}
			},
		{ "outCenter", Imath::identity44f, 0, "",
			eval_outCenter,
			{
				{ "blend" },
				{ "lengthA" },
				{ "lengthB" },
				{ "negate" },
				{ "scaleA" },
				{ "scaleB" },
				{ "roll" },
				{ "maxstretch" },
				{ "slide" },
				{ "softness" },
				{ "reverse" },
				{ "root" },
				{ "ikref" },
				{ "upv" },
				{ "fk0" },
				{ "fk1" },
				{ "fk2" },
				{ "inAparent" },
				{ "inBparent" },
				{ "inCenterparent" },
				{ "inEffparent" },
			}
		},
		{ "outEff", Imath::identity44f, 0, "",
			eval_outEff,
			{
				{ "blend" },
				{ "lengthA" },
				{ "lengthB" },
				{ "negate" },
				{ "scaleA" },
				{ "scaleB" },
				{ "roll" },
				{ "maxstretch" },
				{ "slide" },
				{ "softness" },
				{ "reverse" },
				{ "root" },
				{ "ikref" },
				{ "upv" },
				{ "fk0" },
				{ "fk1" },
				{ "fk2" },
				{ "inAparent" },
				{ "inBparent" },
				{ "inCenterparent" },
				{ "inEffparent" },
			}
		}
	});
}
