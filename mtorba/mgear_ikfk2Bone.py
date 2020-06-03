from mtorba.exporter import NodeExporter, Input

exporter = NodeExporter("mgear_ikfk2Bone", [
	"blend",
	"lengthA",
	"lengthB",
	"negate",
	"scaleA",
	"scaleB",
	"roll",
	"maxstretch",
	"slide",
	"softness",
	"reverse",
	"root",
	"ikref",
	"upv",
	"fk0",
	"fk1",
	"fk2",
	"inAparent",
	"inBparent",
	"inCenterparent",
	"inEffparent"], ["outA", "outB", "outCenter", "outEff"])
