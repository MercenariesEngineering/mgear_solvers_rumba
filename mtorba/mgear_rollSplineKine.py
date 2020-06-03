from mtorba.exporter import NodeExporter, Input

exporter = NodeExporter("mgear_rollSplineKine", [
	"ctlParent",
	"inputs",
	"inputsRoll",
	"outputParent",
	"u",
	"resample",
	"subdiv",
	"absolute",
	], ["output"])
