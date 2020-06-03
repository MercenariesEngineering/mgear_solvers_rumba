from mtorba.exporter import NodeExporter, Input

exporter = NodeExporter("mgear_slideCurve2", [
	"inputGeometry",
	"master_crv",
	"master_mat",
	"slave_length",
	"master_length",
	"position",
	"maxstretch",
	"maxsquash",
	"softness",
], ["outputGeometry"])
