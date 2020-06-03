from mtorba.exporter import NodeExporter, Input

exporter = NodeExporter("mgear_squashStretch2", [
	"global_scale",
	"blend",
	"driver",
	"driver_min",
	"driver_ctr",
	"driver_max",
	"axis",
	"squash",
	"stretch"], ["output"])
