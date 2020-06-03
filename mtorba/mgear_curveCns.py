from mtorba.exporter import NodeExporter, Input

exporter = NodeExporter("mgear_curveCns", [
	"inputGeometry",
	"inputs"], ["outputGeometry"])
