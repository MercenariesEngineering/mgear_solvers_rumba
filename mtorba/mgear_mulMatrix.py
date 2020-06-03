from mtorba.exporter import NodeExporter, Input

exporter = NodeExporter("mgear_mulMatrix", [
	"matrixA",
	"matrixB"
], ["output"])
