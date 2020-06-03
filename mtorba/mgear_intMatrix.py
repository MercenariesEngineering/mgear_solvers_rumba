from mtorba.exporter import NodeExporter, Input

exporter = NodeExporter("mgear_intMatrix", [
	"blend",
	"matrixA",
	"matrixB"], ["output"])
