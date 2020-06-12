from mtorba.exporter import Input
from mtorba.exporters.geometryFilter import geometryFilterExporter

exporter = geometryFilterExporter("mgear_curveCns", [
	Input("inputs", "inputs", "Array")], ["outputGeometry"], weights_attribute_name="weights_name")
