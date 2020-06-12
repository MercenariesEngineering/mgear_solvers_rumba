from mtorba.exporter import Input
from mtorba.exporters.geometryFilter import geometryFilterExporter

exporter = geometryFilterExporter("mgear_slideCurve2", [
	Input("master_crv", "master_crv", "Spline"),
	Input("master_mat", "master_mat", "M44f"),
	Input("slave_length", "slave_length", "Float"),
	Input("master_length", "master_length", "Float"),
	Input("position", "position", "Float"),
	Input("maxstretch", "maxstretch", "Float"),
	Input("maxsquash", "maxsquash", "Float"),
	Input("softness", "softness", "Float")
], ["outputGeometry"], weights_attribute_name="weights_name")
