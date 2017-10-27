within UWmBody.UWSensors;

model TransformAbsoluteVector "Transform absolute vector in to another frame"
  extends Modelica.Icons.RotationalSensor;
  UWmBody.UWInterfaces.Frame_a frame_a "Coordinate system from which absolute kinematic quantities are measured" annotation(Placement(transformation(extent = {{-116, -16}, {-84, 16}})));
  UWmBody.UWInterfaces.Frame_resolve frame_resolve if frame_r_in == UWmBody.UWTypes.ResolveInFrameA.frame_resolve or frame_r_out == UWmBody.UWTypes.ResolveInFrameA.frame_resolve "Coordinate system in which r_in or r_out is optionally resolved" annotation(Placement(transformation(extent = {{84, -16}, {116, 16}}), iconTransformation(extent = {{84, -15}, {116, 17}})));
  Blocks.Interfaces.RealInput r_in[3] "Input vector resolved in frame defined by frame_r_in" annotation(Placement(transformation(extent = {{-20, -20}, {20, 20}}, rotation = -90, origin = {0, 120})));
  Blocks.Interfaces.RealOutput r_out[3] "Input vector r_in resolved in frame defined by frame_r_out" annotation(Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = -90, origin = {0, -110})));
  parameter UWmBody.UWTypes.ResolveInFrameA frame_r_in = UWmBody.UWTypes.ResolveInFrameA.frame_a "Frame in which vector r_in is resolved (world, frame_a, or frame_resolve)";
  parameter UWmBody.UWTypes.ResolveInFrameA frame_r_out = frame_r_in "Frame in which vector r_in shall be resolved and provided as r_out (world, frame_a, or frame_resolve)";
protected
  Internal.BasicTransformAbsoluteVector basicTransformVector(frame_r_in = frame_r_in, frame_r_out = frame_r_out) annotation(Placement(transformation(extent = {{-10, -10}, {10, 10}})));
  UWmBody.UWInterfaces.ZeroPosition zeroPosition if not (frame_r_in == UWmBody.UWTypes.ResolveInFrameA.frame_resolve or frame_r_out == UWmBody.UWTypes.ResolveInFrameA.frame_resolve) annotation(Placement(transformation(extent = {{40, 18}, {60, 38}})));
equation
  connect(basicTransformVector.frame_a, frame_a) annotation(Line(points = {{-10, 0}, {-100, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(basicTransformVector.frame_resolve, frame_resolve) annotation(Line(points = {{10, 0}, {100, 0}}, color = {95, 95, 95}, pattern = LinePattern.Dot));
  connect(zeroPosition.frame_resolve, basicTransformVector.frame_resolve) annotation(Line(points = {{40, 28}, {32, 28}, {32, 0}, {10, 0}}, color = {95, 95, 95}, pattern = LinePattern.Dot));
  connect(basicTransformVector.r_out, r_out) annotation(Line(points = {{0, -11}, {0, -110}}, color = {0, 0, 127}));
  connect(basicTransformVector.r_in, r_in) annotation(Line(points = {{0, 12}, {0, 120}}, color = {0, 0, 127}));
  annotation(Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Line(visible = true, points = {{0, 100}, {0, 70}}, color = {0, 0, 127}), Text(visible = true, textColor = {64, 64, 64}, extent = {{-104, 96}, {-18, 124}}, textString = "r_in"), Text(visible = true, textColor = {64, 64, 64}, extent = {{-124, -104}, {2, -76}}, textString = "r_out"), Line(visible = true, points = {{95, 0}, {95, 0}, {70, 0}, {70, 0}}, pattern = LinePattern.Dot), Text(visible = true, textColor = {64, 64, 64}, extent = {{58, 22}, {189, 47}}, textString = "resolve"), Line(visible = true, points = {{-70, 0}, {-96, 0}, {-96, 0}}), Text(visible = true, textColor = {64, 64, 64}, extent = {{-116, 20}, {-80, 45}}, textString = "a"), Line(visible = true, points = {{0, -70}, {0, -110}}, color = {0, 0, 127})}), Documentation(info = "<html>
<p>
The input vector \"Real r_in[3]\" is assumed to be an absolute kinematic quantity
of frame_a that is defined to be resolved in the frame defined
with parameter \"frame_r_in\". This model resolves vector r_in in the
coordinate system defined with parameter \"frame_r_out\" and returns the
transformed output vector as \"Real r_out[3]\";
</p>
</html>"));
end TransformAbsoluteVector;
