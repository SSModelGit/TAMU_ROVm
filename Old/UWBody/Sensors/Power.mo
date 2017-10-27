within UWBody.Sensors;

model Power "Measure power flowing from frame_a to frame_b"
  extends Modelica.Icons.RotationalSensor;
  extends UWBody.Interfaces.PartialTwoFrames;
  Modelica.Blocks.Interfaces.RealOutput power(quantity = "Power", unit = "W") "Power at frame_a as output signal" annotation(Placement(transformation(origin = {-80, -110}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
equation
  Connections.branch(frame_a.R, frame_b.R);
  frame_a.r_0 = frame_b.r_0;
  frame_a.R = frame_b.R;
  zeros(3) = frame_a.f + frame_b.f;
  zeros(3) = frame_a.t + frame_b.t;
  power = frame_a.f * Frames.resolve2(frame_a.R, der(frame_a.r_0)) + frame_a.t * Frames.angularVelocity2(frame_a.R);
  annotation(Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Line(visible = true, points = {{-70, 0}, {-101, 0}}), Line(visible = true, points = {{70, 0}, {100, 0}}), Line(visible = true, points = {{-80, 0}, {-80, -110}}, color = {0, 0, 127}), Text(visible = true, textColor = {64, 64, 64}, extent = {{-60, -114}, {16, -92}}, textString = "power"), Text(visible = true, textColor = {64, 64, 64}, extent = {{-128, 80}, {126, 120}}, textString = "%name")}), Documentation(info = "<html>
<p>
This component provides the power flowing from frame_a to frame_b
as output signal <b>power</b>.
</p>
</html>"));
end Power;
