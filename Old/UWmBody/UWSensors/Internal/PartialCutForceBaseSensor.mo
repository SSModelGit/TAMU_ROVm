within UWmBody.UWSensors.Internal;

partial model PartialCutForceBaseSensor "Base model to measure the cut force and/or torque between two frames, defined by equations (frame_resolve must be connected exactly once)"
  extends Modelica.Icons.RotationalSensor;
  UWmBody.UWInterfaces.Frame_a frame_a "Coordinate system a" annotation(Placement(transformation(extent = {{-116, -16}, {-84, 16}})));
  UWmBody.UWInterfaces.Frame_b frame_b "Coordinate system b" annotation(Placement(transformation(extent = {{84, -16}, {116, 16}})));
  UWmBody.UWInterfaces.Frame_resolve frame_resolve "The output vector is optionally resolved in this frame (cut-force/-torque are set to zero)" annotation(Placement(transformation(origin = {80, -100}, extent = {{-16, -16}, {16, 16}}, rotation = 270)));
  parameter UWmBody.UWTypes.ResolveInFrameA resolveInFrame = UWmBody.UWTypes.ResolveInFrameA.frame_a "Frame in which output vector is resolved (world, frame_a, or frame_resolve)";
protected
  outer UWmBody.UWWorld world;
equation
  Connections.branch(frame_a.R, frame_b.R);
  assert(cardinality(frame_a) > 0, "Connector frame_a of cut-force/-torque sensor object is not connected");
  assert(cardinality(frame_b) > 0, "Connector frame_b of cut-force/-torque sensor object is not connected");
  // frame_a and frame_b are identical
  frame_a.r_0 = frame_b.r_0;
  frame_a.R = frame_b.R;
  // force and torque balance
  zeros(3) = frame_a.f + frame_b.f;
  zeros(3) = frame_a.t + frame_b.t;
  frame_resolve.f = zeros(3);
  frame_resolve.t = zeros(3);
  annotation(Documentation(info = "<html>
<p>
This is a base class for 3-dim. mechanical components with two frames
and one output port in order to measure the cut-force and/or
cut-torque acting between the two frames and
to provide the measured signals as output for further processing
with the blocks of package Modelica.Blocks.
</p>
</html>"), Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Line(visible = true, points = {{-70, 0}, {-101, 0}}), Line(visible = true, points = {{70, 0}, {100, 0}}), Text(visible = true, textColor = {64, 64, 64}, extent = {{-132, 80}, {129, 120}}, textString = "%name"), Text(visible = true, textColor = {64, 64, 64}, extent = {{-118, 30}, {-82, 55}}, textString = "a"), Text(visible = true, textColor = {64, 64, 64}, extent = {{83, 30}, {119, 55}}, textString = "b"), Text(visible = true, textColor = {64, 64, 64}, extent = {{70, -91}, {201, -66}}, textString = "resolve"), Line(visible = true, points = {{80, 0}, {80, -100}}, color = {95, 95, 95}, pattern = LinePattern.Dot)}));
end PartialCutForceBaseSensor;
