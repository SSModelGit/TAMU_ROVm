within UWmBody.UWSensors.Internal;

model BasicTransformAbsoluteVector "Transform absolute vector in to another frame"
  import UWmBody.UWFrames;
  import UWmBody.UWTypes.ResolveInFrameA;
  extends Modelica.Icons.RotationalSensor;
  parameter UWmBody.UWTypes.ResolveInFrameA frame_r_in = UWmBody.UWTypes.ResolveInFrameA.frame_a "Frame in which vector r_in is resolved (world, frame_a, or frame_resolve)";
  parameter UWmBody.UWTypes.ResolveInFrameA frame_r_out = frame_r_in "Frame in which vector r_out (= r_in in other frame) is resolved (world, frame_a, or frame_resolve)";
  UWmBody.UWInterfaces.Frame_a frame_a "Coordinate system from which absolute kinematic quantities are measured" annotation(Placement(transformation(extent = {{-116, -16}, {-84, 16}})));
  UWmBody.UWInterfaces.Frame_resolve frame_resolve "Coordinate system in which vector is optionally resolved" annotation(Placement(transformation(extent = {{-16, -16}, {16, 16}}, origin = {100, 0}), iconTransformation(extent = {{-16, -16}, {16, 16}}, origin = {100, 0})));
  Blocks.Interfaces.RealInput r_in[3] "Input vector resolved in frame defined by frame_r_in" annotation(Placement(transformation(extent = {{-20, -20}, {20, 20}}, rotation = -90, origin = {0, 120})));
  Blocks.Interfaces.RealOutput r_out[3] "Input vector r_in resolved in frame defined by frame_r_out" annotation(Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = -90, origin = {0, -110})));
protected
  UWmBody.UWFrames.Orientation R1 "Orientation object from world frame to frame in which r_in is resolved";
equation
  assert(cardinality(frame_a) > 0, "Connector frame_a must be connected at least once");
  assert(cardinality(frame_resolve) == 1, "Connector frame_resolve must be connected exactly once");
  frame_a.f = zeros(3);
  frame_a.t = zeros(3);
  frame_resolve.f = zeros(3);
  frame_resolve.t = zeros(3);
  if frame_r_out == frame_r_in then
    r_out = r_in;
    R1 = Frames.nullRotation();
  else
    if frame_r_in == ResolveInFrameA.world then
      R1 = Frames.nullRotation();
    elseif frame_r_in == ResolveInFrameA.frame_a then
      R1 = frame_a.R;
    elseif frame_r_in == ResolveInFrameA.frame_resolve then
      R1 = frame_resolve.R;
    else
      assert(false, "Wrong value for parameter frame_r_in");
      R1 = Frames.nullRotation();
    end if;
    if frame_r_out == ResolveInFrameA.world then
      r_out = Frames.resolve1(R1, r_in);
    elseif frame_r_out == ResolveInFrameA.frame_a then
      r_out = Frames.resolveRelative(r_in, R1, frame_a.R);
    elseif frame_r_out == ResolveInFrameA.frame_resolve then
      r_out = Frames.resolveRelative(r_in, R1, frame_resolve.R);
    else
      assert(false, "Wrong value for parameter frame_r_out");
      r_out = zeros(3);
    end if;
  end if;
  annotation(Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Text(visible = true, textColor = {64, 64, 64}, extent = {{-128, -112}, {-2, -84}}, textString = "r_out"), Text(visible = true, textColor = {64, 64, 64}, extent = {{-108, 109}, {-22, 137}}, textString = "r_in"), Line(visible = true, points = {{0, 100}, {0, 70}}, color = {0, 0, 127}), Line(visible = true, points = {{0, -70}, {0, -110}}, color = {0, 0, 127}), Text(visible = true, textColor = {64, 64, 64}, extent = {{58, 22}, {189, 47}}, textString = "resolve"), Text(visible = true, textColor = {64, 64, 64}, extent = {{-116, 20}, {-80, 45}}, textString = "a"), Line(visible = true, points = {{-70, 0}, {-96, 0}, {-96, 0}}), Line(visible = true, points = {{95, 0}, {95, 0}, {70, 0}, {70, 0}}, pattern = LinePattern.Dot)}));
end BasicTransformAbsoluteVector;
