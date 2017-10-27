within UWBody.Sensors.Internal;

model BasicTransformRelativeVector "Transform relative vector in to another frame"
  import UWBody.Frames;
  import UWBody.Types.ResolveInFrameAB;
  extends UWBody.Sensors.Internal.PartialRelativeBaseSensor;
  parameter UWBody.Types.ResolveInFrameAB frame_r_in = UWBody.Types.ResolveInFrameAB.frame_a "Frame in which vector r_in is resolved (world, frame_a, frame_b, or frame_resolve)";
  parameter UWBody.Types.ResolveInFrameAB frame_r_out = frame_r_in "Frame in which vector r_out (= r_in in other frame) is resolved (world, frame_a, frame_b, or frame_resolve)";
  Modelica.Blocks.Interfaces.RealInput r_in[3] "Input vector resolved in frame defined by frame_r_in" annotation(Placement(transformation(extent = {{-20, -20}, {20, 20}}, rotation = -90, origin = {0, 120})));
  Modelica.Blocks.Interfaces.RealOutput r_out[3] "Input vector r_in resolved in frame defined by frame_r_out" annotation(Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = -90, origin = {0, -110})));
protected
  UWBody.Frames.Orientation R1 "Orientation object from world frame to frame in which r_in is resolved";
equation
  if frame_r_out == frame_r_in then
    r_out = r_in;
    R1 = Frames.nullRotation();
  else
    if frame_r_in == ResolveInFrameAB.world then
      R1 = Frames.nullRotation();
    elseif frame_r_in == ResolveInFrameAB.frame_a then
      R1 = frame_a.R;
    elseif frame_r_in == ResolveInFrameAB.frame_b then
      R1 = frame_b.R;
    else
      R1 = frame_resolve.R;
    end if;
    if frame_r_out == ResolveInFrameAB.world then
      r_out = Frames.resolve1(R1, r_in);
    elseif frame_r_out == ResolveInFrameAB.frame_a then
      r_out = Frames.resolveRelative(r_in, R1, frame_a.R);
    elseif frame_r_out == ResolveInFrameAB.frame_b then
      r_out = Frames.resolveRelative(r_in, R1, frame_b.R);
    else
      r_out = Frames.resolveRelative(r_in, R1, frame_resolve.R);
    end if;
  end if;
  annotation(Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Text(visible = true, textColor = {64, 64, 64}, extent = {{-128, -120}, {-2, -92}}, textString = "r_out"), Text(visible = true, textColor = {64, 64, 64}, extent = {{-108, 116}, {-22, 144}}, textString = "r_in"), Line(visible = true, points = {{0, 100}, {0, 70}}, color = {0, 0, 127})}));
end BasicTransformRelativeVector;
