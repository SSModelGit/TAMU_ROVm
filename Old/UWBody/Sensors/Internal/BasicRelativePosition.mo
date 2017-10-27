within UWBody.Sensors.Internal;

model BasicRelativePosition "Measure relative position vector (same as Sensors.RelativePosition, but frame_resolve is not conditional and must be connected)"
  import UWBody.Types.ResolveInFrameAB;
  extends UWBody.Sensors.Internal.PartialRelativeBaseSensor;
  Modelica.Blocks.Interfaces.RealOutput r_rel[3](each final quantity = "Length", each final unit = "m") "Relative position vector frame_b.r_0 - frame_a.r_0 resolved in frame defined by resolveInFrame" annotation(Placement(transformation(origin = {0, -110}, extent = {{-10, -10}, {10, 10}}, rotation = 270)));
  parameter UWBody.Types.ResolveInFrameAB resolveInFrame = UWBody.Types.ResolveInFrameAB.frame_a "Frame in which output vector r_rel is resolved (world, frame_a, frame_b, or frame_resolve)";
equation
  if resolveInFrame == ResolveInFrameAB.frame_a then
    r_rel = Frames.resolve2(frame_a.R, frame_b.r_0 - frame_a.r_0);
  elseif resolveInFrame == ResolveInFrameAB.frame_b then
    r_rel = Frames.resolve2(frame_b.R, frame_b.r_0 - frame_a.r_0);
  elseif resolveInFrame == ResolveInFrameAB.world then
    r_rel = frame_b.r_0 - frame_a.r_0;
  elseif resolveInFrame == ResolveInFrameAB.frame_resolve then
    r_rel = Frames.resolve2(frame_resolve.R, frame_b.r_0 - frame_a.r_0);
  else
    assert(false, "Wrong value for parameter resolveInFrame");
    r_rel = zeros(3);
  end if;
  annotation(Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Text(visible = true, textColor = {64, 64, 64}, extent = {{12, -106}, {96, -76}}, textString = "r_rel"), Text(visible = true, textColor = {64, 64, 64}, extent = {{-127, 80}, {134, 120}}, textString = "%name")}));
end BasicRelativePosition;
