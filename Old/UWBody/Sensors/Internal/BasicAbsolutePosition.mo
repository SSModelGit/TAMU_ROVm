within UWBody.Sensors.Internal;

model BasicAbsolutePosition "Measure absolute position vector (same as Sensors.AbsolutePosition, but frame_resolve is not conditional and must be connected)"
  import UWBody.Types.ResolveInFrameA;
  extends UWBody.Sensors.Internal.PartialAbsoluteBaseSensor;
  Modelica.Blocks.Interfaces.RealOutput r[3](each final quantity = "Length", each final unit = "m") "Absolute position vector frame_a.r_0 resolved in frame defined by resolveInFrame" annotation(Placement(transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}})));
  parameter UWBody.Types.ResolveInFrameA resolveInFrame = UWBody.Types.ResolveInFrameA.frame_a "Frame in which output vector r is resolved (world, frame_a, or frame_resolve)";
equation
  if resolveInFrame == ResolveInFrameA.world then
    r = frame_a.r_0;
  elseif resolveInFrame == ResolveInFrameA.frame_a then
    r = Frames.resolve2(frame_a.R, frame_a.r_0);
  elseif resolveInFrame == ResolveInFrameA.frame_resolve then
    r = Frames.resolve2(frame_resolve.R, frame_a.r_0);
  else
    assert(false, "Wrong value for parameter resolveInFrame");
    r = zeros(3);
  end if;
  annotation(Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Text(visible = true, textColor = {64, 64, 64}, extent = {{61, 17}, {145, 47}}, textString = "r"), Text(visible = true, textColor = {64, 64, 64}, extent = {{-127, 80}, {134, 120}}, textString = "%name")}));
end BasicAbsolutePosition;
