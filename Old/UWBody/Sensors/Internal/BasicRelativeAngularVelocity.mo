within UWBody.Sensors.Internal;

model BasicRelativeAngularVelocity "Measure relative angular velocity"
  import UWBody.Frames;
  import UWBody.Types.ResolveInFrameAB;
  extends UWBody.Sensors.Internal.PartialRelativeBaseSensor;
  Modelica.Blocks.Interfaces.RealOutput w_rel[3](each final quantity = "AngularVelocity", each final unit = "rad/s") "Relative angular velocity vector" annotation(Placement(transformation(origin = {0, -110}, extent = {{-10, -10}, {10, 10}}, rotation = 270)));
  parameter UWBody.Types.ResolveInFrameAB resolveInFrame = UWBody.Types.ResolveInFrameAB.frame_a "Frame in which output vector w_rel is resolved (world, frame_a, frame_b, or frame_resolve)";
protected
  UWBody.Frames.Orientation R_rel "Relative orientation object from frame_a to frame_b";
equation
  R_rel = Frames.relativeRotation(frame_a.R, frame_b.R);
  if resolveInFrame == ResolveInFrameAB.frame_a then
    w_rel = Frames.angularVelocity1(R_rel);
  elseif resolveInFrame == ResolveInFrameAB.frame_b then
    w_rel = Frames.angularVelocity2(R_rel);
  elseif resolveInFrame == ResolveInFrameAB.world then
    w_rel = Frames.resolve1(frame_a.R, Frames.angularVelocity1(R_rel));
  elseif resolveInFrame == ResolveInFrameAB.frame_resolve then
    w_rel = Frames.resolveRelative(Frames.angularVelocity1(R_rel), frame_a.R, frame_resolve.R);
  else
    assert(false, "Wrong value for parameter resolveInFrame");
    w_rel = zeros(3);
  end if;
  annotation(Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Text(visible = true, textColor = {64, 64, 64}, extent = {{12, -106}, {96, -76}}, textString = "w_rel"), Text(visible = true, textColor = {64, 64, 64}, extent = {{-130, 80}, {130, 120}}, textString = "%name")}));
end BasicRelativeAngularVelocity;
