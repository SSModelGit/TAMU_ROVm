within UWmBody.UWSensors.Internal;

model BasicAbsoluteAngularVelocity "Measure absolute angular velocity"
  import UWmBody.UWFrames;
  import UWmBody.UWTypes.ResolveInFrameA;
  extends UWmBody.UWSensors.Internal.PartialAbsoluteBaseSensor;
  Modelica.Blocks.Interfaces.RealOutput w[3](each final quantity = "AngularVelocity", each final unit = "rad/s") "Absolute angular velocity vector" annotation(Placement(transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}})));
  parameter UWmBody.UWTypes.ResolveInFrameA resolveInFrame = UWmBody.UWTypes.ResolveInFrameA.frame_a "Frame in which output vector w is resolved (world, frame_a, or frame_resolve)";
equation
  if resolveInFrame == ResolveInFrameA.world then
    w = Frames.angularVelocity1(frame_a.R);
  elseif resolveInFrame == ResolveInFrameA.frame_a then
    w = Frames.angularVelocity2(frame_a.R);
  elseif resolveInFrame == ResolveInFrameA.frame_resolve then
    w = Frames.resolveRelative(Frames.angularVelocity1(frame_a.R), frame_a.R, frame_resolve.R);
  else
    assert(false, "Wrong value for parameter resolveInFrame");
    w = zeros(3);
  end if;
  annotation(Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Text(visible = true, textColor = {64, 64, 64}, extent = {{62, 18}, {146, 48}}, textString = "w"), Text(visible = true, textColor = {64, 64, 64}, extent = {{-130, 80}, {131, 120}}, textString = "%name")}));
end BasicAbsoluteAngularVelocity;
