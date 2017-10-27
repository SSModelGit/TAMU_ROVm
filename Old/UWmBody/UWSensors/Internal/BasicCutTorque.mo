within UWmBody.UWSensors.Internal;

model BasicCutTorque "Measure cut torque vector (frame_resolve must be connected)"
  import UWmBody.UWTypes.ResolveInFrameA;
  import UWmBody.UWFrames;
  extends UWmBody.UWSensors.Internal.PartialCutForceBaseSensor;
  Modelica.Blocks.Interfaces.RealOutput torque[3](each final quantity = "Torque", each final unit = "N.m") "Cut torque resolved in frame defined by resolveInFrame" annotation(Placement(transformation(origin = {-80, -110}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
  parameter Boolean positiveSign = true "= true, if torque with positive sign is returned (= frame_a.t), otherwise with negative sign (= frame_b.t)";
protected
  parameter Integer csign = if positiveSign then +1 else -1;
equation
  if resolveInFrame == ResolveInFrameA.world then
    torque = Frames.resolve1(frame_a.R, frame_a.t) * csign;
  elseif resolveInFrame == ResolveInFrameA.frame_a then
    torque = frame_a.t * csign;
  elseif resolveInFrame == ResolveInFrameA.frame_resolve then
    torque = Frames.resolveRelative(frame_a.t, frame_a.R, frame_resolve.R) * csign;
  else
    assert(false, "Wrong value for parameter resolveInFrame");
    torque = zeros(3);
  end if;
  annotation(Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Text(visible = true, textColor = {64, 64, 64}, extent = {{-190, -96}, {-74, -70}}, textString = "torque"), Line(visible = true, points = {{-80, -110}, {-80, 0}}, color = {0, 0, 127})}));
end BasicCutTorque;
