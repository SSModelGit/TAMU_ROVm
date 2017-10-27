within UWBody.Sensors.Internal;

model BasicCutForce "Measure cut force vector (frame_resolve must be connected)"
  import UWBody.Types.ResolveInFrameA;
  import UWBody.Frames;
  extends UWBody.Sensors.Internal.PartialCutForceBaseSensor;
  Modelica.Blocks.Interfaces.RealOutput force[3](each final quantity = "Force", each final unit = "N") "Cut force resolved in frame defined by resolveInFrame" annotation(Placement(transformation(origin = {-80, -110}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
  parameter Boolean positiveSign = true "= true, if force with positive sign is returned (= frame_a.f), otherwise with negative sign (= frame_b.f)";
protected
  parameter Integer csign = if positiveSign then +1 else -1;
equation
  if resolveInFrame == ResolveInFrameA.world then
    force = Frames.resolve1(frame_a.R, frame_a.f) * csign;
  elseif resolveInFrame == ResolveInFrameA.frame_a then
    force = frame_a.f * csign;
  elseif resolveInFrame == ResolveInFrameA.frame_resolve then
    force = Frames.resolveRelative(frame_a.f, frame_a.R, frame_resolve.R) * csign;
  else
    assert(false, "Wrong value for parameter resolveInFrame");
    force = zeros(3);
  end if;
  annotation(Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Text(visible = true, textColor = {64, 64, 64}, extent = {{-190, -96}, {-74, -70}}, textString = "force"), Line(visible = true, points = {{-80, -110}, {-80, 0}}, color = {0, 0, 127})}), Documentation(info = "<html>

</html>"));
end BasicCutForce;
