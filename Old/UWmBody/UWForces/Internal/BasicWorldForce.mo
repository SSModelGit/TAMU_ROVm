within UWmBody.UWForces.Internal;

model BasicWorldForce "External force acting at frame_b, defined by 3 input signals"
  import UWmBody.UWTypes.ResolveInFrameB;
  extends UWInterfaces.PartialOneFrame_b;
  UWInterfaces.Frame_resolve frame_resolve "The input signals are optionally resolved in this frame" annotation(Placement(transformation(origin = {0, -100}, extent = {{-16, -16}, {16, 16}}, rotation = 270)));
  Modelica.Blocks.Interfaces.RealInput force[3](each final quantity = "Force", each final unit = "N") "x-, y-, z-coordinates of force resolved in frame defined by resolveInFrame" annotation(Placement(transformation(extent = {{-140, -20}, {-100, 20}})));
  parameter UWmBody.UWTypes.ResolveInFrameB resolveInFrame = UWmBody.UWTypes.ResolveInFrameB.world "Frame in which force is resolved (1: world, 2: frame_b, 3: frame_resolve)";
equation
  assert(cardinality(frame_resolve) > 0, "Connector frame_resolve must be connected at least once and frame_resolve.r_0/.R must be set");
  frame_resolve.f = zeros(3);
  frame_resolve.t = zeros(3);
  if resolveInFrame == ResolveInFrameB.world then
    frame_b.f = -UWFrames.resolve2(frame_b.R, force);
  elseif resolveInFrame == ResolveInFrameB.frame_b then
    frame_b.f = -force;
  elseif resolveInFrame == ResolveInFrameB.frame_resolve then
    frame_b.f = -UWFrames.resolveRelative(force, frame_resolve.R, frame_b.R);
  else
    assert(false, "Wrong value for parameter resolveInFrame");
    frame_b.f = zeros(3);
  end if;
  frame_b.t = zeros(3);
  annotation(Diagram(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {5, 5}), graphics = {Polygon(visible = true, lineColor = {64, 64, 64}, fillColor = {64, 64, 64}, fillPattern = FillPattern.Solid, points = {{-100, 10}, {40, 10}, {40, 25}, {100, 0}, {40, -25}, {40, -10}, {-100, -10}, {-100, 10}}), Line(visible = true, points = {{0, -10}, {0, -97}}, color = {95, 95, 95}, pattern = LinePattern.Dot)}), Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {5, 5}), graphics = {Text(visible = true, textColor = {192, 192, 192}, extent = {{-89, -76}, {91, -46}}, textString = "resolve"), Polygon(visible = true, lineColor = {64, 64, 64}, fillColor = {64, 64, 64}, fillPattern = FillPattern.Solid, points = {{-100, 10}, {40, 10}, {40, 25}, {94, 0}, {40, -25}, {40, -10}, {-100, -10}, {-100, 10}}), Text(visible = true, textColor = {64, 64, 64}, extent = {{-149, 42}, {136, 82}}, textString = "%name"), Line(visible = true, points = {{0, -10}, {0, -95}}, color = {95, 95, 95}, pattern = LinePattern.Dot)}), Documentation(info = "<html>
<p>
The 3 signals of the <b>force</b> connector are interpreted
as the x-, y- and z-coordinates of a <b>force</b> acting at the frame
connector to which this component is attached.
Via parameter <b>resolveInFrame</b> it is defined, in which frame these
coordinates shall be resolved:
</p>

<table border=1 cellspacing=0 cellpadding=2>
<tr><th><b>Types.ResolveInFrameB.</b></th><th><b>Meaning</b></th></tr>
<tr><td valign=\"top\">world</td>
    <td valign=\"top\">Resolve input force in world frame (= default)</td></tr>

<tr><td valign=\"top\">frame_b</td>
    <td valign=\"top\">Resolve input force in frame_b</td></tr>

<tr><td valign=\"top\">frame_resolve</td>
    <td valign=\"top\">Resolve input force in frame_resolve (frame_resolve must be connected)</td></tr>
</table>

<p>
If resolveInFrame = Types.ResolveInFrameB.frame_resolve, the force coordinates
are with respect to the frame, that is connected to <b>frame_resolve</b>.
</p>

<p>
If resolveInFrame is not Types.ResolveInFrameB.frame_resolve, then the position
vector and the orientation object of frame_resolve must be set to constant
values from the outside in order that the model remains balanced
(these constant values are ignored).
</p>

</html>"));
end BasicWorldForce;
