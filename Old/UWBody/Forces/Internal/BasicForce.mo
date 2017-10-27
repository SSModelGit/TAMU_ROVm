within UWBody.Forces.Internal;

model BasicForce "Force acting between two frames, defined by 3 input signals"
  extends UWBody.Interfaces.PartialTwoFrames;
  import UWBody.Types.ResolveInFrameAB;
  Interfaces.Frame_resolve frame_resolve "The input signals are optionally resolved in this frame" annotation(Placement(transformation(origin = {40, 100}, extent = {{-16, -16}, {16, 16}}, rotation = 90)));
  Modelica.Blocks.Interfaces.RealInput force[3](each final quantity = "Force", each final unit = "N") "x-, y-, z-coordinates of force resolved in frame defined by resolveInFrame" annotation(Placement(transformation(origin = {-60, 120}, extent = {{-20, -20}, {20, 20}}, rotation = 270)));
  parameter UWBody.Types.ResolveInFrameAB resolveInFrame = UWBody.Types.ResolveInFrameAB.frame_b "Frame in which force is resolved (1: world, 2: frame_a, 3: frame_b, 4: frame_resolve)";
  Modelica.SIunits.Position r_0[3] "Position vector from origin of frame_a to origin of frame_b resolved in world frame";
  Modelica.SIunits.Force f_b_0[3] "frame_b.f resolved in world frame";
equation
  assert(cardinality(frame_resolve) > 0, "Connector frame_resolve must be connected at least once and frame_resolve.r_0/.R must be set");
  frame_resolve.f = zeros(3);
  frame_resolve.t = zeros(3);
  if resolveInFrame == ResolveInFrameAB.frame_a then
    f_b_0 = -Frames.resolve1(frame_a.R, force);
    frame_b.f = Frames.resolve2(frame_b.R, f_b_0);
  elseif resolveInFrame == ResolveInFrameAB.frame_b then
    f_b_0 = -Frames.resolve1(frame_b.R, force);
    frame_b.f = -force;
  elseif resolveInFrame == ResolveInFrameAB.world then
    f_b_0 = -force;
    frame_b.f = Frames.resolve2(frame_b.R, f_b_0);
  elseif resolveInFrame == ResolveInFrameAB.frame_resolve then
    f_b_0 = -Frames.resolve1(frame_resolve.R, force);
    frame_b.f = Frames.resolve2(frame_b.R, f_b_0);
  else
    assert(false, "Wrong value for parameter resolveInFrame");
    f_b_0 = zeros(3);
    frame_b.f = zeros(3);
  end if;
  frame_b.t = zeros(3);
  // Force and torque balance
  r_0 = frame_b.r_0 - frame_a.r_0;
  zeros(3) = frame_a.f + Frames.resolve2(frame_a.R, f_b_0);
  zeros(3) = frame_a.t + Frames.resolve2(frame_a.R, cross(r_0, f_b_0));
  annotation(Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Rectangle(visible = true, origin = {-0.5, -0.5}, lineColor = {255, 255, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-98, -98}, {99, 99}}), Text(visible = true, textColor = {192, 192, 192}, extent = {{-92, 35}, {87, 61}}, textString = "resolve"), Text(visible = true, textColor = {64, 64, 64}, extent = {{-140, -92}, {145, -52}}, textString = "%name"), Line(visible = true, points = {{40, 100}, {40, 0}}, color = {95, 95, 95}, pattern = LinePattern.Dot), Line(visible = true, points = {{-60, 100}, {40, 100}}, color = {95, 95, 95}, pattern = LinePattern.Dot), Line(visible = true, points = {{20, 0}, {75, 0}}, color = {64, 64, 64}, thickness = 10, arrowSize = 30), Line(visible = true, points = {{-70, 0}, {-20, 0}}, color = {64, 64, 64}, thickness = 10, arrowSize = 30), Line(visible = true, points = {{20, 0}, {95, 0}}, color = {64, 64, 64}, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 40), Line(visible = true, points = {{-95, 0}, {-20, 0}}, color = {64, 64, 64}, arrow = {Arrow.Filled, Arrow.None}, arrowSize = 40)}), Documentation(info = "<html>
<p>
The <b>3</b> signals of the <b>force</b> connector are interpreted
as the x-, y- and z-coordinates of a <b>force</b> acting at the frame
connector to which frame_b of this component is attached.
Via parameter <b>resolveInFrame</b> it is defined, in which frame these
coordinates shall be resolved:
</p>

<table border=1 cellspacing=0 cellpadding=2>
<tr><th><b>Types.ResolveInFrameAB.</b></th><th><b>Meaning</b></th></tr>
<tr><td valign=\"top\">world</td>
    <td valign=\"top\">Resolve input force in world frame</td></tr>

<tr><td valign=\"top\">frame_a</td>
    <td valign=\"top\">Resolve input force in frame_a</td></tr>

<tr><td valign=\"top\">frame_b</td>
    <td valign=\"top\">Resolve input force in frame_b (= default)</td></tr>

<tr><td valign=\"top\">frame_resolve</td>
    <td valign=\"top\">Resolve input force in frame_resolve (frame_resolve must be connected)</td></tr>
</table>

<p>
If resolveInFrame = ResolveInFrameAB.frame_resolve, the force coordinates
are with respect to the frame, that is connected to <b>frame_resolve</b>.
</p>

<p>
If resolveInFrame is not ResolveInFrameAB.frame_resolve, then the position
vector and the orientation object of frame_resolve must be set to constant
values from the outside in order that the model remains balanced
(these constant values are ignored).
</p>

</html>"));
end BasicForce;
