within UWBody.Forces.Internal;

model BasicTorque "Torque acting between two frames, defined by 3 input signals"
  import UWBody.Types.ResolveInFrameAB;
  extends UWBody.Interfaces.PartialTwoFrames;
  Interfaces.Frame_resolve frame_resolve "The input signals are optionally resolved in this frame" annotation(Placement(transformation(origin = {40, 100}, extent = {{-16, -16}, {16, 16}}, rotation = 90)));
  Modelica.Blocks.Interfaces.RealInput torque[3](each final quantity = "Torque", each final unit = "N.m") "x-, y-, z-coordinates of torque resolved in frame defined by resolveInFrame" annotation(Placement(transformation(origin = {-60, 120}, extent = {{-20, -20}, {20, 20}}, rotation = 270)));
  parameter UWBody.Types.ResolveInFrameAB resolveInFrame = UWBody.Types.ResolveInFrameAB.frame_b "Frame in which torque is resolved (1: world, 2: frame_a, 3: frame_b, 4: frame_resolve)";
  SI.Position r_0[3] "Position vector from origin of frame_a to origin of frame_b resolved in world frame";
  SI.Torque t_b_0[3] "frame_b.t resolved in world frame";
equation
  assert(cardinality(frame_resolve) > 0, "Connector frame_resolve must be connected at least once and frame_resolve.r_0/.R must be set");
  frame_resolve.f = zeros(3);
  frame_resolve.t = zeros(3);
  r_0 = frame_b.r_0 - frame_a.r_0;
  frame_a.f = zeros(3);
  frame_b.f = zeros(3);
  if resolveInFrame == ResolveInFrameAB.frame_a then
    t_b_0 = -Frames.resolve1(frame_a.R, torque);
    frame_b.t = Frames.resolve2(frame_b.R, t_b_0);
  elseif resolveInFrame == ResolveInFrameAB.frame_b then
    t_b_0 = -Frames.resolve1(frame_b.R, torque);
    frame_b.t = -torque;
  elseif resolveInFrame == ResolveInFrameAB.world then
    t_b_0 = -torque;
    frame_b.t = Frames.resolve2(frame_b.R, t_b_0);
  elseif resolveInFrame == ResolveInFrameAB.frame_resolve then
    t_b_0 = -Frames.resolve1(frame_resolve.R, torque);
    frame_b.t = Frames.resolve2(frame_b.R, t_b_0);
  else
    assert(false, "Wrong value for parameter resolveInFrame");
    t_b_0 = zeros(3);
    frame_b.t = zeros(3);
  end if;
  // torque balance
  zeros(3) = frame_a.t + Frames.resolve2(frame_a.R, t_b_0);
  annotation(Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Rectangle(visible = true, origin = {-0.5, -0.5}, lineColor = {255, 255, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-98, -98}, {99, 99}}), Line(visible = true, points = {{40, 100}, {80, 42.513}}, color = {95, 95, 95}, pattern = LinePattern.Dot), Line(visible = true, points = {{-60, 100}, {40, 100}}, color = {95, 95, 95}, pattern = LinePattern.Dot), Ellipse(visible = true, origin = {0, -100}, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, extent = {{-140, -140}, {140, 140}}, startAngle = 60, endAngle = 120), Polygon(visible = true, origin = {0, -100}, fillColor = {128, 128, 128}, fillPattern = FillPattern.Solid, points = {{-12.5, 8.75}, {0, 13.75}, {12.5, 8.75}, {12.5, -8.75}, {0, -13.75}, {-12.5, -8.75}}), Polygon(visible = true, rotation = 60, lineColor = {64, 64, 64}, fillColor = {64, 64, 64}, fillPattern = FillPattern.Solid, points = {{53.392, -68.478}, {43.392, -38.478}, {63.392, -38.478}}), Polygon(visible = true, origin = {0, -100}, fillColor = {191, 191, 191}, fillPattern = FillPattern.Solid, points = {{-12.549, -11.229}, {-12.549, 8.771}, {-12.549, 8.771}, {-0.049, 13.771}, {-0.049, 13.771}, {12.451, 8.771}, {12.451, 8.771}, {12.451, -11.229}, {12.451, -11.229}, {14.951, -11.229}, {14.951, -11.229}, {24.661, 9.043}, {17.206, 23.399}, {15.55, 34.167}, {29.265, 88.818}, {44.409, 152.161}, {45.781, 156.611}, {54.892, 162.823}, {60, 175.661}, {55.997, 188.913}, {44.677, 196.23}, {29.63, 194.021}, {21.064, 184.033}, {20.105, 172.348}, {24.799, 162.409}, {23.297, 157.033}, {6.853, 90.448}, {-6.675, 35.271}, {-9.574, 29.059}, {-22.964, 19.948}, {-24.207, 3.521}, {-15.049, -11.229}, {-15.049, -11.229}, {-12.549, -11.229}}, smooth = Smooth.Bezier), Ellipse(visible = true, origin = {0, -100}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{27.5, 164.188}, {52.5, 189.188}}), Polygon(visible = true, origin = {17.312, -12.455}, lineColor = {128, 128, 128}, fillColor = {255, 255, 255}, pattern = LinePattern.None, fillPattern = FillPattern.Sphere, points = {{-27.312, -59.815}, {-21.575, -57.545}, {-14.064, -57.545}, {-5.944, -60.627}, {0.146, -66.92}, {0.146, -66.92}, {-3.37, -60.294}, {-4.06, -53.116}, {24.507, 64.424}, {26.537, 69.702}, {26.537, 69.702}, {21.868, 69.702}, {16.923, 70.433}, {13.057, 71.813}, {9.078, 74.371}, {9.078, 74.371}, {9.078, 70.108}, {-19.935, -44.005}, {-22.558, -55.739}, {-27.312, -59.815}}, smooth = Smooth.Bezier), Text(visible = true, origin = {0, 5}, textColor = {64, 64, 64}, extent = {{-142.5, -165}, {142.5, -125}}, textString = "%name"), Text(visible = true, textColor = {192, 192, 192}, extent = {{-100, 50}, {30, 80}}, textString = "resolve"), Polygon(visible = true, rotation = -60, lineColor = {64, 64, 64}, fillColor = {64, 64, 64}, fillPattern = FillPattern.Solid, points = {{-53.392, -68.478}, {-63.392, -38.478}, {-43.392, -38.478}})}), Documentation(info = "<html>
<p>
The <b>3</b> signals of the <b>torque</b> connector are interpreted
as the x-, y- and z-coordinates of a <b>torque</b> acting at the frame
connector to which frame_b of this component is attached.
Via parameter <b>resolveInFrame</b> it is defined, in which frame these
coordinates shall be resolved:
</p>

<table border=1 cellspacing=0 cellpadding=2>
<tr><th><b>Types.ResolveInFrameAB.</b></th><th><b>Meaning</b></th></tr>
<tr><td valign=\"top\">world</td>
    <td valign=\"top\">Resolve input torque in world frame</td></tr>

<tr><td valign=\"top\">frame_a</td>
    <td valign=\"top\">Resolve input torque in frame_a</td></tr>

<tr><td valign=\"top\">frame_b</td>
    <td valign=\"top\">Resolve input torque in frame_b (= default)</td></tr>

<tr><td valign=\"top\">frame_resolve</td>
    <td valign=\"top\">Resolve input torque in frame_resolve (frame_resolve must be connected)</td></tr>
</table>

<p>
If resolveInFrame = ResolveInFrameAB.frame_resolve, the torque coordinates
are with respect to the frame, that is connected to <b>frame_resolve</b>.
</p>

<p>
If resolveInFrame is not ResolveInFrameAB.frame_resolve, then the position
vector and the orientation object of frame_resolve must be set to constant
values from the outside in order that the model remains balanced
(these constant values are ignored).
</p>
</html>"));
end BasicTorque;
