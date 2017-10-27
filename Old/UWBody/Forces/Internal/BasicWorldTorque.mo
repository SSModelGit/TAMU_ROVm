within UWBody.Forces.Internal;

model BasicWorldTorque "External torque acting at frame_b, defined by 3 input signals"
  import UWBody.Types.ResolveInFrameB;
  extends Interfaces.PartialOneFrame_b;
  Interfaces.Frame_resolve frame_resolve "The input signals are optionally resolved in this frame" annotation(Placement(transformation(origin = {0, 100}, extent = {{16, -16}, {-16, 16}}, rotation = 270)));
  Modelica.Blocks.Interfaces.RealInput torque[3](each final quantity = "Torque", each final unit = "N.m") "x-, y-, z-coordinates of torque resolved in frame defined by resolveInFrame" annotation(Placement(transformation(extent = {{-140, -20}, {-100, 20}})));
  parameter UWBody.Types.ResolveInFrameB resolveInFrame = UWBody.Types.ResolveInFrameB.world "Frame in which torque is resolved (1: world, 2: frame_b, 3: frame_resolve)";
equation
  assert(cardinality(frame_resolve) > 0, "Connector frame_resolve must be connected at least once and frame_resolve.r_0/.R must be set");
  frame_resolve.f = zeros(3);
  frame_resolve.t = zeros(3);
  if resolveInFrame == ResolveInFrameB.world then
    frame_b.t = -Frames.resolve2(frame_b.R, torque);
  elseif resolveInFrame == ResolveInFrameB.frame_b then
    frame_b.t = -torque;
  elseif resolveInFrame == ResolveInFrameB.frame_resolve then
    frame_b.t = -Frames.resolveRelative(torque, frame_resolve.R, frame_b.R);
  else
    assert(false, "Wrong value for parameter resolveInFrame");
    frame_b.t = zeros(3);
  end if;
  frame_b.f = zeros(3);
  annotation(Diagram(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Line(visible = true, points = {{0, 97}, {0, 82}}, color = {95, 95, 95}, pattern = LinePattern.Dot), Line(visible = true, origin = {-4.674, 43.38}, points = {{-95.326, -43.38}, {-55.326, 16.62}, {4.674, 36.62}, {54.674, 16.62}, {101.304, -36.478}}, color = {64, 64, 64}, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 30, smooth = Smooth.Bezier)}), Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Text(visible = true, origin = {8.333, 0}, textColor = {192, 192, 192}, extent = {{-108.333, 50}, {21.667, 80}}, textString = "resolve"), Line(visible = true, points = {{0, 95}, {0, 82}}, color = {95, 95, 95}, pattern = LinePattern.Dot), Ellipse(visible = true, origin = {0, -100}, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, extent = {{-140, -140}, {140, 140}}, startAngle = 60, endAngle = 120), Polygon(visible = true, origin = {0, -100}, fillColor = {128, 128, 128}, fillPattern = FillPattern.Solid, points = {{-12.5, 8.75}, {0, 13.75}, {12.5, 8.75}, {12.5, -8.75}, {0, -13.75}, {-12.5, -8.75}}), Polygon(visible = true, rotation = 60, lineColor = {64, 64, 64}, fillColor = {64, 64, 64}, fillPattern = FillPattern.Solid, points = {{53.392, -68.478}, {43.392, -38.478}, {63.392, -38.478}}), Polygon(visible = true, origin = {0, -100}, fillColor = {191, 191, 191}, fillPattern = FillPattern.Solid, points = {{-12.549, -11.229}, {-12.549, 8.771}, {-12.549, 8.771}, {-0.049, 13.771}, {-0.049, 13.771}, {12.451, 8.771}, {12.451, 8.771}, {12.451, -11.229}, {12.451, -11.229}, {14.951, -11.229}, {14.951, -11.229}, {24.661, 9.043}, {17.206, 23.399}, {15.55, 34.167}, {29.265, 88.818}, {44.409, 152.161}, {45.781, 156.611}, {54.892, 162.823}, {60, 175.661}, {55.997, 188.913}, {44.677, 196.23}, {29.63, 194.021}, {21.064, 184.033}, {20.105, 172.348}, {24.799, 162.409}, {23.297, 157.033}, {6.853, 90.448}, {-6.675, 35.271}, {-9.574, 29.059}, {-22.964, 19.948}, {-24.207, 3.521}, {-15.049, -11.229}, {-15.049, -11.229}, {-12.549, -11.229}}, smooth = Smooth.Bezier), Ellipse(visible = true, origin = {0, -100}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{27.5, 164.188}, {52.5, 189.188}}), Polygon(visible = true, origin = {17.312, -12.455}, lineColor = {128, 128, 128}, fillColor = {255, 255, 255}, pattern = LinePattern.None, fillPattern = FillPattern.Sphere, points = {{-27.312, -59.815}, {-21.575, -57.545}, {-14.064, -57.545}, {-5.944, -60.627}, {0.146, -66.92}, {0.146, -66.92}, {-3.37, -60.294}, {-4.06, -53.116}, {24.507, 64.424}, {26.537, 69.702}, {26.537, 69.702}, {21.868, 69.702}, {16.923, 70.433}, {13.057, 71.813}, {9.078, 74.371}, {9.078, 74.371}, {9.078, 70.108}, {-19.935, -44.005}, {-22.558, -55.739}, {-27.312, -59.815}}, smooth = Smooth.Bezier), Text(visible = true, textColor = {64, 64, 64}, extent = {{-142.5, -160}, {142.5, -120}}, textString = "%name")}), Documentation(info = "<html>
<p>
The 3 signals of the <b>torque</b> connector are interpreted
as the x-, y- and z-coordinates of a <b>torque</b> acting at the frame
connector to which this component is attached.
Via parameter <b>resolveInFrame</b> it is defined, in which frame these
coordinates shall be resolved:
</p>

<table border=1 cellspacing=0 cellpadding=2>
<tr><th><b>Types.ResolveInFrameB.</b></th><th><b>Meaning</b></th></tr>
<tr><td valign=\"top\">world</td>
    <td valign=\"top\">Resolve input torque in world frame (= default)</td></tr>

<tr><td valign=\"top\">frame_b</td>
    <td valign=\"top\">Resolve input torque in frame_b</td></tr>

<tr><td valign=\"top\">frame_resolve</td>
    <td valign=\"top\">Resolve input torque in frame_resolve (frame_resolve must be connected)</td></tr>
</table>

<p>
If resolveInFrame = Types.ResolveInFrameB.frame_resolve, the torque coordinates
are with respect to the frame, that is connected to <b>frame_resolve</b>.
</p>

<p>
If resolveInFrame is not Types.ResolveInFrameB.frame_resolve, then the position
vector and the orientation object of frame_resolve must be set to constant
values from the outside in order that the model remains balanced
(these constant values are ignored).
</p>

</html>"));
end BasicWorldTorque;
