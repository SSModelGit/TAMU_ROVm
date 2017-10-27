within UWmBody.UWVisualizers;

model Ground "Visualizing the ground (box in z=0)"
  parameter Boolean animation = true "= true, if animation of ground shall be enabled";
  parameter Modelica.SIunits.Position length = 10 "Length and width of box (center is at x=y=0)" annotation(Dialog(enable = animation));
  parameter Modelica.SIunits.Position height = 0.02 "Height of box (upper surface is at z=0, lower surface is at z=-height)" annotation(Dialog(enable = animation));
  parameter UWmBody.UWTypes.Color groundColor = {0, 255, 0} "Color of box" annotation(Dialog(colorSelector = true, enable = animation));
  UWmBody.UWVisualizers.FixedShape ground(lengthDirection = {1, 0, 0}, widthDirection = {0, 1, 0}, animation = animation, r_shape = {-length / 2, 0, -height}, length = length, height = height, color = groundColor, width = length) annotation(Placement(transformation(extent = {{-20, 0}, {0, 20}})));
  UWmBody.UWParts.Fixed fixed annotation(Placement(transformation(extent = {{-60, 0}, {-40, 20}})));
equation
  connect(fixed.frame_b, ground.frame_a) annotation(Line(points = {{-40, 10}, {-20, 10}}, color = {95, 95, 95}, thickness = 0.5));
  annotation(Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Polygon(visible = true, lineColor = {255, 255, 255}, fillColor = {126, 181, 78}, fillPattern = FillPattern.Solid, points = {{-100, -30}, {20, -90}, {100, 0}, {-10, 40}}), Text(visible = true, textColor = {64, 64, 64}, extent = {{20, 70}, {60, 100}}, textString = "z", horizontalAlignment = TextAlignment.Left), Polygon(visible = true, lineColor = {255, 255, 255}, fillColor = {14, 111, 1}, fillPattern = FillPattern.Solid, points = {{100, -10}, {20, -100}, {20, -90}, {100, 0}}), Polygon(visible = true, lineColor = {255, 255, 255}, fillColor = {14, 111, 1}, fillPattern = FillPattern.Solid, points = {{-100, -40}, {20, -100}, {20, -90}, {-100, -30}}), Line(visible = true, origin = {6, -8}, points = {{-6, -10}, {-6, 108}}, color = {64, 64, 64}, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 20), Text(visible = true, textColor = {64, 64, 64}, extent = {{-150, -145}, {150, -105}}, textString = "%name")}), Documentation(info = "<html>
<p>
This shape visualizes the x-y plane by a box
</p>

<blockquote>
<img src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Visualizers/Ground.png\">
</blockquote>
</html>"));
end Ground;
