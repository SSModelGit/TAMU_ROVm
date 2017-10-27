within UWBody.Visualizers;

model FixedArrow "Visualizing an arrow with dynamically varying size in frame_a"
  import UWBody.Types;
  extends UWBody.Interfaces.PartialVisualizer;
  parameter Boolean animation = true "= true, if animation shall be enabled";
  input SI.Position r_tail[3] = {0, 0, 0} "Vector from frame_a to arrow tail, resolved in frame_a" annotation(Dialog(group = "if animation = true", enable = animation));
  input Types.Axis n = {1, 0, 0} "Vector in arrow direction, resolved in frame_a" annotation(Dialog(group = "if animation = true", enable = animation));
  input SI.Length length = 0.1 "Length of complete arrow" annotation(Dialog(group = "if animation = true", enable = animation));
  input SI.Diameter diameter = world.defaultArrowDiameter "Diameter of arrow line" annotation(Dialog(group = "if animation = true", enable = animation));
  input Types.Color color = {0, 0, 255} "Color of arrow" annotation(Dialog(colorSelector = true, group = "if animation = true", enable = animation));
  input Types.SpecularCoefficient specularCoefficient = world.defaultSpecularCoefficient "Reflection of ambient light (= 0: light is completely absorbed)" annotation(Dialog(group = "if animation = true", enable = animation));
protected
  SI.Length headLength = min(length, diameter * Types.Defaults.ArrowHeadLengthFraction);
  SI.Length headWidth = diameter * Types.Defaults.ArrowHeadWidthFraction;
  SI.Length lineLength = max(0, length - headLength);
  Visualizers.Advanced.Shape arrowLine(shapeType = "cylinder", length = lineLength, width = diameter, height = diameter, lengthDirection = n, widthDirection = {0, 1, 0}, color = color, specularCoefficient = specularCoefficient, r_shape = r_tail, r = frame_a.r_0, R = frame_a.R) if world.enableAnimation and animation;
  Visualizers.Advanced.Shape arrowHead(shapeType = "cone", length = headLength, width = headWidth, height = headWidth, lengthDirection = n, widthDirection = {0, 1, 0}, color = color, specularCoefficient = specularCoefficient, r_shape = r_tail + Modelica.Math.Vectors.normalize(n) * lineLength, r = frame_a.r_0, R = frame_a.R) if world.enableAnimation and animation;
equation
  frame_a.f = zeros(3);
  frame_a.t = zeros(3);
  annotation(Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Rectangle(visible = true, origin = {0, 1.034}, lineColor = {128, 128, 128}, fillColor = {128, 128, 128}, fillPattern = FillPattern.Solid, extent = {{-100, -31.034}, {20, 28.966}}), Polygon(visible = true, lineColor = {128, 128, 128}, fillColor = {128, 128, 128}, fillPattern = FillPattern.Solid, points = {{20, 60}, {100, 0}, {20, -60}, {20, 60}}), Text(visible = true, textColor = {64, 64, 64}, extent = {{-150, 65}, {150, 105}}, textString = "%name"), Text(visible = true, textColor = {64, 64, 64}, extent = {{-150, -105}, {150, -75}}, textString = "%length")}), Documentation(info = "<html>
<p>
Model <b>FixedArrow</b> defines an arrow that is
shown at the location of its frame_a.
<br>&nbsp;
</p>

<p>
<IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Visualizers/Arrow.png\" ALT=\"model Visualizers.FixedArrow\">
</p>

<p>
The direction of the arrow specified with vector
<b>n</b> is with respect to frame_a, i.e., the local frame to which the
arrow component is attached. The direction and length of the arrow, its diameter
and color can vary dynamically by
providing appropriate expressions in the input fields of the
parameter menu.
</p>
</html>"));
end FixedArrow;
