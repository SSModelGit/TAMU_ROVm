within UWmBody.UWVisualizers;

model SignalArrow "Visualizing an arrow with dynamically varying size in frame_a based on input signal"
  import UWmBody.UWTypes;
  extends UWmBody.UWInterfaces.PartialVisualizer;
  parameter Boolean animation = true "= true, if animation shall be enabled";
  input SI.Position r_tail[3] = {0, 0, 0} "Vector from frame_a to arrow tail, resolved in frame_a" annotation(Dialog(group = "if animation = true", enable = animation));
  input SI.Diameter diameter = world.defaultArrowDiameter "Diameter of arrow line" annotation(Dialog(group = "if animation = true", enable = animation));
  input UWmBody.UWTypes.Color color = {0, 0, 255} "Color of arrow" annotation(Dialog(colorSelector = true, group = "if animation = true", enable = animation));
  input UWTypes.SpecularCoefficient specularCoefficient = world.defaultSpecularCoefficient "Reflection of ambient light (= 0: light is completely absorbed)" annotation(Dialog(group = "if animation = true", enable = animation));
  Modelica.Blocks.Interfaces.RealInput r_head[3](each final quantity = "Length", each final unit = "m") "Position vector from origin of frame_a to head of arrow, resolved in frame_a" annotation(Placement(transformation(origin = {0, -120}, extent = {{-20, -20}, {20, 20}}, rotation = 90), visible = true, iconTransformation(origin = {-0, -0}, extent = {{-140, -20}, {-100, 20}}, rotation = 90)));
protected
  Visualizers.Advanced.Arrow arrow(R = frame_a.R, r = frame_a.r_0, r_tail = r_tail, r_head = r_head, diameter = diameter, color = color, specularCoefficient = specularCoefficient) if world.enableAnimation and animation;
equation
  frame_a.f = zeros(3);
  frame_a.t = zeros(3);
  annotation(Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Rectangle(visible = true, origin = {0, 1.034}, lineColor = {128, 128, 128}, fillColor = {128, 128, 128}, fillPattern = FillPattern.Solid, extent = {{-100, -31.034}, {20, 28.966}}), Polygon(visible = true, lineColor = {128, 128, 128}, fillColor = {128, 128, 128}, fillPattern = FillPattern.Solid, points = {{20, 60}, {100, 0}, {20, -60}, {20, 60}}), Text(visible = true, textColor = {64, 64, 64}, extent = {{-150, 65}, {150, 105}}, textString = "%name"), Line(visible = true, points = {{0, -120}, {0, -30}}, color = {0, 36, 164})}), Documentation(info = "<html>
<p>
Model <b>SignalArrow</b> defines an arrow that is dynamically visualized
at the location where its frame_a is attached. The
position vector from the tail to the head of the arrow,
resolved in frame_a, is defined via the signal vector of
the connector <b>r_head</b> (Real r_head[3]):<br>&nbsp;
</p>

<p>
<IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Visualizers/Arrow.png\" ALT=\"model Visualizers.SignalArrow\">
</p>
<p>
The tail of the arrow is defined with parameter <b>r_tail</b>
with respect to frame_a (vector from the origin of frame_a to the arrow tail).
</p>
</html>"));
end SignalArrow;
