within UWBody.Visualizers.Internal;

model FixedLines "Visualizing a set of lines as cylinders (e.g., used to display characters)"
  import UWBody;
  import UWBody.Types;
  extends UWBody.Interfaces.PartialVisualizer;
  parameter Boolean animation = true "= true, if animation shall be enabled";
  input Real scale(min = 0) = 1 "The 'lines' are visualized 'scale' times bigger" annotation(Dialog(group = "if animation = true", enable = animation));
  input SI.Position lines[:, 2, 2] = {[0, 0; 1, 1], [0, 1; 1, 0]} "List of start and end points of cylinders resolved along n_x and n_y" annotation(Dialog(group = "if animation = true", enable = animation));
  input SI.Distance diameter(min = 0) = 0.05 "Diameter of the cylinders defined by lines" annotation(Dialog(group = "if animation = true", enable = animation));
  input SI.Position r_lines[3] = {0, 0, 0} "Position vector from origin of frame_a to the origin of the 'lines' frame, resolved in frame_a" annotation(Dialog(group = "if animation = true", enable = animation));
  input Real n_x[3](each final unit = "1") = {1, 0, 0} "Vector in direction of x-axis of 'lines' frame, resolved in frame_a." annotation(Dialog(group = "if animation = true", enable = animation));
  input Real n_y[3](each final unit = "1") = {0, 1, 0} "Vector in direction of y-axis of 'lines' frame, resolved in frame_a." annotation(Dialog(group = "if animation = true", enable = animation));
  input UWBody.Types.Color color = {0, 128, 255} "Color of cylinders" annotation(Dialog(colorSelector = true, group = "if animation = true", enable = animation));
  input Types.SpecularCoefficient specularCoefficient = world.defaultSpecularCoefficient "Reflection of ambient light (= 0: light is completely absorbed)" annotation(Dialog(group = "if animation = true", enable = animation));
protected
  Lines x_label(lines = scale * lines, diameter = scale * diameter, color = color, specularCoefficient = specularCoefficient, r_lines = r_lines, n_x = n_x, n_y = n_y, r = frame_a.r_0, R = frame_a.R) if world.enableAnimation and animation;
equation
  frame_a.f = zeros(3);
  frame_a.t = zeros(3);
  annotation(Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Rectangle(visible = true, lineColor = {128, 128, 128}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, -100}, {100, 100}}), Text(visible = true, textColor = {64, 64, 64}, extent = {{-150, 105}, {150, 145}}, textString = "%name"), Polygon(visible = true, lineColor = {0, 127, 255}, fillColor = {0, 127, 255}, fillPattern = FillPattern.Solid, points = {{-82, -24}, {-20, 46}, {-10, 38}, {-72, -32}, {-82, -24}}), Polygon(visible = true, lineColor = {0, 127, 255}, fillColor = {0, 127, 255}, fillPattern = FillPattern.Solid, points = {{-24, -34}, {-82, 40}, {-72, 46}, {-14, -26}, {-24, -34}}), Polygon(visible = true, lineColor = {0, 127, 255}, fillColor = {0, 127, 255}, fillPattern = FillPattern.Solid, points = {{42, -18}, {10, 40}, {20, 48}, {50, -6}, {42, -18}}), Polygon(visible = true, lineColor = {0, 127, 255}, fillColor = {0, 127, 255}, fillPattern = FillPattern.Solid, points = {{10, -68}, {84, 48}, {96, 42}, {24, -72}, {10, -68}})}), Documentation(info = "<html>
<p>
With model <b>FixedLines</b> a set of lines is defined
that are located relatively to frame_a. Every line
is represented by a cylinder. This allows to define simple shaped
3-dimensional characters. An example is shown in the
following figure:<br>&nbsp;
</p>
<IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/FixedLines.png\" ALT=\"model Visualizers.FixedLines\">
<p>&nbsp;<br>
The two letters \"x\" and \"y\" are constructed with 4 lines
by providing the following data for parameter <b>lines</b>
</p>
<pre>
   lines = {[0, 0; 1, 1],[0, 1; 1, 0],[1.5, -0.5; 2.5, 1],[1.5, 1; 2, 0.25]}
</pre>
<p>
Via parameter vectors <b>n_x</b> and <b>n_y</b> a two-dimensional
coordinate system is defined. The points defined with parameter
<b>lines</b> are with respect to this coordinate system. For example
\"[0, 0; 1, 1]\" defines a line that starts at {0,0} and ends at {1,1}.
The diameter and color of all line cylinders are identical.
</p>
</html>"));
end FixedLines;
