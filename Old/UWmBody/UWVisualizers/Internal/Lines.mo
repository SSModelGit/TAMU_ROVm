within UWmBody.UWVisualizers.Internal;

model Lines "Visualizing a set of lines as cylinders with variable size, e.g., used to display characters (no Frame connector)"
  import Modelica.Mechanics.MultiBody;
  import Types = UWmBody.UWTypes;
  import Frames = UWmBody.UWFrames;
  import T = UWmBody.UWFrames.TransformationMatrices;
  input UWmBody.UWFrames.Orientation R = Frames.nullRotation() "Orientation object to rotate the world frame into the object frame" annotation(Dialog);
  input SI.Position r[3] = {0, 0, 0} "Position vector from origin of world frame to origin of object frame, resolved in world frame" annotation(Dialog);
  input SI.Position r_lines[3] = {0, 0, 0} "Position vector from origin of object frame to the origin of 'lines' frame, resolved in object frame" annotation(Dialog);
  input Real n_x[3](each final unit = "1") = {1, 0, 0} "Vector in direction of x-axis of 'lines' frame, resolved in object frame" annotation(Dialog);
  input Real n_y[3](each final unit = "1") = {0, 1, 0} "Vector in direction of y-axis of 'lines' frame, resolved in object frame" annotation(Dialog);
  input SI.Position lines[:, 2, 2] = zeros(0, 2, 2) "List of start and end points of cylinders resolved in an x-y frame defined by n_x, n_y, e.g., {[0,0;1,1], [0,1;1,0], [2,0; 3,1]}" annotation(Dialog);
  input SI.Length diameter(min = 0) = 0.05 "Diameter of the cylinders defined by lines" annotation(Dialog);
  input UWmBody.UWTypes.Color color = {0, 128, 255} "Color of cylinders" annotation(Dialog(colorSelector = true));
  input UWTypes.SpecularCoefficient specularCoefficient = 0.7 "Reflection of ambient light (= 0: light is completely absorbed)" annotation(Dialog);
protected
  parameter Integer n = size(lines, 1) "Number of cylinders";
  T.Orientation R_rel = T.from_nxy(n_x, n_y);
  T.Orientation R_lines = T.absoluteRotation(R.T, R_rel);
  Modelica.SIunits.Position r_abs[3] = r + T.resolve1(R.T, r_lines);
  UWmBody.UWVisualizers.Advanced.Shape cylinders[n](each shapeType = "cylinder", lengthDirection = array(T.resolve1(R_rel, vector([lines[i, 2, :] - lines[i, 1, :]; 0])) for i in 1:n), length = array(Modelica.Math.Vectors.length(lines[i, 2, :] - lines[i, 1, :]) for i in 1:n), r = array(r_abs + T.resolve1(R_lines, vector([lines[i, 1, :]; 0])) for i in 1:n), each width = diameter, each height = diameter, each widthDirection = {0, 1, 0}, each color = color, each R = R, each specularCoefficient = specularCoefficient);
  annotation(Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Rectangle(visible = true, lineColor = {128, 128, 128}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, -100}, {100, 100}}), Polygon(visible = true, lineColor = {0, 127, 255}, fillColor = {0, 127, 255}, fillPattern = FillPattern.Solid, points = {{-24, -34}, {-82, 40}, {-72, 46}, {-14, -26}, {-24, -34}}), Polygon(visible = true, lineColor = {0, 127, 255}, fillColor = {0, 127, 255}, fillPattern = FillPattern.Solid, points = {{-82, -24}, {-20, 46}, {-10, 38}, {-72, -32}, {-82, -24}}), Polygon(visible = true, lineColor = {0, 127, 255}, fillColor = {0, 127, 255}, fillPattern = FillPattern.Solid, points = {{42, -18}, {10, 40}, {20, 48}, {50, -6}, {42, -18}}), Polygon(visible = true, lineColor = {0, 127, 255}, fillColor = {0, 127, 255}, fillPattern = FillPattern.Solid, points = {{10, -68}, {84, 48}, {96, 42}, {24, -72}, {10, -68}}), Text(visible = true, textColor = {64, 64, 64}, extent = {{-150, 105}, {150, 145}}, textString = "%name")}), Documentation(info = "<html>
<p>
With model <b>Lines</b> a set of dynamic lines is defined
that are located relatively to frame_a. Every line
is represented by a cylinder. This allows, e.g., to define simple shaped
3-dimensional characters. Note, if the lines are fixed relatively to frame_a,
it is more convenient to use model <b>Visualizers.FixedLines</b>.
An example for dynamic lines is shown in the following figure:<br>&nbsp;
</p>
<IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/FixedLines.png\" ALT=\"model Visualizers.FixedLines\">
<p>&nbsp;<br>
The two letters \"x\" and \"y\" are constructed with 4 lines
by providing the following data for input variable <b>lines</b>
</p>
<pre>
   lines = {[0, 0; 1, 1],[0, 1; 1, 0],[1.5, -0.5; 2.5, 1],[1.5, 1; 2, 0.25]}
</pre>
<p>
Via vectors <b>n_x</b> and <b>n_y</b> a two-dimensional
coordinate system is defined. The points defined with variable
<b>lines</b> are with respect to this coordinate system. For example
\"[0, 0; 1, 1]\" defines a line that starts at {0,0} and ends at {1,1}.
The diameter and color of all line cylinders are identical
and are defined by parameters.
</p>

</html>"));
end Lines;
