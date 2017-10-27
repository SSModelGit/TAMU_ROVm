within UWBody.Visualizers.Advanced;

model Arrow "Visualizing an arrow with variable size; all data have to be set as modifiers (see info layer)"
  import UWBody.Types;
  import UWBody.Frames;
  import T = UWBody.Frames.TransformationMatrices;
  import Modelica.SIunits.Conversions.to_unit1;
  input Frames.Orientation R = Frames.nullRotation() "Orientation object to rotate the world frame into the arrow frame" annotation(Dialog);
  input SI.Position r[3] = {0, 0, 0} "Position vector from origin of world frame to origin of arrow frame, resolved in world frame" annotation(Dialog);
  input SI.Position r_tail[3] = {0, 0, 0} "Position vector from origin of arrow frame to arrow tail, resolved in arrow frame" annotation(Dialog);
  input SI.Position r_head[3] = {0, 0, 0} "Position vector from arrow tail to the head of the arrow, resolved in arrow frame" annotation(Dialog);
  input SI.Diameter diameter = world.defaultArrowDiameter "Diameter of arrow line" annotation(Dialog);
  input UWBody.Types.Color color = UWBody.Types.Defaults.ArrowColor "Color of arrow" annotation(Dialog(colorSelector = true));
  input Types.SpecularCoefficient specularCoefficient = world.defaultSpecularCoefficient "Material property describing the reflecting of ambient light (= 0 means, that light is completely absorbed)" annotation(Dialog);
protected
  outer UWBody.World world;
  SI.Length length = Modelica.Math.Vectors.length(r_head) "Length of arrow";
  Real e_x[3](each final unit = "1", start = {1, 0, 0}) = noEvent(if length < 1.e-10 then {1, 0, 0} else r_head / length);
  Real rxvisobj[3](each final unit = "1") = transpose(R.T) * e_x "X-axis unit vector of shape, resolved in world frame" annotation(HideResult = true);
  SI.Position rvisobj[3] = r + T.resolve1(R.T, r_tail) "Position vector from world frame to shape frame, resolved in world frame" annotation(HideResult = true);
  SI.Length arrowLength = noEvent(max(0, length - diameter * Types.Defaults.ArrowHeadLengthFraction)) annotation(HideResult = true);
  Visualizers.Advanced.Shape arrowLine(length = arrowLength, width = diameter, height = diameter, lengthDirection = to_unit1(r_head), widthDirection = {0, 1, 0}, shapeType = "cylinder", color = color, specularCoefficient = specularCoefficient, r_shape = r_tail, r = r, R = R) if world.enableAnimation;
  Visualizers.Advanced.Shape arrowHead(length = noEvent(max(0, min(length, diameter * Types.Defaults.ArrowHeadLengthFraction))), width = noEvent(max(0, diameter * UWBody.Types.Defaults.ArrowHeadWidthFraction)), height = noEvent(max(0, diameter * UWBody.Types.Defaults.ArrowHeadWidthFraction)), lengthDirection = to_unit1(r_head), widthDirection = {0, 1, 0}, shapeType = "cone", color = color, specularCoefficient = specularCoefficient, r = rvisobj + rxvisobj * arrowLength, R = R) if world.enableAnimation;
  annotation(Documentation(info = "<html>
<p>
Model <b>Arrow</b> defines an arrow that is dynamically
visualized at the defined location (see variables below).
</p>

<p>
<IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Visualizers/Arrow.png\" ALT=\"model Visualizers.Advanced.Arrow\">
</p>

<p>
The variables under heading <b>Parameters</b> below
are declared as (time varying) <b>input</b> variables.
If the default equation is not appropriate, a corresponding
modifier equation has to be provided in the
model where an <b>Arrow</b> instance is used, e.g., in the form
</p>
<pre>
    Visualizers.Advanced.Arrow arrow(diameter = sin(time));
</pre>

<p>
Variable <b>color</b> is an Integer vector with 3 elements,
{r, g, b}, and specifies the color of the shape.
{r,g,b} are the \"red\", \"green\" and \"blue\" color parts.
Note, r g, b are given in the range 0 .. 255.
The predefined type <b>MultiBody.Types.Color</b> contains
a menu definition of the colors used in the MultiBody
library (will be replaced by a color editor).
</p>
</html>"), Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Rectangle(visible = true, lineColor = {128, 128, 128}, fillColor = {128, 128, 128}, fillPattern = FillPattern.Solid, extent = {{-100, -30}, {20, 28}}), Polygon(visible = true, lineColor = {128, 128, 128}, fillColor = {128, 128, 128}, fillPattern = FillPattern.Solid, points = {{20, 60}, {100, 0}, {20, -60}, {20, 60}}), Text(visible = true, textColor = {64, 64, 64}, extent = {{-150, 65}, {150, 105}}, textString = "%name")}));
end Arrow;
