within UWmBody.UWVisualizers.Advanced;

model DoubleArrow "Visualizing a double arrow with variable size; all data have to be set as modifiers (see info layer)"
  import UWmBody.UWTypes;
  import UWmBody.UWFrames;
  import T = UWmBody.UWFrames.TransformationMatrices;
  import Modelica.SIunits.Conversions.to_unit1;
  input UWFrames.Orientation R = Frames.nullRotation() "Orientation object to rotate the world frame into the arrow frame" annotation(Dialog);
  input SI.Position r[3] = {0, 0, 0} "Position vector from origin of world frame to origin of arrow frame, resolved in world frame" annotation(Dialog);
  input SI.Position r_tail[3] = {0, 0, 0} "Position vector from origin of arrow frame to double arrow tail, resolved in arrow frame" annotation(Dialog);
  input SI.Position r_head[3] = {0, 0, 0} "Position vector from double arrow tail to the head of the double arrow, resolved in arrow frame" annotation(Dialog);
  input SI.Diameter diameter = world.defaultArrowDiameter "Diameter of arrow line" annotation(Dialog);
  input UWmBody.UWTypes.Color color = UWmBody.UWTypes.Defaults.ArrowColor "Color of double arrow" annotation(Dialog(colorSelector = true));
  input UWTypes.SpecularCoefficient specularCoefficient = world.defaultSpecularCoefficient "Material property describing the reflecting of ambient light (= 0 means, that light is completely absorbed)" annotation(Dialog);
protected
  outer UWmBody.UWWorld world;
  SI.Length length = Modelica.Math.Vectors.length(r_head) "Length of arrow";
  Real e_x[3](each final unit = "1", start = {1, 0, 0}) = noEvent(if length < 1.e-10 then {1, 0, 0} else r_head / length);
  Real rxvisobj[3](each final unit = "1") = transpose(R.T) * e_x "X-axis unit vector of shape, resolved in world frame" annotation(HideResult = true);
  SI.Position rvisobj[3] = r + T.resolve1(R.T, r_tail) "Position vector from world frame to shape frame, resolved in world frame" annotation(HideResult = true);
  SI.Length headLength = noEvent(max(0, min(length, diameter * MultiBody.Types.Defaults.ArrowHeadLengthFraction)));
  SI.Length headWidth = noEvent(max(0, diameter * MultiBody.Types.Defaults.ArrowHeadWidthFraction));
  SI.Length arrowLength = noEvent(max(0, length - 1.5 * diameter * MultiBody.Types.Defaults.ArrowHeadLengthFraction));
  Visualizers.Advanced.Shape arrowLine(length = arrowLength, width = diameter, height = diameter, lengthDirection = to_unit1(r_head), widthDirection = {0, 1, 0}, shapeType = "cylinder", color = color, specularCoefficient = specularCoefficient, r_shape = r_tail, r = r, R = R) if world.enableAnimation;
  Visualizers.Advanced.Shape arrowHead1(length = headLength, width = headWidth, height = headWidth, lengthDirection = to_unit1(r_head), widthDirection = {0, 1, 0}, shapeType = "cone", color = color, specularCoefficient = specularCoefficient, r = rvisobj + rxvisobj * arrowLength, R = R) if world.enableAnimation;
  Visualizers.Advanced.Shape arrowHead2(length = headLength, width = headWidth, height = headWidth, lengthDirection = to_unit1(r_head), widthDirection = {0, 1, 0}, shapeType = "cone", color = color, specularCoefficient = specularCoefficient, r = rvisobj + rxvisobj * (arrowLength + 0.5 * headLength), R = R) if world.enableAnimation;
  annotation(Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Rectangle(visible = true, lineColor = {128, 128, 128}, fillColor = {128, 128, 128}, fillPattern = FillPattern.Solid, extent = {{-100, -28}, {0, 28}}), Polygon(visible = true, lineColor = {128, 128, 128}, fillColor = {128, 128, 128}, fillPattern = FillPattern.Solid, points = {{40, 60}, {100, 0}, {40, -60}, {40, 60}}), Text(visible = true, textColor = {64, 64, 64}, extent = {{-150, 65}, {150, 105}}, textString = "%name"), Polygon(visible = true, lineColor = {128, 128, 128}, fillColor = {128, 128, 128}, fillPattern = FillPattern.Solid, points = {{0, 60}, {60, 0}, {0, -60}, {0, 60}})}), Documentation(info = "<html>
<p>
Model <b>DoubleArrow</b> defines a double arrow that is dynamically
visualized at the defined location (see variables below).
</p>

<p>
<IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Visualizers/DoubleArrow.png\" ALT=\"model Visualizers.Advanced.DoubleArrow\">
</p>

<p>
The variables under heading <b>Parameters</b> below
are declared as (time varying) <b>input</b> variables.
If the default equation is not appropriate, a corresponding
modifier equation has to be provided in the
model where an <b>Arrow</b> instance is used, e.g., in the form
</p>
<pre>
    Visualizers.Advanced.DoubleArrow doubleArrow(diameter = sin(time));
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
</html>"));
end DoubleArrow;
