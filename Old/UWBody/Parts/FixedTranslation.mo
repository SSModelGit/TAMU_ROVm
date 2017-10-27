within UWBody.Parts;

model FixedTranslation "Fixed translation of frame_b with respect to frame_a"
  import UWBody.Types;
  import Modelica.SIunits.Conversions.to_unit1;
  Interfaces.Frame_a frame_a "Coordinate system fixed to the component with one cut-force and cut-torque" annotation(Placement(transformation(extent = {{-116, -16}, {-84, 16}})));
  Interfaces.Frame_b frame_b "Coordinate system fixed to the component with one cut-force and cut-torque" annotation(Placement(transformation(extent = {{84, -16}, {116, 16}})));
  parameter Boolean animation = true "= true, if animation shall be enabled";
  parameter SI.Position r[3](start = {0, 0, 0}) "Vector from frame_a to frame_b resolved in frame_a";
  parameter Types.ShapeType shapeType = "cylinder" "Type of shape" annotation(Dialog(tab = "Animation", group = "if animation = true", enable = animation));
  parameter SI.Position r_shape[3] = {0, 0, 0} "Vector from frame_a to shape origin, resolved in frame_a" annotation(Dialog(tab = "Animation", group = "if animation = true", enable = animation));
  parameter Types.Axis lengthDirection = to_unit1(r - r_shape) "Vector in length direction of shape, resolved in frame_a" annotation(Evaluate = true, Dialog(tab = "Animation", group = "if animation = true", enable = animation));
  parameter Types.Axis widthDirection = {0, 1, 0} "Vector in width direction of shape, resolved in frame_a" annotation(Evaluate = true, Dialog(tab = "Animation", group = "if animation = true", enable = animation));
  parameter SI.Length length = Modelica.Math.Vectors.length(r - r_shape) "Length of shape" annotation(Dialog(tab = "Animation", group = "if animation = true", enable = animation));
  parameter SI.Distance width = length / world.defaultWidthFraction "Width of shape" annotation(Dialog(tab = "Animation", group = "if animation = true", enable = animation));
  parameter SI.Distance height = width "Height of shape" annotation(Dialog(tab = "Animation", group = "if animation = true", enable = animation));
  parameter Types.ShapeExtra extra = 0.0 "Additional parameter depending on shapeType (see docu of Visualizers.Advanced.Shape)" annotation(Dialog(tab = "Animation", group = "if animation = true", enable = animation));
  input Types.Color color = UWBody.Types.Defaults.RodColor "Color of shape" annotation(Dialog(colorSelector = true, tab = "Animation", group = "if animation = true", enable = animation));
  input Types.SpecularCoefficient specularCoefficient = world.defaultSpecularCoefficient "Reflection of ambient light (= 0: light is completely absorbed)" annotation(Dialog(tab = "Animation", group = "if animation = true", enable = animation));
protected
  outer UWBody.World world;
  Visualizers.Advanced.Shape shape(shapeType = shapeType, color = color, specularCoefficient = specularCoefficient, r_shape = r_shape, lengthDirection = lengthDirection, widthDirection = widthDirection, length = length, width = width, height = height, extra = extra, r = frame_a.r_0, R = frame_a.R) if world.enableAnimation and animation;
equation
  Connections.branch(frame_a.R, frame_b.R);
  assert(cardinality(frame_a) > 0 or cardinality(frame_b) > 0, "Neither connector frame_a nor frame_b of FixedTranslation object is connected");
  frame_b.r_0 = frame_a.r_0 + Frames.resolve1(frame_a.R, r);
  frame_b.R = frame_a.R;
  /* Force and torque balance */
  zeros(3) = frame_a.f + frame_b.f;
  zeros(3) = frame_a.t + frame_b.t + cross(r, frame_b.f);
  annotation(Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Rectangle(visible = true, lineColor = {64, 64, 64}, fillColor = {64, 64, 64}, fillPattern = FillPattern.Solid, extent = {{-99, -5}, {101, 5}}), Text(visible = true, textColor = {64, 64, 64}, extent = {{-150, 45}, {150, 85}}, textString = "%name"), Text(visible = true, textColor = {64, 64, 64}, extent = {{-150, -50}, {150, -20}}, textString = "r=%r"), Text(visible = true, textColor = {128, 128, 128}, extent = {{-89, 13}, {-53, 38}}, textString = "a"), Text(visible = true, textColor = {128, 128, 128}, extent = {{57, 14}, {93, 39}}, textString = "b")}), Diagram(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(extent = {{-100, 5}, {100, -5}}, lineColor = {0, 0, 0}, fillColor = {0, 0, 0}, fillPattern = FillPattern.Solid), Line(points = {{-95, 20}, {-58, 20}}, color = {128, 128, 128}, arrow = {Arrow.None, Arrow.Filled}), Line(points = {{-94, 18}, {-94, 50}}, color = {128, 128, 128}, arrow = {Arrow.None, Arrow.Filled}), Text(extent = {{-72, 35}, {-58, 24}}, lineColor = {128, 128, 128}, textString = "x"), Text(extent = {{-113, 57}, {-98, 45}}, lineColor = {128, 128, 128}, textString = "y"), Line(points = {{-100, -4}, {-100, -69}}, color = {128, 128, 128}), Line(points = {{-100, -63}, {90, -63}}, color = {128, 128, 128}), Text(extent = {{-22, -39}, {16, -63}}, lineColor = {128, 128, 128}, textString = "r"), Polygon(points = {{88, -59}, {88, -68}, {100, -63}, {88, -59}}, lineColor = {0, 0, 0}, fillColor = {0, 0, 0}, fillPattern = FillPattern.Solid), Line(points = {{100, -3}, {100, -68}}, color = {128, 128, 128}), Line(points = {{69, 20}, {106, 20}}, color = {128, 128, 128}, arrow = {Arrow.None, Arrow.Filled}), Line(points = {{70, 18}, {70, 50}}, color = {128, 128, 128}, arrow = {Arrow.None, Arrow.Filled}), Text(extent = {{92, 35}, {106, 24}}, lineColor = {128, 128, 128}, textString = "x"), Text(extent = {{51, 57}, {66, 45}}, lineColor = {128, 128, 128}, textString = "y")}), Documentation(info = "<html>
<p>
Component for a <b>fixed translation</b> of frame_b with respect
to frame_a, i.e., the relationship between connectors frame_a and frame_b
remains constant and frame_a is always <b>parallel</b> to frame_b.
</p>
<p>
By default, this component is visualized by a cylinder connecting
frame_a and frame_b, as shown in the figure below. Note, that the
two visualized frames are not part of the component animation and that
the animation may be switched off via parameter animation = <b>false</b>.
</p>

<p>
<IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/FixedTranslation.png\" ALT=\"Parts.FixedTranslation\">
</p>
</html>"));
end FixedTranslation;
