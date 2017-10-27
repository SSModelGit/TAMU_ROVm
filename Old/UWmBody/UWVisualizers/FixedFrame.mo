within UWmBody.UWVisualizers;

model FixedFrame "Visualizing a coordinate system including axes labels (visualization data may vary dynamically)"
  import UWmBody.UWTypes;
  extends UWmBody.UWInterfaces.PartialVisualizer;
  parameter Boolean animation = true "= true, if animation shall be enabled";
  parameter Boolean showLabels = true "= true, if labels shall be shown" annotation(Dialog(group = "if animation = true", enable = animation));
  input SI.Distance length = 0.5 "Length of axes arrows" annotation(Dialog(group = "if animation = true", enable = animation));
  input SI.Distance diameter = length / world.defaultFrameDiameterFraction "Diameter of axes arrows" annotation(Dialog(group = "if animation = true", enable = animation));
  input UWTypes.Color color_x = UWmBody.UWTypes.Defaults.FrameColor "Color of x-arrow" annotation(Dialog(colorSelector = true, group = "if animation = true", enable = animation));
  input UWTypes.Color color_y = color_x "Color of y-arrow" annotation(Dialog(colorSelector = true, group = "if animation = true", enable = animation));
  input UWTypes.Color color_z = color_x "Color of z-arrow" annotation(Dialog(colorSelector = true, group = "if animation = true", enable = animation));
  input UWTypes.SpecularCoefficient specularCoefficient = world.defaultSpecularCoefficient "Reflection of ambient light (= 0: light is completely absorbed)" annotation(Dialog(group = "if animation = true", enable = animation));
protected
  parameter Boolean animation2 = world.enableAnimation and animation;
  parameter Boolean showLabels2 = world.enableAnimation and animation and showLabels;
  // Parameters to define axes
  SI.Length headLength = min(length, diameter * Types.Defaults.FrameHeadLengthFraction);
  SI.Length headWidth = diameter * Types.Defaults.FrameHeadWidthFraction;
  SI.Length lineLength = max(0, length - headLength);
  SI.Length lineWidth = diameter;
  // Parameters to define axes labels
  SI.Length scaledLabel = UWmBody.UWTypes.Defaults.FrameLabelHeightFraction * diameter;
  SI.Length labelStart = 1.05 * length;
  // x-axis
  Visualizers.Advanced.Shape x_arrowLine(shapeType = "cylinder", length = lineLength, width = lineWidth, height = lineWidth, lengthDirection = {1, 0, 0}, widthDirection = {0, 1, 0}, color = color_x, specularCoefficient = specularCoefficient, r = frame_a.r_0, R = frame_a.R) if animation2;
  Visualizers.Advanced.Shape x_arrowHead(shapeType = "cone", length = headLength, width = headWidth, height = headWidth, lengthDirection = {1, 0, 0}, widthDirection = {0, 1, 0}, color = color_x, specularCoefficient = specularCoefficient, r = frame_a.r_0 + Frames.resolve1(frame_a.R, {lineLength, 0, 0}), R = frame_a.R) if animation2;
  Visualizers.Internal.Lines x_label(lines = scaledLabel * {[0, 0; 1, 1], [0, 1; 1, 0]}, diameter = diameter, color = color_x, specularCoefficient = specularCoefficient, r_lines = {labelStart, 0, 0}, n_x = {1, 0, 0}, n_y = {0, 1, 0}, r = frame_a.r_0, R = frame_a.R) if showLabels2;
  // y-axis
  Visualizers.Advanced.Shape y_arrowLine(shapeType = "cylinder", length = lineLength, width = lineWidth, height = lineWidth, lengthDirection = {0, 1, 0}, widthDirection = {1, 0, 0}, color = color_y, specularCoefficient = specularCoefficient, r = frame_a.r_0, R = frame_a.R) if animation2;
  Visualizers.Advanced.Shape y_arrowHead(shapeType = "cone", length = headLength, width = headWidth, height = headWidth, lengthDirection = {0, 1, 0}, widthDirection = {1, 0, 0}, color = color_y, specularCoefficient = specularCoefficient, r = frame_a.r_0 + Frames.resolve1(frame_a.R, {0, lineLength, 0}), R = frame_a.R) if animation2;
  Visualizers.Internal.Lines y_label(lines = scaledLabel * {[0, 0; 1, 1.5], [0, 1.5; 0.5, 0.75]}, diameter = diameter, color = color_y, specularCoefficient = specularCoefficient, r_lines = {0, labelStart, 0}, n_x = {0, 1, 0}, n_y = {-1, 0, 0}, r = frame_a.r_0, R = frame_a.R) if showLabels2;
  // z-axis
  Visualizers.Advanced.Shape z_arrowLine(shapeType = "cylinder", length = lineLength, width = lineWidth, height = lineWidth, lengthDirection = {0, 0, 1}, widthDirection = {0, 1, 0}, color = color_z, specularCoefficient = specularCoefficient, r = frame_a.r_0, R = frame_a.R) if animation2;
  Visualizers.Advanced.Shape z_arrowHead(shapeType = "cone", length = headLength, width = headWidth, height = headWidth, lengthDirection = {0, 0, 1}, widthDirection = {0, 1, 0}, color = color_z, specularCoefficient = specularCoefficient, r = frame_a.r_0 + Frames.resolve1(frame_a.R, {0, 0, lineLength}), R = frame_a.R) if animation2;
  Visualizers.Internal.Lines z_label(lines = scaledLabel * {[0, 0; 1, 0], [0, 1; 1, 1], [0, 1; 1, 0]}, diameter = diameter, color = color_z, specularCoefficient = specularCoefficient, r_lines = {0, 0, labelStart}, n_x = {0, 0, 1}, n_y = {0, 1, 0}, r = frame_a.r_0, R = frame_a.R) if showLabels2;
equation
  frame_a.f = zeros(3);
  frame_a.t = zeros(3);
  annotation(Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Rectangle(visible = true, lineColor = {0, 127, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, -100}, {100, 100}}), Line(visible = true, origin = {2, -2}, points = {{-2, -18}, {-2, 92}}, color = {47, 167, 41}, thickness = 0.5, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 30), Text(visible = true, textColor = {47, 167, 41}, extent = {{20, 60}, {60, 100}}, textString = "y", horizontalAlignment = TextAlignment.Left), Text(visible = true, textColor = {64, 64, 64}, extent = {{80, -50}, {120, -10}}, textString = "x", horizontalAlignment = TextAlignment.Left), Line(visible = true, points = {{0, -20}, {90, -70}}, color = {64, 64, 64}, thickness = 0.5, arrow = {Arrow.None, Arrow.Filled}, arrowSize = 30), Line(visible = true, origin = {2, -2}, points = {{-92, -68}, {-2, -18}}, color = {10, 90, 224}, thickness = 0.5, arrow = {Arrow.Filled, Arrow.None}, arrowSize = 30), Text(visible = true, textColor = {10, 90, 224}, extent = {{-120, -50}, {-80, -10}}, textString = "z", horizontalAlignment = TextAlignment.Right), Text(visible = true, textColor = {64, 64, 64}, extent = {{-150, 105}, {150, 145}}, textString = "%name")}), Documentation(info = "<html>
<p>
Model <b>FixedFrame</b> visualizes the three axes of
its coordinate system <b>frame_a</b> together with appropriate axes
labels. A typical example is shown in the following figure:
<br>&nbsp;
</p>

<p>
<IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/FixedFrame.png\" ALT=\"model Visualizers.FixedFrame\">
</p>

<p>
The sizes of the axes, the axes colors and the specular coefficient
(= reflection factor for
ambient light) can vary dynamically by
providing appropriate expressions in the input fields of the
parameter menu.
</p>
</html>"));
end FixedFrame;
