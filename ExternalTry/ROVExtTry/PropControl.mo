within ExternalTry.ROVExtTry;

model PropControl
  parameter Real c = 1 "amplitude of signal";
  Modelica.Blocks.Interfaces.RealOutput u annotation(Placement(visible = true, transformation(origin = {140.0, -0.0}, extent = {{-10.0, -10.0}, {10.0, 10.0}}, rotation = 0), iconTransformation(origin = {107.5, 0.0}, extent = {{-7.5, -7.5}, {7.5, 7.5}}, rotation = 0)));
equation
  u = ExternalTry.ROVExtTry.PropSignal(c);
  annotation(Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}), graphics = {Rectangle(visible = true, fillColor = {255, 255, 255}, extent = {{-100, -100}, {100, 100}}), Text(visible = true, origin = {1.288, -117.457}, extent = {{-54.297, -12.543}, {54.297, 12.543}}, textString = "%name", fontName = "Liberation Sans"), Text(visible = true, origin = {0.076, 0.278}, extent = {{-100.076, -100.278}, {100.076, 100.278}}, textString = "PropSignal()", fontName = "Liberation Sans")}), Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
end PropControl;
