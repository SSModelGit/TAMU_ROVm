within UWBody.Interfaces;

model FlangeWithBearingAdaptor "Adaptor to allow direct connections to the sub-connectors of FlangeWithBearing"
  parameter Boolean includeBearingConnector = false "= true, if bearing frame connector is present, otherwise not present";
  UWBody.Interfaces.FlangeWithBearing flangeAndFrame(includeBearingConnector = includeBearingConnector) "Compound connector consisting of 1-dim. rotational flange and 3-dim. frame mounting" annotation(Placement(transformation(extent = {{-130, -30}, {-70, 30}}, origin = {0, 0}, rotation = 0), visible = true, iconTransformation(origin = {0, 0}, extent = {{-130, -30}, {-70, 30}}, rotation = 0)));
  Modelica.Mechanics.Rotational.Interfaces.Flange_b flange "1-dim. rotational flange" annotation(Placement(transformation(extent = {{-10, -10}, {10, 10}})));
  Frame_a frame if includeBearingConnector "3-dim. frame in which the 1-dim. shaft is mounted" annotation(Placement(transformation(origin = {0, -100}, extent = {{-16, -16}, {16, 16}}, rotation = 90)));
equation
  connect(flange, flangeAndFrame.flange) annotation(Line(points = {{50, 0}, {-50, 0}}, visible = true, origin = {-50, 0}, color = {64, 64, 64}));
  connect(frame, flangeAndFrame.bearingFrame) annotation(Line(points = {{0, -100}, {0, -40}, {-100, -40}, {-100, 0}}, thickness = 0.5, visible = true, color = {64, 64, 64}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10})), defaultComponentName = "adaptor", Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Rectangle(visible = true, lineColor = {255, 255, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, -100}, {20, 30}}), Line(visible = true, points = {{-100, -10}, {-100, -40}, {0, -40}, {0, -100}}, color = {64, 64, 64}), Line(visible = true, points = {{-90, 0}, {0, 0}}, color = {64, 64, 64}), Text(visible = true, textColor = {64, 64, 64}, extent = {{-216, 36}, {86, 76}}, textString = "%name")}), Documentation(info = "<html>
<p>
Adaptor object to make a more visible connection to the flange and frame
subconnectors of a
<a href=\"modelica://UWBody.Interfaces.FlangeWithBearing\">FlangeWithBearing</a>
connector.
</p>
</html>"));
end FlangeWithBearingAdaptor;
