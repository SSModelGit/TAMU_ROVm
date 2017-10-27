within UWBody.Interfaces;

connector FlangeWithBearing "Connector consisting of 1-dim. rotational flange and its bearing frame"
  parameter Boolean includeBearingConnector = false "= true, if bearing frame connector is present, otherwise not present";
  Modelica.Mechanics.Rotational.Interfaces.Flange_a flange "1-dim. rotational flange";
  UWBody.Interfaces.Frame bearingFrame if includeBearingConnector "3-dim. frame in which the 1-dim. shaft is mounted";
  annotation(defaultComponentName = "flange", Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Rectangle(visible = true, lineColor = {135, 135, 135}, lineThickness = 0.5, extent = {{-20, -1}, {20, 1}}), Rectangle(visible = true, lineColor = {255, 255, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-100, -100}, {100, 100}}), Rectangle(visible = true, lineColor = {64, 64, 64}, fillColor = {192, 192, 192}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-100, -25}, {100, 25}}), Line(visible = true, points = {{-80, 60}, {80, 60}}, color = {64, 64, 64}), Line(visible = true, points = {{-80, -60}, {80, -60}}, color = {64, 64, 64}), Line(visible = true, points = {{0, 100}, {0, 60}}, color = {64, 64, 64}), Line(visible = true, points = {{0, -60}, {0, -100}}, color = {64, 64, 64}), Rectangle(visible = true, lineColor = {135, 135, 135}, extent = {{-100, -100}, {100, 100}}), Rectangle(visible = true, lineColor = {64, 64, 64}, extent = {{-100, -25}, {100, 25}})}), Diagram(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Line(visible = true, points = {{-50, -40}, {50, -40}}, thickness = 0.5), Line(visible = true, points = {{-50, 40}, {50, 40}}, thickness = 0.5), Text(visible = true, textColor = {64, 64, 64}, extent = {{-158, -124}, {158, -66}}, textString = "%name"), Rectangle(visible = true, lineColor = {255, 255, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, lineThickness = 0.5, extent = {{-60, -60}, {60, 60}}), Rectangle(visible = true, lineColor = {64, 64, 64}, fillColor = {192, 192, 192}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-60, -15}, {60, 15}}), Line(visible = true, points = {{0, 60}, {0, 40}}, color = {64, 64, 64}), Line(visible = true, points = {{0, -40}, {0, -60}}, color = {64, 64, 64}), Line(visible = true, points = {{-50, 40}, {50, 40}}, color = {64, 64, 64}), Line(visible = true, points = {{-50, -40}, {50, -40}}, color = {64, 64, 64}), Rectangle(visible = true, lineColor = {135, 135, 135}, extent = {{-60, -60}, {60, 60}}), Rectangle(visible = true, lineColor = {64, 64, 64}, extent = {{-60, -15}, {60, 15}})}), Documentation(info = "<html>
<p>
This hierarchical connector models a 1-dim. rotational flange
connector and its optional bearing defined by a 3-dim. frame connector.
If a connection to the subconnectors should be clearly visible,
connect first an  instance of
<a href=\"modelica://UWBody.Interfaces.FlangeWithBearingAdaptor\">FlangeWithBearingAdaptor</a>
to the FlangeWithBearing connector.
</p>
</html>"));
end FlangeWithBearing;
