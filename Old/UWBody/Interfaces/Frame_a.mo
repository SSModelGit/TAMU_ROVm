within UWBody.Interfaces;

connector Frame_a "Coordinate system fixed to the component with one cut-force and cut-torque (filled rectangular icon)"
  extends Frame;
  annotation(defaultComponentName = "frame_a", Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.16, grid = {10, 10}), graphics = {Rectangle(visible = true, lineColor = {95, 95, 95}, lineThickness = 0.5, extent = {{-10, -10}, {10, 10}}), Rectangle(visible = true, lineColor = {64, 64, 64}, fillColor = {192, 192, 192}, fillPattern = FillPattern.Solid, extent = {{-30, -100}, {30, 100}})}), Diagram(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.16, grid = {10, 10}), graphics = {Text(visible = true, textColor = {64, 64, 64}, extent = {{-140, -88}, {140, -50}}, textString = "%name"), Rectangle(visible = true, lineColor = {64, 64, 64}, fillColor = {192, 192, 192}, fillPattern = FillPattern.Solid, extent = {{-12, -40}, {12, 40}})}), Documentation(info = "<html>
<p>
Basic definition of a coordinate system that is fixed to a mechanical
component. In the origin of the coordinate system the cut-force
and the cut-torque is acting.
This component has a filled rectangular icon.
</p>
</html>"));
end Frame_a;
