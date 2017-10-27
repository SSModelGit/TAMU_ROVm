within UWmBody.UWInterfaces;

partial function partialDragForce
  extends Modelica.Icons.Function;
  input Modelica.SIunits.Velocity v[:];
  output Modelica.SIunits.Force f_d[:];
  annotation(Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
end partialDragForce;
