within UWmBody.UWInterfaces;

partial function partialBuoyantAcceleration
  extends Modelica.Icons.Function;
  input Modelica.SIunits.Density d "Density of submerged object";
  output Modelica.SIunits.Acceleration buoyant_a[3] "Buoyant acceleration experienced by submerged object from surrounding fluid";
  annotation(Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
end partialBuoyantAcceleration;
