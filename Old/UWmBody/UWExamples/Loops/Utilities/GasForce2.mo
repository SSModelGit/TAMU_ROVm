within UWmBody.UWExamples.Loops.Utilities;

model GasForce2 "Rough approximation of gas force in a cylinder"
  import Modelica.Constants.pi;
  extends Modelica.Mechanics.Translational.Interfaces.PartialCompliant;
  parameter SI.Length L "Length of cylinder";
  parameter SI.Length d "Diameter of cylinder";
  parameter SIunits.Volume k0 = 0.01 "Volume V = k0 + k1*(1-x), with x = 1 - s_rel/L";
  parameter SIunits.Volume k1 = 1 "Volume V = k0 + k1*(1-x), with x = 1 - s_rel/L";
  parameter SIunits.HeatCapacity k = 1 "Gas constant (p*V = k*T)";
  /*
      parameter Real k0=0.01;
      parameter Real k1=1;
      parameter Real k=1;
    */
  Real x "Normalized position of cylinder (= 1 - s_rel/L)";
  SI.Density dens;
  Modelica.SIunits.AbsolutePressure press "Cylinder pressure";
  SI.Volume V;
  SI.Temperature T;
  Modelica.SIunits.Velocity v_rel;
protected
  Modelica.SIunits.SpecificHeatCapacity R_air = Modelica.Constants.R / 0.0289651159;
equation
  x = 1 - s_rel / L;
  v_rel = der(s_rel);
  press = 1.0E5 * (if v_rel < 0 then if x < 0.987 then 177.4132 * x ^ 4 - 287.2189 * x ^ 3 + 151.8252 * x ^ 2 - 24.9973 * x + 2.4 else 2836360 * x ^ 4 - 10569296 * x ^ 3 + 14761814 * x ^ 2 - 9158505 * x + 2129670 else if x > 0.93 then (-3929704 * x ^ 4) + 14748765 * x ^ 3 - 20747000 * x ^ 2 + 12964477 * x - 3036495 else 145.930 * x ^ 4 - 131.707 * x ^ 3 + 17.3438 * x ^ 2 + 17.9272 * x + 2.4);
  f = -press * pi * d ^ 2 / 4;
  V = k0 + k1 * (1 - x);
  dens = press / (R_air * T);
  press * V = k * T;
  assert(s_rel >= (-1.e-12), "flange_b.s - flange_a.s (= " + String(s_rel, significantDigits = 14) + ") >= 0 required for GasForce component.\n" + "Most likely, the component has to be flipped.");
  assert(s_rel <= L + 1.e-12, " flange_b.s - flange_a.s (= " + String(s_rel, significantDigits = 14) + ") <= L (= " + String(L, significantDigits = 14) + ") required for GasForce component.\n" + "Most likely, parameter L is not correct.");
  annotation(Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Rectangle(visible = true, fillColor = {192, 192, 192}, fillPattern = FillPattern.Solid, extent = {{-90, -50}, {90, 50}}), Polygon(visible = true, lineColor = {190, 20, 31}, fillColor = {190, 20, 31}, fillPattern = FillPattern.Solid, points = {{-13, 4}, {-16, 4}, {-65, 4}, {-65, 15}, {-90, 0}, {-65, -15}, {-65, -4}, {-13, -4}, {-13, 4}}), Text(visible = true, textColor = {128, 128, 128}, extent = {{-135, 19}, {-99, 44}}, textString = "a"), Text(visible = true, textColor = {128, 128, 128}, extent = {{97, 15}, {133, 40}}, textString = "b"), Polygon(visible = true, lineColor = {190, 20, 31}, fillColor = {190, 20, 31}, fillPattern = FillPattern.Solid, points = {{12, 4}, {70, 4}, {65, 4}, {65, 15}, {90, 0}, {65, -15}, {65, -4}, {12, -4}, {12, 4}}), Text(visible = true, textColor = {64, 64, 64}, extent = {{-150, 65}, {150, 105}}, textString = "%name")}), Diagram(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Rectangle(visible = true, fillColor = {192, 192, 192}, fillPattern = FillPattern.Solid, extent = {{-90, -50}, {90, 50}}), Polygon(visible = true, lineColor = {190, 20, 31}, fillColor = {190, 20, 31}, fillPattern = FillPattern.Solid, points = {{12, 5}, {70, 5}, {65, 5}, {65, 16}, {90, 1}, {65, -14}, {65, -3}, {12, -3}, {12, 5}}), Polygon(visible = true, lineColor = {190, 20, 31}, fillColor = {190, 20, 31}, fillPattern = FillPattern.Solid, points = {{-13, 5}, {-16, 5}, {-65, 5}, {-65, 16}, {-90, 1}, {-65, -14}, {-65, -3}, {-13, -3}, {-13, 5}})}), Documentation(info = "<html>
<p>
The gas force in a cylinder is computed as function of the relative
distance of the two flanges. It is required that s_rel = flange_b.s - flange_a.s
is in the range
</p>
<pre>
    0 &le; s_rel &le; L
</pre>
<p>
where the parameter L is the length
of the cylinder. If this assumption is not fulfilled, an error occurs.
</p>
</html>"));
end GasForce2;
