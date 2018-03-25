within ExternalTry;

model ExtBlock
  parameter Modelica.SIunits.AngularVelocity w_start = 0 "Angular velocity at start time";
  parameter Modelica.SIunits.AngularVelocity w_end = 10 "Angular velocity at end time";
  parameter Real A = 1 "amplitude of signal";
  parameter Real M = 10 "time period for signal";
  Modelica.Blocks.Interfaces.RealOutput u annotation(Placement(visible = true, transformation(origin = {140.0, -0.0}, extent = {{-10.0, -10.0}, {10.0, 10.0}}, rotation = 0), iconTransformation(origin = {107.5, 0.0}, extent = {{-7.5, -7.5}, {7.5, 7.5}}, rotation = 0)));
equation
  u = ExternalTry.ExtCall(w_start, w_end, A, M, time);
  annotation(Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}), graphics = {Rectangle(visible = true, fillColor = {255, 255, 255}, extent = {{-100, -100}, {100, 100}})}));
end ExtBlock;
