within UWmBody.UWExamples.Elementary;

model RollingWheel "Single wheel rolling on ground starting from an initial speed"
  extends Modelica.Icons.Example;
  UWmBody.UWParts.RollingWheel wheel1(wheelRadius = 0.3, wheelMass = 2, wheel_I_axis = 0.06, wheel_I_long = 0.12, hollowFraction = 0.6, x(start = 0.2), y(start = 0.2), der_angles(start = {0, 5, 1})) annotation(Placement(transformation(extent = {{-20, 0}, {0, 20}})));
  inner UWmBody.UWWorld world(label2 = "z", n = {0, 0, -1}) annotation(Placement(transformation(extent = {{-60, 0}, {-40, 20}})));
  UWmBody.UWVisualizers.Ground ground(length = 4) annotation(Placement(transformation(extent = {{-20, -40}, {0, -20}})));
  annotation(__Wolfram(PlotSet(plots = {Plot(name = "Wheel position from above", preferred = true, subPlots = {SubPlot(curves = {Curve(x = wheel1.x, y = wheel1.y, legend = "Wheel position from above, in the x-y plane")})}), Plot(name = "Wheel forces", subPlots = {SubPlot(curves = {Curve(x = time, y = wheel1.rollingWheel.f_n, legend = "Normal force acting on wheel")}), SubPlot(curves = {Curve(x = time, y = wheel1.rollingWheel.f_lat, legend = "Contact force on wheel in lateral direction"), Curve(x = time, y = wheel1.rollingWheel.f_long, legend = "Contact force on wheel in longitudinal direction")})})})), experiment(StopTime = 4), Documentation(info = "<html>
<p>
Demonstrates how a single wheel is rolling on ground
(starting with an initial velocity).
</p>
</html>"));
end RollingWheel;
