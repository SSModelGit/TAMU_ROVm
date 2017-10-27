within UWmBody.UWExamples.Elementary;

model RollingWheelSetDriving "Rolling wheel set that is driven by torques driving the wheels"
  extends Modelica.Icons.Example;
  UWmBody.UWVisualizers.Ground ground(length = 3, groundColor = {0, 255, 0}) annotation(Placement(transformation(extent = {{-20, -60}, {0, -40}}, origin = {-30, -20}, rotation = 0), visible = true));
  inner UWmBody.UWWorld world(label2 = "z", n = {0, 0, -1}) annotation(Placement(transformation(extent = {{-80, -60}, {-60, -40}}, origin = {0, -20}, rotation = 0), visible = true));
  UWmBody.UWParts.RollingWheelSet wheelSet(wheelRadius = 0.1, wheelMass = 0.5, wheel_I_axis = 0.01, wheel_I_long = 0.02, wheelDistance = 0.5, x(start = 0.1, fixed = true), y(start = 0.1, fixed = true), phi(fixed = true), theta1(fixed = true), theta2(fixed = true), der_theta1(fixed = true), der_theta2(fixed = true)) annotation(Placement(transformation(extent = {{-20, -20}, {0, 0}}, origin = {0, -30}, rotation = 0), visible = true));
  UWmBody.UWParts.Body body(m = 0.01, r_CM = {0, 0, 0}, animation = false) annotation(Placement(transformation(extent = {{40, 56}, {60, 76}}, origin = {0, -26}, rotation = 0), visible = true));
  UWmBody.UWParts.FixedTranslation fixedTranslation(r = {0.2, 0, 0}, animation = true, width = 0.04) annotation(Placement(transformation(extent = {{0, 56}, {20, 76}}, origin = {0, -26}, rotation = 0), visible = true));
  Modelica.Blocks.Sources.Sine sine1(freqHz = 1, amplitude = 2) annotation(Placement(transformation(extent = {{-80, 20}, {-60, 40}}, origin = {0, -20}, rotation = 0), visible = true));
  Modelica.Blocks.Sources.Sine sine2(freqHz = 1, amplitude = 2, phase = 1.5707963267949) annotation(Placement(transformation(extent = {{60, 20}, {40, 40}}, origin = {0, -20}, rotation = 0), visible = true));
  Modelica.Mechanics.Rotational.Sources.Torque2 torque1 annotation(Placement(transformation(extent = {{-40, 4}, {-20, 24}}, origin = {0, -24}, rotation = 0), visible = true));
  Modelica.Mechanics.Rotational.Sources.Torque2 torque2 annotation(Placement(transformation(extent = {{24, 4}, {4, 24}}, origin = {-4, -24}, rotation = 0), visible = true));
  UWmBody.UWVisualizers.FixedShape shape(final lengthDirection = {0, 1, 0}, final widthDirection = {1, 0, 0}, final shapeType = "pipe", final r_shape = {0, -wheelSet.wheelWidth, 0}, final length = 2 * wheelSet.wheelWidth, final width = 2 * wheelSet.wheelRadius, final height = 2 * wheelSet.wheelRadius, final color = {0, 128, 255}, final extra = 0.8) annotation(Placement(transformation(extent = {{-10, -10}, {10, 10}}, origin = {50, 70}, rotation = 0), visible = true));
equation
  connect(fixedTranslation.frame_a, wheelSet.frameMiddle) annotation(Line(points = {{-0, 60}, {-10, 60}, {-10, -20}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {0, -20}));
  connect(fixedTranslation.frame_b, body.frame_a) annotation(Line(points = {{-10, 0}, {10, 0}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {30, 40}));
  connect(wheelSet.axis1, torque1.flange_a) annotation(Line(points = {{-20, -10}, {-40, -10}, {-40, 10}}, visible = true, origin = {0, -20}, color = {64, 64, 64}));
  connect(torque1.flange_b, wheelSet.support) annotation(Line(points = {{-6.667, 7.267}, {3.333, 7.267}, {3.333, -14.533}}, visible = true, origin = {-13.333, -17.267}, color = {64, 64, 64}));
  connect(wheelSet.axis2, torque2.flange_a) annotation(Line(points = {{-0, -10}, {20, -10}, {20, 10}}, visible = true, origin = {0, -20}, color = {64, 64, 64}));
  connect(wheelSet.support, torque2.flange_b) annotation(Line(points = {{-3.333, -14.533}, {-3.333, 7.267}, {6.667, 7.267}}, visible = true, origin = {-6.667, -17.267}, color = {64, 64, 64}));
  connect(sine1.y, torque1.tau) annotation(Line(points = {{-59, 30}, {-30, 30}, {-30, 14}}, color = {1, 37, 163}, visible = true, origin = {0, -20}));
  connect(sine2.y, torque2.tau) annotation(Line(points = {{39, 30}, {10, 30}, {10, 14}}, color = {1, 37, 163}, visible = true, origin = {0, -20}));
  connect(shape.frame_a, fixedTranslation.frame_b) annotation(Line(points = {{38, 90}, {28, 90}, {28, 60}, {18, 60}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {2, -20}));
  annotation(__Wolfram(PlotSet(plots = {Plot(name = "Torques and forces", preferred = true, subPlots = {SubPlot(curves = {Curve(x = time, y = torque1.tau, legend = "Torque on wheel1"), Curve(x = time, y = torque2.tau, legend = "Torque on wheel2"), Curve(x = time, y = wheelSet.wheelSetJoint.support.tau, legend = "Total torque on wheels")}), SubPlot(curves = {Curve(x = time, y = wheelSet.wheelSetJoint.rolling1.f_lat, legend = "Lateral force on wheel1"), Curve(x = time, y = wheelSet.wheelSetJoint.rolling1.f_long, legend = "Longitudinal force on wheel1")}), SubPlot(curves = {Curve(x = time, y = wheelSet.wheelSetJoint.rolling2.f_lat, legend = "Lateral force on wheel2"), Curve(x = time, y = wheelSet.wheelSetJoint.rolling2.f_long, legend = "Longitudinal force on wheel2")})})})), experiment(StopTime = 3), Documentation(info = "<html>
<p>
Demonstrates how a RollingWheelSet (two wheels rigidly coupled together) is rolling
on ground when driven by some external torques.
</p>
</html>"));
end RollingWheelSetDriving;
