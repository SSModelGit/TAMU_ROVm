within UWBody.Examples.Elementary;

model HeatLosses "Demonstrate the modeling of heat losses"
  extends Modelica.Icons.Example;
  inner World world annotation(Placement(transformation(extent = {{-100, 40}, {-80, 60}}, origin = {-0, 20}, rotation = 0), visible = true));
  Parts.Body body1(m = 1, r_CM = {0, -0.2, 0}, cylinderDiameter = 0.05, sphereDiameter = 0.15, I_11 = 0.1, I_22 = 0.1, I_33 = 0.1, r_0(start = {0.3, -0.2, 0}, each fixed = true), v_0(each fixed = true), angles_fixed = true, w_0_fixed = true, w_0_start(each displayUnit = "deg/s") = {0, 0, 0.034906585039887}) annotation(Placement(transformation(origin = {-40, -10}, extent = {{-10, 10}, {10, -10}}, rotation = 270), visible = true));
  Parts.FixedTranslation bar1(r = {0.3, 0, 0}) annotation(Placement(transformation(extent = {{-66, 40}, {-46, 60}}, origin = {-0, 20}, rotation = 0), visible = true));
  Parts.FixedTranslation bar2(r = {0.3, 0, 0}) annotation(Placement(transformation(extent = {{-20, 40}, {0, 60}}, origin = {-0, 20}, rotation = 0), visible = true));
  Forces.Spring spring1(s_unstretched = 0.1, coilWidth = 0.01, c = 30, numberOfWindings = 10, width = 0.1) annotation(Placement(transformation(origin = {-25, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 270), visible = true));
  Forces.Damper damper1(d = 2, useHeatPort = true) annotation(Placement(transformation(origin = {-55, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 270), visible = true));
  Forces.SpringDamperParallel springDamper(d = 2, c = 30, s_unstretched = 0.1, width = 0.1, coilWidth = 0.01, numberOfWindings = 10, useHeatPort = true) annotation(Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = -90, origin = {20, 30}), visible = true));
  Parts.Body body2(m = 1, r_CM = {0, -0.2, 0}, cylinderDiameter = 0.05, sphereDiameter = 0.15, I_11 = 0.1, I_22 = 0.1, I_33 = 0.1, v_0(each fixed = true), angles_fixed = true, w_0_fixed = true, w_0_start(each displayUnit = "deg/s") = {0, 0, 0.034906585039887}, r_0(start = {0.6, -0.2, 0}, each fixed = true)) annotation(Placement(transformation(origin = {20, -10}, extent = {{-10, 10}, {10, -10}}, rotation = 270), visible = true));
  Parts.FixedTranslation bar3(r = {0.3, 0, 0}) annotation(Placement(transformation(extent = {{34, 40}, {54, 60}}, origin = {-4, 20}, rotation = 0), visible = true));
  Forces.SpringDamperSeries springDamperSeries(d = 2, c = 30, s_unstretched = 0.1, useHeatPort = true) annotation(Placement(transformation(extent = {{-10, 10}, {10, -10}}, rotation = -90, origin = {80, 30}), visible = true));
  Parts.Body body3(m = 1, r_CM = {0, -0.2, 0}, cylinderDiameter = 0.05, sphereDiameter = 0.15, I_11 = 0.1, I_22 = 0.1, I_33 = 0.1, v_0(each fixed = true), angles_fixed = true, w_0_fixed = true, w_0_start(each displayUnit = "deg/s") = {0, 0, 0.034906585039887}, r_0(start = {0.9, -0.2, 0}, each fixed = true)) annotation(Placement(transformation(origin = {80, -10}, extent = {{-10, 10}, {10, -10}}, rotation = 270), visible = true));
  Forces.Spring spring(s_unstretched = 0.2, width = 0.05, c = 30) annotation(Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = -90, origin = {60, 30}), visible = true));
  Modelica.Blocks.Sources.Constant const(k = 20) annotation(Placement(transformation(extent = {{-8, -100}, {12, -80}}, origin = {-2, 10}, rotation = 0), visible = true));
  Modelica.Thermal.HeatTransfer.Components.Convection convection annotation(Placement(transformation(extent = {{26, -52}, {46, -72}}, origin = {4, 12}, rotation = 0), visible = true));
  Modelica.Thermal.HeatTransfer.Celsius.FixedTemperature TAmbient(T = 25) "Ambient temperature" annotation(Placement(transformation(extent = {{80, -72}, {60, -52}}, origin = {-0, 12}, rotation = 0), visible = true));
equation
  connect(world.frame_b, bar1.frame_a) annotation(Line(points = {{-80, 50}, {-66, 50}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-0, 20}));
  connect(bar1.frame_b, bar2.frame_a) annotation(Line(points = {{-46, 50}, {-20, 50}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-0, 20}));
  connect(damper1.frame_a, bar1.frame_b) annotation(Line(points = {{-55, 20}, {-55, 30}, {-40, 30}, {-40, 50}, {-46, 50}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-0, 20}));
  connect(spring1.frame_a, bar1.frame_b) annotation(Line(points = {{-25, 20}, {-25, 30}, {-40, 30}, {-40, 50}, {-46, 50}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-0, 20}));
  connect(damper1.frame_b, body1.frame_a) annotation(Line(points = {{-55, 0}, {-55, -10}, {-40, -10}, {-40, -20}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-0, 20}));
  connect(spring1.frame_b, body1.frame_a) annotation(Line(points = {{-25, 0}, {-25, -10}, {-40, -10}, {-40, -20}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-0, 20}));
  connect(bar2.frame_b, springDamper.frame_a) annotation(Line(points = {{-13.333, 10}, {6.667, 10}, {6.667, -20}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {13.333, 60}));
  connect(springDamper.frame_b, body2.frame_a) annotation(Line(points = {{20, 0}, {20, -20}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-0, 20}));
  connect(bar3.frame_b, springDamperSeries.frame_a) annotation(Line(points = {{-20, 10}, {10, 10}, {10, -20}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {70, 60}));
  connect(springDamperSeries.frame_b, body3.frame_a) annotation(Line(points = {{80, 0}, {80, -20}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-0, 20}));
  connect(bar3.frame_a, bar2.frame_b) annotation(Line(points = {{15, 0}, {-15, 0}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {15, 70}));
  connect(bar3.frame_b, spring.frame_a) annotation(Line(points = {{-6.667, 10}, {3.333, 10}, {3.333, -20}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {56.667, 60}));
  connect(spring.frame_b, body3.frame_a) annotation(Line(points = {{60, -2}, {60, -12}, {80, -12}, {80, -22}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-0, 22}));
  connect(const.y, convection.Gc) annotation(Line(points = {{-19.333, -6.667}, {9.667, -6.667}, {9.667, 13.333}}, color = {1, 37, 163}, visible = true, origin = {30.333, -73.333}));
  connect(TAmbient.port, convection.fluid) annotation(Line(points = {{5, 0}, {-5, 0}}, color = {191, 0, 0}, visible = true, origin = {55, -50}));
  connect(damper1.heatPort, convection.solid) annotation(Line(points = {{-60, 20}, {-60, -70}, {35, -70}}, color = {191, 0, 0}, visible = true, origin = {-5, 20}));
  connect(springDamper.heatPort, convection.solid) annotation(Line(points = {{14, 20}, {4, 20}, {4, -70}, {34, -70}}, color = {191, 0, 0}, visible = true, origin = {-4, 20}));
  connect(springDamperSeries.heatPort, convection.solid) annotation(Line(points = {{86, 20}, {96, 20}, {96, -45}, {-4, -45}, {-4, -70}, {26, -70}}, color = {191, 0, 0}, visible = true, origin = {4, 20}));
  annotation(__Wolfram(PlotSet(plots = {Plot(name = "Heat losses", preferred = true, subPlots = {SubPlot(curves = {Curve(x = time, y = body1.r_0[2], legend = "Elongation of body1"), Curve(x = time, y = body2.r_0[2], legend = "Elongation of body2"), Curve(x = time, y = body3.r_0[2], legend = "Elongation of body3")}), SubPlot(curves = {Curve(x = time, y = damper1.f, legend = "Force from damper1 connected to body1"), Curve(x = time, y = springDamper.f, legend = "Force from springDamper connected to body2"), Curve(x = time, y = springDamperSeries.f, legend = "Force from springDamperSeries connected to body3")}), SubPlot(curves = {Curve(x = time, y = damper1.heatPort.Q_flow, legend = "Heat flow from damper1 connected to body1"), Curve(x = time, y = springDamper.heatPort.Q_flow, legend = "Heat flow from springDamper connected to body2"), Curve(x = time, y = springDamperSeries.heatPort.Q_flow, legend = "Heat flow from springDamperSeries connected to body3")})})})), Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10})), experiment(StopTime = 3), Documentation(info = "<html>
<p>
This model demonstrates how to model the dissipated power of a multi-body
force element by enabling the heatPort of all components and connecting these heatPorts via
a convection element to the environment. The total heat flow generated by the
elements of this multi-body system and transported to the environment
is present in variable convection.fluid.
</p>
</html>"));
end HeatLosses;
