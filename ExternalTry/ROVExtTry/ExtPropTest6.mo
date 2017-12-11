within ExternalTry.ROVExtTry;

model ExtPropTest6
  Modelica.Blocks.Continuous.FirstOrder firstOrder(T = 1, k = 1) annotation(Placement(visible = true, transformation(origin = {-50, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.FirstOrder firstOrder1(T = 1, k = 1) annotation(Placement(visible = true, transformation(origin = {31.658, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.FirstOrder firstOrder2(T = 1, k = 1) annotation(Placement(visible = true, transformation(origin = {-50, 5}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.FirstOrder firstOrder3(T = 1, k = 1) annotation(Placement(visible = true, transformation(origin = {48.603, 5}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.FirstOrder firstOrder4(T = 1, k = 1) annotation(Placement(visible = true, transformation(origin = {-50, -55}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Continuous.FirstOrder firstOrder5(T = 1, k = 1) annotation(Placement(visible = true, transformation(origin = {48.603, -55}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  PropControl6 propControl6 annotation(Placement(visible = true, transformation(origin = {-135, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  ROVm.Propeller.Examples.BasicProp basicProp(m_Propeller = 5, A_Propeller = 0.2, n = Modelica.Math.Vectors.normalize({1, 0, 1}), r = 1, direction = 1) annotation(Placement(visible = true, transformation(origin = {-30, 45}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  ROVm.Propeller.Examples.BasicProp basicProp1(m_Propeller = 5, A_Propeller = 0.2, n = Modelica.Math.Vectors.normalize({1, 0, 1}), r = 1, direction = 1) annotation(Placement(visible = true, transformation(origin = {-27.29, -7.574}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  ROVm.Propeller.Examples.BasicProp basicProp2(m_Propeller = 5, A_Propeller = 0.2, n = Modelica.Math.Vectors.normalize({1, 0, 1}), r = 1, direction = 1) annotation(Placement(visible = true, transformation(origin = {57.247, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  ROVm.Propeller.Examples.BasicProp basicProp3(m_Propeller = 5, A_Propeller = 0.2, n = Modelica.Math.Vectors.normalize({1, 0, 1}), r = 1, direction = 1) annotation(Placement(visible = true, transformation(origin = {70, -15}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  ROVm.Propeller.Examples.BasicProp basicProp4(m_Propeller = 5, A_Propeller = 0.2, n = Modelica.Math.Vectors.normalize({1, 0, 1}), r = 1, direction = 1) annotation(Placement(visible = true, transformation(origin = {76.595, -65}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  ROVm.Propeller.Examples.BasicProp basicProp5(m_Propeller = 5, A_Propeller = 0.2, n = Modelica.Math.Vectors.normalize({1, 0, 1}), r = 1, direction = 1) annotation(Placement(visible = true, transformation(origin = {-27.166, -68.092}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner RBodyInFluid.Fields.WaterField waterField annotation(Placement(visible = true, transformation(origin = {-130, 65}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner Modelica.Mechanics.MultiBody.World world annotation(Placement(visible = true, transformation(origin = {-132.071, 36.541}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(propControl6.u[1], firstOrder.u) annotation(Line(visible = true, origin = {-79.062, 35}, points = {{-45.188, -35}, {14.062, -35}, {14.062, 35}, {17.062, 35}}, color = {1, 37, 163}));
  connect(propControl6.u[3], firstOrder2.u) annotation(Line(visible = true, origin = {-79.062, 2.5}, points = {{-45.188, -2.5}, {14.062, -2.5}, {14.062, 2.5}, {17.062, 2.5}}, color = {1, 37, 163}));
  connect(propControl6.u[5], firstOrder4.u) annotation(Line(visible = true, origin = {-79.062, -27.5}, points = {{-45.188, 27.5}, {14.062, 27.5}, {14.062, -27.5}, {17.062, -27.5}}, color = {1, 37, 163}));
  connect(propControl6.u[2], firstOrder1.u) annotation(Line(visible = true, origin = {-17.819, 35}, points = {{-106.431, -35}, {34.477, -35}, {34.477, 35}, {37.477, 35}}, color = {1, 37, 163}));
  connect(propControl6.u[4], firstOrder3.u) annotation(Line(visible = true, origin = {-5.11, 2.5}, points = {{-119.14, -2.5}, {38.713, -2.5}, {38.713, 2.5}, {41.713, 2.5}}, color = {1, 37, 163}));
  connect(propControl6.u[6], firstOrder5.u) annotation(Line(visible = true, origin = {-5.11, -27.5}, points = {{-119.14, 27.5}, {38.713, 27.5}, {38.713, -27.5}, {41.713, -27.5}}, color = {1, 37, 163}));
  connect(firstOrder.y, basicProp.u) annotation(Line(visible = true, origin = {-33, 64.667}, points = {{-6, 5.333}, {3, 5.333}, {3, -10.667}}, color = {1, 37, 163}));
  connect(firstOrder2.y, basicProp1.u) annotation(Line(visible = true, origin = {-35.097, 2.617}, points = {{-3.903, 2.383}, {-3.903, -1.191}, {7.807, -1.191}}, color = {1, 37, 163}));
  connect(firstOrder4.y, basicProp5.u) annotation(Line(visible = true, origin = {-35.055, -57.728}, points = {{-3.945, 2.728}, {-3.945, -1.364}, {7.889, -1.364}}, color = {1, 37, 163}));
  connect(firstOrder5.y, basicProp4.u) annotation(Line(visible = true, origin = {67.6, -53.91}, points = {{-7.997, -1.09}, {-4.997, -1.09}, {-4.997, 2.135}, {8.995, 2.135}, {8.995, -2.09}}, color = {1, 37, 163}));
  connect(firstOrder3.y, basicProp3.u) annotation(Line(visible = true, origin = {63.069, -2.333}, points = {{-3.466, 7.333}, {-3.466, -3.667}, {6.931, -3.667}}, color = {1, 37, 163}));
  connect(firstOrder1.y, basicProp2.u) annotation(Line(visible = true, origin = {52.384, 66.333}, points = {{-9.726, 3.667}, {4.863, 3.667}, {4.863, -7.333}}, color = {1, 37, 163}));
  annotation(Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
end ExtPropTest6;
