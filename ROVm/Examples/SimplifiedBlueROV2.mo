within ROVm.Examples;

model SimplifiedBlueROV2
  extends ROVm.Templates.BlueROV2Simplified(propPos5.r = {0.05, 0.125, 0.15}, propPos6.r = {0.05, 0.125, -0.15}, propPos3.r = {-0.2, -0.05, 0.15}, propPos1.r = {0.2, -0.05, 0.15}, propPos2.r = {0.2, -0.05, -0.15}, propPos4.r = {-0.2, -0.05, -0.15}, rovBody.density.displayUnit = "g/cm3", rovBody.mu_d = 100, rovBody.k_d = 500, rovBody.I_11 = 0.2, rovBody.I_22 = 0.23, rovBody.I_33 = 0.25, basicProp5.direction_b = 1, basicProp4.direction_b = 1, basicProp2.direction_b = 1);
  Modelica.Blocks.Math.MatrixGain matrixGain(K = 14.8 * identity(6)) annotation(Placement(visible = true, transformation(origin = {-88.133, 87.349}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteAngles rov_angles annotation(Placement(visible = true, transformation(origin = {170, -25}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Sensors.AbsolutePosition rov_position(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world) annotation(Placement(visible = true, transformation(origin = {170, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteVelocity absoluteVelocity annotation(Placement(visible = true, transformation(origin = {170, 45}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteAngularVelocity absoluteAngularVelocity annotation(Placement(visible = true, transformation(origin = {170, -46.751}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput u[6] annotation(Placement(visible = true, transformation(origin = {-145, 87.074}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-95.342, 82.927}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
equation
  connect(matrixGain.y[1], firstOrder1.u) annotation(Line(visible = true, origin = {-8.66, 86.996}, points = {{-68.473, 0.354}, {21.824, 0.354}, {21.824, -0.354}, {24.824, -0.354}}, color = {1, 37, 163}));
  connect(matrixGain.y[2], firstOrder2.u) annotation(Line(visible = true, origin = {-69.903, 86.996}, points = {{-7.23, 0.354}, {1.41, 0.354}, {1.41, -0.354}, {4.41, -0.354}}, color = {1, 37, 163}));
  connect(matrixGain.y[3], firstOrder3.u) annotation(Line(visible = true, origin = {-17.354, 53.652}, points = {{-59.779, 33.697}, {19.124, 33.697}, {19.124, -33.697}, {21.531, -33.697}}, color = {1, 37, 163}));
  connect(matrixGain.y[4], firstOrder4.u) annotation(Line(visible = true, origin = {-69.903, 49.496}, points = {{-7.23, 37.854}, {1.41, 37.854}, {1.41, -37.854}, {4.41, -37.854}}, color = {1, 37, 163}));
  connect(matrixGain.y[5], firstOrder5.u) annotation(Line(visible = true, origin = {-18.718, 18.675}, points = {{-58.415, 68.675}, {18.472, 68.675}, {18.472, -68.675}, {21.472, -68.675}}, color = {1, 37, 163}));
  connect(matrixGain.y[6], firstOrder6.u) annotation(Line(visible = true, origin = {-74.566, 40.562}, points = {{-2.566, 46.787}, {0.434, 46.787}, {0.434, 33.775}, {-0.434, 33.775}, {-0.434, -80.562}, {2.566, -80.562}}, color = {1, 37, 163}));
  connect(absoluteVelocity.frame_a, rovBody.frame_a) annotation(Line(visible = true, origin = {117.244, 22.5}, points = {{42.756, 22.5}, {-15.256, 22.5}, {-15.256, -22.5}, {-12.244, -22.5}}, color = {95, 95, 95}));
  connect(rov_position.frame_a, rovBody.frame_a) annotation(Line(visible = true, origin = {117.244, 10}, points = {{42.756, 10}, {-15.256, 10}, {-15.256, -10}, {-12.244, -10}}, color = {95, 95, 95}));
  connect(rov_angles.frame_a, rovBody.frame_a) annotation(Line(visible = true, origin = {117.244, -12.5}, points = {{42.756, -12.5}, {-15.256, -12.5}, {-15.256, 12.5}, {-12.244, 12.5}}, color = {95, 95, 95}));
  connect(absoluteAngularVelocity.frame_a, rovBody.frame_a) annotation(Line(visible = true, origin = {117.244, -23.375}, points = {{42.756, -23.375}, {-15.256, -23.375}, {-15.256, 23.375}, {-12.244, 23.375}}, color = {95, 95, 95}));
  connect(u, matrixGain.u) annotation(Line(visible = true, origin = {-112.85, 87.212}, points = {{-32.15, -0.138}, {9.717, -0.138}, {9.717, 0.138}, {12.717, 0.138}}, color = {1, 37, 163}));
  annotation(experiment(StopTime = 80));
end SimplifiedBlueROV2;
