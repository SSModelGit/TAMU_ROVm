within ROVm.Examples;

model DataBasedSimplifiedBlueROV2
  extends ROVm.Templates.BlueROV2Simplified(propPos5.r = {0.05, 0.125, 0.15}, propPos6.r = {0.05, 0.125, -0.15}, propPos3.r = {-0.2, -0.05, 0.15}, propPos1.r = {0.2, -0.05, 0.15}, propPos2.r = {0.2, -0.05, -0.15}, propPos4.r = {-0.2, -0.05, -0.15}, rovBody.density.displayUnit = "g/cm3", rovBody.mu_d = 100, rovBody.k_d = 500, rovBody.I_11 = 0.2, rovBody.I_22 = 0.23, rovBody.I_33 = 0.25, basicProp5.direction_b = 1, basicProp4.direction_b = 1, basicProp2.direction_b = 1);
  Modelica.Blocks.Tables.CombiTable1Ds experimentData(tableName = "inout_cell_mat_parsed", tableOnFile = true, columns = 2:11, fileName = Modelica.Utilities.Files.loadResource("modelica://ROVm.Examples.DataBasedBlueROV2/../../Resources/PARSED_DATA_SPLIT/LOG15/IN_OUT_LOG15_PARSED_8.mat")) annotation(Placement(visible = true, transformation(origin = {-126.722, 87.349}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Clock clock annotation(Placement(visible = true, transformation(origin = {-130, 53.016}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.MatrixGain matrixGain(K = 14.8 * identity(6)) annotation(Placement(visible = true, transformation(origin = {-88.133, 87.349}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteAngles rov_angles annotation(Placement(visible = true, transformation(origin = {170, -25}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Sensors.AbsolutePosition rov_position(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world) annotation(Placement(visible = true, transformation(origin = {170, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteVelocity absoluteVelocity annotation(Placement(visible = true, transformation(origin = {170, 45}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteAngularVelocity absoluteAngularVelocity annotation(Placement(visible = true, transformation(origin = {170, -46.751}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(experimentData.y[1], matrixGain.u[1]) annotation(Line(visible = true, origin = {-107.927, 87.349}, points = {{-7.794, 0}, {7.794, 0}}, color = {1, 37, 163}));
  connect(experimentData.y[2], matrixGain.u[2]) annotation(Line(visible = true, origin = {-107.927, 87.349}, points = {{-7.794, 0}, {7.794, 0}}, color = {1, 37, 163}));
  connect(experimentData.y[3], matrixGain.u[3]) annotation(Line(visible = true, origin = {-107.927, 87.349}, points = {{-7.794, 0}, {7.794, 0}}, color = {1, 37, 163}));
  connect(experimentData.y[4], matrixGain.u[4]) annotation(Line(visible = true, origin = {-107.927, 87.349}, points = {{-7.794, 0}, {7.794, 0}}, color = {1, 37, 163}));
  connect(experimentData.y[5], matrixGain.u[5]) annotation(Line(visible = true, origin = {-107.927, 87.349}, points = {{-7.794, 0}, {7.794, 0}}, color = {1, 37, 163}));
  connect(experimentData.y[6], matrixGain.u[6]) annotation(Line(visible = true, origin = {-107.927, 87.349}, points = {{-7.794, 0}, {7.794, 0}}, color = {1, 37, 163}));
  connect(clock.y, experimentData.u) annotation(Line(visible = true, origin = {-128.861, 68.798}, points = {{9.861, -15.782}, {12.861, -15.782}, {12.861, -2.77}, {-12.861, -2.77}, {-12.861, 18.552}, {-9.861, 18.552}}, color = {1, 37, 163}));
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
  annotation(experiment(StopTime = 80));
end DataBasedSimplifiedBlueROV2;
