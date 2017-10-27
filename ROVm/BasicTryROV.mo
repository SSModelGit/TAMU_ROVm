within ROVm;

model BasicTryROV
  inner Modelica.Mechanics.MultiBody.World world(gravityType = Modelica.Mechanics.MultiBody.Types.GravityTypes.UniformGravity) annotation(Placement(visible = true, transformation(origin = {-138.043, -95}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Forces.WorldForce thrust1(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b) annotation(Placement(visible = true, transformation(origin = {-40, 35}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Forces.WorldForce thrust2(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b) annotation(Placement(visible = true, transformation(origin = {42.06, 36.606}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Forces.WorldForce thrust3(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b) annotation(Placement(visible = true, transformation(origin = {-42.411, -55}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Forces.WorldForce thrust4(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b) annotation(Placement(visible = true, transformation(origin = {37.647, -55}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Forces.WorldForce thrust5(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b) annotation(Placement(visible = true, transformation(origin = {-35, -5}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Forces.WorldForce thrust6(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b) annotation(Placement(visible = true, transformation(origin = {30, -10}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.MatrixGain thrustMag1(K = sqrt(2) / 2 * diagonal({-1, 1, -1})) annotation(Placement(visible = true, transformation(origin = {-72.68, 35}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.MatrixGain thrustMag2(K = sqrt(2) / 2 * diagonal({-1, 1, 1})) annotation(Placement(visible = true, transformation(origin = {70, 36.895}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.MatrixGain thrustMag3(K = sqrt(2) / 2 * diagonal({1, 1, -1})) annotation(Placement(visible = true, transformation(origin = {-70, -55}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.MatrixGain thrustMag4(K = sqrt(2) / 2 * identity(3)) annotation(Placement(visible = true, transformation(origin = {65.159, -55.757}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.MatrixGain thrustMag5(K = identity(3)) annotation(Placement(visible = true, transformation(origin = {-67.523, -5}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.MatrixGain thrustMag6(K = diagonal({1, -1, 1})) annotation(Placement(visible = true, transformation(origin = {60, -10}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Forces.WorldForce buoyancyForce annotation(Placement(visible = true, transformation(origin = {-7.116, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Forces.WorldForceAndTorque fluidDragForce(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b) annotation(Placement(visible = true, transformation(origin = {-5, -77.987}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.BodyShape basicROV(m = 5, r_CM = {0, -0.05, 0}, shapeType = "modelica://ROVm/Resources/AssemBROV2R1.stl", r = {0, -0.1, 0}) annotation(Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = -90)), Include = "#include \"AssemBROV2R1.stl\" ", IncludeDirectory = "modelica://ROVm/Resources/Include");
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteVelocity absoluteVelocity(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.frame_a) annotation(Placement(visible = true, transformation(origin = {30, -85}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.SIunits.Velocity unitV[3];
  Modelica.SIunits.Velocity scalarV;
  parameter Modelica.SIunits.DimensionlessRatio c_d = 1;
  parameter Modelica.SIunits.Density rho = 1000;
  parameter Modelica.SIunits.Area cutA = 0.5;
  parameter Real thrustMagMax[6] = {0, 0, 3, 3, 0, 0} "Array of maximum possible propeller thrust magnitudes - [Prop 1, 2, 3, 4, 5, 6]";
  Real thrustMag[6] "Array of propeller thrust force magnitudes - [Prop 1, 2, 3, 4, 5, 6]";
  Modelica.Mechanics.MultiBody.Visualizers.FixedShape fixedShape(length = 0.5, width = sqrt(0.1), height = sqrt(0.1), color = {0, 0, 255}, lengthDirection = {-1, 0, 0}, shapeType = "box") annotation(Placement(visible = true, transformation(origin = {0, -28.274}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation prop1Loc(r = {0.2, 0, -0.1}, animation = false) annotation(Placement(visible = true, transformation(origin = {-17.63, 35}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation prop2Loc(r = {0.2, 0, 0.1}, animation = false) annotation(Placement(visible = true, transformation(origin = {20, 36.922}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation prop3Loc(r = {-0.2, 0, -0.1}, animation = false) annotation(Placement(visible = true, transformation(origin = {-17.546, -55}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation prop4Loc(r = {-0.2, 0, 0.1}, animation = false) annotation(Placement(visible = true, transformation(origin = {15, -55}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation prop5Loc(r = {0, 0.1, -0.1}, animation = false) annotation(Placement(visible = true, transformation(origin = {-35, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation prop6Loc(r = {0, 0.1, 0.1}, animation = false) annotation(Placement(visible = true, transformation(origin = {30, 6.786}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Sensors.AbsoluteAngularVelocity absoluteAngularVelocity annotation(Placement(visible = true, transformation(origin = {70, -85}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
protected
  parameter Real lateralPropVector[3] = {1, 0, 1};
  parameter Real verticalPropVector[3] = {0, 1, 0};
algorithm
  thrustMag := thrustMagMax;
equation
  connect(thrustMag1.y, thrust1.force) annotation(Line(visible = true, origin = {-56.84, 35}, points = {{-4.84, 0}, {4.84, 0}}, color = {1, 37, 163}));
  connect(thrustMag5.y, thrust5.force) annotation(Line(visible = true, origin = {-51.761, -5}, points = {{-4.761, 0}, {4.761, 0}}, color = {1, 37, 163}));
  connect(thrustMag3.y, thrust3.force) annotation(Line(visible = true, origin = {-56.706, -55}, points = {{-2.294, 0}, {2.294, 0}}, color = {1, 37, 163}));
  connect(thrustMag2.y, thrust2.force) annotation(Line(visible = true, origin = {55.795, 36.751}, points = {{3.205, 0.145}, {-0.735, 0.145}, {-0.735, -0.145}, {-1.735, -0.145}}, color = {1, 37, 163}));
  connect(thrustMag6.y, thrust6.force) annotation(Line(visible = true, origin = {45.5, -10}, points = {{3.5, 0}, {-3.5, 0}}, color = {1, 37, 163}));
  connect(thrustMag4.y, thrust4.force) annotation(Line(visible = true, origin = {51.275, -55.379}, points = {{2.884, -0.378}, {-0.628, -0.378}, {-0.628, 0.379}, {-1.628, 0.379}}, color = {1, 37, 163}));
  connect(buoyancyForce.frame_b, basicROV.frame_a) annotation(Line(visible = true, origin = {2.931, 31.205}, points = {{-0.046, 28.795}, {2.954, 28.795}, {2.954, -18.192}, {-2.931, -18.192}, {-2.931, -21.205}}, color = {95, 95, 95}));
  connect(absoluteVelocity.frame_a, basicROV.frame_a) annotation(Line(visible = true, origin = {10.8, -26.795}, points = {{9.2, -58.205}, {6.2, -58.205}, {6.2, 39.808}, {-10.8, 39.808}, {-10.8, 36.795}}, color = {95, 95, 95}));
  connect(fixedShape.frame_a, basicROV.frame_a) annotation(Line(visible = true, origin = {-7.29, -4.105}, points = {{-2.71, -24.169}, {-5.935, -24.169}, {-5.935, 17.117}, {7.29, 17.117}, {7.29, 14.105}}, color = {95, 95, 95}));
  connect(fluidDragForce.frame_b, basicROV.frame_a) annotation(Line(visible = true, origin = {6.2, -23.99}, points = {{-1.2, -53.997}, {6.8, -53.997}, {6.8, 37.002}, {-6.2, 37.002}, {-6.2, 33.99}}, color = {95, 95, 95}));
  connect(thrust2.frame_b, prop2Loc.frame_a) annotation(Line(visible = true, origin = {31.015, 36.764}, points = {{1.045, -0.158}, {-0.015, -0.158}, {-0.015, 0.158}, {-1.015, 0.158}}, color = {95, 95, 95}));
  connect(prop2Loc.frame_b, basicROV.frame_a) annotation(Line(visible = true, origin = {3.333, 27.948}, points = {{6.667, 8.974}, {-3.333, 8.974}, {-3.333, -17.948}}, color = {95, 95, 95}));
  connect(prop1Loc.frame_b, basicROV.frame_a) annotation(Line(visible = true, origin = {-2.543, 26.667}, points = {{-5.086, 8.333}, {2.543, 8.333}, {2.543, -16.667}}, color = {95, 95, 95}));
  connect(thrust1.frame_b, prop1Loc.frame_a) annotation(Line(visible = true, origin = {-28.815, 35}, points = {{-1.185, 0}, {1.185, 0}}, color = {95, 95, 95}));
  connect(thrust4.frame_b, prop4Loc.frame_a) annotation(Line(visible = true, origin = {26.323, -55}, points = {{1.323, 0}, {-1.323, 0}}, color = {95, 95, 95}));
  connect(prop4Loc.frame_b, basicROV.frame_a) annotation(Line(visible = true, origin = {4.968, -22.568}, points = {{0.032, -32.432}, {-3.08, -32.432}, {-3.08, -19.432}, {8.032, -19.432}, {8.032, 35.58}, {-4.968, 35.58}, {-4.968, 32.568}}, color = {95, 95, 95}));
  connect(thrust3.frame_b, prop3Loc.frame_a) annotation(Line(visible = true, origin = {-29.978, -55}, points = {{-2.433, 0}, {2.433, 0}}, color = {95, 95, 95}));
  connect(prop3Loc.frame_b, basicROV.frame_a) annotation(Line(visible = true, origin = {-6.059, -22.568}, points = {{-1.487, -32.432}, {1.626, -32.432}, {1.626, -19.432}, {-6.941, -19.432}, {-6.941, 35.58}, {6.059, 35.58}, {6.059, 32.568}}, color = {95, 95, 95}));
  connect(thrust5.frame_b, prop5Loc.frame_a) annotation(Line(visible = true, origin = {-31.8, -10.657}, points = {{6.8, 5.657}, {9.8, 5.657}, {9.8, -0.985}, {-13.2, -0.985}, {-13.2, -9.343}}, color = {95, 95, 95}));
  connect(prop5Loc.frame_b, basicROV.frame_a) annotation(Line(visible = true, origin = {-13.755, -0.795}, points = {{-11.245, -19.205}, {-8.132, -19.205}, {-8.132, 13.808}, {13.755, 13.808}, {13.755, 10.795}}, color = {95, 95, 95}));
  connect(thrust6.frame_b, prop6Loc.frame_a) annotation(Line(visible = true, origin = {26.667, 1.191}, points = {{-6.667, -11.191}, {-6.667, 5.595}, {13.333, 5.595}}, color = {95, 95, 95}));
  connect(prop6Loc.frame_b, basicROV.frame_a) annotation(Line(visible = true, origin = {10.755, 9.919}, points = {{9.245, -3.133}, {6.132, -3.133}, {6.132, 3.093}, {-10.755, 3.093}, {-10.755, 0.081}}, color = {95, 95, 95}));
  scalarV = Modelica.Math.Vectors.length(absoluteVelocity.v);
  unitV = Modelica.Math.Vectors.normalize(absoluteVelocity.v);
  fluidDragForce.force = -0.5 * scalarV * scalarV * rho * c_d * cutA * unitV;
  fluidDragForce.torque = -0.5 * Modelica.Math.Vectors.length(absoluteAngularVelocity.w) * Modelica.Math.Vectors.length(absoluteAngularVelocity.w) * rho * c_d * cutA * Modelica.Math.Vectors.normalize(absoluteAngularVelocity.w);
  buoyancyForce.force = -basicROV.m * basicROV.body.g_0;
  thrustMag1.u = thrustMag[1] * lateralPropVector;
  thrustMag2.u = thrustMag[2] * lateralPropVector;
  thrustMag3.u = thrustMag[3] * lateralPropVector;
  thrustMag4.u = thrustMag[4] * lateralPropVector;
  thrustMag5.u = thrustMag[5] * verticalPropVector;
  thrustMag6.u = thrustMag[6] * verticalPropVector;
  connect(absoluteAngularVelocity.frame_a, basicROV.frame_a) annotation(Line(visible = true, origin = {26.813, -38.634}, points = {{33.187, -46.366}, {20.122, -46.366}, {20.122, -29.353}, {-9.903, -29.353}, {-9.903, 51.403}, {-26.813, 51.403}, {-26.813, 48.634}}, color = {95, 95, 95}));
  annotation(experiment(Interval = 0.001, __Wolfram_Algorithm = "rk4"), Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
  annotation(IncludeDirectory = "modelica://ROVm/Resources/Include", Include = "#include \"AssemBROV2R1.stl\" ");
end BasicTryROV;
