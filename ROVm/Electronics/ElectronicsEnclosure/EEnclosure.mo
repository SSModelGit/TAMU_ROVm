within ROVm.Electronics.ElectronicsEnclosure;

model EEnclosure
  import SI = Modelica.SIunits;
  Modelica.Electrical.Analog.Interfaces.PositivePin pin_p annotation(Placement(visible = true, transformation(origin = {-100, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Interfaces.NegativePin pin_n annotation(Placement(visible = true, transformation(origin = {100, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_lb annotation(Placement(visible = true, transformation(origin = {-149, -61}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {-100, -70}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  RBodyInFluid.Parts.BasicBodyShape eEnclosure(A = A_EEnclosure, animation = animation, r_CM = r_CM_EEnclosure, m = m_EEnclosure, c_d = c_d_EEnclosure, density = d_EEnclosure, r = r_CM_EEnclosure * 2, color = color) annotation(Placement(visible = true, transformation(origin = {2.301, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Basic.Resistor resistor(R = R_EEnclosure) annotation(Placement(visible = true, transformation(origin = {0, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  // frames to TopPieces
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_lf annotation(Placement(visible = true, transformation(origin = {-147.707, 60}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {-100, 50}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_rf annotation(Placement(visible = true, transformation(origin = {149, 60}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {100, 50}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_rb annotation(Placement(visible = true, transformation(origin = {149, -59}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {100, -70}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation(r = lf_pos, animation = animationFT) annotation(Placement(visible = true, transformation(origin = {-105, 60}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation1(r = lb_pos, animation = animationFT) annotation(Placement(visible = true, transformation(origin = {-110, -60}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation2(r = rf_pos, animation = animationFT) annotation(Placement(visible = true, transformation(origin = {95, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation3(r = rb_pos, animation = animationFT) annotation(Placement(visible = true, transformation(origin = {88.348, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  // movement vectors
  SI.Position r_0[3](start = {0, 0, 0}, each stateSelect = if enforceStates then StateSelect.always else StateSelect.avoid) "Position vector from origin of world frame to origin of frame_a" annotation(Dialog(tab = "Initialization", showStartAttribute = true));
  SI.Velocity v_0[3](start = {0, 0, 0}, each stateSelect = if enforceStates then StateSelect.always else StateSelect.avoid) "Absolute velocity of frame_a, resolved in world frame (= der(r_0))" annotation(Dialog(tab = "Initialization", showStartAttribute = true));
  SI.Acceleration a_0[3](start = {0, 0, 0}) "Absolute acceleration of frame_a resolved in world frame (= der(v_0))" annotation(Dialog(tab = "Initialization", showStartAttribute = true));
  // parameters for external frames and translations
  parameter SI.Length lf_pos[3];
  parameter SI.Length lb_pos[3];
  parameter SI.Length rf_pos[3];
  parameter SI.Length rb_pos[3];
  // parameters general
  parameter Boolean animation = true;
  parameter Boolean animationFT = false;
  parameter Boolean enforceStates = false "= true, if absolute variables of body object shall be used as states (StateSelect.always)" annotation(Evaluate = true, Dialog(tab = "Advanced"));
  parameter Boolean useQuaternions = true "= true, if quaternions shall be used as potential states otherwise use 3 angles as potential states" annotation(Evaluate = true, Dialog(tab = "Advanced"));
  parameter SI.Length r_CM_EEnclosure[3] "r_CM from frame_a to center of mass of electronics enclosure";
  parameter SI.Mass m_EEnclosure "Mass of electronics enclosure";
  parameter SI.Density d_EEnclosure "Density of electronics enclosure";
  parameter SI.Area A_EEnclosure "Overall cross sectional area of the electronics enclosure";
  parameter SI.DimensionlessRatio c_d_EEnclosure "Drag coefficient of the electronics enclosure";
  parameter SI.Resistance R_EEnclosure "Resistance of electronics enclosure";
  // animation parameters
  input Modelica.Mechanics.MultiBody.Types.Color color = Modelica.Mechanics.MultiBody.Types.Defaults.BodyColor "Color of shape" annotation(Dialog(colorSelector = true, tab = "Animation", group = "if animation = true", enable = animation));
equation
  r_0 = frame_lb.r_0;
  v_0 = der(r_0);
  a_0 = der(v_0);
  connect(resistor.n, pin_n) annotation(Line(visible = true, origin = {55, 100}, points = {{-45, 0}, {45, 0}}, color = {10, 90, 224}));
  connect(resistor.p, pin_p) annotation(Line(visible = true, origin = {-55, 100}, points = {{45, 0}, {-45, 0}}, color = {10, 90, 224}));
  connect(frame_rb, fixedTranslation3.frame_b) annotation(Line(visible = true, origin = {112.567, -59.5}, points = {{36.433, 0.5}, {-11.107, 0.5}, {-11.107, -0.5}, {-14.219, -0.5}}, color = {95, 95, 95}));
  connect(frame_rf, fixedTranslation2.frame_b) annotation(Line(visible = true, origin = {127, 60}, points = {{22, 0}, {-22, 0}}, color = {95, 95, 95}));
  connect(fixedTranslation2.frame_a, eEnclosure.frame_a) annotation(Line(visible = true, origin = {13.97, 30}, points = {{71.031, 30}, {-24.681, 30}, {-24.681, -30}, {-21.669, -30}}, color = {95, 95, 95}));
  connect(fixedTranslation3.frame_a, eEnclosure.frame_a) annotation(Line(visible = true, origin = {12.306, -30}, points = {{66.041, -30}, {-23.018, -30}, {-23.018, 30}, {-20.005, 30}}, color = {95, 95, 95}));
  connect(frame_lf, fixedTranslation.frame_b) annotation(Line(visible = true, origin = {-131.353, 60}, points = {{-16.353, 0}, {16.353, 0}}, color = {95, 95, 95}));
  connect(frame_lb, fixedTranslation1.frame_b) annotation(Line(visible = true, origin = {-128.806, -60.5}, points = {{-20.194, -0.5}, {5.694, -0.5}, {5.694, 0.5}, {8.806, 0.5}}, color = {95, 95, 95}));
  connect(fixedTranslation1.frame_a, eEnclosure.frame_a) annotation(Line(visible = true, origin = {-32.281, -30}, points = {{-67.719, -30}, {21.569, -30}, {21.569, 30}, {24.582, 30}}, color = {95, 95, 95}));
  connect(fixedTranslation.frame_a, eEnclosure.frame_a) annotation(Line(visible = true, origin = {-31.03, 30}, points = {{-63.969, 30}, {20.319, 30}, {20.319, -30}, {23.331, -30}}, color = {95, 95, 95}));
  annotation(Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}), graphics = {Rectangle(visible = true, origin = {0, -10}, fillColor = {0, 170, 255}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-90, -60}, {90, 60}}), Line(visible = true, origin = {-36.667, 85.318}, points = {{-53.333, 14.682}, {26.667, 14.682}, {26.667, -29.364}}, thickness = 2.5), Line(visible = true, origin = {36.667, 85.441}, points = {{53.333, 14.559}, {-26.667, 14.559}, {-26.667, -29.119}}, thickness = 2.5)}), Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
end EEnclosure;
