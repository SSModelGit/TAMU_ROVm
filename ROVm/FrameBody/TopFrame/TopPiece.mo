within ROVm.FrameBody.TopFrame;

model TopPiece
  import SI = Modelica.SIunits;
  RBodyInFluid.Parts.BasicBody fairing(m = m_Fairing, r_CM = r_CM_Fairing, density = d_Fairing, c_d = c_d_Fairing, A = A_Fairing, animation = animation) annotation(Placement(visible = true, transformation(origin = {-42.568, -37.312}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_SF "Left front connection to side plates, as viewed from the back" annotation(Placement(visible = true, transformation(origin = {-149, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {-99.698, 71.429}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_EC "Right front connection to side plate, as viewed from the back" annotation(Placement(visible = true, transformation(origin = {149, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {100.302, 71.429}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation angledPropPosition(r = r_CM_TP - thickness, animation = animationFT) annotation(Placement(visible = true, transformation(origin = {0, -55}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  RBodyInFluid.Parts.BasicBodyShape topPlate(density = d_plate, r = r_TP, r_CM = r_CM_TP, m = m_TP, A = A_TP, c_d = c_d_TP, animation = animation) annotation(Placement(visible = true, transformation(origin = {-1.549, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_propA "Connection to the battery enclosure" annotation(Placement(visible = true, transformation(origin = {0, -104}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {0, -98.316}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  SI.Position r_0[3](start = {0, 0, 0}, each stateSelect = if enforceStates then StateSelect.always else StateSelect.avoid) "Position vector from origin of world frame to origin of frame_a" annotation(Dialog(tab = "Initialization", showStartAttribute = true));
  SI.Velocity v_0[3](start = {0, 0, 0}, each stateSelect = if enforceStates then StateSelect.always else StateSelect.avoid) "Absolute velocity of frame_a, resolved in world frame (= der(r_0))" annotation(Dialog(tab = "Initialization", showStartAttribute = true));
  SI.Acceleration a_0[3](start = {0, 0, 0}) "Absolute acceleration of frame_a resolved in world frame (= der(v_0))" annotation(Dialog(tab = "Initialization", showStartAttribute = true));
  parameter Boolean animation = true;
  parameter Boolean animationFT = false;
  parameter SI.Density d_plate = 2700;
  parameter SI.Length r_TP[3] = {0, 0, 0.15};
  parameter SI.Length TP_Width[3] = {0.3, 0, 0};
  parameter SI.Length r_CM_TP[3] = r_TP / 2;
  parameter SI.Length r_CM_Fairing[3] = {0, 0.05, 0};
  parameter SI.Mass m_TP = 0.25;
  parameter SI.Area A_TP = 0.0009;
  parameter SI.DimensionlessRatio c_d_TP = 1;
  parameter SI.DimensionlessRatio c_d_Fairing = 1;
  parameter SI.Length thickness[3] = {0, 0.05, 0};
  parameter Boolean enforceStates = false "= true, if absolute variables of body object shall be used as states (StateSelect.always)" annotation(Evaluate = true, Dialog(tab = "Advanced"));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation(r = r_CM_TP, animation = animationFT) annotation(Placement(visible = true, transformation(origin = {-110, -37.272}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
protected
  parameter SI.Length r_Fairing[3] = r_TP;
  parameter SI.Density d_Fairing = 0.7;
  parameter SI.Mass m_Fairing = 0.5;
  parameter SI.Area A_Fairing = 0.002;
equation
  r_0 = frame_SF.r_0;
  v_0 = der(r_0);
  a_0 = der(a_0);
  connect(topPlate.frame_b, frame_EC) annotation(Line(visible = true, origin = {78.726, 0}, points = {{-70.274, 0}, {70.274, 0}}, color = {95, 95, 95}));
  connect(topPlate.frame_a, frame_SF) annotation(Line(visible = true, origin = {-80.274, 0}, points = {{68.726, 0}, {-68.726, 0}}, color = {95, 95, 95}));
  connect(topPlate.frame_a, angledPropPosition.frame_a) annotation(Line(visible = true, origin = {-8.134, -25.8}, points = {{-3.415, 25.8}, {-6.427, 25.8}, {-6.427, -16.2}, {8.134, -16.2}, {8.134, -19.2}}, color = {95, 95, 95}));
  connect(angledPropPosition.frame_b, frame_propA) annotation(Line(visible = true, origin = {0, -84.5}, points = {{0, 19.5}, {0, -19.5}}, color = {95, 95, 95}));
  connect(fixedTranslation.frame_a, topPlate.frame_a) annotation(Line(visible = true, origin = {-94.387, -18.636}, points = {{-25.613, -18.636}, {-28.613, -18.636}, {-28.613, 18.636}, {82.838, 18.636}}, color = {95, 95, 95}));
  connect(fixedTranslation.frame_b, fairing.frame_a) annotation(Line(visible = true, origin = {-65.932, -37.292}, points = {{-34.068, 0.02}, {10.352, 0.02}, {10.352, -0.02}, {13.364, -0.02}}, color = {95, 95, 95}));
  annotation(Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}), graphics = {Polygon(visible = true, origin = {5.031, 9.008}, fillColor = {0, 0, 255}, fillPattern = FillPattern.Vertical, points = {{-39.634, 63.511}, {-39.634, -79.008}, {32.149, -79.008}, {32.149, 30.992}, {14.969, 63.511}}), Line(visible = true, origin = {-78.078, 18.283}, points = {{43.843, -18.283}, {-21.922, -18.283}, {-21.922, 36.566}}, color = {116, 116, 116}, thickness = 5), Line(visible = true, origin = {71.795, 13.804}, points = {{-34.615, -13.804}, {-21.795, -13.804}, {28.205, -13.804}, {28.205, 41.413}}, color = {116, 116, 116}, thickness = 5), Line(visible = true, origin = {0, -75.861}, points = {{0, 5.861}, {0, -5.861}}, pattern = LinePattern.Dot, thickness = 1)}), Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
end TopPiece;
