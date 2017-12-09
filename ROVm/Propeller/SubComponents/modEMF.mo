within ROVm.Propeller.SubComponents;

model modEMF "Electromotoric force (electric/mechanic transformer), controlled by real input"
  import SI = Modelica.SIunits;
  parameter Boolean useSupport = false "= true, if support flange enabled, otherwise implicitly grounded" annotation(Evaluate = true, HideResult = true, choices(checkBox = true));
  SI.Voltage v "Voltage drop between the two pins";
  SI.Current i "Current flowing from positive to negative pin";
  SI.Angle phi "Angle of shaft flange with respect to support (= flange.phi - support.phi)";
  SI.AngularVelocity w "Angular velocity of flange relative to support";
  Modelica.Electrical.Analog.Interfaces.PositivePin p annotation(Placement(transformation(origin = {0, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Electrical.Analog.Interfaces.NegativePin n annotation(Placement(transformation(origin = {0, -100}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.Rotational.Interfaces.Flange_b flange annotation(Placement(transformation(extent = {{90, -10}, {110, 10}})));
  Modelica.Mechanics.Rotational.Interfaces.Support support if useSupport "Support/housing of emf shaft" annotation(Placement(transformation(extent = {{-110, -10}, {-90, 10}})));
  Modelica.Blocks.Interfaces.RealInput kC "External input to transformation coefficient" annotation(Placement(visible = true, transformation(origin = {-97.321, 100}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-80, 100}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
protected
  parameter SI.ElectricalTorqueConstant k(start = 1) "Transformation coefficient";
  Modelica.Mechanics.Rotational.Components.Fixed fixed if not useSupport annotation(Placement(transformation(extent = {{-90, -20}, {-70, 0}})));
  Modelica.Mechanics.Rotational.Interfaces.InternalSupport internalSupport(tau = -flange.tau) annotation(Placement(transformation(extent = {{-90, -10}, {-70, 10}})));
equation
  v = p.v - n.v;
  0 = p.i + n.i;
  i = p.i;
  phi = flange.phi - internalSupport.phi;
  w = der(phi);
  k * kC * w = v;
  flange.tau = -k * kC * i;
  connect(internalSupport.flange, support) annotation(Line(points = {{-80, 0}, {-100, 0}}));
  connect(internalSupport.flange, fixed.flange) annotation(Line(points = {{-80, 0}, {-80, -10}}));
  annotation(defaultComponentName = "emf", Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Rectangle(visible = true, lineColor = {64, 64, 64}, fillColor = {191, 191, 191}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-100, -50}, {-40, -30}}), Rectangle(visible = true, lineColor = {64, 64, 64}, fillColor = {191, 191, 191}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-85, -10}, {-36, 10}}), Line(visible = true, points = {{0, 90}, {0, 40}}, color = {64, 64, 64}), Rectangle(visible = true, lineColor = {64, 64, 64}, fillColor = {191, 191, 191}, fillPattern = FillPattern.HorizontalCylinder, extent = {{35, -10}, {100, 10}}), Ellipse(visible = true, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-40, -40}, {40, 40}}), Line(visible = true, points = {{0, -90}, {0, -40}}, color = {64, 64, 64}), Text(visible = true, textColor = {64, 64, 64}, extent = {{0, -90}, {199, -50}}, textString = "%name"), Text(visible = true, textColor = {160, 160, 164}, extent = {{0, 46}, {189, 80}}, textString = "k=%k"), Line(visible = not useSupport, points = {{-100, -30}, {-40, -30}}, color = {64, 64, 64}), Line(visible = not useSupport, points = {{-70, -30}, {-70, -10}}, color = {64, 64, 64})}), Diagram(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Polygon(points = {{-17, 95}, {-20, 85}, {-23, 95}, {-17, 95}}, lineColor = {160, 160, 164}, fillColor = {160, 160, 164}, fillPattern = FillPattern.Solid), Line(points = {{-20, 110}, {-20, 85}}, color = {160, 160, 164}), Text(extent = {{-40, 110}, {-30, 90}}, lineColor = {160, 160, 164}, textString = "i"), Line(points = {{9, 75}, {19, 75}}, color = {192, 192, 192}), Line(points = {{-20, -110}, {-20, -85}}, color = {160, 160, 164}), Polygon(points = {{-17, -100}, {-20, -110}, {-23, -100}, {-17, -100}}, lineColor = {160, 160, 164}, fillColor = {160, 160, 164}, fillPattern = FillPattern.Solid), Text(extent = {{-40, -110}, {-30, -90}}, lineColor = {160, 160, 164}, textString = "i"), Line(points = {{8, -79}, {18, -79}}, color = {192, 192, 192}), Line(points = {{14, 80}, {14, 70}}, color = {192, 192, 192})}), Documentation(info = "<html>
<p>EMF transforms electrical energy into rotational mechanical energy. It is used as basic building block of an electrical motor. The mechanical connector flange can be connected to elements of the Modelica.Mechanics.Rotational library. flange.tau is the cut-torque, flange.phi is the angle at the rotational connection.</p>
</html>", revisions = "<html>
<ul>
<li><i> 1998   </i>
       by Martin Otter<br> initially implemented<br>
       </li>
</ul>
</html>"));
end modEMF;
