within UWmBody.UWExamples.Systems.RobotR3.Components;

model GearType1 "Motor inertia and gearbox model for r3 joints 1,2,3"
  extends Modelica.Mechanics.Rotational.Interfaces.PartialTwoFlanges;
  parameter Real i = -105 "Gear ratio";
  parameter Real c(unit = "N.m/rad") = 43 "Spring constant";
  parameter Real d(unit = "N.m.s/rad") = 0.005 "Damper constant";
  parameter SI.Torque Rv0 = 0.4 "Viscous friction torque at zero velocity";
  parameter Real Rv1(unit = "N.m.s/rad") = 0.13 / 160 "Viscous friction coefficient (R=Rv0+Rv1*abs(qd))";
  parameter Real peak = 1 "Maximum static friction torque is peak*Rv0 (peak >= 1)";
  SI.AngularAcceleration a_rel = der(spring.w_rel) "Relative angular acceleration of spring";
  constant SI.AngularVelocity unitAngularVelocity = 1;
  constant SI.Torque unitTorque = 1;
  Modelica.Mechanics.Rotational.Components.IdealGear gear(ratio = i, useSupport = false) annotation(Placement(transformation(extent = {{50, -10}, {70, 10}})));
  Modelica.Mechanics.Rotational.Components.SpringDamper spring(c = c, d = d) annotation(Placement(transformation(extent = {{0, -10}, {20, 10}}, origin = {0, 0}, rotation = 0), visible = true));
  Modelica.Mechanics.Rotational.Components.BearingFriction bearingFriction(tau_pos = [0, Rv0 / unitTorque; 1, (Rv0 + Rv1 * unitAngularVelocity) / unitTorque], useSupport = false) annotation(Placement(transformation(extent = {{-60, -10}, {-40, 10}}, origin = {0, 0}, rotation = 0), visible = true));
equation
  connect(spring.flange_b, gear.flange_a) annotation(Line(points = {{-15, 0}, {15, 0}}, color = {128, 128, 128}, thickness = 0.5, visible = true, origin = {35, 0}));
  connect(bearingFriction.flange_b, spring.flange_a) annotation(Line(points = {{-20, 0}, {20, 0}}, color = {128, 128, 128}, thickness = 0.5, visible = true, origin = {-20, 0}));
  connect(gear.flange_b, flange_b) annotation(Line(points = {{70, 0}, {100, 0}}, color = {128, 128, 128}, thickness = 0.5));
  connect(bearingFriction.flange_a, flange_a) annotation(Line(points = {{20, 0}, {-20, 0}}, color = {128, 128, 128}, thickness = 0.5, visible = true, origin = {-80, 0}));
initial equation
  spring.w_rel = 0;
  a_rel = 0;
  annotation(Documentation(info = "<html>
<p>
Models the gearbox used in the first three joints with all its effects,
like elasticity and friction.
Coulomb friction is approximated by a friction element acting
at the \"motor\"-side. In reality, bearing friction should be
also incorporated at the driven side of the gearbox. However,
this would require considerable more effort for the measurement
of the friction parameters.
Default values for all parameters are given for joint 1.
Model relativeStates is used to define the relative angle
and relative angular velocity across the spring (=gear elasticity)
as state variables. The reason is, that a default initial
value of zero of these states makes always sense.
If the absolute angle and the absolute angular velocity of model
Jmotor would be used as states, and the load angle (= joint angle of
robot) is NOT zero, one has always to ensure that the initial values
of the motor angle and of the joint angle are modified correspondingly.
Otherwise, the spring has an unrealistic deflection at initial time.
Since relative quantities are used as state variables, this simplifies
the definition of initial values considerably.
</p>
</html>"), Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Rectangle(visible = true, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-90, -10}, {-60, 10}}), Polygon(visible = true, fillColor = {192, 192, 192}, fillPattern = FillPattern.HorizontalCylinder, points = {{-60, 10}, {-60, 20}, {-40, 40}, {-40, -40}, {-60, -20}, {-60, 10}}), Rectangle(visible = true, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-40, -60}, {40, 60}}, radius = 10), Polygon(visible = true, fillColor = {192, 192, 192}, fillPattern = FillPattern.HorizontalCylinder, points = {{60, 20}, {40, 40}, {40, -40}, {60, -20}, {60, 20}}), Rectangle(visible = true, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, fillPattern = FillPattern.HorizontalCylinder, extent = {{60, -10}, {90, 10}}), Polygon(visible = true, fillColor = {64, 64, 64}, fillPattern = FillPattern.Solid, points = {{-60, -90}, {-50, -90}, {-20, -30}, {20, -30}, {48, -90}, {60, -90}, {60, -100}, {-60, -100}, {-60, -90}}), Text(visible = true, textColor = {64, 64, 64}, extent = {{-0, 68}, {0, 108}}, textString = "%name"), Text(visible = true, textColor = {10, 90, 224}, extent = {{-36, -30}, {36, 40}}, textString = "1")}), Diagram(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Text(extent = {{72, 30}, {130, 22}}, lineColor = {0, 0, 0}, textString = "flange of joint axis"), Text(extent = {{-128, 26}, {-70, 18}}, lineColor = {0, 0, 0}, textString = "flange of motor axis")}));
end GearType1;
