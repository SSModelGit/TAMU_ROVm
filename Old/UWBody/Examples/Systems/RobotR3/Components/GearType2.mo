within UWBody.Examples.Systems.RobotR3.Components;

model GearType2 "Motor inertia and gearbox model for r3 joints 4,5,6"
  extends Modelica.Mechanics.Rotational.Interfaces.PartialTwoFlanges;
  parameter Real i = -99 "Gear ratio";
  parameter SI.Torque Rv0 = 21.8 "Viscous friction torque at zero velocity";
  parameter Real Rv1 = 9.8 "Viscous friction coefficient in [Nms/rad] (R=Rv0+Rv1*abs(qd))";
  parameter Real peak = 26.7 / 21.8 "Maximum static friction torque is peak*Rv0 (peak >= 1)";
  constant SI.AngularVelocity unitAngularVelocity = 1;
  constant SI.Torque unitTorque = 1;
  Modelica.Mechanics.Rotational.Components.IdealGear gear(ratio = i, useSupport = false) annotation(Placement(transformation(extent = {{-28, -10}, {-8, 10}})));
  Modelica.Mechanics.Rotational.Components.BearingFriction bearingFriction(tau_pos = [0, Rv0 / unitTorque; 1, (Rv0 + Rv1 * unitAngularVelocity) / unitTorque], peak = peak, useSupport = false) annotation(Placement(transformation(extent = {{30, -10}, {50, 10}})));
equation
  connect(gear.flange_b, bearingFriction.flange_a) annotation(Line(points = {{-8, 0}, {30, 0}}, color = {128, 128, 128}, thickness = 0.5));
  connect(bearingFriction.flange_b, flange_b) annotation(Line(points = {{50, 0}, {100, 0}}, color = {128, 128, 128}, thickness = 0.5));
  connect(gear.flange_a, flange_a) annotation(Line(points = {{-28, 0}, {-100, 0}}, color = {128, 128, 128}, thickness = 0.5));
  annotation(Documentation(info = "<html>
<p>
The elasticity and damping in the gearboxes of the outermost
three joints of the robot is neglected.
Default values for all parameters are given for joint 4.
</p>
</html>"), Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Rectangle(visible = true, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-90, -10}, {-60, 10}}), Polygon(visible = true, fillColor = {192, 192, 192}, fillPattern = FillPattern.HorizontalCylinder, points = {{-60, 10}, {-60, 20}, {-40, 40}, {-40, -40}, {-60, -20}, {-60, 10}}), Rectangle(visible = true, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-40, -60}, {40, 60}}, radius = 10), Polygon(visible = true, fillColor = {192, 192, 192}, fillPattern = FillPattern.HorizontalCylinder, points = {{60, 20}, {40, 40}, {40, -40}, {60, -20}, {60, 20}}), Rectangle(visible = true, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, fillPattern = FillPattern.HorizontalCylinder, extent = {{60, -10}, {90, 10}}), Polygon(visible = true, fillColor = {64, 64, 64}, fillPattern = FillPattern.Solid, points = {{-60, -90}, {-50, -90}, {-20, -30}, {20, -30}, {48, -90}, {60, -90}, {60, -100}, {-60, -100}, {-60, -90}}), Text(visible = true, textColor = {64, 64, 64}, extent = {{-0, 68}, {0, 108}}, textString = "%name"), Text(visible = true, textColor = {10, 90, 224}, extent = {{-36, -30}, {38, 40}}, textString = "2")}));
end GearType2;
