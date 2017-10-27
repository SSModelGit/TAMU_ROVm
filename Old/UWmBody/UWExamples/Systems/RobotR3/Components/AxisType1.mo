within UWmBody.UWExamples.Systems.RobotR3.Components;

model AxisType1 "Axis model of the r3 joints 1,2,3"
  extends AxisType2(redeclare GearType1 gear(c = c, d = cd));
  parameter Real c(unit = "N.m/rad") = 43 "Spring constant" annotation(Dialog(group = "Gear"));
  parameter Real cd(unit = "N.m.s/rad") = 0.005 "Damper constant" annotation(Dialog(group = "Gear"));
  annotation(Documentation(info = "<html>
<p>
Model of axis 1, 2, 3 of the robot r3. An axis consists of a gearbox with modelled gear elasticity and bearing friction,
a model of the electrical motor and a continuous-time cascade controller.
</p>
</html>"));
end AxisType1;
