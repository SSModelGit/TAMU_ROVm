within UWmBody.UWExamples.Systems.RobotR3.Components;

expandable connector AxisControlBus "Data bus for one robot axis"
  extends Modelica.Icons.SignalSubBus;
  Boolean motion_ref "= true, if reference motion is not in rest" annotation(HideResult = false);
  Modelica.SIunits.Angle angle_ref "Reference angle of axis flange" annotation(HideResult = false);
  Modelica.SIunits.Angle angle "Angle of axis flange" annotation(HideResult = false);
  SI.AngularVelocity speed_ref "Reference speed of axis flange" annotation(HideResult = false);
  SI.AngularVelocity speed "Speed of axis flange" annotation(HideResult = false);
  SI.AngularAcceleration acceleration_ref "Reference acceleration of axis flange" annotation(HideResult = false);
  SI.AngularAcceleration acceleration "Acceleration of axis flange" annotation(HideResult = false);
  SI.Current current_ref "Reference current of motor" annotation(HideResult = false);
  SI.Current current "Current of motor" annotation(HideResult = false);
  Modelica.SIunits.Angle motorAngle "Angle of motor flange" annotation(HideResult = false);
  SI.AngularVelocity motorSpeed "Speed of motor flange" annotation(HideResult = false);
  annotation(defaultComponentPrefixes = "protected", Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(extent = {{-20, 2}, {22, -2}}, lineColor = {255, 204, 51}, lineThickness = 0.5)}), Documentation(info = "<html>
<p>
Signal bus that is used to communicate all signals for <b>one</b> axis.
This is an expandable connector which has a \"default\" set of
signals. Note, the input/output causalities of the signals are
determined from the connections to this bus.
</p>

</html>"));
end AxisControlBus;
