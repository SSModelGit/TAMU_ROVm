within UWBody.Examples.Systems.RobotR3.Components;

expandable connector ControlBus "Data bus for all axes of robot"
  extends Modelica.Icons.SignalBus;
  UWBody.Examples.Systems.RobotR3.Components.AxisControlBus axisControlBus1 "Bus of axis 1";
  UWBody.Examples.Systems.RobotR3.Components.AxisControlBus axisControlBus2 "Bus of axis 2";
  UWBody.Examples.Systems.RobotR3.Components.AxisControlBus axisControlBus3 "Bus of axis 3";
  UWBody.Examples.Systems.RobotR3.Components.AxisControlBus axisControlBus4 "Bus of axis 4";
  UWBody.Examples.Systems.RobotR3.Components.AxisControlBus axisControlBus5 "Bus of axis 5";
  UWBody.Examples.Systems.RobotR3.Components.AxisControlBus axisControlBus6 "Bus of axis 6";
  annotation(Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(extent = {{-20, 2}, {22, -2}}, lineColor = {255, 204, 51}, lineThickness = 0.5)}), Documentation(info = "<html>
<p>
Signal bus that is used to communicate <b>all signals</b> of the robot.
This is an expandable connector which has a \"default\" set of
signals. Note, the input/output causalities of the signals are
determined from the connections to this bus.
</p>
</html>"));
end ControlBus;
