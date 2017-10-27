within UWmBody.UWSensors.Internal;

model ZeroForceAndTorque "Set force and torque to zero"
  extends Modelica.Blocks.Icons.Block;
  UWInterfaces.Frame_a frame_a annotation(Placement(transformation(extent = {{-116, -16}, {-84, 16}})));
equation
  frame_a.f = zeros(3);
  frame_a.t = zeros(3);
  annotation(Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Text(visible = true, textColor = {64, 64, 64}, extent = {{-74, -20}, {80, 24}}, textString = "f = t = 0")}));
end ZeroForceAndTorque;
