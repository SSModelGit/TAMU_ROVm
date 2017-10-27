within UWBody.Joints.Internal;

model InitAngularVelocity "Internal model to initialize w_rel_b for Joints.FreeMotionScalarInit"
  extends Modelica.Blocks.Icons.Block;
  import SI = Modelica.SIunits;
  import UWBody.Frames;
  input Frames.Orientation R_a annotation(Dialog);
  input Frames.Orientation R_b annotation(Dialog);
  Modelica.Blocks.Interfaces.RealOutput w_rel_b[3](each final quantity = "AngularVelocity", each final unit = "rad/s") annotation(Placement(transformation(extent = {{100, -10}, {120, 10}})));
equation
  Frames.angularVelocity2(R_b) = Frames.resolve2(R_b, Frames.angularVelocity1(R_a)) + w_rel_b;
  annotation(Icon(graphics = {Text(visible = true, textColor = {64, 64, 64}, extent = {{-86, -12}, {84, 16}}, textString = "w_rel_b")}, coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10})));
end InitAngularVelocity;
