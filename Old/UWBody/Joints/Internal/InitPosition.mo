within UWBody.Joints.Internal;

model InitPosition "Internal model to initialize r_rel_a for Joints.FreeMotionScalarInit"
  extends Modelica.Blocks.Icons.Block;
  import SI = Modelica.SIunits;
  import UWBody.Frames;
  input SI.Position r_a_0[3] annotation(Dialog);
  input SI.Position r_b_0[3] annotation(Dialog);
  input Frames.Orientation R_a annotation(Dialog);
  Modelica.Blocks.Interfaces.RealOutput r_rel_a[3](each final quantity = "Length", each final unit = "m") annotation(Placement(transformation(extent = {{100, -10}, {120, 10}})));
equation
  r_b_0 = r_a_0 + Frames.resolve1(R_a, {r_rel_a[1], r_rel_a[2], r_rel_a[3]});
  annotation(Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Text(visible = true, textColor = {64, 64, 64}, extent = {{-88, -12}, {82, 16}}, textString = "r_rel_a")}));
end InitPosition;
