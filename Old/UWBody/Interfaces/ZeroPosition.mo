within UWBody.Interfaces;

model ZeroPosition "Set absolute position vector of frame_resolve to a zero vector and the orientation object to a null rotation"
  extends Modelica.Blocks.Icons.Block;
  Interfaces.Frame_resolve frame_resolve annotation(Placement(transformation(extent = {{-116, -16}, {-84, 16}})));
equation
  Connections.root(frame_resolve.R);
  frame_resolve.R = UWBody.Frames.nullRotation();
  frame_resolve.r_0 = zeros(3);
  annotation(Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Text(extent = {{-74, 24}, {80, -20}}, lineColor = {0, 0, 0}, textString = "r = 0")}));
end ZeroPosition;
