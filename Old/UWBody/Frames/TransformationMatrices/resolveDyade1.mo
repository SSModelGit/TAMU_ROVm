within UWBody.Frames.TransformationMatrices;

function resolveDyade1 "Transform second order tensor from frame 2 to frame 1"
  extends Modelica.Icons.Function;
  input TransformationMatrices.Orientation T "Orientation object to rotate frame 1 into frame 2";
  input Real D2[3, 3] "Second order tensor resolved in frame 2";
  output Real D1[3, 3] "Second order tensor resolved in frame 1";
algorithm
  D1 := transpose(T) * D2 * T;
  annotation(Inline = true);
end resolveDyade1;
