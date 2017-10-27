within UWmBody.UWFrames.TransformationMatrices;

function resolveDyade2 "Transform second order tensor from frame 1 to frame 2"
  extends Modelica.Icons.Function;
  input TransformationMatrices.Orientation T "Orientation object to rotate frame 1 into frame 2";
  input Real D1[3, 3] "Second order tensor resolved in frame 1";
  output Real D2[3, 3] "Second order tensor resolved in frame 2";
algorithm
  D2 := T * D1 * transpose(T);
  annotation(Inline = true);
end resolveDyade2;
