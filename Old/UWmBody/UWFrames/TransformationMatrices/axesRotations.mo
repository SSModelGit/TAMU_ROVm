within UWmBody.UWFrames.TransformationMatrices;

function axesRotations "Return rotation object to rotate in sequence around 3 axes"
  extends Modelica.Icons.Function;
  input Integer sequence[3](min = {1, 1, 1}, max = {3, 3, 3}) = {1, 2, 3} "Sequence of rotations from frame 1 to frame 2 along axis sequence[i]";
  input Modelica.SIunits.Angle angles[3] = {0, 0, 0} "Rotation angles around the axes defined in 'sequence'";
  output TransformationMatrices.Orientation T "Orientation object to rotate frame 1 into frame 2";
algorithm
  T := absoluteRotation(absoluteRotation(axisRotation(sequence[1], angles[1]), axisRotation(sequence[2], angles[2])), axisRotation(sequence[3], angles[3]));
  annotation(Inline = true);
end axesRotations;
