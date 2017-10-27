within UWmBody.UWFrames.TransformationMatrices;

function relativeRotation "Return relative orientation object"
  extends Modelica.Icons.Function;
  input TransformationMatrices.Orientation T1 "Orientation object to rotate frame 0 into frame 1";
  input TransformationMatrices.Orientation T2 "Orientation object to rotate frame 0 into frame 2";
  output TransformationMatrices.Orientation T_rel "Orientation object to rotate frame 1 into frame 2";
algorithm
  T_rel := T2 * transpose(T1);
  annotation(Inline = true);
end relativeRotation;
