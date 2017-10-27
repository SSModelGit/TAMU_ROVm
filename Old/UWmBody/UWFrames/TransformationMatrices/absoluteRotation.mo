within UWmBody.UWFrames.TransformationMatrices;

function absoluteRotation "Return absolute orientation object from another absolute and a relative orientation object"
  extends Modelica.Icons.Function;
  input TransformationMatrices.Orientation T1 "Orientation object to rotate frame 0 into frame 1";
  input TransformationMatrices.Orientation T_rel "Orientation object to rotate frame 1 into frame 2";
  output TransformationMatrices.Orientation T2 "Orientation object to rotate frame 0 into frame 2";
algorithm
  T2 := T_rel * T1;
  annotation(Inline = true);
end absoluteRotation;
