within UWmBody.UWFrames.TransformationMatrices;

function inverseRotation "Return inverse orientation object"
  extends Modelica.Icons.Function;
  input TransformationMatrices.Orientation T "Orientation object to rotate frame 1 into frame 2";
  output TransformationMatrices.Orientation T_inv "Orientation object to rotate frame 2 into frame 1";
algorithm
  T_inv := transpose(T);
  annotation(Inline = true);
end inverseRotation;
