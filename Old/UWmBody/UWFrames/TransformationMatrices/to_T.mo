within UWmBody.UWFrames.TransformationMatrices;

function to_T "Return transformation matrix T from orientation object R"
  extends Modelica.Icons.Function;
  input TransformationMatrices.Orientation R "Orientation object to rotate frame 1 into frame 2";
  output Real T[3, 3] "Transformation matrix to transform vector from frame 1 to frame 2 (v2=T*v1)";
algorithm
  T := R;
  annotation(Inline = true);
end to_T;
