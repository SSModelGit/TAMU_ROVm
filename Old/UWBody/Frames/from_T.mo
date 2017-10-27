within UWBody.Frames;

function from_T "Return orientation object R from transformation matrix T"
  extends Modelica.Icons.Function;
  input Real T[3, 3] "Transformation matrix to transform vector from frame 1 to frame 2 (v2=T*v1)";
  input Modelica.SIunits.AngularVelocity w[3] "Angular velocity from frame 2 with respect to frame 1, resolved in frame 2 (skew(w)=T*der(transpose(T)))";
  output Orientation R "Orientation object to rotate frame 1 into frame 2";
algorithm
  R := Orientation(T = T, w = w);
  annotation(Inline = true);
end from_T;
