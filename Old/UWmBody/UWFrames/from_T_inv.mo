within UWmBody.UWFrames;

function from_T_inv "Return orientation object R from inverse transformation matrix T_inv"
  extends Modelica.Icons.Function;
  input Real T_inv[3, 3] "Inverse transformation matrix to transform vector from frame 2 to frame 1 (v1=T_inv*v2)";
  input Modelica.SIunits.AngularVelocity w[3] "Angular velocity from frame 1 with respect to frame 2, resolved in frame 1 (skew(w)=T_inv*der(transpose(T_inv)))";
  output Orientation R "Orientation object to rotate frame 1 into frame 2";
algorithm
  R := Orientation(T = transpose(T_inv), w = -w);
  annotation(Inline = true);
end from_T_inv;
