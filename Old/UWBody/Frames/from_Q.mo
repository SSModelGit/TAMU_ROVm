within UWBody.Frames;

function from_Q "Return orientation object R from quaternion orientation object Q"
  extends Modelica.Icons.Function;
  input Quaternions.Orientation Q "Quaternions orientation object to rotate frame 1 into frame 2";
  input Modelica.SIunits.AngularVelocity w[3] "Angular velocity from frame 2 with respect to frame 1, resolved in frame 2";
  output Orientation R "Orientation object to rotate frame 1 into frame 2";
algorithm
  /*
      T := (2*Q[4]*Q[4] - 1)*identity(3) + 2*([Q[1:3]]*transpose([Q[1:3]]) - Q[4]*
        skew(Q[1:3]));
    */
  R := Orientation([2 * (Q[1] * Q[1] + Q[4] * Q[4]) - 1, 2 * (Q[1] * Q[2] + Q[3] * Q[4]), 2 * (Q[1] * Q[3] - Q[2] * Q[4]); 2 * (Q[2] * Q[1] - Q[3] * Q[4]), 2 * (Q[2] * Q[2] + Q[4] * Q[4]) - 1, 2 * (Q[2] * Q[3] + Q[1] * Q[4]); 2 * (Q[3] * Q[1] + Q[2] * Q[4]), 2 * (Q[3] * Q[2] - Q[1] * Q[4]), 2 * (Q[3] * Q[3] + Q[4] * Q[4]) - 1], w = w);
  annotation(Inline = true);
end from_Q;
