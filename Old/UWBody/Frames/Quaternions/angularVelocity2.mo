within UWBody.Frames.Quaternions;

function angularVelocity2 "Compute angular velocity resolved in frame 2 from quaternions orientation object and its derivative"
  extends Modelica.Icons.Function;
  input Quaternions.Orientation Q "Quaternions orientation object to rotate frame 1 into frame 2";
  input der_Orientation der_Q "Derivative of Q";
  output Modelica.SIunits.AngularVelocity w[3] "Angular velocity of frame 2 with respect to frame 1 resolved in frame 2";
algorithm
  w := 2 * ([Q[4], Q[3], -Q[2], -Q[1]; -Q[3], Q[4], Q[1], -Q[2]; Q[2], -Q[1], Q[4], -Q[3]] * der_Q);
  annotation(Inline = true);
end angularVelocity2;
