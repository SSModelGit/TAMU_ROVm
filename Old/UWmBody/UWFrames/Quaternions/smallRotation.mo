within UWmBody.UWFrames.Quaternions;

function smallRotation "Return rotation angles valid for a small rotation"
  extends Modelica.Icons.Function;
  input Quaternions.Orientation Q "Quaternions orientation object to rotate frame 1 into frame 2";
  output Modelica.SIunits.Angle phi[3] "The rotation angles around x-, y-, and z-axis of frame 1 to rotate frame 1 into frame 2 for a small relative rotation";
algorithm
  phi := 2 * {Q[1], Q[2], Q[3]};
  annotation(Inline = true);
end smallRotation;
