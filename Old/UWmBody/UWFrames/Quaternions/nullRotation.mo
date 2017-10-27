within UWmBody.UWFrames.Quaternions;

function nullRotation "Return quaternion orientation object that does not rotate a frame"
  extends Modelica.Icons.Function;
  output Quaternions.Orientation Q "Quaternions orientation object to rotate frame 1 into frame 2";
algorithm
  Q := {0, 0, 0, 1};
  annotation(Inline = true);
end nullRotation;
