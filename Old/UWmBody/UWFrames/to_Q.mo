within UWmBody.UWFrames;

function to_Q "Return quaternion orientation object Q from orientation object R"
  extends Modelica.Icons.Function;
  input Orientation R "Orientation object to rotate frame 1 into frame 2";
  input Quaternions.Orientation Q_guess = Quaternions.nullRotation() "Guess value for output Q (there are 2 solutions; the one closer to Q_guess is used";
  output Quaternions.Orientation Q "Quaternions orientation object to rotate frame 1 into frame 2";
algorithm
  Q := Quaternions.from_T(R.T, Q_guess);
  annotation(Inline = true);
end to_Q;
