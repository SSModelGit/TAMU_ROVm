within UWmBody.UWFrames.Internal;

function resolve2_der "Derivative of function Frames.resolve2(..)"
  import Frames = UWmBody.UWFrames;
  extends Modelica.Icons.Function;
  input Orientation R "Orientation object to rotate frame 1 into frame 2";
  input Real v1[3] "Vector resolved in frame 1";
  input Real v1_der[3] "= der(v1)";
  output Real v2_der[3] "Derivative of vector v resolved in frame 2";
algorithm
  v2_der := Frames.resolve2(R, v1_der) - cross(R.w, Frames.resolve2(R, v1));
  annotation(Inline = true);
end resolve2_der;
