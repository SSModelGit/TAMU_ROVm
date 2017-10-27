within UWmBody.UWFrames;

function resolve2 "Transform vector from frame 1 to frame 2"
  extends Modelica.Icons.Function;
  input Orientation R "Orientation object to rotate frame 1 into frame 2";
  input Real v1[3] "Vector in frame 1";
  output Real v2[3] "Vector in frame 2";
algorithm
  v2 := R.T * v1;
  annotation(derivative(noDerivative = R) = Internal.resolve2_der, InlineAfterIndexReduction = true, Documentation(info = "<html>
<h4>Syntax</h4>
<blockquote><pre>
v2 = Frames.<b>resolve2</b>(R12, v1);
</pre></blockquote>

<h4>Description</h4>
<p>
The function call <code>Frames.<b>resolve2</b>(R12, v1)</code> returns vector v
resolved in frame 2 (= v2) from vector v resolved in frame 1 (= v1) using the
orientation object R12 that describes the orientation  to rotate frame 1 into frame 2.
</p>
</html>"));
end resolve2;
