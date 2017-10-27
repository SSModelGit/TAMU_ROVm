within UWBody.Frames;

function resolve1 "Transform vector from frame 2 to frame 1"
  extends Modelica.Icons.Function;
  input Orientation R "Orientation object to rotate frame 1 into frame 2";
  input Real v2[3] "Vector in frame 2";
  output Real v1[3] "Vector in frame 1";
algorithm
  v1 := transpose(R.T) * v2;
  annotation(derivative(noDerivative = R) = Internal.resolve1_der, InlineAfterIndexReduction = true, Documentation(info = "<html>
<h4>Syntax</h4>
<blockquote><pre>
v1 = Frames.<b>resolve1</b>(R12, v2);
</pre></blockquote>

<h4>Description</h4>
<p>
The function call <code>Frames.<b>resolve1</b>(R12, v2)</code> returns vector v
resolved in frame 1 (= v1) from vector v resolved in frame 2 (= v2) using the
orientation object R12 that describes the orientation to rotate frame 1 into frame 2.
</p>
</html>"));
end resolve1;
