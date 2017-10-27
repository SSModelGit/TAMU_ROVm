within UWBody.Frames;

function resolveRelative "Transform vector from frame 1 to frame 2 using absolute orientation objects of frame 1 and of frame 2"
  extends Modelica.Icons.Function;
  input Real v1[3] "Vector in frame 1";
  input Orientation R1 "Orientation object to rotate frame 0 into frame 1";
  input Orientation R2 "Orientation object to rotate frame 0 into frame 2";
  output Real v2[3] "Vector in frame 2";
algorithm
  v2 := resolve2(R2, resolve1(R1, v1));
  annotation(derivative(noDerivative = R1, noDerivative = R2) = Internal.resolveRelative_der, InlineAfterIndexReduction = true, Documentation(info = "<html>
<h4>Syntax</h4>
<blockquote><pre>
v2 = Frames.<b>resolveRelative</b>(R1, R2, v1);
</pre></blockquote>

<h4>Description</h4>
<p>
The function call <code>Frames.<b>resolveRelative</b>(R1,R2,v1)</code> returns vector v
resolved in frame 2 (= v1) from vector v resolved in frame 1 (= v1) given the
orientation object R1 that describes the orientation to rotate frame 0 into frame 1 and
orientation object R2 that describes the orientation to rotate frame 0 into frame 2.
</p>
</html>"));
end resolveRelative;
