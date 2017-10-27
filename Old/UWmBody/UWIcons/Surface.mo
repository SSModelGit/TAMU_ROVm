within UWmBody.UWIcons;

model Surface "Surface icon"
  annotation(Icon(graphics = {Polygon(visible = true, lineColor = {10, 90, 224}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, points = {{-98, 92}, {28, -8}, {96, 146}, {104, -118}, {-18, -34}, {-52, -130}, {-102, 40}}, smooth = Smooth.Bezier), Text(visible = true, textColor = {64, 64, 64}, extent = {{-150, 100}, {150, 140}}, textString = "%name")}, coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10})), Documentation(info = "<html>
<p>
Model <b>Surface</b> defines a moveable, parametrized surface in 3-dim. space
that is used for animation. This object is specified by:
</p>

<ul>
<li> The surface frame (orientation object \"R\" and origin \"r_0\")
     in which the data is specified.</li>
<li> A set of two parameters, one in u- and one in v-direction,
     that defines the control points. </li>
<li> A time-varying position of each control point with respect to
     the surface frame.</li>
</ul>

<p>
The parameter values (u,v) are given by the ordinal numbers of the
corresponding control point in u- or in v-direction, respectively.
The surface is then defined by the replaceable function \"surfaceCharacteristic\" with the
interface <a href=\"modelica://Modelica.Mechanics.MultiBody.Interfaces.partialSurfaceCharacteristic\">partialSurfaceCharacteristic</a>
that returns the x-, y-, z- coordinate of every control point in form of 3 positional
arrays and an optional color array, if every control point shall have a different
color:
</p>

<pre>
  Real X[nu,nv], Y[nu,nv], Z[nu,nv], colorArray[nu,nv];
</pre>

<p>
Models <a href=\"modelica://Modelica.Mechanics.MultiBody.Visualizers.Torus\">Torus</a>
and <a href=\"modelica://Modelica.Mechanics.MultiBody.Visualizers.VoluminousWheel\">VoluminousWheel</a>
demonstrate how new visualizer objects can be constructed with the Surface model.<br>
The direct usage of the Surface model, as well as of the Torus and the VoluminousWheel models, are demonstrated with example
<a href=\"modelica://Modelica.Mechanics.MultiBody.Examples.Elementary.Surfaces\">Examples.Elementary.Surfaces</a>.
</p>
</html>"));
end Surface;
