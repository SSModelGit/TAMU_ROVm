within UWmBody.UsersGuide;

class Upgrade "Upgrade from Former Versions"
  extends Modelica.Icons.Information;
  annotation(Documentation(info = "<html>
<p>
If different versions of the MultiBody library are not
compatible to each other, corresponding conversion scripts are
provided. As a result, models build with an older version
of the MultiBody library are automatically converted to the
new version when the model is loaded. The user is prompted
whether automatic conversion shall take place or not.
Problems are not to be expected. Still one should first make
a copy of such a model as backup before the conversion
is performed.
</p>
<p>
<b>Upgrade from ModelicaAdditions.MultiBody</b>
</p>
<p>
There is now also a conversion script from the \"old\"
<b>ModelicaAdditions.MultiBody</b> library to the
\"new\" Modelica.Mechanics.MultiBody library. This script is also
automatically invoked. Since the differences between the \"old\" and the
\"new\" MultiBody library are so large, not everything is
converted and it might be that some pieces have to
be adapted manually. Still, this script is useful, since
many class names, parameters and modifiers are
automatically converted.
</p>
<p>
Components from the following sublibraries
are automatically converted
to the Modelica.Mechanics.MultiBody library:
</p>
<ul>
<li> ModelicaAdditions.MultiBody.Parts</li>
<li> ModelicaAdditions.MultiBody.Joints</li>
<li> ModelicaAdditions.MultiBody.Forces</li>
<li> Part of ModelicaAdditions.MultiBody.Interfaces</li>
</ul>
<p>
Models using the ModelicaAdditions.MultiBody library
that are programmed with <b>equations</b> are only partly converted:
The Frame connectors will be converted to the \"new\"
Frame connectors of the MultiBody library, but the equations
that reference variables of the Frame connectors will
<b>not</b> be converted. For a manual conversion, the following
table might be helpful showing how the <b>variables</b> of the
\"old\" and the \"new\" <b>Frame connectors</b> are
related to each other (resolve2 and angularVelocity2 are
functions from library Modelica.Mechanics.MultiBody.Frames):
</p>
<table border=1 cellspacing=0 cellpadding=2>
<tr><th><b>ModelicaAdditions.MultiBody.<br>
           Interfaces.Frame_a</b></th>
    <th><b>MultiBody.Interfaces.Frame_a</b></th></tr>
<tr>
  <td valign=\"top\">frame_a.<b>r0</b></td>
  <td valign=\"top\">= frame_a.r_0 (is converted)</td>
</tr>
<tr>
  <td valign=\"top\">frame_a.<b>S</b></td>
  <td valign=\"top\">= transpose(frame_a.R)</td>
</tr>
<tr>
  <td valign=\"top\">frame_a.<b>v</b></td>
  <td valign=\"top\">= resolve2(frame_a.R, <b>der</b>(frame_a.r_0))</td>
</tr>
<tr>
  <td valign=\"top\">frame_a.<b>w</b></td>
  <td valign=\"top\">= angularVelocity2(frame_a.R)</td>
</tr>
<tr>
  <td valign=\"top\">frame_a.<b>a</b></td>
  <td valign=\"top\">= resolve2(frame_a.R, <b>der</b>(v_0)); v_0 = der(r_0)</td>
</tr>
<tr>
  <td valign=\"top\">frame_a.<b>z</b></td>
  <td valign=\"top\">= <b>der</b>(w);  w = angulaVelocity2(frame_a.R)</td>
</tr>
<tr>
  <td valign=\"top\">frame_a.<b>f</b></td>
  <td valign=\"top\">= frame_a.f (no conversion needed)</td>
</tr>
<tr>
  <td valign=\"top\">frame_a.<b>t</b></td>
  <td valign=\"top\">= frame_a.t (no conversion needed)</td>
</tr>
</table>
<p>
<b>Upgrade from MultiBody 0.99 (and earlier) to 1.0 (and later)</b>
</p>
<p>
The conversion from MultiBody 0.99 to 1.0 does not work in some rare
cases, where own components are implemented using functions of the
MultiBody.Frames package. In this case, the conversion has to be
performed manually. The changes in 1.0 with regards to 0.99 are:
</p>
<p>
The definition of the Modelica.Mechanics.MultiBody.Frames.Orientation object has changed.
In 0.99 this was just an alias type for a transformation matrix
(now Modelica.Mechanics.MultiBody.Frames.TransformationMatrices.Orientation).
In 1.0 the orientation object is a record holding the
transformation matrix from frame 1 to frame 2 and the angular
velocity of the transformation matrix resolved in frame 2.
The reason is that this allows to compute the angular velocity
in many cases by standard recursive formulas and not by
differentiation of the transformation matrix. This is usually
much more efficient. As a consequence, the following
calls in 0.99 should be changed:
</p>
<pre>
   Frames.angularVelocity1(T,der(T)) -> Frames.angularVelocity1(T)
   Frames.angularVelocity2(T,der(T)) -> Frames.angularVelocity2(T)
   Frames.from_T(T)                  -> Frames.from_T2(T,der(T))
</pre>
</html>"));
end Upgrade;
