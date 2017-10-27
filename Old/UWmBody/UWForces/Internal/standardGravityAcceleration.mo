within UWmBody.UWForces.Internal;

function standardGravityAcceleration "Standard gravity fields (no/parallel/point field)"
  extends Modelica.Icons.Function;
  extends UWmBody.UWInterfaces.partialGravityAcceleration;
  import UWmBody.UWTypes.GravityTypes;
  input GravityTypes gravityType "Type of gravity field" annotation(Dialog);
  input Modelica.SIunits.Acceleration g[3] "Constant gravity acceleration, resolved in world frame, if gravityType=UniformGravity" annotation(Dialog);
  input Real mue(unit = "m3/s2") "Field constant of point gravity field, if gravityType=PointGravity" annotation(Dialog);
  //  input GravityTypes gravityType "Type of gravity field" annotation(Dialog);
  // input Modelica.SIunits.Acceleration g[3] "Constant gravity acceleration, resolved in world frame, if gravityType=UniformGravity" annotation(Dialog);
  /* input Modelica.SIunits.Density pho_fluid "Density of fluid surrounding submerged object" annotation(Dialog);
protected
  final parameter Modelica.SIunits.RelativeDensity d_star = pho_fluid / d;
  input UWTypes.Axis dir annotation(Dialog); */
algorithm
  gravity := if gravityType == GravityTypes.UniformGravity then g else if gravityType == GravityTypes.PointGravity then -mue / (r * r) * (r / Modelica.Math.Vectors.length(r)) else zeros(3);
  /* annotation(Inline = true, Documentation(info = "<html>
<p>
This function defines the standard gravity fields for the World object.
</p>

<table border=1 cellspacing=0 cellpadding=2>
<tr><td><b><i>gravityType</i></b></td>
    <td><b><i>gravity [m/s2]</i></b></td>
    <td><b><i>description</i></b></td></tr>
<tr><td>Types.GravityType.NoGravity</td>
    <td>= {0,0,0}</td>
    <td>No gravity</td></tr>

<tr><td>Types.GravityType.UniformGravity</td>
    <td>= g</td>
    <td> Constant parallel gravity field</td></tr>

<tr><td>Types.GravityType.PointGravity</td>
    <td>= -(mue/(r*r))*r/|r|</td>
    <td> Point gravity field with spherical mass</td></tr>
</table>

</html>")); */
end standardGravityAcceleration;
