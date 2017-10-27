package UWmBody "Library to model 3-dimensional mechanical systems"
  import SI = Modelica.SIunits;
  import Cv = Modelica.SIunits.Conversions;
  import C = Modelica.Constants;
  import Rotational = Modelica.Mechanics.Rotational;
  import Translational = Modelica.Mechanics.Translational;
  annotation(Documentation(info = "<html>
<p>
Library <b>MultiBody</b> is a <b>free</b> Modelica package providing
3-dimensional mechanical components to model in a convenient way
<b>mechanical systems</b>, such as robots, mechanisms, vehicles.
Typical animations generated with this library are shown
in the next figure:
</p>

<p>
<img src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/MultiBody.png\">
</p>

<p>
For an introduction, have especially a look at:
</p>
<ul>
<li> <a href=\"modelica://Modelica.Mechanics.MultiBody.UsersGuide\">MultiBody.UsersGuide</a>
     discusses the most important aspects how to use this library.</li>
<li> <a href=\"modelica://Modelica.Mechanics.MultiBody.Examples\">MultiBody.Examples</a>
     contains examples that demonstrate the usage of this library.</li>
</ul>

<p>
Copyright &copy; 1998-2016, Modelica Association and DLR.
</p>
<p>
<i>This Modelica package is <u>free</u> software and the use is completely at <u>your own risk</u>; it can be redistributed and/or modified under the terms of the Modelica License 2. For license conditions (including the disclaimer of warranty) see <a href=\"modelica://Modelica.UsersGuide.ModelicaLicense2\">Modelica.UsersGuide.ModelicaLicense2</a> or visit <a href=\"https://www.modelica.org/licenses/ModelicaLicense2\"> https://www.modelica.org/licenses/ModelicaLicense2</a>.</i>
</p>
</html>"), Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Rectangle(visible = true, lineColor = {0, 255, 255}, extent = {{-100, -100}, {100, 100}}, radius = 25), Rectangle(visible = true, lineColor = {0, 0, 255}, fillColor = {0, 0, 128}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-100, -100}, {100, 100}}, radius = 25), Polygon(visible = true, lineColor = {0, 128, 0}, fillColor = {0, 128, 0}, fillPattern = FillPattern.Solid, points = {{-58, 76}, {6, 76}, {-26, 50}, {-58, 76}}), Line(visible = true, points = {{-26, 50}, {28, -50}}, color = {255, 255, 0}), Ellipse(visible = true, fillColor = {255, 255, 255}, fillPattern = FillPattern.Sphere, extent = {{-4, -78}, {60, -14}})}));
end UWmBody;
