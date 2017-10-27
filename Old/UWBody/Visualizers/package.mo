within UWBody;

package Visualizers "3-dimensional visual objects used for animation"
  extends Modelica.Icons.Package;
  annotation(Documentation(info = "<html>
<p>
Package <b>Visualizers</b> contains components to visualize
3-dimensional shapes. These components are the basis for the
animation features of the MultiBody library.
</p>
<h4>Content</h4>
<table border=1 cellspacing=0 cellpadding=2>
  <tr><td valign=\"top\"><a href=\"modelica://UWBody.Visualizers.FixedShape\">FixedShape</a><br>
             <a href=\"modelica://UWBody.Visualizers.FixedShape2\">FixedShape2</a></td>
      <td valign=\"top\">Visualizing an elementary shape with dynamically varying shape attributes.
      FixedShape has one connector frame_a, whereas FixedShape2 has additionally
          a frame_b for easier connection to further visual objects.
          The following shape types are supported:<br>&nbsp;<br>
      <IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/FixedShape.png\" ALT=\"model Visualizers.FixedShape\">
      </td>
  </tr>
  <tr><td valign=\"top\"><a href=\"modelica://UWBody.Visualizers.FixedFrame\">FixedFrame</a></td>
      <td valign=\"top\">Visualizing a coordinate system including axes labels with fixed sizes:<br>
      <IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/FixedFrame2.png\"
       ALT=\"model Visualizers.FixedFrame\">
      </td>
  </tr>
  <tr><td valign=\"top\"><a href=\"modelica://UWBody.Visualizers.FixedArrow\">FixedArrow</a>,<br>
<a href=\"modelica://UWBody.Visualizers.SignalArrow\">SignalArrow</a></td>
      <td valign=\"top\">Visualizing an arrow. Model \"FixedArrow\" provides
      a fixed sized arrow, model \"SignalArrow\" provides
      an arrow with dynamically varying length that is defined
      by an input signal vector:<br>
      <IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Visualizers/Arrow.png\">
      </td>
  </tr>

  <tr><td valign=\"top\"><a href=\"modelica://UWBody.Visualizers.Ground\">Ground</a></td>
      <td valign=\"top\">Visualizing the x-y plane by a box:<br>
      <IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Visualizers/GroundSmall.png\">
      </td>
  </tr>

  <tr><td valign=\"top\"><a href=\"modelica://UWBody.Visualizers.Torus\">Torus</a></td>
      <td valign=\"top\">Visualizing a torus:<br>
      <IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Visualizers/TorusIcon.png\">
      </td>
  </tr>

  <tr><td valign=\"top\"><a href=\"modelica://UWBody.Visualizers.VoluminousWheel\">VoluminousWheel</a></td>
      <td valign=\"top\">Visualizing a wheel:<br>
      <IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Visualizers/VoluminousWheelIcon.png\">
      </td>
  </tr>

  <tr><td valign=\"top\"><a href=\"modelica://UWBody.Visualizers.PipeWithScalarField\">PipeWithScalarField</a></td>
      <td valign=\"top\">Visualizing a pipe with a scalar field represented by a color coding:<br>
      <IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Visualizers/PipeWithScalarFieldIcon.png\">
      </td>
  </tr>

<tr><td valign=\"top\"><a href=\"modelica://UWBody.Visualizers.Advanced\">Advanced</a></td>
      <td valign=\"top\"> <b>Package</b> that contains components to visualize
          3-dimensional shapes where all parts of the shape
          can vary dynamically. Basic knowledge of Modelica is
          needed in order to utilize the components of this package.
      </td>
  </tr>
</table>
<p>
The colors of the visualization components are declared with
the predefined type <b>MultiBody.Types.Color</b>.
This is a vector with 3 elements,
{r, g, b}, and specifies the color of the shape.
{r,g,b} are the \"red\", \"green\" and \"blue\" color parts.
Note, r g, b are given as Integer[3] in the ranges 0 .. 255,
respectively.
</p>
</html>"), Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Polygon(visible = true, origin = {4.391, -1}, lineColor = {128, 128, 128}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, points = {{-14.391, 86}, {75.609, 66}, {15.609, 26}, {-84.391, 56}}), Polygon(visible = true, origin = {18.391, -30}, lineColor = {128, 128, 128}, fillColor = {128, 128, 128}, fillPattern = FillPattern.Solid, points = {{61.609, 95}, {1.609, 55}, {1.609, -55}, {61.609, -5}}), Polygon(visible = true, origin = {-11.843, -48.757}, lineColor = {128, 128, 128}, fillColor = {191, 191, 191}, fillPattern = FillPattern.Solid, points = {{31.843, 73.757}, {31.843, -36.243}, {-68.157, 3.757}, {-68.157, 103.757}})}));
end Visualizers;
