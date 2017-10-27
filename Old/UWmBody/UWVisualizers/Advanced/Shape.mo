within UWmBody.UWVisualizers.Advanced;

model Shape "Visualizing an elementary object with variable size; all data have to be set as modifiers (see info layer)"
  extends ModelicaServices.Animation.Shape;
  extends Modelica.Utilities.Internal.PartialModelicaServices.Animation.PartialShape;
  annotation(Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Rectangle(visible = true, lineColor = {255, 255, 255}, fillColor = {71, 152, 255}, fillPattern = FillPattern.Solid, extent = {{-100, -100}, {80, 70}}), Polygon(visible = true, lineColor = {255, 255, 255}, fillColor = {156, 203, 255}, fillPattern = FillPattern.Solid, points = {{-80, 100}, {100, 100}, {80, 70}, {-100, 70}}), Polygon(visible = true, lineColor = {255, 255, 255}, fillColor = {10, 90, 224}, fillPattern = FillPattern.Solid, points = {{100, 100}, {100, -60}, {80, -100}, {80, 70}, {100, 100}}), Text(visible = true, textColor = {64, 64, 64}, extent = {{-100, -42}, {80, 20}}, textString = "%shapeType"), Text(visible = true, textColor = {64, 64, 64}, extent = {{-150, 110}, {150, 150}}, textString = "%name")}), Documentation(info = "<html>
<p>
Model <b>Shape</b> defines a visual shape that is
shown at the location of its reference coordinate system, called
'object frame' below. All describing variables such
as size and color can vary dynamically (with the only exception
of parameter shapeType). The default equations in the
declarations should be modified by providing appropriate modifier
equations. Model <b>Shape</b> is usually used as a basic building block to
implement simpler to use graphical components.
</p>
<p>
The following shapes are supported via
parameter <b>shapeType</b> (e.g., shapeType=\"box\"):<br>&nbsp;
</p>

<p>
<IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Shape.png\" ALT=\"model Visualizers.FixedShape\">
</p>

<p>&nbsp;<br>
The dark blue arrows in the figure above are directed along
variable <b>lengthDirection</b>. The light blue arrows are directed
along variable <b>widthDirection</b>. The <b>coordinate systems</b>
in the figure represent frame_a of the Shape component.
</p>

<p>
Additionally, <b>external shapes</b> can be specified as (not all options might be supported by all tools):
</p>

<ul>
<li> <b>\"1\", \"2\", ...</b><br>
     define external shapes specified in DXF format in files \"1.dxf\", \"2.dxf\", ...
     The DXF-files must be found either in the current directory or in the directory where
     the Shape instance is stored that references the DXF file.
     This (very limited) option should not be used for new models. Example:<br>
    shapeType=\"1\".<br></li>

<li> \"<b>modelica:</b>//&lt;Modelica-name&gt;/&lt;relative-path-file-name&gt;\"<br>
     characterizes the file that is stored under the location of the
     &lt;Modelica-name&gt; library path with the given relative file name.
     Example:<br> shapeType = \"modelica://Modelica/Resources/Data/Shapes/Engine/piston.dxf\".<br></li>

<li> \"<b>file:</b>//&lt;absolute-file-name&gt;\"<br>
     characterizes an absolute file name in the file system. Example:<br>
     shapeType=\"file://C:/users/myname/shapes/piston.dxf\".</li>
</ul>

<p>
The supported file formats are tool dependent. Most tools support at least DXF-files
but may support other format as well (such as stl, obj, 3ds).
Since visualization files contain color and other data, the corresponding
information in the model is usually ignored.
For information about DXF files, see <a href=\"https://en.wikipedia.org/wiki/AutoCAD_DXF\">Wikipedia</a>.
As a default it is assumed that the DXF coordinates are in the \"frame_a\"-system and in meters, and that the 3dfaces are two-sided.
Some tools support only 3dface (for geometry) and layer (for advanced coloring).
</p>

<p>
The sizes of any of the above components are specified by the
<b>length</b>, <b>width</b> and <b>height</b> variables.
Via variable <b>extra</b> additional data can be defined:
</p>
<table border=1 cellspacing=0 cellpadding=2>
<tr><th><b>shapeType</b></th><th>Meaning of parameter <b>extra</b></th></tr>
<tr>
  <td valign=\"top\">\"cylinder\"</td>
  <td valign=\"top\">if extra &gt; 0, a black line is included in the
      cylinder to show the rotation of it.</td>
</tr>
<tr>
  <td valign=\"top\">\"cone\"</td>
  <td valign=\"top\">extra = diameter-left-side / diameter-right-side, i.e.,<br>
      extra = 1: cylinder<br>
      extra = 0: \"real\" cone.</td>
</tr>
<tr>
  <td valign=\"top\">\"pipe\"</td>
  <td valign=\"top\">extra = outer-diameter / inner-diameter, i.e, <br>
      extra = 1: cylinder that is completely hollow<br>
      extra = 0: cylinder without a hole.</td>
</tr>
<tr>
  <td valign=\"top\">\"gearwheel\"</td>
  <td valign=\"top\">extra is the number of teeth of the (external) gear.
If extra &lt; 0, an internal gear is visualized with |extra| teeth.
The axis of the gearwheel is along \"lengthDirection\", and usually:
width = height = 2*radiusOfGearWheel.</td>
</tr>
<tr>
  <td valign=\"top\">\"spring\"</td>
  <td valign=\"top\">extra is the number of windings of the spring.
      Additionally, \"height\" is <b>not</b> the \"height\" but
      2*coil-width.</td>
</tr>
<tr>
  <td valign=\"top\">external shape</td>
  <td valign=\"top\">extra = 0: Visualization from file is not scaled.<br>
                   extra = 1: Visualization from file is scaled with \"length\", \"width\" and height\"
                              of the shape</td>
</tr>
</table>
<p>
Parameter <b>color</b> is a vector with 3 elements,
{r, g, b}, and specifies the color of the shape.
{r,g,b} are the \"red\", \"green\" and \"blue\" color parts.
Note, r g, b are given as Integer[3] in the ranges 0 .. 255,
respectively. The predefined type
<a href=\"modelica://Modelica.Mechanics.MultiBody.Types.Color\">MultiBody.Types.Color</a> contains a menu
definition of the colors used in the MultiBody library together with a color editor.
</p>

<p>
The variables under heading <b>Parameters</b> below
are declared as (time varying) <b>input</b> variables.
If the default equation is not appropriate, a corresponding
modifier equation has to be provided in the
model where a <b>Shape</b> instance is used, e.g., in the form
</p>
<pre>
    Visualizers.Advanced.Shape shape(length = sin(time));
</pre>
</html>"));
end Shape;
