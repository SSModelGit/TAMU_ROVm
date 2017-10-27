within UWBody.Visualizers;

model FixedShape2 "Visualizing an elementary shape with dynamically varying shape attributes (has two frame connectors)"
  import UWBody.Frames;
  import UWBody.Types;
  import Modelica.SIunits.Conversions.to_unit1;
  Interfaces.Frame_a frame_a "Coordinate system a (all shape definition vectors are resolved in this frame)" annotation(Placement(transformation(extent = {{-116, -16}, {-84, 16}})));
  Interfaces.Frame_b frame_b "Coordinate system b" annotation(Placement(transformation(extent = {{84, -16}, {116, 16}})));
  parameter Boolean animation = true "= true, if animation shall be enabled";
  parameter Types.ShapeType shapeType = "box" "Type of shape" annotation(Dialog(group = "if animation = true", enable = animation));
  input SI.Position r[3] = {1, 0, 0} "Vector from frame_a to frame_b resolved in frame_a";
  input SI.Position r_shape[3] = {0, 0, 0} "Vector from frame_a to shape origin, resolved in frame_a" annotation(Dialog(group = "if animation = true", enable = animation));
  input Types.Axis lengthDirection = to_unit1(r - r_shape) "Vector in length direction of shape, resolved in frame_a" annotation(Evaluate = true, Dialog(group = "if animation = true", enable = animation));
  input Types.Axis widthDirection = {0, 1, 0} "Vector in width direction of shape, resolved in frame_a" annotation(Evaluate = true, Dialog(group = "if animation = true", enable = animation));
  input SI.Length length = Modelica.Math.Vectors.length(r - r_shape) "Length of shape" annotation(Dialog(group = "if animation = true", enable = animation));
  input SI.Distance width = 0.1 "Width of shape" annotation(Dialog(group = "if animation = true", enable = animation));
  input SI.Distance height = width "Height of shape" annotation(Dialog(group = "if animation = true", enable = animation));
  input Types.ShapeExtra extra = 0.0 "Additional data for cylinder, cone, pipe, gearwheel and spring" annotation(Dialog(group = "if animation = true", enable = animation));
  input Types.Color color = {0, 128, 255} "Color of shape" annotation(Dialog(colorSelector = true, group = "if animation = true", enable = animation));
  input Types.SpecularCoefficient specularCoefficient = world.defaultSpecularCoefficient "Reflection of ambient light (= 0: light is completely absorbed)" annotation(Dialog(group = "if animation = true", enable = animation));
protected
  outer UWBody.World world;
  Advanced.Shape shape(shapeType = shapeType, r_shape = r_shape, lengthDirection = lengthDirection, widthDirection = widthDirection, length = length, width = width, height = height, extra = extra, color = color, specularCoefficient = specularCoefficient, r = frame_a.r_0, R = frame_a.R) if world.enableAnimation and animation;
equation
  Connections.branch(frame_a.R, frame_b.R);
  assert(cardinality(frame_a) > 0 or cardinality(frame_b) > 0, "Neither connector frame_a nor frame_b of
    UWBody.Visualizers.FixedShape2 object is connected");
  frame_b.r_0 = frame_a.r_0 + Frames.resolve1(frame_a.R, r);
  frame_b.R = frame_a.R;
  /* Force and torque balance */
  zeros(3) = frame_a.f + frame_b.f;
  zeros(3) = frame_a.t + frame_b.t + cross(r, frame_b.f);
  annotation(Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {5, 5}), graphics = {Text(visible = true, textColor = {64, 64, 64}, extent = {{-150, 75}, {150, 115}}, textString = "%name"), Text(visible = true, textColor = {64, 64, 64}, extent = {{-150, -90}, {150, -60}}, textString = "r=%r"), Polygon(visible = true, lineColor = {255, 255, 255}, fillColor = {71, 152, 255}, fillPattern = FillPattern.Solid, points = {{-100, 50}, {-100, -44}, {-35, -25}, {75, -60}, {75, 50}, {-35, 30}, {-100, 50}}), Polygon(visible = true, lineColor = {255, 255, 255}, fillColor = {156, 203, 255}, fillPattern = FillPattern.Solid, points = {{-100, 50}, {-75, 70}, {-10, 50}, {100, 70}, {75, 50}, {-35, 30}}), Text(visible = true, textColor = {64, 64, 64}, extent = {{-86, -10}, {-50, 15}}, textString = "a"), Text(visible = true, textColor = {64, 64, 64}, extent = {{37, -10}, {73, 15}}, textString = "b"), Polygon(visible = true, lineColor = {255, 255, 255}, fillColor = {10, 90, 224}, fillPattern = FillPattern.Solid, points = {{100, 70}, {75, 50}, {75, -60}, {100, -40}, {100, 70}})}), Documentation(info = "<html>
<p>
Model <b>FixedShape2</b> defines a visual shape that is
shown at the location of its frame_a. This model is identical
to <b>FixedShape</b> with the only difference that an
additional frame_b is present which is parallel to frame_a.
This makes it more convenient to connect several visual
shapes together when building up more complex visual
objects. All describing data such as size and color can vary dynamically by
providing appropriate expressions in the input fields of the
parameter menu. The only exception is parameter shapeType
that cannot be changed during simulation.
The following shapes are currently supported via
parameter <b>shapeType</b> (e.g., shapeType=\"box\"):<br>&nbsp;
</p>

<p>
<IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Shape.png\" ALT=\"model Visualizers.FixedShape2\">
</p>

<p>&nbsp;<br>
The dark blue arrows in the figure above are directed along
variable <b>lengthDirection</b>. The light blue arrows are directed
along variable <b>widthDirection</b>. The <b>coordinate systems</b>
in the figure represent frame_a of the FixedShape component.
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
<a href=\"modelica://UWBody.Types.Color\">MultiBody.Types.Color</a> contains a menu
definition of the colors used in the UWBody library together with a color editor.
</p>
<p>
In the following figure the relationships between
frame_a and frame_b are shown. The origin of frame_b
with respect to frame_a is specified via parameter
vector <b>r</b>.
</p>

<p>
<IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/FixedTranslation.png\" ALT=\"Parts.FixedTranslation\">
</p>
</html>"));
end FixedShape2;
