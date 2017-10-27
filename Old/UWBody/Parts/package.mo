within UWBody;

package Parts "Rigid components such as bodies with mass and inertia and massless rods"
  extends Modelica.Icons.Package;
  annotation(Documentation(info = "<html>
<p>
Package <b>Parts</b> contains <b>rigid components</b> of a
multi-body system. These components may be used to build up
more complicated structures. For example, a part may be built up of
a \"Body\" and of several \"FixedTranslation\" components.
</p>
<h4>Content</h4>
<table border=1 cellspacing=0 cellpadding=2>
  <tr><th><b><i>Model</i></b></th><th><b><i>Description</i></b></th></tr>
  <tr><td valign=\"top\"><a href=\"modelica://UWBody.Parts.Fixed\">Fixed</a></td>
      <td valign=\"top\">Frame fixed in world frame at a given position.
          It is visualized with a shape, see <b>shapeType</b> below
         (the frames on the two
          sides do not belong to the component):<br>&nbsp;<br>
      <IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Fixed.png\" ALT=\"model Parts.Fixed\">
      </td>
  </tr>
  <tr><td valign=\"top\"><a href=\"modelica://UWBody.Parts.FixedTranslation\">FixedTranslation</a></td>
      <td valign=\"top\">Fixed translation of frame_b with respect to frame_a.
          It is visualized with a shape, see <b>shapeType</b> below
          (the frames on the two sides do not belong to the component):<br>&nbsp;<br>
      <IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/FixedTranslation.png\" ALT=\"model Parts.FixedTranslation\">
      </td>
  </tr>
  <tr><td valign=\"top\"><a href=\"modelica://UWBody.Parts.FixedRotation\">FixedRotation</a></td>
      <td valign=\"top\">Fixed translation and fixed rotation of frame_b with respect to frame_a
          It is visualized with a shape, see <b>shapeType</b>  below
          (the frames on the two sides do not belong to the component):<br>&nbsp;<br>
      <IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/FixedRotation.png\" ALT=\"model Parts.FixedRotation\">
      </td>
  </tr>
  <tr><td valign=\"top\"><a href=\"modelica://UWBody.Parts.Body\">Body</a></td>
      <td valign=\"top\">Rigid body with mass, inertia tensor and one frame connector.
          It is visualized with a cylinder and a sphere at the
          center of mass:<br>&nbsp;<br>
      <IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Body.png\" ALT=\"model Parts.Body\">
      </td>
  </tr>
  <tr><td valign=\"top\"><a href=\"modelica://UWBody.Parts.BodyShape\">BodyShape</a></td>
      <td valign=\"top\">Rigid body with mass, inertia tensor, different shapes
          (see <b>shapeType</b> below)
          for animation, and two frame connectors:<br>&nbsp;<br>
      <IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/BodyShape.png\" ALT=\"model Parts.BodyShape\">
      </td>
  </tr>
  <tr><td valign=\"top\"><a href=\"modelica://UWBody.Parts.Fixed\">Fixed BodyBox</a></td>
      <td valign=\"top\">Rigid body with box shape (mass and animation properties are computed
          from box data and from density):<br>&nbsp;<br>
      <IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/BodyBox.png\" ALT=\"model Parts.BodyBox\">
      </td>
  </tr>
  <tr><td valign=\"top\"><a href=\"modelica://UWBody.Parts.BodyCylinder\">BodyCylinder</a></td>
      <td valign=\"top\">Rigid body with cylinder shape (mass and animation properties
          are computed from cylinder data and from density):<br>&nbsp;<br>
      <IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/BodyCylinder.png\" ALT=\"model Parts.BodyCylinder\">
      </td>
  </tr>
  <tr><td valign=\"top\"><a href=\"modelica://UWBody.Parts.PointMass\">PointMass</a></td>
      <td valign=\"top\">Rigid body where inertia tensor and rotation is neglected:<br>&nbsp;<br>
      <IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Parts/PointMass.png\" ALT=\"model Parts.PointMass\">
      </td>
  </tr>
  <tr><td valign=\"top\"><a href=\"modelica://UWBody.Parts.Mounting1D\">Mounting1D</a></td>
      <td valign=\"top\"> Propagate 1-dim. support torque to 3-dim. system
      </td>
  </tr>
  <tr><td valign=\"top\"><a href=\"modelica://UWBody.Parts.Rotor1D\">Rotor1D</a></td>
      <td valign=\"top\">1D inertia attachable on 3-dim. bodies (without neglecting dynamic effects)<br>
      <IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Parts/Rotor1D.png\" ALT=\"model Parts.Rotor1D\">
      </td>
  </tr>
  <tr><td valign=\"top\"><a href=\"modelica://UWBody.Parts.BevelGear1D\">BevelGear1D</a></td>
      <td valign=\"top\">1D gearbox with arbitrary shaft directions (3D bearing frame)
      </td>
  </tr>
</table>
<p>
Components <b>Fixed</b>, <b>FixedTranslation</b>, <b>FixedRotation</b>
and <b>BodyShape</b> are visualized according to parameter
<b>shapeType</b>, that may have the following values (e.g., shapeType = \"box\"): <br>&nbsp;<br>
</p>
<IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/FixedShape.png\" ALT=\"model Visualizers.FixedShape\">
<p>
All the details of the visualization shape parameters are
given in
<a href=\"modelica://UWBody.Visualizers.FixedShape\">Visualizers.FixedShape</a>
</p>
<p>
Colors in all animation parts are defined via parameter <b>color</b>.
This is an Integer vector with 3 elements, {r, g, b}, and specifies the
color of the shape. {r,g,b} are the \"red\", \"green\" and \"blue\" color parts,
given in the ranges 0 .. 255, respectively. The predefined type
<b>MultiBody.Types.Color</b> contains a menu
definition of the colors used in the MultiBody library
(this will be replaced by a color editor).
</p>
</html>"), Icon(graphics = {Rectangle(extent = {{-80, 28}, {2, -16}}, lineColor = {95, 95, 95}, fillPattern = FillPattern.HorizontalCylinder, fillColor = {215, 215, 215}, radius = 10), Ellipse(extent = {{-8, 52}, {86, -42}}, lineColor = {95, 95, 95}, fillPattern = FillPattern.Sphere, fillColor = {215, 215, 215})}));
end Parts;
