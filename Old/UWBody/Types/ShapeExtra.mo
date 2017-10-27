within UWBody.Types;

type ShapeExtra = Modelica.Icons.TypeReal "Type of the additional data that can be defined for an elementary ShapeType" annotation(Documentation(info = "<html>
   <p>
   This type is used in shapes of visual objects to define
   extra data depending on the shape type. Usually, input
   variable <b>extra</b> is used as instance name:
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
   
   </html>"));
