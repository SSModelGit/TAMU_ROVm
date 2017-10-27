within UWmBody.UWTypes;

type GravityTypes = enumeration(NoGravity "No gravity field", UniformGravity "Uniform gravity field", PointGravity "Point gravity field") "Enumeration defining the type of the gravity field" annotation(Documentation(info = "<html>
   <table border=1 cellspacing=0 cellpadding=2>
   <tr><th><b>Types.GravityTypes.</b></th><th><b>Meaning</b></th></tr>
   <tr><td valign=\"top\">NoGravity</td>
       <td valign=\"top\">No gravity field</td></tr>
   
   <tr><td valign=\"top\">UniformGravity</td>
       <td valign=\"top\">Gravity field is described by a vector of constant gravity acceleration</td></tr>
   
   <tr><td valign=\"top\">PointGravity</td>
       <td valign=\"top\">Central gravity field. The gravity acceleration vector is directed to
           the field center and the gravity is proportional to 1/r^2, where
           r is the distance to the field center.</td></tr>
   </table>
   </html>"));
