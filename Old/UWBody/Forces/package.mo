within UWBody;

package Forces "Components that exert forces and/or torques between frames"
  extends Modelica.Icons.SourcesPackage;
  annotation(Documentation(info = "<html>
<p>
This package contains components that exert forces and torques
between two frame connectors, e.g., between two parts.
</p>
<h4>Content</h4>
<table border=1 cellspacing=0 cellpadding=2>
  <tr><th><b><i>Model</i></b></th><th><b><i>Description</i></b></th></tr>
  <tr><td valign=\"top\"><a href=\"modelica://UWBody.Forces.WorldForce\">WorldForce</a></td>
      <td valign=\"top\"> External force acting at the frame to which this component
           is connected and defined by 3 input signals,
           that are interpreted as one vector resolved in frame world, frame_b or frame_resolve. <br>
           <IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Forces/ArrowForce.png\"></td>
  </tr>
  <tr><td valign=\"top\"><a href=\"modelica://UWBody.Forces.WorldTorque\">WorldTorque</a></td>
      <td valign=\"top\"> External torque acting at the frame to which this component
           is connected and defined by 3 input signals,
           that are interpreted as one vector resolved in frame world, frame_b or frame_resolve. <br>
           <IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Forces/ArrowTorque.png\"></td>
  </tr>
  <tr><td valign=\"top\"><a href=\"modelica://UWBody.Forces.WorldForceAndTorque\">WorldForceAndTorque</a></td>
      <td valign=\"top\"> External force and external torque acting at the frame
           to which this component
           is connected and defined by 3+3 input signals,
           that are interpreted as a force and as a torque vector
           resolved in frame world, frame_b or frame_resolve. <br>
           <IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Forces/ArrowForce.png\"><br>
           <IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Forces/ArrowTorque.png\">
      </td>
  </tr>
  <tr><td valign=\"top\"><a href=\"modelica://UWBody.Forces.Force\">Force</a></td>
      <td valign=\"top\"> Force acting between two frames defined by 3 input signals
           resolved in frame world, frame_a, frame_b or in frame_resolve. <br>
           <IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Forces/ArrowForce2.png\"></td>
  </tr>
  <tr><td valign=\"top\"><a href=\"modelica://UWBody.Forces.Torque\">Torque</a></td>
      <td valign=\"top\"> Torque acting between two frames defined by 3 input signals
           resolved in frame world, frame_a, frame_b or in frame_resolve. <br>
           <IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Forces/ArrowTorque2.png\"></td>
  </tr>
  <tr><td valign=\"top\"><a href=\"modelica://UWBody.Forces.ForceAndTorque\">ForceAndTorque</a></td>
      <td valign=\"top\"> Force and torque acting between two frames defined by 3+3 input signals
           resolved in frame world, frame_a, frame_b or in frame_resolve. <br>
           <IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Forces/ArrowForce2.png\"><br>
           <IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Forces/ArrowTorque2.png\"></td>
  </tr>
  <tr><td valign=\"top\"><a href=\"modelica://UWBody.Forces.LineForceWithMass\">LineForceWithMass</a></td>
      <td valign=\"top\">  General line force component with an optional point mass
            on the connection line. The force law can be defined by a
            component of Modelica.Mechanics.Translational<br>
           <IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Forces/LineForceWithMass.png\"></td>
  </tr>
  <tr><td valign=\"top\"><a href=\"modelica://UWBody.Forces.LineForceWithTwoMasses\">LineForceWithTwoMasses</a></td>
      <td valign=\"top\">  General line force component with two optional point masses
            on the connection line. The force law can be defined by a
            component of Modelica.Mechanics.Translational<br>
           <IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Forces/LineForceWithTwoMasses.png\"></td>
  </tr>
  <tr><td valign=\"top\"><a href=\"modelica://UWBody.Forces.Spring\">Spring</a></td>
      <td valign=\"top\"> Linear translational spring with optional mass <br>
           <IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Forces/Spring2.png\"></td>
  </tr>
  <tr><td valign=\"top\"><a href=\"modelica://UWBody.Forces.Damper\">Damper</a></td>
      <td valign=\"top\"> Linear (velocity dependent) damper <br>
           <IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Forces/Damper2.png\"></td>
  </tr>
  <tr><td valign=\"top\"><a href=\"modelica://UWBody.Forces.SpringDamperParallel\">SpringDamperParallel</a></td>
      <td valign=\"top\"> Linear spring and damper in parallel connection </td>
  </tr>
  <tr><td valign=\"top\"><a href=\"modelica://UWBody.Forces.SpringDamperSeries\">SpringDamperSeries</a></td>
      <td valign=\"top\"> Linear spring and damper in series connection </td>
  </tr>
</table>
</html>"));
end Forces;
