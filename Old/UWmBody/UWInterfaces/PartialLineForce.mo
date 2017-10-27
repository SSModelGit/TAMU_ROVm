within UWmBody.UWInterfaces;

partial model PartialLineForce "Base model for line force elements"
  import Frames = UWmBody.UWFrames;
  parameter SI.Position s_small = 1.E-6 "Prevent zero-division if relative distance s=0" annotation(Dialog(tab = "Advanced"));
  parameter Boolean fixedRotationAtFrame_a = false "=true, if rotation frame_a.R is fixed (to directly connect line forces)" annotation(Evaluate = true, choices(checkBox = true), Dialog(tab = "Advanced", group = "If enabled, can give wrong results, see MultiBody.UsersGuide.Tutorial.ConnectionOfLineForces"));
  parameter Boolean fixedRotationAtFrame_b = false "=true, if rotation frame_b.R is fixed (to directly connect line forces)" annotation(Evaluate = true, choices(checkBox = true), Dialog(tab = "Advanced", group = "If enabled, can give wrong results, see MultiBody.UsersGuide.Tutorial.ConnectionOfLineForces"));
  UWInterfaces.Frame_a frame_a "Coordinate system fixed to the force element with one cut-force and cut-torque" annotation(Placement(transformation(extent = {{-116, -16}, {-84, 16}})));
  UWInterfaces.Frame_b frame_b "Coordinate system fixed to the force element with one cut-force and cut-torque" annotation(Placement(transformation(extent = {{84, -16}, {116, 16}})));
  SI.Force f "Line force acting on frame_a and on frame_b (positive, if acting on frame_b and directed from frame_a to frame_b)";
  SI.Position s "(Guarded) distance between the origin of frame_a and the origin of frame_b (>= s_small))";
  Real e_a[3](each final unit = "1") "Unit vector on the line connecting the origin of frame_a with the origin of frame_b resolved in frame_a (directed from frame_a to frame_b)";
  Modelica.SIunits.Position r_rel_a[3] "Position vector from origin of frame_a to origin of frame_b, resolved in frame_a";
protected
  outer UWmBody.UWWorld world;
equation
  assert(cardinality(frame_a) > 0, "Connector frame_a of line force object is not connected");
  assert(cardinality(frame_b) > 0, "Connector frame_b of line force object is not connected");
  // Determine distance s and n_a
  r_rel_a = Frames.resolve2(frame_a.R, frame_b.r_0 - frame_a.r_0);
  s = noEvent(max(Modelica.Math.Vectors.length(r_rel_a), s_small));
  e_a = r_rel_a / s;
  /* Determine forces and torques at frame_a and frame_b */
  frame_a.f = -e_a * f;
  frame_b.f = -Frames.resolve2(Frames.relativeRotation(frame_a.R, frame_b.R), frame_a.f);
  // Additional equations, if direct connections of line forces
  if fixedRotationAtFrame_a then
    Connections.root(frame_a.R);
    frame_a.R = Frames.nullRotation();
  else
    frame_a.t = zeros(3);
  end if;
  if fixedRotationAtFrame_b then
    Connections.root(frame_b.R);
    frame_b.R = Frames.nullRotation();
  else
    frame_b.t = zeros(3);
  end if;
  annotation(Documentation(info = "<html>
<p>
All <b>line force</b> elements should be based on this base model.
This model defines frame_a and frame_b, computes the relative
distance <b>s</b> and provides the force and torque
balance of the cut-forces and cut-torques at frame_a and
frame_b, respectively. In sub-models, only the line force <b>f</b>,
acting at frame_b on the line from frame_a to frame_b, as a function
of the relative distance <b>s</b> and its derivative <b>der</b>(<b>s</b>)
has to be defined. Example:
</p>
<pre>
   <b>model</b> Spring
      <b>parameter</b> Real c \"spring constant\",
      <b>parameter</b> Real s_unstretched \"unstretched spring length\";
      <b>extends</b> Modelica.Mechanics.MultiBody.Interfaces.PartialLineForce;
   <b>equation</b>
      f = c*(s-s_unstretched);
   <b>end</b> Spring;
</pre>
</html>"), Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Text(extent = {{-136, -44}, {-100, -19}}, lineColor = {128, 128, 128}, textString = "a"), Text(extent = {{100, -42}, {136, -17}}, lineColor = {128, 128, 128}, textString = "b"), Ellipse(visible = fixedRotationAtFrame_a, extent = {{-70, 30}, {-130, -30}}, lineColor = {255, 0, 0}), Text(visible = fixedRotationAtFrame_a, extent = {{-62, 50}, {-140, 30}}, lineColor = {255, 0, 0}, textString = "R=0"), Ellipse(visible = fixedRotationAtFrame_b, extent = {{70, 30}, {130, -30}}, lineColor = {255, 0, 0}), Text(visible = fixedRotationAtFrame_b, extent = {{62, 50}, {140, 30}}, lineColor = {255, 0, 0}, textString = "R=0")}));
end PartialLineForce;
