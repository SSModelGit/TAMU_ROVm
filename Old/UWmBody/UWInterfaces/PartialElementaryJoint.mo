within UWmBody.UWInterfaces;

partial model PartialElementaryJoint "Base model for elementary joints (has two frames + outer world + assert to guarantee that the joint is connected)"
  UWInterfaces.Frame_a frame_a "Coordinate system fixed to the joint with one cut-force and cut-torque" annotation(Placement(transformation(extent = {{-116, -16}, {-84, 16}})));
  UWInterfaces.Frame_b frame_b "Coordinate system fixed to the joint with one cut-force and cut-torque" annotation(Placement(transformation(extent = {{84, -16}, {116, 16}})));
protected
  outer UWmBody.UWWorld world;
equation
  Connections.branch(frame_a.R, frame_b.R);
  assert(cardinality(frame_a) > 0, "Connector frame_a of joint object is not connected");
  assert(cardinality(frame_b) > 0, "Connector frame_b of joint object is not connected");
  annotation(Documentation(info = "<html>
<p>
All <b>elementary joints</b> should inherit from this base model, i.e.,
joints that are directly defined by equations, provided they compute
either the rotation object of frame_b from the rotation object of frame_a
and from relative quantities (or vice versa), or there is a constraint
equation between the rotation objects of the two frames.
In other cases, a joint object should inherit from
<b>Interfaces.PartialTwoFrames</b> (e.g., joint Spherical, because there
is no constraint between the rotation objects of frame_a and frame_b
or joint Cylindrical because it is not an elementary joint).
</p>
<p>
This partial model provides two frame connectors, a \"Connections.branch\"
between frame_a and frame_b, access to the world
object and an assert to check that both frame connectors are connected.
</p>
</html>"));
end PartialElementaryJoint;
