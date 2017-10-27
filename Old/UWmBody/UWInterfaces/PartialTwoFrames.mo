within UWmBody.UWInterfaces;

partial model PartialTwoFrames "Base model for components providing two frame connectors + outer world + assert to guarantee that the component is connected"
  UWInterfaces.Frame_a frame_a "Coordinate system fixed to the component with one cut-force and cut-torque" annotation(Placement(transformation(extent = {{-116, -16}, {-84, 16}})));
  UWInterfaces.Frame_b frame_b "Coordinate system fixed to the component with one cut-force and cut-torque" annotation(Placement(transformation(extent = {{84, -16}, {116, 16}})));
protected
  outer UWmBody.UWWorld world;
equation
  assert(cardinality(frame_a) > 0, "Connector frame_a of component is not connected");
  assert(cardinality(frame_b) > 0, "Connector frame_b of component is not connected");
  annotation(Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Text(extent = {{-136, -25}, {-100, -50}}, lineColor = {128, 128, 128}, textString = "a"), Text(extent = {{100, -25}, {136, -50}}, lineColor = {128, 128, 128}, textString = "b")}), Documentation(info = "<html>
<p>
This partial model provides two frame connectors, access to the world
object and an assert to check that both frame connectors are connected.
Therefore, inherit from this partial model if the two frame connectors are
needed and if the two frame connectors should be connected for a correct model.
</p>
</html>"));
end PartialTwoFrames;
