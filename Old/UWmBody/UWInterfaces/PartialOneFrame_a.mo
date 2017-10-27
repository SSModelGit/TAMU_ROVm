within UWmBody.UWInterfaces;

partial model PartialOneFrame_a "Base model for components providing one frame_a connector + outer world + assert to guarantee that the component is connected"
  UWInterfaces.Frame_a frame_a "Coordinate system fixed to the component with one cut-force and cut-torque" annotation(Placement(transformation(extent = {{-116, -16}, {-84, 16}})));
protected
  outer UWmBody.UWWorld world;
equation
  assert(cardinality(frame_a) > 0, "Connector frame_a of component is not connected");
  annotation(Documentation(info = "<html>
<p>
This partial model provides one frame_a connector, access to the world
object and an assert to check that the frame_a connector is connected.
Therefore, inherit from this partial model if the frame_a connector is
needed and if this connector should be connected for a correct model.
</p>
</html>"));
end PartialOneFrame_a;
