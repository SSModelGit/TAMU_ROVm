within UWmBody.UWForces.Internal;

function standardVIscousDrag "Viscous Drag = a * v"
  extends Modelica.Icons.Function;
  extends UWmBody.UWInterfaces.partialDragForce;
  annotation(Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
end standardVIscousDrag;
