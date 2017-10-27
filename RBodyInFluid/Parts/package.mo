within RBodyInFluid;

package Parts
  extends RBodyInFluid.Icons.RBFPackage;
  annotation(Icon(graphics = {Rectangle(visible = true, lineColor = {95, 95, 95}, fillColor = {230, 230, 230}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-80, -16}, {2, 28}}, radius = 10), Ellipse(visible = true, lineColor = {95, 95, 95}, fillColor = {230, 230, 230}, fillPattern = FillPattern.Sphere, extent = {{-8, -42}, {86, 52}})}, coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10})));
end Parts;
