within RBodyInFluid.Fields;

model WaterField
  extends Icons.Field;
  parameter Modelica.SIunits.Density pho_field = 1000 "Density of water in field";
  parameter Modelica.Mechanics.MultiBody.Types.AxisLabel label1 = "x" "Label of horizontal axis in icon";
  parameter Modelica.Mechanics.MultiBody.Types.AxisLabel label2 = "y" "Label of vertical axis in icon";
  parameter Modelica.Mechanics.MultiBody.Types.AxisLabel label3 = "z" "Label of protruding axis in icon";
  replaceable function waterBuoyantForce = RBodyInFluid.Fields.Internal.standardBuoyantForce(pho_fluid = pho_field, g = world.g * Modelica.Math.Vectors.normalizeWithAssert(world.n)) constrainedby RBodyInFluid.Interfaces.partialBuoyantForce;
  replaceable function waterDragForce = RBodyInFluid.Fields.Internal.standardViscousDrag(pho_fluid = pho_field) constrainedby RBodyInFluid.Interfaces.partialViscousDrag;
protected
  outer Modelica.Mechanics.MultiBody.World world;
  annotation(defaultComponentName = "waterField", defaultComponentPrefixes = "inner", missingInnerMessage = "No \"waterField\" component is defined.", Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
end WaterField;
