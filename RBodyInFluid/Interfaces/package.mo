within RBodyInFluid;

package Interfaces
  extends RBodyInFluid.Icons.RBFPackage;
  annotation(Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = false, initialScale = 0.1, grid = {10, 10}), graphics = {Polygon(visible = true, origin = {20, 0}, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, points = {{-10, 70}, {10, 70}, {40, 20}, {80, 20}, {80, -20}, {40, -20}, {10, -70}, {-10, -70}}), Polygon(visible = true, lineColor = {64, 64, 64}, fillColor = {128, 128, 128}, fillPattern = FillPattern.Solid, points = {{-100, 20}, {-60, 20}, {-30, 70}, {-10, 70}, {-10, -70}, {-30, -70}, {-60, -20}, {-100, -20}})}));
end Interfaces;
