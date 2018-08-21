within ROVm.Examples.ConstantMotionProfiles;

record ForwardMotion
  parameter Real k1 = 0.3316;
  parameter Real k2 = 0.2967;
  parameter Real k3 = 0.4104;
  parameter Real k4 = 0.3755;
  parameter Real k5 = 0.0897;
  parameter Real k6 = -0.0897;
  annotation(Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
end ForwardMotion;
