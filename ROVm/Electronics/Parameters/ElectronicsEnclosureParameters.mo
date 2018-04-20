within ROVm.Electronics.Parameters;

record ElectronicsEnclosureParameters "Record of parameters specific to the electronics enclosure"
  import SI = Modelica.SIunits;
  // parameters for external frames
  parameter SI.Length lf_pos[3] = {0.9, 0, -0.2};
  parameter SI.Length lb_pos[3] = {0.1, 0, -0.2};
  parameter SI.Length rf_pos[3] = {0.9, 0, 0.2};
  parameter SI.Length rb_pos[3] = {0.1, 0, 0.2};
  // general parameters
  parameter SI.Length r_CM_EEnclosure[3] = {0.5, 0, 0} "r_CM from frame_a to center of mass of electronics enclosure";
  parameter SI.Mass m_EEnclosure = 2 "Mass of electronics enclosure";
  parameter SI.Density d_EEnclosure = 427 "Density of electronics enclosure";
  parameter SI.Area A_EEnclosure = 0.3 "Overall cross sectional area of the electronics enclosure";
  parameter SI.DimensionlessRatio c_d_EEnclosure = 0 "Drag coefficient of the electronics enclosure";
  parameter SI.Resistance R_EEnclosure = 100 "Resistance of electronics enclosure";
  annotation(Diagram(coordinateSystem(extent = {{-148.5, -105}, {148.5, 105}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})));
end ElectronicsEnclosureParameters;
