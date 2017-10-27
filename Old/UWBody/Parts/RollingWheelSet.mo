within UWBody.Parts;

model RollingWheelSet "Ideal rolling wheel set consisting of two ideal rolling wheels connected together by an axis"
  UWBody.Interfaces.Frame_a frameMiddle "Frame fixed in middle of axis connecting both wheels (y-axis: along wheel axis, z-axis: upwards)" annotation(Placement(transformation(extent = {{-16, 16}, {16, -16}}), iconTransformation(extent = {{-16, -16}, {16, 16}})));
  parameter Boolean animation = true "= true, if animation of wheel set shall be enabled";
  parameter SI.Radius wheelRadius "Radius of one wheel";
  parameter SI.Mass wheelMass "Mass of one wheel";
  parameter SI.Inertia wheel_I_axis "Inertia along one wheel axis";
  parameter SI.Inertia wheel_I_long "Inertia perpendicular to one wheel axis";
  parameter SI.Distance wheelDistance "Distance between the two wheels";
  parameter StateSelect stateSelect = StateSelect.always "Priority to use the generalized coordinates as states";
  Modelica.SIunits.Position x(start = 0, fixed = true, stateSelect = stateSelect) "x coordinate of center between wheels";
  Modelica.SIunits.Position y(start = 0, fixed = true, stateSelect = stateSelect) "y coordinate of center between wheels";
  Modelica.SIunits.Angle phi(start = 0, fixed = true, stateSelect = stateSelect) "Orientation angle of wheel axis along z-axis";
  Modelica.SIunits.Angle theta1(start = 0, fixed = true, stateSelect = stateSelect) "Angle of wheel 1";
  Modelica.SIunits.Angle theta2(start = 0, fixed = true, stateSelect = stateSelect) "Angle of wheel 2";
  Modelica.SIunits.AngularVelocity der_theta1(start = 0, fixed = true, stateSelect = stateSelect) "Derivative of theta 1";
  Modelica.SIunits.AngularVelocity der_theta2(start = 0, fixed = true, stateSelect = stateSelect) "Derivative of theta 2";
  parameter SI.Distance wheelWidth = 0.01 "Width of one wheel" annotation(Dialog(tab = "Animation", group = "if animation = true", enable = animation));
  parameter Real hollowFraction = 0.8 "1.0: Completely hollow, 0.0: rigid cylinder" annotation(Dialog(tab = "Animation", group = "if animation = true", enable = animation));
  parameter UWBody.Types.Color wheelColor = {30, 30, 30} "Color of wheels" annotation(Dialog(colorSelector = true, tab = "Animation", group = "if animation = true", enable = animation));
  UWBody.Interfaces.Frame_a frame1 "Frame fixed in center point of left wheel (y-axis: along wheel axis, z-axis: upwards)" annotation(Placement(transformation(extent = {{-96, 16}, {-64, -16}}, origin = {0, 0}, rotation = 0), iconTransformation(extent = {{-96, 16}, {-64, -16}}, origin = {-0, -0}, rotation = 0), visible = true));
  UWBody.Interfaces.Frame_b frame2 "Frame fixed in center point of right wheel (y-axis: along wheel axis, z-axis: upwards)" annotation(Placement(transformation(extent = {{64, 16}, {96, -16}}, origin = {0, 0}, rotation = 0), visible = true, iconTransformation(origin = {0, 0}, extent = {{64, 16}, {96, -16}}, rotation = 0)));
  UWBody.Parts.Body body2(final r_CM = {0, 0, 0}, final I_21 = 0, final I_31 = 0, final I_32 = 0, animation = false, final m = wheelMass, final I_11 = wheel_I_long, final I_22 = wheel_I_axis, final I_33 = wheel_I_long) annotation(Placement(transformation(extent = {{10, -10}, {-10, 10}}, rotation = -90, origin = {60, 30})));
  UWBody.Visualizers.FixedShape shape2(final animation = animation, final lengthDirection = {0, 1, 0}, final widthDirection = {1, 0, 0}, final color = wheelColor, final extra = hollowFraction, final shapeType = "pipe", final r_shape = {0, -wheelWidth, 0}, final length = 2 * wheelWidth, final width = 2 * wheelRadius, final height = 2 * wheelRadius) if animation annotation(Placement(transformation(extent = {{10, -10}, {-10, 10}}, rotation = 90, origin = {60, -38})));
  UWBody.Parts.Body body1(final r_CM = {0, 0, 0}, final I_21 = 0, final I_31 = 0, final I_32 = 0, animation = false, final m = wheelMass, final I_11 = wheel_I_long, final I_22 = wheel_I_axis, final I_33 = wheel_I_long) annotation(Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {-60, 30})));
  UWBody.Visualizers.FixedShape shape1(final animation = animation, final lengthDirection = {0, 1, 0}, final widthDirection = {1, 0, 0}, final color = wheelColor, final extra = hollowFraction, final shapeType = "pipe", final r_shape = {0, -wheelWidth, 0}, final length = 2 * wheelWidth, final width = 2 * wheelRadius, final height = 2 * wheelRadius) if animation annotation(Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = -90, origin = {-60, -40})));
  Modelica.Mechanics.Rotational.Interfaces.Flange_a axis1 "1-dim. rotational flange that drives the left wheel" annotation(Placement(transformation(extent = {{-110, 90}, {-90, 110}}, origin = {0, 0}, rotation = 0), visible = true, iconTransformation(origin = {0, 0}, extent = {{-110, 90}, {-90, 110}}, rotation = 0)));
  Modelica.Mechanics.Rotational.Interfaces.Flange_a axis2 "1-dim. rotational flange that drives the right wheel" annotation(Placement(transformation(extent = {{90, 90}, {110, 110}}, origin = {0, 0}, rotation = 0), visible = true, iconTransformation(origin = {0, 0}, extent = {{90, 90}, {110, 110}}, rotation = 0)));
  UWBody.Joints.RollingWheelSet wheelSetJoint(animation = false, wheelRadius = wheelRadius, wheelDistance = wheelDistance, stateSelect = StateSelect.default, x(fixed = false), y(fixed = false), phi(fixed = false), theta1(fixed = false), theta2(fixed = false), der_theta1(fixed = false), der_theta2(fixed = false)) annotation(Placement(transformation(extent = {{-10, -42}, {10, -22}})));
  Modelica.Mechanics.Rotational.Interfaces.Flange_b support "Support of 1D axes" annotation(Placement(transformation(extent = {{-10, 70}, {10, 90}}, origin = {0, 0}, rotation = 0), iconTransformation(extent = {{-10, 72}, {10, 92}}, origin = {0, 0}, rotation = 0), visible = true));
equation
  wheelSetJoint.x = x;
  wheelSetJoint.y = y;
  wheelSetJoint.phi = phi;
  wheelSetJoint.theta1 = theta1;
  wheelSetJoint.theta2 = theta2;
  der_theta1 = der(theta1);
  der_theta2 = der(theta2);
  connect(body2.frame_a, frame2) annotation(Line(points = {{60, 20}, {60, 0}, {80, 0}}, color = {95, 95, 95}, thickness = 0.5, visible = true));
  connect(body1.frame_a, frame1) annotation(Line(points = {{-60, 20}, {-60, 0}, {-80, 0}}, color = {95, 95, 95}, thickness = 0.5, visible = true));
  connect(shape1.frame_a, frame1) annotation(Line(points = {{-60, -30}, {-60, 0}, {-80, 0}}, color = {95, 95, 95}, thickness = 0.5, visible = true));
  connect(shape2.frame_a, frame2) annotation(Line(points = {{60, -28}, {60, 0}, {80, 0}}, color = {95, 95, 95}, thickness = 0.5, visible = true));
  connect(wheelSetJoint.frame2, frame2) annotation(Line(points = {{8, -32}, {40, -32}, {40, 0}, {80, 0}}, color = {95, 95, 95}, thickness = 0.5, visible = true));
  connect(wheelSetJoint.frame1, frame1) annotation(Line(points = {{-8, -32}, {-40, -32}, {-40, 0}, {-80, 0}}, color = {95, 95, 95}, thickness = 0.5, visible = true));
  connect(wheelSetJoint.axis1, axis1) annotation(Line(points = {{-10, -22}, {-20, -22}, {-20, 50}, {-80, 50}, {-80, 100}, {-100, 100}}, visible = true));
  connect(wheelSetJoint.axis2, axis2) annotation(Line(points = {{10, -22}, {24, -22}, {24, 50}, {80, 50}, {80, 100}, {100, 100}}, visible = true));
  connect(wheelSetJoint.support, support) annotation(Line(points = {{0, -24}, {0, -14}, {16, -14}, {16, 58}, {0, 58}, {0, 80}}, visible = true));
  connect(wheelSetJoint.frameMiddle, frameMiddle) annotation(Line(points = {{0, -32}, {-4, -32}, {-4, 0}, {0, 0}}, color = {95, 95, 95}, thickness = 0.5));
  annotation(defaultComponentName = "wheelSet", Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}, initialScale = 0.1, grid = {10, 10}), graphics = {Ellipse(visible = true, origin = {110, 0}, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-50, -80}, {10, 80}}), Rectangle(visible = true, origin = {85.5, 0}, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, pattern = LinePattern.None, fillPattern = FillPattern.HorizontalCylinder, extent = {{-7.5, -80}, {4.5, 80}}), Ellipse(visible = true, origin = {66, 0}, lineColor = {64, 64, 64}, fillColor = {191, 191, 191}, fillPattern = FillPattern.Solid, extent = {{-16, -80}, {44, 80}}), Ellipse(visible = true, origin = {64.615, 0}, lineColor = {192, 192, 192}, fillColor = {128, 128, 128}, fillPattern = FillPattern.Solid, extent = {{-1.923, -50}, {32.692, 50}}), Rectangle(visible = true, origin = {7.148, 2.5}, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, pattern = LinePattern.None, fillPattern = FillPattern.HorizontalCylinder, extent = {{-70.575, -7.5}, {72.852, 2.5}}), Ellipse(visible = true, origin = {-50, 0}, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, fillPattern = FillPattern.HorizontalCylinder, extent = {{-50, -80}, {10, 80}}), Rectangle(visible = true, origin = {-74.5, 0}, lineColor = {64, 64, 64}, fillColor = {255, 255, 255}, pattern = LinePattern.None, fillPattern = FillPattern.HorizontalCylinder, extent = {{-7.5, -80}, {4.5, 80}}), Ellipse(visible = true, origin = {-94, 0}, lineColor = {64, 64, 64}, fillColor = {191, 191, 191}, fillPattern = FillPattern.Solid, extent = {{-16, -80}, {44, 80}}), Ellipse(visible = true, origin = {-95.385, 0}, lineColor = {128, 128, 128}, fillColor = {128, 128, 128}, fillPattern = FillPattern.Solid, extent = {{-1.923, -50}, {32.692, 50}}), Rectangle(visible = true, fillColor = {175, 175, 175}, fillPattern = FillPattern.Solid, extent = {{-100, -100}, {100, -80}}), Text(visible = true, textColor = {64, 64, 64}, extent = {{-150, -145}, {150, -105}}, textString = "%name"), Line(visible = true, points = {{86, 24}, {64, 24}, {64, 10}, {56, 10}}), Line(visible = true, points = {{86, -24}, {64, -24}, {64, -12}, {56, -12}}), Line(visible = true, points = {{-100, 100}, {-80, 100}, {-80, 0}}), Line(visible = true, points = {{100, 100}, {80, 100}, {80, 0}}), Line(visible = true, points = {{0, 80}, {0, 40}, {-20, 40}, {-20, 2}}), Line(visible = true, points = {{0, 40}, {20, 40}, {20, 2}})}), Diagram(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Line(points = {{0, -106}, {0, -78}}, color = {0, 0, 255}), Polygon(points = {{0, -60}, {-6, -78}, {6, -78}, {0, -60}}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid), Text(extent = {{12, -68}, {30, -80}}, lineColor = {0, 0, 255}, textString = "x"), Line(points = {{6, -100}, {-26, -100}}, color = {0, 0, 255}), Polygon(points = {{-22, -94}, {-22, -106}, {-40, -100}, {-22, -94}}, lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid), Text(extent = {{-46, -80}, {-28, -92}}, lineColor = {0, 0, 255}, textString = "y")}), Documentation(info = "<html>
<p>
Two wheels are connected by an axis and can rotate around this axis.
The wheels are rolling on the x-y plane. The coordinate system attached
to the center of the wheel axis (frameMiddle) is constrained so that it
is always parallel to the x-y plane. If all generalized coordinates are zero,
frameMiddle is parallel to the world frame.
</p>
</html>"));
end RollingWheelSet;
