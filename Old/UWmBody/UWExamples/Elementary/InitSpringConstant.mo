within UWmBody.UWExamples.Elementary;

model InitSpringConstant "Determine spring constant such that system is in steady state at given position"
  extends Modelica.Icons.Example;
  inner UWmBody.UWWorld world(gravityType = UWmBody.UWTypes.GravityTypes.UniformGravity) annotation(Placement(transformation(extent = {{-80, 0}, {-60, 20}}, origin = {10, -30}, rotation = 0), visible = true));
  UWmBody.UWJoints.Revolute rev(useAxisFlange = true, n = {0, 0, 1}, phi(fixed = true), w(fixed = true), a(fixed = true)) annotation(Placement(transformation(extent = {{-40, 0}, {-20, 20}}, origin = {10, -30}, rotation = 0), visible = true));
  Modelica.Mechanics.Rotational.Components.Damper damper(d = 0.1) annotation(Placement(transformation(extent = {{-40, 40}, {-20, 60}}, origin = {10, -30}, rotation = 0), visible = true));
  UWmBody.UWParts.BodyShape body(r = {1, 0, 0}, r_CM = {0.5, 0, 0}, m = 1) annotation(Placement(transformation(extent = {{0, 0}, {20, 20}}, origin = {10, -30}, rotation = 0), visible = true));
  UWmBody.UWParts.Fixed fixed(r = {1, 0.2, 0}, width = 0.02) annotation(Placement(transformation(origin = {50, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 270), visible = true));
  UWmBody.UWForces.Spring spring(s_unstretched = 0.1, c(fixed = false, start = 100)) annotation(Placement(transformation(origin = {50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 270), visible = true));
equation
  connect(world.frame_b, rev.frame_a) annotation(Line(points = {{-10, 0}, {10, 0}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {-40, -20}));
  connect(damper.flange_b, rev.axis) annotation(Line(points = {{-26, 50}, {-16, 50}, {-16, 26}, {-36, 26}, {-36, 20}}, visible = true, color = {64, 64, 64}, origin = {16, -30}));
  connect(rev.support, damper.flange_a) annotation(Line(points = {{-34, 20}, {-34, 26}, {-48, 26}, {-48, 50}, {-38, 50}}, visible = true, color = {64, 64, 64}, origin = {8, -30}));
  connect(rev.frame_b, body.frame_a) annotation(Line(points = {{-10, 0}, {10, 0}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {0, -20}));
  connect(fixed.frame_b, spring.frame_a) annotation(Line(points = {{0, 5}, {-0, -5}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {50, 15}));
  connect(body.frame_b, spring.frame_b) annotation(Line(points = {{30, 10}, {50, 10}, {50, 20}}, color = {95, 95, 95}, thickness = 0.5, visible = true, origin = {0, -30}));
  annotation(__Wolfram(PlotSet(plots = {Plot(name = "Forces and initial conditions", preferred = true, subPlots = {SubPlot(curves = {Curve(x = time, y = body.body.frame_a.f[2], legend = "Force from gravity on body")}, range = Range(xmin = 0, xmax = 1.2, ymin = auto, ymax = auto)), SubPlot(curves = {Curve(x = time, y = spring.c, legend = "Calculated spring constant"), Curve(x = time, y = spring.spring.f, legend = "Force from spring on end point of the body")}, range = Range(xmin = 0, xmax = 1.2, ymin = auto, ymax = auto)), SubPlot(curves = {Curve(x = time, y = rev.phi, legend = "Angle of the revolute joint (set to 0 at initialization)"), Curve(x = time, y = der(rev.phi), legend = "Angular velocity of the revolute joint (set to 0 at initialization)"), Curve(x = time, y = der(rev.w), legend = "Angular acceleration of the revolute joint (set to 0 at initialization)")}, range = Range(xmin = 0, xmax = 1.2, ymin = auto, ymax = auto))})})), Documentation(info = "<html>
<p>
This example demonstrates a non-standard type of initialization
by calculating a spring constant such
that a simple pendulum is at a defined position in steady state.
</p>
<p>
The goal is that the pendulum should be in steady state
when the rotation angle of the pendulum is zero. The spring
constant of the spring shall be calculated during initialization
such that this goal is reached.
</p>
<p>
The pendulum has one degree of freedom, i.e., two states.
Therefore, two additional equations have to be provided
for initialization. However, parameter \"c\" of the spring
component is defined with attribute \"fixed = <b>false</b>\", i.e.,
the value of this parameter is computed during initialization.
Therefore, there is one additional equation required during
initialization. The 3 initial equations are the rotational
angle of the revolute joint and its first and second
derivative. The latter ones are zero, in order to initialize
in steady state. By setting the start values of phi, w, a to zero and
their fixed attributes to true, the required
3 initial equations are defined.
</p>
<p>
After translation, this model is initialized in steady-state.
The spring constant is computed as c = 49.05 N/m.
An animation of this simulation is shown in the figure below.
</p>

<IMG src=\"modelica://Modelica/Resources/Images/Mechanics/MultiBody/Examples/Elementary/InitSpringConstant.png\"
ALT=\"model Examples.Elementary.InitSpringConstant\">
</html>"), experiment(StopTime = 1.01));
end InitSpringConstant;
