model MultiRoomHeating4x2_Scheduled
  import Modelica.Thermal.HeatTransfer.Components.*;
  import Modelica.Thermal.HeatTransfer.Sources.*;
  import Modelica.Blocks.Sources.*;

  // ---------- Parameters ----------
  parameter Real C        = 12000 "Room air heat capacity (J/K)";
  parameter Real G_wall   = 15    "Coupling conductance between adjacent rooms (W/K)";
  parameter Real G_out    = 20    "Loss conductance from each room to outside (W/K)";
  parameter Real T_out_K  = 10 + 273.15 "Outside temperature (K)";
  parameter Real T0_K     = 18 + 273.15 "Initial room temperature (K)";
  parameter Real Q_heater = 2000  "Heater power when ON (W)";
  parameter Real reassignment = 300 "How often to reassign heaters (s)";

  // ---------- Rooms (thermal masses) ----------
  HeatCapacitor room1(C = C, T(start = T0_K));
  HeatCapacitor room2(C = C, T(start = T0_K));
  HeatCapacitor room3(C = C, T(start = T0_K));
  HeatCapacitor room4(C = C, T(start = T0_K));

  // ---------- Room-to-room couplings (1-2-3-4 line) ----------
  ThermalConductor r12(G = G_wall);
  ThermalConductor r23(G = G_wall);
  ThermalConductor r34(G = G_wall);

  // ---------- Outside connection for each room ----------
  FixedTemperature outside(T = T_out_K);
  ThermalConductor r1_out(G = G_out);
  ThermalConductor r2_out(G = G_out);
  ThermalConductor r3_out(G = G_out);
  ThermalConductor r4_out(G = G_out);

  // ---------- One heater per room ----------
  PrescribedHeatFlow h1;
  PrescribedHeatFlow h2;
  PrescribedHeatFlow h3;
  PrescribedHeatFlow h4;

  // commanded heater powers (W)
  Real q1, q2, q3, q4;

  // bridge Reals into the PrescribedHeatFlow inputs
  RealExpression q1Expr(y = q1);
  RealExpression q2Expr(y = q2);
  RealExpression q3Expr(y = q3);
  RealExpression q4Expr(y = q4);

  // ---------- Scheduler/controller state ----------
  // no start/fixed here – initial values come from when initial()
  Boolean on1;
  Boolean on2;
  Boolean on3;
  Boolean on4;

  // round-robin mode:
  // 1 → heat rooms (1,2)
  // 2 → heat rooms (3,4)
  // 3 → heat rooms (1,3)
  Integer mode;

algorithm
  // initial heater assignment at t = 0
  when initial() then
    mode := 1;
    on1 := true;   // heat room 1
    on2 := true;   // and room 2
    on3 := false;
    on4 := false;
  end when;

  // every 'reassignment' seconds, move to the next mode (round-robin)
  when sample(0, reassignment) then
    if pre(mode) == 1 then
      // mode 2: heat rooms 3 & 4
      mode := 2;
      on1 := false; on2 := false; on3 := true;  on4 := true;

    elseif pre(mode) == 2 then
      // mode 3: heat rooms 1 & 3
      mode := 3;
      on1 := true;  on2 := false; on3 := true;  on4 := false;

    else
      // mode 1 again: heat rooms 1 & 2
      mode := 1;
      on1 := true;  on2 := true;  on3 := false; on4 := false;
    end if;
  end when;

equation
  // ---------- thermal network ----------
  connect(room1.port, r12.port_a); connect(r12.port_b, room2.port);
  connect(room2.port, r23.port_a); connect(r23.port_b, room3.port);
  connect(room3.port, r34.port_a); connect(r34.port_b, room4.port);

  connect(room1.port, r1_out.port_a); connect(r1_out.port_b, outside.port);
  connect(room2.port, r2_out.port_a); connect(r2_out.port_b, outside.port);
  connect(room3.port, r3_out.port_a); connect(r3_out.port_b, outside.port);
  connect(room4.port, r4_out.port_a); connect(r4_out.port_b, outside.port);

  // ---------- heaters physically attached (one per room) ----------
  connect(h1.port, room1.port);
  connect(h2.port, room2.port);
  connect(h3.port, room3.port);
  connect(h4.port, room4.port);

  // power commands from controller
  q1 = if on1 then Q_heater else 0;
  q2 = if on2 then Q_heater else 0;
  q3 = if on3 then Q_heater else 0;
  q4 = if on4 then Q_heater else 0;

  connect(q1Expr.y, h1.Q_flow);
  connect(q2Expr.y, h2.Q_flow);
  connect(q3Expr.y, h3.Q_flow);
  connect(q4Expr.y, h4.Q_flow);

  annotation (experiment(StartTime = 0, StopTime = 7200, Tolerance = 1e-6, Interval = 1));
end MultiRoomHeating4x2_Scheduled;
