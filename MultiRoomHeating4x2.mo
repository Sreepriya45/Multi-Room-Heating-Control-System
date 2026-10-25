model MultiRoomHeating4x2
  import Modelica.Thermal.HeatTransfer.Components.*;
  import Modelica.Thermal.HeatTransfer.Sources.*;
  import Modelica.Blocks.Sources.*;

  // ---------- Parameters (tweak these) ----------
  parameter Real C        = 12000 "Room air heat capacity (J/K)";
  parameter Real G_wall   = 15    "Coupling conductance between adjacent rooms (W/K)";
  parameter Real G_out    = 20    "Loss conductance from each room to outside (W/K)";
  parameter Real T_out_K  = 10 + 273.15 "Outside temperature (K)";
  parameter Real T0_K     = 18 + 273.15 "Initial room temperature (K)";
  parameter Real Q_heater = 2000  "Heater power (W)";

  // ---------- Rooms (capacitances) ----------
  HeatCapacitor room1(C = C, T(start = T0_K));
  HeatCapacitor room2(C = C, T(start = T0_K));
  HeatCapacitor room3(C = C, T(start = T0_K));
  HeatCapacitor room4(C = C, T(start = T0_K));

  // ---------- Room-to-room couplings ----------
  ThermalConductor r12(G = G_wall);
  ThermalConductor r23(G = G_wall);
  ThermalConductor r34(G = G_wall);

  // ---------- Outside connection for each room ----------
  FixedTemperature outside(T = T_out_K);
  ThermalConductor r1_out(G = G_out);
  ThermalConductor r2_out(G = G_out);
  ThermalConductor r3_out(G = G_out);
  ThermalConductor r4_out(G = G_out);

  // ---------- Heaters (two) ----------
  // h1: ON from 0s, then turns OFF at 300s (step drops to 0)
  PrescribedHeatFlow h1;
  Step h1Step(offset = Q_heater, height = -Q_heater, startTime = 300);

  // h2: stays ON the whole time (constant 2000 W)
  PrescribedHeatFlow h2;
  Step h2Step(offset = Q_heater, height = 0, startTime = 0);

equation
  // Room-to-room connections (line of 4: 1-2-3-4)
  connect(room1.port, r12.port_a);
  connect(r12.port_b, room2.port);

  connect(room2.port, r23.port_a);
  connect(r23.port_b, room3.port);

  connect(room3.port, r34.port_a);
  connect(r34.port_b, room4.port);

  // Losses to outside for each room
  connect(room1.port, r1_out.port_a);  connect(r1_out.port_b, outside.port);
  connect(room2.port, r2_out.port_a);  connect(r2_out.port_b, outside.port);
  connect(room3.port, r3_out.port_a);  connect(r3_out.port_b, outside.port);
  connect(room4.port, r4_out.port_a);  connect(r4_out.port_b, outside.port);

  // Heaters attached to Room 1 and Room 3 (you can change this later)
  connect(h1.port, room1.port);
  connect(h2.port, room3.port);

  // Heater power inputs
  connect(h1Step.y, h1.Q_flow);
  connect(h2Step.y, h2.Q_flow);

  annotation (experiment(StartTime=0, StopTime=1800, Tolerance=1e-6, Interval=1));
end MultiRoomHeating4x2;
