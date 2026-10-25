model MultiRoomHeating4x2_Scheduled_Energy
  import Modelica.Thermal.HeatTransfer.Components.*;
  import Modelica.Thermal.HeatTransfer.Sources.*;
  import Modelica.Blocks.Sources.*;

  // ---------- Parameters ----------
  parameter Real C        = 12000 "Room air heat capacity (J/K)";
  parameter Real G_wall   = 15    "Coupling conductance between adjacent rooms (W/K)";
  parameter Real G_out    = 20    "Loss conductance from each room to outside (W/K)";
  parameter Real Q_heater = 2000  "Heater power when ON (W)";
  parameter Real T_out_K  = 10 + 273.15 "Outside temperature (K)";
  parameter Real T0_K     = 18 + 273.15 "Initial room temperature (K)";

  // comfort band used only for metrics
  parameter Real T_low    = 20 + 273.15 "Lower comfort bound (K)";
  parameter Real T_high   = 24 + 273.15 "Upper comfort bound (K)";

  // how often the central scheduler reassigns heaters
  parameter Real reassignment = 300 "Reassignment period (s)";

  // ---------- Rooms (thermal masses) ----------
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

  // ---------- Energy & comfort metrics ----------
  Real E_total(start = 0) "Total heater energy [J]";
  Real E_kWh "Total heater energy [kWh]";

  Boolean ok1;
  Boolean ok2;
  Boolean ok3;
  Boolean ok4;

  Real timeComfort1(start = 0);
  Real timeComfort2(start = 0);
  Real timeComfort3(start = 0);
  Real timeComfort4(start = 0);

protected 
  // scheduler/controller state
  Boolean on1(start = true,  fixed = true);
  Boolean on2(start = false, fixed = true);
  Boolean on3(start = true,  fixed = true);
  Boolean on4(start = false, fixed = true);

  Integer coldest1(start = 1, fixed = true);
  Integer coldest2(start = 3, fixed = true);

  Real temps[4];

algorithm 
  // initialize which rooms are heated at t=0
  when initial() then
    on1 := true;
    on2 := false;
    on3 := true;
    on4 := false;
  end when;

  // every 'reassignment' seconds, select the two coldest rooms
  when sample(0, reassignment) then
    // snapshot current temperatures
    temps[1] := room1.T;
    temps[2] := room2.T;
    temps[3] := room3.T;
    temps[4] := room4.T;

    // find coldest
    coldest1 := 1;
    for i in 2:4 loop
      if temps[i] < temps[coldest1] then
        coldest1 := i;
      end if;
    end for;

    // find second coldest (not equal to coldest1)
    if coldest1 <> 1 then
      coldest2 := 1;
    else
      coldest2 := 2;
    end if;

    for i in 1:4 loop
      if i <> coldest1 then
        if temps[i] < temps[coldest2] then
          coldest2 := i;
        end if;
      end if;
    end for;

    // turn heaters ON in those two rooms, OFF in others
    on1 := (coldest1 == 1) or (coldest2 == 1);
    on2 := (coldest1 == 2) or (coldest2 == 2);
    on3 := (coldest1 == 3) or (coldest2 == 3);
    on4 := (coldest1 == 4) or (coldest2 == 4);
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

  // ---------- heaters physically attached ----------
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

  // ---------- energy metric ----------
  der(E_total) = q1 + q2 + q3 + q4;
  E_kWh       = E_total / 3.6e6;

  // ---------- comfort metrics ----------
  ok1 = (room1.T >= T_low) and (room1.T <= T_high);
  ok2 = (room2.T >= T_low) and (room2.T <= T_high);
  ok3 = (room3.T >= T_low) and (room3.T <= T_high);
  ok4 = (room4.T >= T_low) and (room4.T <= T_high);

  der(timeComfort1) = if ok1 then 1 else 0;
  der(timeComfort2) = if ok2 then 1 else 0;
  der(timeComfort3) = if ok3 then 1 else 0;
  der(timeComfort4) = if ok4 then 1 else 0;

  annotation (experiment(StartTime=0, StopTime=7200, Tolerance=1e-6, Interval=1));
end MultiRoomHeating4x2_Scheduled_Energy;
