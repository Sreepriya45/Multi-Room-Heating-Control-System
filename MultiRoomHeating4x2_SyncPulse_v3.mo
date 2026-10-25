model MultiRoomHeating4x2_SyncPulse_v3
  import Modelica.Thermal.HeatTransfer.Components.*;
  import Modelica.Thermal.HeatTransfer.Sources.*;
  import Modelica.Blocks.Sources.*;

  // -------- Parameters --------
  parameter Real C        = 12000 "Room air heat capacity (J/K)";
  parameter Real G_wall   = 15    "Coupling conductance (W/K)";
  parameter Real G_out    = 20    "Loss conductance to outside (W/K)";
  parameter Real T_out_K  = 10 + 273.15 "Outside temperature (K)";
  parameter Real T0_K     = 18 + 273.15 "Initial room temperature (K)";
  parameter Real Q_heater = 2000 "Heater power when ON (W)";
  parameter Real Ts       = 300  "Controller period (s)";

  // -------- Plant --------
  HeatCapacitor room1(C=C, T(start=T0_K));
  HeatCapacitor room2(C=C, T(start=T0_K));
  HeatCapacitor room3(C=C, T(start=T0_K));
  HeatCapacitor room4(C=C, T(start=T0_K));

  ThermalConductor r12(G=G_wall), r23(G=G_wall), r34(G=G_wall);

  FixedTemperature outside(T=T_out_K);
  ThermalConductor r1_out(G=G_out), r2_out(G=G_out), r3_out(G=G_out), r4_out(G=G_out);

  PrescribedHeatFlow h1, h2, h3, h4;

  // Heater powers (continuous)
  Real q1, q2, q3, q4;
  RealExpression q1Expr(y=q1), q2Expr(y=q2), q3Expr(y=q3), q4Expr(y=q4);

  // -------- Explicit periodic "clock" --------
  BooleanPulse clk(period=Ts, width=1);

protected 
  // controller state
  Boolean on1(start=true, fixed=true), on2(start=false, fixed=true),
          on3(start=true, fixed=true), on4(start=false, fixed=true);
  Integer coldest1(start=1, fixed=true), coldest2(start=3, fixed=true);

  // sampled temps and helper array
  Real T1s, T2s, T3s, T4s;
  Real temps[4];

algorithm
  when initial() then
    on1 := true; on2 := false; on3 := true; on4 := false;
  end when;

  // Synchronous controller: runs on rising edges of clk
  when clk.y then
    // sample temps at this tick
    T1s := room1.T;  T2s := room2.T;  T3s := room3.T;  T4s := room4.T;

    // fill array (avoid inline array literals)
    temps[1] := T1s;  temps[2] := T2s;  temps[3] := T3s;  temps[4] := T4s;

    // ---- find coldest1 (min index) ----
    coldest1 := 1;
    for i in 2:4 loop
      if temps[i] < temps[coldest1] then
        coldest1 := i;
      end if;
    end for;

    // ---- find coldest2 among remaining ----
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

    // set ON/OFF (only two heaters ON)
    on1 := (coldest1==1) or (coldest2==1);
    on2 := (coldest1==2) or (coldest2==2);
    on3 := (coldest1==3) or (coldest2==3);
    on4 := (coldest1==4) or (coldest2==4);
  end when;

equation
  // Plant connections
  connect(room1.port, r12.port_a); connect(r12.port_b, room2.port);
  connect(room2.port, r23.port_a); connect(r23.port_b, room3.port);
  connect(room3.port, r34.port_a); connect(r34.port_b, room4.port);

  connect(room1.port, r1_out.port_a); connect(r1_out.port_b, outside.port);
  connect(room2.port, r2_out.port_a); connect(r2_out.port_b, outside.port);
  connect(room3.port, r3_out.port_a); connect(r3_out.port_b, outside.port);
  connect(room4.port, r4_out.port_a); connect(r4_out.port_b, outside.port);

  connect(h1.port, room1.port);
  connect(h2.port, room2.port);
  connect(h3.port, room3.port);
  connect(h4.port, room4.port);

  // zero-order hold of commands between ticks
  q1 = if on1 then Q_heater else 0.0;
  q2 = if on2 then Q_heater else 0.0;
  q3 = if on3 then Q_heater else 0.0;
  q4 = if on4 then Q_heater else 0.0;

  connect(q1Expr.y, h1.Q_flow);
  connect(q2Expr.y, h2.Q_flow);
  connect(q3Expr.y, h3.Q_flow);
  connect(q4Expr.y, h4.Q_flow);

  annotation (experiment(StartTime=0, StopTime=7200, Tolerance=1e-6, Interval=1));
end MultiRoomHeating4x2_SyncPulse_v3;
