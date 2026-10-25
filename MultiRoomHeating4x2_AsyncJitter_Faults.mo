model MultiRoomHeating4x2_AsyncJitter_Faults
  import Modelica.Thermal.HeatTransfer.Components.*;
  import Modelica.Thermal.HeatTransfer.Sources.*;
  import Modelica.Blocks.Sources.*;

  // ---------------- Parameters ----------------
  parameter Real C        = 12000;
  parameter Real G_wall   = 15;
  parameter Real G_out    = 20;
  parameter Real T_out_K  = 10 + 273.15;
  parameter Real T0_K     = 18 + 273.15;
  parameter Real Q_heater = 2000;
  parameter Real T_low    = 20 + 273.15;
  parameter Real T_high   = 24 + 273.15;

  // jittered periods for each room (asynchronous)
  parameter Real P1 = 300 +  20;
  parameter Real P2 = 300 -  15;
  parameter Real P3 = 300 +  25;
  parameter Real P4 = 300 -  10;

  // communication check period
  parameter Real T_comm = 300;

  // times when each room loses communication (faults)
  parameter Real tFault1 = 3600;
  parameter Real tFault2 = 4200;
  parameter Real tFault3 = 4800;
  parameter Real tFault4 = 5400;

  // expose these as plot-friendly variables
  Real faultEvent1 = tFault1;
  Real faultEvent2 = tFault2;
  Real faultEvent3 = tFault3;
  Real faultEvent4 = tFault4;

  // ---------------- Rooms and network ----------------
  HeatCapacitor room1(C = C, T(start = T0_K));
  HeatCapacitor room2(C = C, T(start = T0_K));
  HeatCapacitor room3(C = C, T(start = T0_K));
  HeatCapacitor room4(C = C, T(start = T0_K));

  ThermalConductor r12(G = G_wall);
  ThermalConductor r23(G = G_wall);
  ThermalConductor r34(G = G_wall);

  FixedTemperature outside(T = T_out_K);
  ThermalConductor r1_out(G = G_out);
  ThermalConductor r2_out(G = G_out);
  ThermalConductor r3_out(G = G_out);
  ThermalConductor r4_out(G = G_out);

  // ---------------- Heaters ----------------
  PrescribedHeatFlow h1;
  PrescribedHeatFlow h2;
  PrescribedHeatFlow h3;
  PrescribedHeatFlow h4;

  Real q1, q2, q3, q4;
  RealExpression q1E(y = q1);
  RealExpression q2E(y = q2);
  RealExpression q3E(y = q3);
  RealExpression q4E(y = q4);

  // ---------------- Top-level “flags” to plot ----------------
  Boolean commOK1(start = true);
  Boolean commOK2(start = true);
  Boolean commOK3(start = true);
  Boolean commOK4(start = true);

protected 
  // local ON/OFF states
  Boolean on1(start = true);
  Boolean on2(start = false);
  Boolean on3(start = true);
  Boolean on4(start = false);

  // local wake-up times for each controller
  Real t1(start = 0);
  Real t2(start = 50);
  Real t3(start = 100);
  Real t4(start = 150);

  // next time when communication is refreshed
  Real tNextComm(start = T_comm);

algorithm 
  // -------- Local asynchronous updates with jitter --------
  when time >= pre(t1) then
    if commOK1 then
      on1 := if room1.T < T_low then true
             elseif room1.T > T_high then false
             else pre(on1);
    else
      // communication lost → hold previous command
      on1 := pre(on1);
    end if;
    t1 := time + P1;
  end when;

  when time >= pre(t2) then
    if commOK2 then
      on2 := if room2.T < T_low then true
             elseif room2.T > T_high then false
             else pre(on2);
    else
      on2 := pre(on2);
    end if;
    t2 := time + P2;
  end when;

  when time >= pre(t3) then
    if commOK3 then
      on3 := if room3.T < T_low then true
             elseif room3.T > T_high then false
             else pre(on3);
    else
      on3 := pre(on3);
    end if;
    t3 := time + P3;
  end when;

  when time >= pre(t4) then
    if commOK4 then
      on4 := if room4.T < T_low then true
             elseif room4.T > T_high then false
             else pre(on4);
    else
      on4 := pre(on4);
    end if;
    t4 := time + P4;
  end when;

  // -------- Fault events: communication drops --------
  when time >= tFault1 then commOK1 := false; end when;
  when time >= tFault2 then commOK2 := false; end when;
  when time >= tFault3 then commOK3 := false; end when;
  when time >= tFault4 then commOK4 := false; end when;

  // -------- Periodic communication refresh --------
  when time >= pre(tNextComm) then
    commOK1 := true;
    commOK2 := true;
    commOK3 := true;
    commOK4 := true;
    tNextComm := time + T_comm;
  end when;

equation
  // network connections
  connect(room1.port, r12.port_a);
  connect(r12.port_b, room2.port);
  connect(room2.port, r23.port_a);
  connect(r23.port_b, room3.port);
  connect(room3.port, r34.port_a);
  connect(r34.port_b, room4.port);

  connect(room1.port, r1_out.port_a);
  connect(r1_out.port_b, outside.port);
  connect(room2.port, r2_out.port_a);
  connect(r2_out.port_b, outside.port);
  connect(room3.port, r3_out.port_a);
  connect(r3_out.port_b, outside.port);
  connect(room4.port, r4_out.port_a);
  connect(r4_out.port_b, outside.port);

  connect(h1.port, room1.port);
  connect(h2.port, room2.port);
  connect(h3.port, room3.port);
  connect(h4.port, room4.port);

  // heater powers
  q1 = if on1 then Q_heater else 0;
  q2 = if on2 then Q_heater else 0;
  q3 = if on3 then Q_heater else 0;
  q4 = if on4 then Q_heater else 0;

  connect(q1E.y, h1.Q_flow);
  connect(q2E.y, h2.Q_flow);
  connect(q3E.y, h3.Q_flow);
  connect(q4E.y, h4.Q_flow);

  annotation (experiment(StartTime = 0, StopTime = 7200, Interval = 1));
end MultiRoomHeating4x2_AsyncJitter_Faults;
