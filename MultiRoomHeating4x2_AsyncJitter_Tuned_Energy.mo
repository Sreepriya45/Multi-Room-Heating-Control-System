model MultiRoomHeating4x2_AsyncJitter_Tuned_Energy
  import Modelica.Thermal.HeatTransfer.Components.*;
  import Modelica.Thermal.HeatTransfer.Sources.*;
  import Modelica.Blocks.Sources.*;

  // ---- Parameters ----
  parameter Real C        = 12000;
  parameter Real G_wall   = 15;
  parameter Real G_out    = 20;
  parameter Real T_out_K  = 10 + 273.15;
  parameter Real T0_K     = 18 + 273.15;
  parameter Real Q_heater = 2000;

  // comfort band (also used by local thermostats)
  parameter Real T_low    = 20 + 273.15;
  parameter Real T_high   = 24 + 273.15;

  // base period and jitter offsets (tuned)
  parameter Real Pbase = 300 "Base period [s]";
  parameter Real J1    = -20 "Jitter room 1 [s]";
  parameter Real J2    =  15 "Jitter room 2 [s]";
  parameter Real J3    =  25 "Jitter room 3 [s]";
  parameter Real J4    =  35 "Jitter room 4 [s]";

  parameter Real O1 =   0 "Initial offset room 1 [s]";
  parameter Real O2 =  60 "Initial offset room 2 [s]";
  parameter Real O3 = 130 "Initial offset room 3 [s]";
  parameter Real O4 = 210 "Initial offset room 4 [s]";

  // derived local periods
  parameter Real P1 = Pbase + J1;
  parameter Real P2 = Pbase + J2;
  parameter Real P3 = Pbase + J3;
  parameter Real P4 = Pbase + J4;

  // ---- Rooms / network ----
  HeatCapacitor room1(C=C, T(start=T0_K));
  HeatCapacitor room2(C=C, T(start=T0_K));
  HeatCapacitor room3(C=C, T(start=T0_K));
  HeatCapacitor room4(C=C, T(start=T0_K));
  ThermalConductor r12(G=G_wall), r23(G=G_wall), r34(G=G_wall);

  FixedTemperature outside(T=T_out_K);
  ThermalConductor r1_out(G=G_out), r2_out(G=G_out), r3_out(G=G_out), r4_out(G=G_out);

  // ---- Heaters ----
  PrescribedHeatFlow h1;
  PrescribedHeatFlow h2;
  PrescribedHeatFlow h3;
  PrescribedHeatFlow h4;

  Real q1, q2, q3, q4;
  RealExpression q1E(y=q1), q2E(y=q2), q3E(y=q3), q4E(y=q4);

  // ---- Energy and comfort metrics ----
  Real E_total(start=0) "Total heater energy [J]";
  Real E_kWh "Total heater energy [kWh]";

  Boolean ok1;
  Boolean ok2;
  Boolean ok3;
  Boolean ok4;

  Real timeComfort1(start=0);
  Real timeComfort2(start=0);
  Real timeComfort3(start=0);
  Real timeComfort4(start=0);

protected 
  // ON/OFF states updated on local ticks
  Boolean on1(start=true), on2(start=false), on3(start=true), on4(start=false);

  // next local wake-up times
  Real t1(start=O1), t2(start=O2), t3(start=O3), t4(start=O4);

  // helper variables for global cap (<=2 heaters)
  Integer nOn;
  Real T[4];
  Boolean b[4];
  Integer idxHot;
  Real Th;

algorithm 
  // ----- Local asynchronous updates -----
  when time >= pre(t1) then
    on1 := if room1.T < T_low then true
           elseif room1.T > T_high then false
           else pre(on1);
    t1  := time + P1;
  end when;

  when time >= pre(t2) then
    on2 := if room2.T < T_low then true
           elseif room2.T > T_high then false
           else pre(on2);
    t2  := time + P2;
  end when;

  when time >= pre(t3) then
    on3 := if room3.T < T_low then true
           elseif room3.T > T_high then false
           else pre(on3);
    t3  := time + P3;
  end when;

  when time >= pre(t4) then
    on4 := if room4.T < T_low then true
           elseif room4.T > T_high then false
           else pre(on4);
    t4  := time + P4;
  end when;

  // ----- Global safety: at most two heaters ON -----
  when change(on1) or change(on2) or change(on3) or change(on4) then
    nOn := (if on1 then 1 else 0)
         + (if on2 then 1 else 0)
         + (if on3 then 1 else 0)
         + (if on4 then 1 else 0);

    if nOn > 2 then
      // snapshot temps & states
      T := {room1.T, room2.T, room3.T, room4.T};
      b := {on1, on2, on3, on4};

      // turn OFF the hottest ON rooms until only 2 remain
      for k in 1:2 loop
        nOn := (if on1 then 1 else 0)
             + (if on2 then 1 else 0)
             + (if on3 then 1 else 0)
             + (if on4 then 1 else 0);
        if nOn <= 2 then
          break;
        end if;

        idxHot := 1;
        Th := -1e9;
        for i in 1:4 loop
          if b[i] and T[i] > Th then
            Th := T[i];
            idxHot := i;
          end if;
        end for;

        if idxHot == 1 then
          on1 := false;
        elseif idxHot == 2 then
          on2 := false;
        elseif idxHot == 3 then
          on3 := false;
        else
          on4 := false;
        end if;

        b[idxHot] := false;
      end for;
    end if;
  end when;

equation 
  // topology
  connect(room1.port,r12.port_a); connect(r12.port_b,room2.port);
  connect(room2.port,r23.port_a); connect(r23.port_b,room3.port);
  connect(room3.port,r34.port_a); connect(r34.port_b,room4.port);

  connect(room1.port,r1_out.port_a); connect(r1_out.port_b,outside.port);
  connect(room2.port,r2_out.port_a); connect(r2_out.port_b,outside.port);
  connect(room3.port,r3_out.port_a); connect(r3_out.port_b,outside.port);
  connect(room4.port,r4_out.port_a); connect(r4_out.port_b,outside.port);

  connect(h1.port,room1.port); connect(h2.port,room2.port);
  connect(h3.port,room3.port); connect(h4.port,room4.port);

  q1 = if on1 then Q_heater else 0;
  q2 = if on2 then Q_heater else 0;
  q3 = if on3 then Q_heater else 0;
  q4 = if on4 then Q_heater else 0;

  connect(q1E.y,h1.Q_flow); connect(q2E.y,h2.Q_flow);
  connect(q3E.y,h3.Q_flow); connect(q4E.y,h4.Q_flow);

  // ---- energy metric ----
  der(E_total) = q1 + q2 + q3 + q4;
  E_kWh       = E_total / 3.6e6;

  // ---- comfort metrics ----
  ok1 = (room1.T >= T_low) and (room1.T <= T_high);
  ok2 = (room2.T >= T_low) and (room2.T <= T_high);
  ok3 = (room3.T >= T_low) and (room3.T <= T_high);
  ok4 = (room4.T >= T_low) and (room4.T <= T_high);

  der(timeComfort1) = if ok1 then 1 else 0;
  der(timeComfort2) = if ok2 then 1 else 0;
  der(timeComfort3) = if ok3 then 1 else 0;
  der(timeComfort4) = if ok4 then 1 else 0;

  annotation (experiment(StartTime=0, StopTime=7200, Tolerance=1e-6, Interval=1));
end MultiRoomHeating4x2_AsyncJitter_Tuned_Energy;
