model MultiRoomHeating4x2_AsyncJitter_Tuned
  import Modelica.Thermal.HeatTransfer.Components.*;
  import Modelica.Thermal.HeatTransfer.Sources.*;
  import Modelica.Blocks.Sources.*;

  // ---- Parameters ----
  parameter Real C        = 12000 "Room air heat capacity (J/K)";
  parameter Real G_wall   = 15    "Coupling conductance between adjacent rooms (W/K)";
  parameter Real G_out    = 20    "Loss conductance from each room to outside (W/K)";
  parameter Real T_out_K  = 10 + 273.15 "Outside temperature (K)";
  parameter Real T0_K     = 18 + 273.15 "Initial room temperature (K)";
  parameter Real Q_heater = 2000  "Heater power when ON (W)";
  parameter Real T_low    = 20 + 273.15 "Comfort band lower limit (K)";
  parameter Real T_high   = 24 + 273.15 "Comfort band upper limit (K)";

  // --- Base period and jitter (for smoother independence) ---
  parameter Real Pbase = 300 "Base decision period (s)";

  // Jitter values: small +/- differences from Pbase
  parameter Real J1 = -20;
  parameter Real J2 = +15;
  parameter Real J3 = -35;
  parameter Real J4 = +25;

  // Effective periods = base + jitter
  parameter Real P1 = Pbase + J1;  // 280 s
  parameter Real P2 = Pbase + J2;  // 315 s
  parameter Real P3 = Pbase + J3;  // 265 s
  parameter Real P4 = Pbase + J4;  // 325 s

  // Initial offsets (start times) for each local clock
  parameter Real O1 = 0;
  parameter Real O2 = 60;
  parameter Real O3 = 130;
  parameter Real O4 = 210;

  // ---- Rooms / thermal network ----
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

  // ---- Heaters ----
  PrescribedHeatFlow h1;
  PrescribedHeatFlow h2;
  PrescribedHeatFlow h3;
  PrescribedHeatFlow h4;

  Real q1, q2, q3, q4;
  RealExpression q1E(y = q1);
  RealExpression q2E(y = q2);
  RealExpression q3E(y = q3);
  RealExpression q4E(y = q4);

protected 
  // ON/OFF states updated on local ticks
  Boolean on1(start = true),  on2(start = false);
  Boolean on3(start = true),  on4(start = false);

  // next local wake-up times
  Real t1(start = O1), t2(start = O2), t3(start = O3), t4(start = O4);

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
      b := {on1,     on2,     on3,     on4    };

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
  // ----- Thermal topology -----
  connect(room1.port, r12.port_a); connect(r12.port_b, room2.port);
  connect(room2.port, r23.port_a); connect(r23.port_b, room3.port);
  connect(room3.port, r34.port_a); connect(r34.port_b, room4.port);

  connect(room1.port, r1_out.port_a); connect(r1_out.port_b, outside.port);
  connect(room2.port, r2_out.port_a); connect(r2_out.port_b, outside.port);
  connect(room3.port, r3_out.port_a); connect(r3_out.port_b, outside.port);
  connect(room4.port, r4_out.port_a); connect(r4_out.port_b, outside.port);

  // ----- Heaters attached -----
  connect(h1.port, room1.port);
  connect(h2.port, room2.port);
  connect(h3.port, room3.port);
  connect(h4.port, room4.port);

  // heater power commands
  q1 = if on1 then Q_heater else 0;
  q2 = if on2 then Q_heater else 0;
  q3 = if on3 then Q_heater else 0;
  q4 = if on4 then Q_heater else 0;

  connect(q1E.y, h1.Q_flow);
  connect(q2E.y, h2.Q_flow);
  connect(q3E.y, h3.Q_flow);
  connect(q4E.y, h4.Q_flow);

  annotation (experiment(
      StartTime = 0,
      StopTime  = 7200,
      Tolerance = 1e-6,
      Interval  = 1));
end MultiRoomHeating4x2_AsyncJitter_Tuned;
