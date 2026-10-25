# Modeling and Control of a Multi-Room Heating System with Two Heaters

### Project Overview

This project focuses on modeling and simulating a four-room heating system using **OpenModelica** to compare synchronous and asynchronous control strategies. The main challenge is maintaining comfortable temperatures across four interconnected rooms while allowing only two heaters to operate at a time.

The project investigates how centralized and decentralized control strategies affect temperature regulation, energy consumption, heater switching, and system reliability. Through simulation experiments, it explores how different timing mechanisms, communication delays, and heater failures influence overall system performance.

### Problem Statement

Managing temperature across multiple rooms becomes challenging when heating resources are limited. In this system, four interconnected rooms share only two available heaters, making it necessary to allocate heating efficiently while maintaining temperatures between 20°C and 24°C.

The project aims to investigate how synchronous and asynchronous control strategies manage limited heating resources and how timing differences, communication delays, and system faults affect comfort, energy efficiency, and reliability.

### Technologies Used

- **OpenModelica:** Modeling and simulation environment.
- **Modelica:** Equation-based modeling language.
- **Modelica Thermal Library:** Modeling heat transfer, thermal capacitance, and heating components.
- **Hybrid Systems Modeling:** Combining continuous temperature dynamics with discrete heater-switching decisions.
- **Control Systems:** Centralized and decentralized control strategies.

### System Design

The system consists of four rooms arranged in a linear configuration:

`Room 1 ↔ Room 2 ↔ Room 3 ↔ Room 4`

Each room exchanges heat with its neighboring rooms and loses heat to the outside environment.

**System Constraints:**
- Four interconnected rooms.
- A maximum of two heaters operating simultaneously.
- Desired temperature range: 20°C–24°C.
- Outside temperature: 10°C.
- Heater power: 2000 W per active heater.

### Control Strategies

**1. Synchronous (Centralized) Control**

The synchronous controller uses a global clock to evaluate all room temperatures at fixed 300-second intervals. It identifies the two coldest rooms and assigns the available heaters to them.

This approach provides predictable heater scheduling but can cause synchronized switching and temperature fluctuations.

**2. Asynchronous (Decentralized) Control**

The asynchronous controller allows individual rooms to monitor their temperatures using independent timers and small timing offsets.

Each room requests heating based on its temperature, while a shared safety constraint ensures that no more than two heaters operate simultaneously.

This approach produces smoother heater switching and improves responsiveness to changing room conditions.

### Experiments Conducted

The project compares both control strategies through simulation experiments.

**1. Temperature Regulation**

Analyzed room temperatures over time to determine how effectively each controller maintains the desired comfort range.

**2. Energy Consumption**

Measured heater usage to compare energy efficiency between synchronous and asynchronous control strategies.

**3. Heater Switching Behavior**

Observed how centralized scheduling and independent room timers affect heater activation patterns.

**4. Timing Jitter**

Investigated how small differences in controller timing influence coordination and system stability.

**5. Fault and Communication Delay Testing**

Evaluated system behavior under simulated communication delays, temporary controller failures, and heater faults.

### Results and Findings

The simulation results showed that asynchronous control generally provided smoother temperature regulation and improved system resilience compared with synchronous control.

| Evaluation Metric | Synchronous Control | Asynchronous Control |
|---|---|---|
| Temperature Comfort | 85% | 92% |
| Heater Switching | Fixed and synchronized | Independent and staggered |
| Energy Consumption | Slightly higher | Slightly lower |
| Fault Tolerance | Sensitive to global failures | More resilient to local failures |
| Temperature Stability | More fluctuations | Smoother regulation |

**Key Findings:**
- Asynchronous control achieved approximately **92% comfort time**, compared with 85% for synchronous control.
- Decentralized decision-making reduced abrupt heater-switching patterns.
- Timing jitter helped prevent controllers from repeatedly switching at the same time.
- Asynchronous control maintained partial functionality during local faults.
- Both controllers were designed to respect the two-heater resource constraint.

### How to Run the Project

1. Install **OpenModelica** from https://openmodelica.org/.
2. Download or clone this GitHub repository.
3. Open **OMEdit (OpenModelica Connection Editor)**.
4. Load the provided `.mo` model files.
5. Select the synchronous or asynchronous experiment model.
6. Run the simulation using the parameters defined in the model.
7. Plot room temperatures, heater states, and energy consumption to compare the results.

*Note: The exact model names and simulation settings depend on the source files included in the repository.*

### Conclusion

This project demonstrates how different control strategies influence the performance of a multi-room heating system with limited resources. The results suggest that asynchronous decentralized control provides smoother temperature regulation, improved comfort, and greater resilience to local failures compared with synchronous centralized control.

Through this individual project, I gained practical experience in **cyber-physical systems, hybrid systems modeling, control algorithms, simulation, and performance evaluation** using OpenModelica.

### Future Improvements

Potential extensions include:

- Implementing model predictive control for improved energy optimization.
- Introducing variable communication delays and sensor failures.
- Modeling more complex building layouts and room characteristics.
- Integrating real-time sensors and embedded hardware for practical testing.


## These are my core project models

	1	MultiRoomHeating4x2.mo - Base thermal model
	2	MultiRoomHeating4x2_Scheduled.mo - Synchronous centralized scheduling
	3	MultiRoomHeating4x2_SyncPulse_v3.mo - Clean synchronous version using timed tick
	4	MultiRoomHeating4x2_AsyncJitter.mo - Basic asynchronous version
	5	MultiRoomHeating4x2_AsyncJitter_Tuned.mo - Improved asynchronous independence
	1	MultiRoomHeating4x2_AsyncJitter_Faults.mo - Communication delay / fault simulation
	1	MultiRoomHeating4x2_AsyncJitter_Tuned_Energy.mo -  Asynchronous energy and comfort 
	2	MultiRoomHeating4x2_Scheduled_Energy.mo - synchronous energy and comfort  
## libraries
I manually installed Modelica_StateGraph2 or the RealTimeCoordinationLibrary:
	•	     RealTimeCoordinationLibrary
	•	     Modelica_StateGraph2  

## Plots:
	•	synchronous_plot.png
	•	 async_plot.png
	•	 energy and comfort _comparison.png (Synchronous)
	•	 energy and comfort_comparison.png(Asynchronous)
	•	 fault_scenario_plot.png
