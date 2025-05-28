# Computer Organization
This is the final project of this course. This final project is required to design 3 types of CPUs by Verilog.
These 3 types of CPUs are: Single Cycle Implementation, Pipelined Implementation, and Datapath with Hazard Detection.

## Contents
- [Single Cycle Implementation](https://github.com/TzuHsiang417/Computer-Organization/blob/main/README.md#single-cycle-implementation)
- [Pipelined Implementation](https://github.com/TzuHsiang417/Computer-Organization/blob/main/README.md#pipelined-implementation)
- [Datapath with Hazard Detection](https://github.com/TzuHsiang417/Computer-Organization/blob/main/README.md#pipelined-implementation)

## Single Cycle Implementation
### Description:
The characteristic of a single-cycle implementation is that each instruction is fully executed before the next instruction begins.  
![Single Cycle Implementation](https://github.com/TzuHsiang417/Computer-Organization/blob/main/picture/Single%20Cycle%20Implementation.png)

**Report:**
[Report_Single Cycle Implementation.pdf](https://github.com/TzuHsiang417/Computer-Organization/blob/main/Report/Report_Single%20Cycle%20Implementation.pdf)

## Pipelined Implementation
### Description:
Pipeline implementation differs from single-cycle implementation by dividing the CPU into 5 stages. In each stage, pipeline registers are introduced to store values passed down from the previous stage. As a result, in pipeline implementation, multiple instructions can be executed simultaneously.  
<img src="https://github.com/TzuHsiang417/Computer-Organization/blob/main/picture/Pipelined%20Implementation.png" width="70%">

**Report:**
[Report_Pipelined Implementation.pdf](https://github.com/TzuHsiang417/Computer-Organization/blob/main/Report/Report_Pipelined%20Implementation.pdf)

## Datapath with Hazard Detection
### Description:
In Pipeline Implementation, hazards may occur, so we use a Forwarding unit to handle them. However, since the Forwarding unit can only forward values within the same clock cycle, it can only address certain types of Data Hazards. Therefore, we also need to add a Hazard detection unit in the ID stage to handle other Data Hazards.  
Branch Hazards cannot be solved by the Forwarding unit or Hazard detection unit, and when a Branch Hazard occurs, a stall will always happen. To reduce the number of stalls, we design a block in the ID stage to predict Branch Hazards in advance.  
Jump Hazards also cannot be solved by the Forwarding unit or Hazard detection unit, so we design a block to address the hazards caused by jumps.  
<img src="https://github.com/TzuHsiang417/Computer-Organization/blob/main/picture/Datapath%20with%20Hazard%20Detection.png" width="70%">

**Report:**
[Report_Datapath with Hazard Detection.pdf](https://github.com/TzuHsiang417/Computer-Organization/blob/main/Report/Report_Datapath%20with%20Hazard%20Detection.pdf)
