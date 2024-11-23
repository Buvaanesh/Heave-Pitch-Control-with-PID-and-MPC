Hello,

This is a custom project that uses the active vehicle suspension system (AVSS) to minimise the pitch and heave of the vehicle under road disturbances.

The AVSS plant model has 8 states, 4 inputs and 2 outputs which are heave and pitch ( same order in the output bus).

The control systems are PID and MPC for the linearised plant. 

The plant model is linearised at Y = (0,0) which is heave = 0, pitch = 0.

The linearisation points and techniques can be played around to further optimise the system in the future. 

The model does not include any artificial noise but incorporates the road disturbance.

The road disturbance is modelled are two scenarios 
	1. with a bump and dip.
	2. with rough road surface.

PS: You may need a  MPC toolbox installed with MATLAB and open the MPC properties to add the given "MPCDesignerSession" file to the block.

Enjoy playing around.



