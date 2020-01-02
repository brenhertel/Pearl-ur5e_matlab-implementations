% This is a first version of optimal kinematic filtering  (IEEE T-RO Meirovitch et al., 2016). 
% The function is an alternative to FIR and IIR low-pass filtering methods and is especially adequate for signals / trajectories representing smooth natural behaviors such as hand movements and handwriting trajectories. 
% 
% The basic usage is: 
% [x, y]=filter_JA(trj_ns); 
% which computes the smoothest representation [1] of the noisy trajectory trj_ns ([Nx2]). The output [x,y] is the Cartesian representation of the trajectory and its five derivatives. The constant lambda is a Lagrange multiplier. 
% Unlike FIR and IIR filtering, kinematic filtering allows to define any desired boundary conditions for the output trajectory. The output is guaranteed to minimize the error with respect to the inputs and those constraints.
% 
% Full usage is:
% [x, y]=filter_JA(trj,lambda,tt,endpoints,vel,acc,direction,method)
% 
% For lambda=0 the output trajectory is based on the minimum jerk model [1], ie ignoring the noisy input and generating the smoothest trajectory wrt to the boundary conditions. 
% For lambda >> 0 the output trajectory is equal to the noisy input.
% 
% ==== Citation ====================== 
% If you use this code, please cite this paper: 
% Y. Meirovitch, D. Bennequin and T. Flash "Geometrical Invariance and Smoothness Maximization for Task-Space Movement Generation", 2016. 
% IEEE-Transaction on Robotics, 2016. 
% =================================
% 
% ==== Examples =====
% 
% Example 1 
% Adaptively changing Lambda and boundary conditions 
% Run: demoBoundaryCondition for a demo 
% demoBoundaryCondition()
% 
% % Example 2 
% % prepare a template to be smoothly regenerated 
% A = [0,0]; 
% B = [1/2,1]; 
% C = [1,0];
% 
% tt_lag = 0:1/100:1; 
% rr1 = [1-tt_lag(1:end-1); tt_lag(1:end-1) ]' * [A;B]; 
% rr2 = [1-tt_lag; tt_lag ]' * [B;C]; 
% rr = [rr1;rr2]; 
% tt = [tt_lag(1:end-1) tt_lag+1];
% 
% % define smoothness-accuracy parameter 
% lambda = 8;
% 
% % regenerate motion with new boundary condition 
% [x, y]=filter_JA(rr,lambda,tt,[A + [.2 0], C+[0 +.1]],[]);
% 
% % plot result 
% figure; 
% plot(rr(:,1),rr(:,2),'linewidth',2); hold all; % original trajectory (input) 
% plot(x(:,1),y(:,1),'linewidth',2); % smooth trajectory (output)
% 
% % Example 3 
% % For first use, call 
% kinematicFiltering()
% % 
% % [1] Smoothness is defined based on Mean Squared Derivative model. For n=3 this is known as the minimum jerk model: Flash and Hogan, 1985.