function demo_MPC_infHor_incremental02
% Infinite horizon LQR with an iterative re-estimation of the system plant linear system (example with pendulum). 
% (similar example as in "LQR-RRT∗: Optimal Sampling-Based Motion Planning with Automatically Derived Extension Heuristics")
%
% If this code is useful for your research, please cite the related publication:
% @incollection{Calinon19chapter,
% 	author="Calinon, S. and Lee, D.",
% 	title="Learning Control",
% 	booktitle="Humanoid Robotics: a Reference",
% 	publisher="Springer",
% 	editor="Vadakkepat, P. and Goswami, A.", 
% 	year="2019",
% 	pages="1261--1312",
% 	doi="10.1007/978-94-007-6046-2_68"
% }
% 
% Copyright (c) 2019 Idiap Research Institute, http://idiap.ch/
% Written by Sylvain Calinon and Danilo Bruno
% 
% This file is part of PbDlib, http://www.idiap.ch/software/pbdlib/
% 
% PbDlib is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License version 3 as
% published by the Free Software Foundation.
% 
% PbDlib is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
% GNU General Public License for more details.
% 
% You should have received a copy of the GNU General Public License
% along with PbDlib. If not, see <http://www.gnu.org/licenses/>.

addpath('./m_fcts/');


%% Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nbData = 200; %Number of datapoints
nbIter = 1; %Number of reproductions

model.nbVarPos = 1; %Dimension of position data (here: q)
model.nbDeriv = 2; %Number of derivatives for the state space 
model.nbVarX = model.nbVarPos * model.nbDeriv; %Dimension of state space (here: [q,dq])
model.nbVarU = 1; %Dimension of control command (here: torque)
model.dt = 1E-2; %Time step duration
model.rfactor = 1E-4;	%Control cost in LQR 

%Control cost matrix
R = eye(model.nbVarU) * model.rfactor;


%% Task description
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
model.Mu = [pi/2; 0]; %pi/2
model.Sigma = diag([1E-2, 1E-1]);
Q = inv(model.Sigma); %Precision matrix


%% iLQR reproduction
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for n=1:nbIter
	u = 0;
	x = [-pi/2; 0];
	
	for t=1:nbData
		%Log data
		r(n).x(:,t) = x;
		
		%Simulation
		ddx = dynSys(x, u); %Compute acceleration
		x(2) = x(2) + ddx .* model.dt; %Update velocity
		x(1) = x(1) + x(2) .* model.dt; %Update position
			
		%Linearization of system plant
		[A, B] = computeTransferMatrices(x, model);

		%Iterative discrete LQR with infinite horizon
		P = solveAlgebraicRiccati_eig_discrete(A, B*(R\B'), (Q+Q')/2);
		K = (B'*P*B + R) \ B'*P*A; %Feedback gain (discrete version)
		u = K * (model.Mu - x); %Compute acceleration (with only feedback terms)
% 		x = A*x + B*u;
		
		r(n).c(t) = (model.Mu-x)'*Q*(model.Mu-x) + u'*R*u; %Value of the cost function
		r(n).u(:,t) = u;
	end
end


%% Plot 2D
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('position',[10 10 700 700],'color',[1 1 1],'name','x1-x2 plot'); hold on; axis off;
plotGMM(model.Mu(1:2,:), model.Sigma(1:2,1:2,:), [.8 0 0], .5);
plot(model.Mu(1,:), model.Mu(2,:), '.','markersize',20,'color',[.8 0 0]);
%Plot reproduction samples
for n=1:nbIter
	plot(r(n).x(1,:), r(n).x(2,:), '-','linewidth',2,'color',ones(1,3)-ones(1,3).*.8.*n./nbIter);
	plot(r(n).x(1,1), r(n).x(2,1), '.','markersize',20,'color',ones(1,3)-ones(1,3).*.8.*n./nbIter);
end
axis equal; 


%% Timeline plot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
labList = {'$x_1$','$x_2$','$x_3$','$\dot{x}_1$','$\dot{x}_2$','$\dot{x}_3$'};
figure('position',[720 10 1000 1300],'color',[1 1 1]); 
for j=1:model.nbVarX
	subplot(model.nbVarX+model.nbVarU,1,j); hold on;
	plot([1,nbData],[model.Mu(j) model.Mu(j)],':','color',[.5 .5 .5]);
	%Plot reproduction samples
	for n=1:nbIter
		plot(r(n).x(j,:), '-','linewidth',1,'color',ones(1,3)-ones(1,3).*.8.*n./nbIter);
	end
	ylabel(labList{j},'fontsize',14,'interpreter','latex');
end
for j=1:model.nbVarU
	subplot(model.nbVarX+model.nbVarU,1,model.nbVarX+j); hold on;
	%Plot reproduction samples
	for n=1:nbIter
		plot(r(n).u(j,:), '-','linewidth',1,'color',ones(1,3)-ones(1,3).*.8.*n./nbIter);
	end
	ylabel(['$u_' num2str(j) '$'],'fontsize',14,'interpreter','latex');
end
xlabel('$t$','fontsize',14,'interpreter','latex');


%% Animated plot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('PaperPosition',[0 0 8 8],'position',[10,10,1000,1000],'color',[1 1 1]); hold on; axis off;
plotArm(model.Mu(1), 1, [0;0;0], .03, [.8 0 0]);
for t=round(linspace(1,nbData,20))
	colTmp = [.9,.9,.9] - [.7,.7,.7] * t/nbData;
	plotArm(r(1).x(1,t), 1, [0;0;t*0.1], .02, colTmp);
end
axis equal;

figure; hold on;
plot(r(1).c,'k-');

pause;
close all;
end

%%%%%%%%%%%%%%%%%
function f = dynSys(x,u)
	b = 0.1;
	g = 9.81;
	f = u - b .* x(2,:) - g .* cos(x(1,:));
end

%%%%%%%%%%%%%%%%%
function [A, B] = computeTransferMatrices(x, model)
	b = 0.1;
	g = 9.81;
	
	dfdx = g .* sin(x(1,:));
	dfddx = -b;
	dfdu = 1;

	Ac = [0, 1; dfdx, dfddx];
	Bc = [0; dfdu];

	%Conversion to discrete linear system
	A = eye(model.nbVarX) + Ac * model.dt;
	B = Bc * model.dt;
end