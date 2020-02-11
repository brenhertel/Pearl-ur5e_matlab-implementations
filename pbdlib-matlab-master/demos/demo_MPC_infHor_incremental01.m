function demo_MPC_infHor_incremental01
% Infinite horizon LQR with an iterative re-estimation of the system plant linear system (example with planar UAV). 
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
nbData = 300; %Number of datapoints
nbIter = 1; %Number of reproductions

model.nbVarPos = 3; %Dimension of position data (here: x=x1,x2,theta)
model.nbDeriv = 2; %Number of derivatives for the state space
model.nbVarX = model.nbVarPos * model.nbDeriv; %Dimension of state space (here: [x,dx])
model.nbVarU = 2; %Dimension of control command (here: force of two propellers)
model.dt = 1E-2; %Time step duration
model.rfactor = 1E-4;	%Control cost in LQR 

%Control cost matrix
R = eye(model.nbVarU) * model.rfactor;


%% Task description
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
model.Mu = [1; 1; 0; zeros(model.nbVarPos,1)];
model.Sigma = blkdiag(eye(model.nbVarPos).*1E-2, eye(model.nbVarPos).*1E-1);
Q = inv(model.Sigma); %Precision matrix
u00 = repmat(0.5 * 2.5 * 9.81, 2, 1);


%% iLQR reproduction
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for n=1:nbIter
	u = zeros(model.nbVarU, 1);
	x = zeros(model.nbVarX, 1);
	for t=1:nbData
		%Log data
		r(n).x(:,t) = x;
		r(n).u(:,t) = u;
		
		%Simulation
		ddx = dynSys(x, u); %Compute acceleration
		x(model.nbVarPos+1:end) = x(model.nbVarPos+1:end) + ddx .* model.dt; %Update velocity
		x(1:model.nbVarPos) = x(1:model.nbVarPos) + x(model.nbVarPos+1:end) .* model.dt; %Update position
	
		%Linearization of system plant 
		u1 = 0.5 * ( (2.5*(ddx(2)+9.81))/cos(x(3)) + (1.2*ddx(3))/0.5 );
		u2 = 0.5 * ( (2.5*(ddx(2)+9.81))/cos(x(3)) - (1.2*ddx(3))/0.5 );
		[A, B] = computeTransferMatrices(x, [u1; u2], model);

		%Iterative discrete LQR with infinite horizon
		P = solveAlgebraicRiccati_eig_discrete(A, B*(R\B'), (Q+Q')/2);
		L = (B'*P*B + R) \ B'*P*A; %Feedback gain (discrete version)
		u = L * (model.Mu-x) + u00; %Compute acceleration (with only feedback terms)
% 		x = A*x + B*u;
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

pause;
close all;
end

%%%%%%%%%%%%%%%%%
function f = dynSys(x,u)
	m = 2.5;
	g = 9.81;
	l = 0.5;
	I = 1.2;
	f = zeros(3,1);
	f(1,1) = -m.^-1 .* (u(1) + u(2)) .* sin(x(3)); %ddx
	f(2,1) = m.^-1 .* (u(1) + u(2)) .* cos(x(3)) - g; %ddy
	f(3,1) = l/I .* (u(1) - u(2)); %ddt
end

%%%%%%%%%%%%%%%%%
function [A, B] = computeTransferMatrices(x, u, model)
	m = 2.5;
% 	g = 9.81;
	l = 0.5;
	I = 1.2;

	%syms x;
	dfdx = zeros(3,1);
	dfdy = zeros(3,1);
	dfdt = zeros(3,1);
	dfdu1 = zeros(3,1);
	dfdu2 = zeros(3,1);

% 		f = dynSys(x(:,t), u(:,t));

	dfdx(1) = 0;
	dfdx(2) = 0;
	dfdx(3) = 0;

	dfdy(1) = 0;
	dfdy(2) = 0;
	dfdy(3) = 0;

	dfdt(1) = -m.^-1 .* (u(1) + u(2)) .* cos(x(3)); %*dx?
	dfdt(2) = -m.^-1 .* (u(1) + u(2)) .* sin(x(3)); %*dx?
	dfdt(3) = 0;

	dfdu1(1) = -m.^-1 .* sin(x(3));
	dfdu1(2) = m.^-1 .* cos(x(3));
	dfdu1(3) = l/I;

	dfdu2(1) = -m.^-1 .* sin(x(3));
	dfdu2(2) = m.^-1 .* cos(x(3));
	dfdu2(3) = -l/I;

	Ac = [zeros(model.nbVarPos), eye(model.nbVarPos); [dfdx, dfdy, dfdt], zeros(model.nbVarPos)];
	Bc = [zeros(model.nbVarPos,model.nbVarU); [dfdu1, dfdu2]];

	%Conversion to discrete linear system
	A = eye(model.nbVarX) + Ac * model.dt;
	B = Bc * model.dt;
end