function demo_MPC_infHor_bicopter01
% Infinite horizon LQR applied on a planar UAV with an iterative linearization of the system plant.
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
% Written by Antonio Paolillo and Sylvain Calinon 
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

close all;
addpath('./m_fcts/');


%% Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nbData = 400; %Number of datapoints
nbIter = 1; %Number of reproductions

model.nbVarPos = 3; %Dimension of position data (here: x1,x2)
model.nbDeriv = 2; %Number of static & dynamic features (D=2 for [x,dx])
model.nbVarX = model.nbVarPos * model.nbDeriv; %Dimension of state vector in the tangent space
model.nbVarU = 2;
model.dt = 1E-2; %Time step duration
model.rfactor = 0.5;	%Control cost in LQR

% UAV parameters
robot.m = 2.5; % mass [kg]
robot.g = 9.81; % gravity acceleration [m/s^2]
robot.l = 0.5; % arms length [m]
robot.I = 1.2; % inertia [kg*m^2]

% Control cost matrix
R = eye(model.nbVarU) * model.rfactor;

%Task description
model.Mu = [2; 5; 0; zeros(model.nbVarPos,1)];
model.Sigma = blkdiag(eye(model.nbVarPos-1).*1E-2, 1E-1,eye(model.nbVarPos).*1E1);

% Control input at the reference point
u0 = 0.5 * robot.m * robot.g * ones(2,1);

animation = true;

% Waitbar
h_bar = waitbar(0, '0.00%','name','PROGRESS');


%% iLQR reproduction
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for n=1:nbIter
	u = zeros(model.nbVarU, 1);
	x = zeros(model.nbVarX, 1);
	for t=1:nbData
		
		frac = t / nbData;
		waitbar(frac, h_bar, sprintf ('Iteration n. %d of %d: %.2f%%', n, nbIter, 100*frac));
		
		% Log data for plots
		r(n).x(:,t) = x;
		r(n).u(:,t) = u;
		r(n).time(t) = t*model.dt;
		
		% Simulate UAV motion
		% 1- Update the system (acceleration)
		ddx = dynSys(x, u, robot);
		
		% 2- Update velocity (integration)
		x(model.nbVarPos+1:end) = x(model.nbVarPos+1:end) + ddx .* model.dt;
		% 3- Update position (integration)
		x(1:model.nbVarPos) = x(1:model.nbVarPos) + x(model.nbVarPos+1:end) .* model.dt;
		
		% Linearization of the system plant around the reference point
		[A, B] = computeTransferMatrices(model.Mu, u0, model, robot);
		% Linearization of the system plant around the current point
		%[A, B] = computeTransferMatrices(x, u, model, robot);
		
		% Iterative discrete LQR with infinite horizon
		% 1- Compute precision matrix
		Q = inv(model.Sigma);
		% 2- Solve Riccati's equation
		P = solveAlgebraicRiccati_eig_discrete(A, B*(R\B'), (Q+Q')/2);
		% 3- Compute the gain
		L = (B'*P*B + R) \ B'*P*A;
		
		% Control law (steering method)
		u = u0 + L * (model.Mu-x);
	end
end

close(h_bar)

%% Animation
if animation
	figure('name', 'animation');
	
	subplot(2,2,[1 3])
	hold on
	h1 = plot(zeros(1,nbData),zeros(1,nbData),'k--');
	h2 = plot(0,0,'r.','markersize',10);
	h3 = plot( zeros(3,1), zeros(3,1), 'b.-', 'linewidth',2,'markersize',10);
	h4 = plot(0,0,'k.','markersize',10,'linewidth',1.5);
	
	margin = robot.l + 0.1;
	var = [r(end).x(1,:), model.Mu(1)];
	x_min = min(var) - margin;
	x_max = max(var) + margin;
	var = [r(end).x(2,:), model.Mu(2,:)];
	y_min = min(var) - margin;
	y_max = max(var) + margin;
	
	axis([x_min x_max y_min y_max])
	xlabel('x [m]')
	ylabel('y [m]')
	axis equal; grid on; axis on; box on;
	
	subplot(2,2,2)
	hold on;
	h5 = plot(0,0,'k.','markersize',5,'linewidth',2);
	h6 = plot(zeros(1,nbData), zeros(1,nbData),'k');
	var = r(end).u(1,:); margin = 0.05;
	x_min = min(r(end).time(:)); x_max = max(r(end).time(:));
	margin = 0.1*abs(max(var)-min(var));
	y_min = min(var) - margin;
	y_max = max(var) + margin;
	axis([x_min x_max y_min y_max]);
	axis on; grid on; box on;
	xlabel('time [s]')
	ylabel('u_1 [kg m s^{-2}]')
	
	subplot(2,2,4)
	hold on;
	h7 = plot(0,0,'k.','markersize',5,'linewidth',2);
	h8 = plot(zeros(1,nbData), zeros(1,nbData),'k');
	var = r(end).u(2,:); margin = 0.05;
	margin = 0.1*abs(max(var)-min(var));
	y_min = min(var) - margin;
	y_max = max(var) + margin;
	axis([x_min x_max y_min y_max]);
	axis on; grid on; box on;
	xlabel('time [s]')
	ylabel('u_2 [kg m s^{-2}]')
	
	for i=1:nbData
		tic
		x_left = [r(end).x(1,i)-robot.l*cos(r(end).x(3,i)); r(end).x(2,i)-robot.l*sin(r(end).x(3,i))];
		x_right= [r(end).x(1,i)+robot.l*cos(r(end).x(3,i)); r(end).x(2,i)+robot.l*sin(r(end).x(3,i))];
		set(h1, 'xdata', [zeros(1,nbData-i),r(end).x(1,1:i)], 'ydata', [zeros(1,nbData-i),r(end).x(2,1:i)]);
		set(h2, 'xdata', model.Mu(1), 'ydata', model.Mu(2));
		set(h3, 'xdata', [x_left(1); r(end).x(1,i); x_right(1)],'ydata',[x_left(2); r(end).x(2,i); x_right(2)])
		set(h4, 'xdata', r(end).x(1,1),'ydata',r(end).x(2,1));
		set(h5, 'xdata', r(end).time(i), 'ydata', r(end).u(1,i));
		set(h6, 'xdata', [zeros(1,nbData-i),r(end).time(1:i)], 'ydata', [zeros(1,nbData-i),r(end).u(1,1:i)]);
		set(h7, 'xdata', r(end).time(i), 'ydata', r(end).u(2,i));
		set(h8, 'xdata', [zeros(1,nbData-i),r(end).time(1:i)], 'ydata', [zeros(1,nbData-i),r(end).u(2,1:i)]);
		drawnow()
		pause(model.dt - 0.001*toc)
		if i==1 || i==nbData
			pause(2)
		end
	end
end

%% Plot 2D
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%figure('position',[10 10 700 700],'color',[1 1 1],'name','x1-x2 plot');
figure('name', 'trajectory')
hold on; axis on; grid on; box on;
plotGMM(model.Mu(1:2,:), model.Sigma(1:2,1:2,:), [1 0 0], .5);
plot(model.Mu(1), model.Mu(2), 'r.','markersize',15);
%Plot reproduction samples
x_min = 0;
x_max = 0;
y_min = 0;
y_max = 0;
for n=1:nbIter
	plot(r(n).x(1,:), r(n).x(2,:), '-','linewidth',2,'color',ones(1,3)-ones(1,3).*.8.*n./nbIter);
	plot(r(n).x(1,1), r(n).x(2,1), '.','markersize',15,'color',ones(1,3)-ones(1,3).*.8.*n./nbIter);
	var = [r(n).x(1,:), model.Mu(1)];
	margin = 0.1 * abs(min(var)-max(var));
	if min(var) - margin < x_min
		x_min = min(var) - margin;
	end
	if max(var) + margin > x_max
		x_max = max(var) + margin;
	end
	var = [r(n).x(2,:), model.Mu(2,:)];
	margin = 0.1 * abs(min(var)-max(var));
	if min(var) - margin < y_min
		y_min = min(var) - margin;
	end
	if max(var) + margin > y_max
		y_max = max(var) + margin;
	end
end
axis([x_min x_max y_min y_max])
xlabel('x [m]')
ylabel('y [m]')
axis equal;


%% Timeline plot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
labList = {'$x_1$','$x_2$','$x_3$','$\dot{x}_1$','$\dot{x}_2$','$\dot{x}_3$'};
%figure('position',[720 10 1000 1300],'color',[1 1 1]);
figure('name', 'system state')
for j=1:model.nbVarX
	subplot(model.nbVarX,1,j);
	axis on; grid on; hold on; box on;
	if j>model.nbVarPos
		plot([1,nbData],[0,0],':','color',[.5 .5 .5]);
	end
	%Plot reproduction samples
	for n=1:nbIter
		plot(r(n).x(j,:), '-','linewidth',1,'color',ones(1,3)-ones(1,3).*.8.*n./nbIter);
	end
	ylabel(labList{j},'fontsize',14,'interpreter','latex');
end
xlabel('$t$','fontsize',14,'interpreter','latex');
% control input
figure('name','system input')
for j=1:model.nbVarU
	subplot(model.nbVarU,1,j);
	axis on; grid on; hold on; box on;
	%Plot reproduction samples
	for n=1:nbIter
		plot(r(n).u(j,:), '-','linewidth',1,'color',ones(1,3)-ones(1,3).*.8.*n./nbIter);
	end
	ylabel(['$u_' num2str(j) '$'],'fontsize',14,'interpreter','latex');
end
xlabel('$t$','fontsize',14,'interpreter','latex');

end


%%%%%%%%%%%%%%%%%
function f = dynSys(x,u,robot)
	m = robot.m;
	g = robot.g;
	l = robot.l;
	I = robot.I;

	f = zeros(3,1);

	f(1,1) = -m.^-1 .* (u(1) + u(2)) .* sin(x(3)); %ddx
	f(2,1) = m.^-1 .* (u(1) + u(2)) .* cos(x(3)) - g; %ddy
	f(3,1) = l/I .* (u(1) - u(2)); %ddt
end

%%%%%%%%%%%%%%%%%
function [A, B] = computeTransferMatrices(x, u, model, robot)
	m = robot.m;
	l = robot.l;
	I = robot.I;

	dfdx = zeros(3,1);
	dfdy = zeros(3,1);
	dfdt = zeros(3,1);
	dfdu1 = zeros(3,1);
	dfdu2 = zeros(3,1);

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