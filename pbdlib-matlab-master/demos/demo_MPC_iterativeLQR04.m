function demo_MPC_iterativeLQR04
% Control of a spring attached to a point with iterative linear quadratic tracking (with feedback and feedforward terms).
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
% Written by Sylvain Calinon, http://calinon.ch/
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
nbVarPos = 2; %Dimension of position data (here: x1,x2)
nbDeriv = 2; %Number of static and dynamic features (nbDeriv=2 for [x,dx] and u=ddx)
nbVar = nbVarPos * (nbDeriv+1); %Dimension of state vector
dt = 1E-2; %Time step duration
rfactor = 1E-4;	%Control cost in LQR
kP = 0; %Stiffness gain (initial estimate)
kV = (2*kP)^.5; %Damping gain (with ideal underdamped damping ratio)
%kV = 2*kP^.5; %Damping (for critically damped system)

nbRepros = 5; %Number of reproductions
nbData = 200; %Number of datapoints


%% Dynamical System settings (discrete version)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Dynamical System settings (discrete version)
Ac = kron([0, 1, 0; -kP, -kV, kP; 0, 0, 0], eye(nbVarPos));
A = eye(nbVar) + Ac * dt;
B = kron([dt^2/2; dt; 0], eye(nbVarPos));
%Control cost matrix
R = eye(nbVarPos) * rfactor;


%% Setting tracking task
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
xTar = [0; 3]; %Equilibrium point of the spring
Mu = [zeros(nbVarPos,1); zeros(nbVarPos,1); xTar];
Gamma = blkdiag(eye(nbVarPos)*1E1, zeros(nbVarPos), zeros(nbVarPos));


%% Iterative LQR reproduction (finite horizon, discrete version)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
P = zeros(nbVar,nbVar,nbData);
P(:,:,end) = Gamma;
d = zeros(nbVar, nbData);
Q = Gamma;

%Backward computation
for t=nbData-1:-1:1	
	P(:,:,t) = Q - A' * (P(:,:,t+1) * B / (B' * P(:,:,t+1) * B + R) * B' * P(:,:,t+1) - P(:,:,t+1)) * A;
	d(:,t) = (A' - A' * P(:,:,t+1) * B / (R + B' * P(:,:,t+1) * B) * B' ) * (P(:,:,t+1) * (A * Mu - Mu) + d(:,t+1));
end

%Reproduction with feedback (FB) and feedforward (FF) terms
for n=1:nbRepros
	X = [10; 10; zeros(2,1); xTar] + [randn(2,1)*3E0; zeros(4,1)]; 
	r(n).X0 = X;
	for t=1:nbData
		r(n).Data(:,t) = X; %Log data
		K = (B' * P(:,:,t) * B + R) \ B' * P(:,:,t) * A; %FB gain
		
% 		%Test ratio between kp and kv
% 		kp = eigs(K(:,1:2));
% 		kv = eigs(K(:,3:4));
% 		ratio = kv ./ (2.*kp).^.5
		
% 		figure; hold on;
% 		plotGMM(zeros(2,1), K(:,1:2), [.8 0 0],.3);
% 		plotGMM(zeros(2,1), K(:,3:4), [.8 0 0],.3);
% 		axis equal;
% 		pause;
% 		close all;

		M = -(B' * P(:,:,t) * B + R) \ B' * (P(:,:,t) * (A * Mu - Mu) + d(:,t)); %FF term
		u = K * (Mu - X) + M; %Acceleration command with FB and FF terms
		X = A * X + B * u; %Update of state vector
	end
end


%% Plot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('position',[10 10 1200 1200],'color',[1 1 1]); hold on; axis off;
for n=1:nbRepros
	plot(r(n).Data(1,:), r(n).Data(2,:), '-','linewidth',1,'color',[.8 0 0]);
	plot(r(n).Data(1,:), r(n).Data(2,:), '.','markersize',6,'color',[.6 0 0]);
end
%plotGMM(Mu(1:2,:), Sigma(1:2,1:2,:), [0.5 0.5 0.5]);
h(1) = plot(Mu(1,:), Mu(2,:), 'k.','markersize',20, 'color', [0.5 0.5 0.5]);
h(2) = plot(xTar(1),xTar(2),'k+','markersize',20,'linewidth',2);
legend(h,{'Target to reach','Equilibrium point of the spring'});
axis equal;

%print('-dpng','graphs/demo_iterativeLQR04.png');
pause;
close all;