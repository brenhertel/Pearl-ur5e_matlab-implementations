function demo_MPC_nullspace_online01
% Batch LQR recomputed in an online manner with nullspace formulation.
%
% If this code is useful for your research, please cite the related publication:
% @article{Girgin19,
% 	author="Girgin, H. and Calinon, S.",
% 	title="Nullspace Structure in Model Predictive Control",
% 	journal="arXiv:1905.09679",
% 	year="2019",
% 	pages="1--16"
% }
% 
% Copyright (c) 2019 Idiap Research Institute, http://idiap.ch/
% Written by Sylvain Calinon and Hakan Girgin
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
nbD = 50; %Size of the time window for MPC computation
nbPoints = 3; %Number of keypoints
nbVarPos = 2; %Dimension of position data (here: x1,x2)
nbDeriv = 2; %Number of static and dynamic features (nbDeriv=2 for [x,dx] and u=ddx)
nbVar = nbVarPos * nbDeriv; %Dimension of state vector
dt = 1E-2; %Time step duration
rfactor = 1E-8; %dt^nbDeriv;	%Control cost in LQR
R = speye((nbD-1)*nbVarPos) * rfactor; %Control cost matrix


%% Dynamical System settings (discrete version)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
A1d = zeros(nbDeriv);
for i=0:nbDeriv-1
	A1d = A1d + diag(ones(nbDeriv-i,1),i) * dt^i * 1/factorial(i); %Discrete 1D
end
B1d = zeros(nbDeriv,1); 
for i=1:nbDeriv
	B1d(nbDeriv-i+1) = dt^i * 1/factorial(i); %Discrete 1D
end
A = kron(A1d, speye(nbVarPos)); %Discrete nD
B = kron(B1d, speye(nbVarPos)); %Discrete nD

%Build Su and Sx transfer matrices
Su = sparse(nbVar*nbD, nbVarPos*(nbD-1));
Sx = kron(ones(nbD,1), speye(nbVar));
M = B;
for n=2:nbD
	id1 = (n-1)*nbVar+1:nbD*nbVar;
	Sx(id1,:) = Sx(id1,:) * A;
	id1 = (n-1)*nbVar+1:n*nbVar; 
	id2 = 1:(n-1)*nbVarPos;
	Su(id1,id2) = M;
	M = [A*M(:,1:nbVarPos), M]; 
end


%% Principal task setting
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tl = linspace(1,nbData,nbPoints+1);
tl = round(tl(2:end)); 
MuQ0 = zeros(nbVar*nbData,1); 
Q0 = zeros(nbVar*nbData);
for t=1:length(tl)
	id0(:,t) = [1:nbVarPos] + (tl(t)-1) * nbVar;
	Q0(id0(:,t), id0(:,t)) = eye(nbVarPos);
	MuQ0(id0(:,t)) = rand(nbVarPos,1) - 0.5;
end


%% Secondary task setting (simulating smooth random perturbations)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Precomputation of basis functions to generate structured stochastic u through Bezier curves
nbRBF = 8;
H = zeros(nbRBF,nbData-1);
tlr = linspace(0,1,nbData-1);
nbDeg = nbRBF - 1;
for i=0:nbDeg
	H(i+1,:) = factorial(nbDeg) ./ (factorial(i) .* factorial(nbDeg-i)) .* (1-tlr).^(nbDeg-i) .* tlr.^i; %Bernstein basis functions
end
w = randn(nbVarPos,nbRBF) .* 1E1; %Random weights
u0 = reshape(w * H, [nbVarPos*(nbData-1), 1]); %Reconstruction of signals by weighted superposition of basis functions

% figure; hold on;
% % plot(u0(1:2:end), u0(2:2:end), 'k-');
% x = zeros(nbVar,1);
% for t=1:nbData-1
% 	x = A * x + B * u0((t-1)*nbVarPos+[1:nbVarPos]); %Update state with first control command
% 	plot(x(1), x(2), 'k.');
% end
% pause; 
% close all;
% return


%% Reproduction
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x = zeros(nbVar,1);
rx = zeros(nbVar,nbData);
for t=1:nbData
	rx(:,t) = x; %Log data 
	
	id = [t:min(t+nbD-1,nbData), ones(1,t-nbData+nbD-1)]; %Time steps involved in the MPC computation
	id2=[];
	for s=1:nbD
		id2 = [id2, [1:nbVar] + (id(s)-1) * nbVar];
	end
	
	id = [t:min(t+nbD-1,nbData-1), ones(1,t-nbData+nbD-1)]; %Time steps involved in the MPC computation
	id3=[];
	for s=1:nbD-1
		id3 = [id3, [1:nbVarPos] + (id(s)-1) * nbVarPos];
	end
	
	MuQ = MuQ0(id2,1);
	Q = Q0(id2,id2);
	
	%Reproduction with nullspace planning
	[V,D] = eig(Q);
	U = V * D.^.5;
	J = U' * Su; %Jacobian

	%Left pseudoinverse solution
	% pinvJ = pinv(J);
	pinvJ = (J' * J + R) \ J'; %Left pseudoinverse
	N = speye((nbD-1)*nbVarPos) - pinvJ * J; %Nullspace projection matrix
	u1 = pinvJ * U' * (MuQ - Sx * x); %Principal task 

	u = u1 + N * u0(id3);
% 	u = u0(id3);
	
	x = A * x + B * u(1:nbVarPos); %Update state with first control command
end


%% Plot 2D
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('position',[10 10 1000 1000]); hold on; axis off;
plot(rx(1,:), rx(2,:), '-','linewidth',2,'color',[0 0 0]);
plot(rx(1,1), rx(2,1), '.','markersize',35,'color',[0 0 0]);
plot(MuQ0(id0(1,:)), MuQ0(id0(2,:)), '.','markersize',35,'color',[.8 0 0]);
for t=1:length(tl)
	text(MuQ0(id0(1,t)), MuQ0(id0(2,t))+2E-2, num2str(t));
end
axis equal; 
% print('-dpng','graphs/MPC_nullspace_online01.png');

pause;
close all;