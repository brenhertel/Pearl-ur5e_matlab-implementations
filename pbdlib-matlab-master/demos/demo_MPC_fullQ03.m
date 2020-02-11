function demo_MPC_fullQ03
% Batch LQT exploiting full Q matrix to constrain the motion of two agents in a ballistic task.
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
nbData = 200; %Number of datapoints
nbAgents = 2; %Number of agents
nbVarPos = 2 * nbAgents; %Dimension of position data (here: x1,x2)
nbDeriv = 2; %Number of static and dynamic features (nbDeriv=2 for [x,dx] and u=ddx)
nbVar = nbVarPos * (nbDeriv+1); %Dimension of state vector
dt = 1E-2; %Time step duration
rfactor = 1E-8; %dt^nbDeriv;	%Control cost in LQR
R = speye((nbData-1)*nbVarPos) * rfactor; %Control cost matrix


%% Dynamical System settings (augmented state space, discrete version)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Ac0 = kron([0, 1, 0; 0, 0, 0; 0, 0, 0], speye(2));
% Ad0 = speye(6) + Ac0 * dt;
% Ad0 = kron(Ad0, speye(nbAgents)); %Discrete nD for several agents

Ac1 = kron([0, 1, 0; 0, 0, 5; 0, 0, 0], speye(2));
Ad1 = speye(6) + Ac1 * dt;

Bc1 = kron([0; 1; 0], speye(2));
Bd1 = Bc1 * dt;

Ad = kron(Ad1, speye(nbAgents)); %Discrete nD for several agents
Bd = kron(Bd1, speye(nbAgents)); %Discrete nD for several agents

% % Bd(:,3:4)=0;
% full(Bd)

% full(Ad0)
% full(Ad)
% Ad(5,9)=0;
% full(Ad)
% return

%Build Sx and Su transfer matrices(for heterogeneous A and B)
A = zeros(nbVar,nbVar,nbData);
B = zeros(nbVar,nbVarPos,nbData);
for t=1:nbData
	A(:,:,t) = Ad;
% 	A(5:8,9:12,t) = 0; %no gravity on agents
% 	A(7:8,11:12,t) = 0; %no gravity on second agent
% 	A(2:2:end,2:2:end,t) = Ad0(2:2:end,2:2:end);
	if t==1
% 		A(:,:,t) = Ad0;
% 		B(:,:,t) = Bd;
		B(:,1:2,t) = Bd(:,1:2);
	end
	if t==1
		B(:,3:4,t) = Bd(:,3:4);
	end
end
[Su, Sx] = transferMatrices(A, B);


%% Task setting
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
MuQ = zeros(nbVar*nbData,1); 
Q = zeros(nbVar*nbData);
id = [1:nbVarPos] + (nbData-1) * nbVar;
Q(id,id) = eye(nbVarPos);
MuQ(id) = rand(nbVarPos,1);

%Constraining the two agents to meet at the end of the motion
MuQ(id(3:4)) = MuQ(id(1:2)); %Proposed meeting point for the two agents
Q(id(1:2), id(3:4)) = -eye(2)*1E0;
Q(id(3:4), id(1:2)) = -eye(2)*1E0;


%% Batch LQR reproduction
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x0 = [rand(4,1); zeros(4,1); 0; -9.81*1E-2; 0; -9.81*0E-2];
u = (Su' * Q * Su + R) \ Su' * Q * (MuQ - Sx * x0); 
rx = reshape(Sx*x0+Su*u, nbVar, nbData); %Reshape data for plotting

% uSigma = inv(Su' * Q * Su + R); 
% xSigma = Su * uSigma * Su';


%% Plot 2D
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('position',[10 10 1000 1000],'color',[1 1 1],'name','x1-x2 plot'); hold on; axis off;
%Agent 1
% for t=1:nbData
% 	plotGMM(rx(1:2,t), xSigma(nbVar*(t-1)+[1,2],nbVar*(t-1)+[1,2]).*1E-8, [.2 .2 .2], .03); %Plot uncertainty
% end	
plot(rx(1,:), rx(2,:), '-','linewidth',2,'color',[0 0 0]);
plot(rx(1,1), rx(2,1), '.','markersize',35,'color',[0 0 0]);
% plot(rx(1,end-3), rx(2,end-2), '.','markersize',25,'color',[0 .6 0]); %meeting point
% plot(MuQ(id(1)), MuQ(id(2)), '.','markersize',35,'color',[.8 0 0]);
%Agent 2
% for t=1:nbData
% 	plotGMM(rx(3:4,t), xSigma(nbVar*(t-1)+[3,4],nbVar*(t-1)+[3,4]).*1E-8, [.2 .2 .2], .03); %Plot uncertainty
% end	
plot(rx(3,:), rx(4,:), '-','linewidth',2,'color',[.6 .6 .6]);
plot(rx(3,1), rx(4,1), '.','markersize',35,'color',[.6 .6 .6]);
plot(rx(3,end-1), rx(4,end), '.','markersize',35,'color',[.4 .8 .4]); %meeting point
% plot(MuQ(id(3)), MuQ(id(4)), '.','markersize',35,'color',[1 .6 .6]);
axis equal;
% print('-dpng','graphs/MPC_fullQ03a.png');

% %Visualize Q
% figure('position',[1030 10 1000 1000],'color',[1 1 1],'name','Covariances'); hold on; box on; 
% set(gca,'linewidth',2); title('Q','fontsize',14);
% colormap(gca, flipud(gray));
% pcolor(abs(Q));
% set(gca,'xtick',[1,size(Q,1)],'ytick',[1,size(Q,1)]);
% axis square; axis([1 size(Q,1) 1 size(Q,1)]); shading flat;

pause;
close all;
end

%%%%%%%%%%%%%%%%%%%%%%%%%
function [Su, Sx] = transferMatrices(A, B)
	[nbVarX, nbVarU, nbData] = size(B);
	Su = zeros(nbVarX*nbData, nbVarU*(nbData-1));
	Sx = kron(ones(nbData,1), speye(nbVarX)); 
	M = speye(nbVarX);
	N = [];
	for t=2:nbData
		id1 = (t-1)*nbVarX+1:nbData*nbVarX;
		Sx(id1,:) = Sx(id1,:) * A(:,:,t-1);
		id1 = (t-1)*nbVarX+1:t*nbVarX; 
		id2 = 1:(t-1)*nbVarU;
% 		N = blkdiag(B(:,:,t-1), N);
		N = blkdiag(N, B(:,:,t-1)); %To be checked!!!
		Su(id1,id2) = M * N;
		M = [A(:,:,t-1) * M, A(:,:,t-1)]; %Also M = [A^(n-1), M] or M = [Sx(id1,:), M]
	end
end