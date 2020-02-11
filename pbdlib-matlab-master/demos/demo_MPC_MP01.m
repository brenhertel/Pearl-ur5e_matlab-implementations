function demo_MPC_MP01
% Batch LQR using sparse movement primitives with radial basis functions
%
% If this code is useful for your research, please cite the related publication:
% @incollection{Calinon19MM,
% 	author="Calinon, S.",
% 	title="Mixture Models for the Analysis, Edition, and Synthesis of Continuous Time Series",
% 	booktitle="Mixture Models and Applications",
% 	publisher="Springer",
% 	editor="Bouguila, N. and Fan, W.", 
% 	year="2019",
% 	pages="39--57",
% 	doi="10.1007/978-3-030-23876-6_3"
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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nbVarPos = 2; %Dimension of position data (here: x1,x2)
nbDeriv = 2; %Number of static & dynamic features (D=2 for [x,dx])
nbVar = nbVarPos * nbDeriv; %Dimension of state vector
nbData = 200; %Number of datapoints in a trajectory

dt = 0.01; %Time step duration
rfactor = 1E2;	%Control cost in LQR
R = speye((nbData-1)*nbVarPos) * rfactor;


%% Dynamical System settings (discrete version)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Integration with higher order Taylor series expansion
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
C = kron([1, 0], eye(nbVarPos));

% %Construct Su and Sx matrices (transfer matrices in batch LQR)
% Su = zeros(nbVar*nbData, nbVarPos*(nbData-1));
% Sx = kron(ones(nbData,1),eye(nbVar)); 
% M = B;
% for n=2:nbData
% 	%Build Sx matrix
% 	id1 = (n-1)*nbVar+1:nbData*nbVar;
% 	Sx(id1,:) = Sx(id1,:) * A;
% 	%Build Su matrix
% 	id1 = (n-1)*nbVar+1:n*nbVar; 
% 	id2 = 1:(n-1)*nbVarPos;
% 	Su(id1,id2) = M;
% 	M = [A*M(:,1:nbVarPos), M];
% end

%Construct CSu and CSx matrices (transfer matrices in batch LQR)
CSu = zeros(nbVarPos*nbData, nbVarPos*(nbData-1));
CSx = kron(ones(nbData,1), [eye(nbVarPos) zeros(nbVarPos)]);
M = B;
for n=2:nbData
	id1 = (n-1)*nbVarPos+1:n*nbVarPos;
	CSx(id1,:) = CSx(id1,:) * A;
	id1 = (n-1)*nbVarPos+1:n*nbVarPos; 
	id2 = 1:(n-1)*nbVarPos;
	CSu(id1,id2) = C * M;
	M = [A*M(:,1:nbVarPos), M]; %Also M = [A^(n-1)*B, M] or M = [Sx(id1,:)*B, M]
end


%% Probabilistic movement primitives with radial basis functions 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Generate data
nbSamples = 4; %Number of demonstrations
for n=1:nbSamples
	t = linspace(.2*n/3, 1.5*pi+.4*n/3, nbData);
	xtmp = (1+n.*1E-2) .* [cos(t); sin(t)] + (.7+n.*1E-2) .* [zeros(1,nbData); cos(t*2-pi/3)] + randn(nbVarPos,nbData) .* 1E-2;
	x(:,n) = xtmp(:);
end

%Compute basis functions Psi and activation weights w
nbFct = 12; 
t = linspace(0,1,nbData);
tMu = linspace(t(1), t(end), nbFct);
phi = zeros(nbData,nbFct);

for i=1:nbFct
	phi(:,i) = gaussPDF(t, tMu(i), 1E-2);
% 	phi(:,i) = mvnpdf(t', tMu(i), 1E-2); 
end
% phi = phi ./ repmat(sum(phi,2),1,nbFct); %Optional rescaling

% for i=0:nbFct-1
% 	phi(:,i+1) = factorial(nbFct-1) ./ (factorial(i) .* factorial(nbFct-1-i)) .* (1-t).^(nbFct-1-i) .* t.^i; %Bernstein basis functions
% end

Psi = kron(phi, eye(nbVarPos)); 

% w = (Psi' * Psi + eye(nbFct).*1E-18) \ Psi' * x; 
w = pinv(Psi) * x; 

%Distribution in parameter space
Mu_w = mean(w,2); %Mean
Sigma_w = cov(w') + eye(nbFct*nbVarPos) .* 1E-4; %Covariance

%Distribution in trajectory space
MuQ = Psi * Mu_w; %Mean
Q = Psi / Sigma_w * Psi'; %Precision matrix
% rank(Q) %Q is sparse


%% Reproduction
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x0 = [x(1:2,1)+.2; zeros(nbVarPos,1)];
% u = (Su' * Q * Su + R) \ (Su' * Q * (MuQ - Sx*x0)); 
% rx = reshape(Sx*x0+Su*u, nbVarPos, nbData);
u = (CSu' * Q * CSu + R) \ (CSu' * Q * (MuQ - CSx*x0)); 
rx = reshape(CSx*x0+CSu*u, nbVarPos, nbData);

% uSigma = inv(Rq) .* 1E-1;
% xSigma = Su * uSigma * Su';


%% 2D Plots
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('position',[10,10,2300,1200]); 
subplot(3,2,[1,3,5]); hold on; axis off;
for n=1:nbSamples
	plot(x(1:2:end,n), x(2:2:end,n), '-','linewidth',4,'color',[.7,.7,.7]);
end
plot(MuQ(1:2:end), MuQ(2:2:end), '-','linewidth',4,'color',[.8 0 0]);
plot(rx(1,:), rx(2,:), '-','linewidth',4,'color',[0,0,0]);
plot(rx(1,1), rx(2,1), '.','markersize',28,'color',[0,0,0]);
axis equal;

%Timeline Plots
for j=1:2
	subplot(3,2,2*(j-1)+2); hold on; %axis off;
	for n=1:nbSamples
		plot(1:nbData, x(j:2:end,n), '-','linewidth',3,'color',[.7,.7,.7]);
	end
% 	%Plot uncertainty
% 	id = j:nbVar:nbVar*nbData;
% 	S = diag(xSigma(id,id));
% 	patch([1:nbData nbData:-1:1], [rx(j,:)-S(:)' fliplr(rx(j,:)+S(:)')], [.8 0 0],'edgecolor',[.6 0 0],'facealpha', .2, 'edgealpha', .2);
	plot(1:nbData, MuQ(j:2:end), '-','linewidth',4,'color',[.8 0 0]);
	plot(1:nbData, rx(j,:), '-','linewidth',4,'color',[0 0 0]);
	set(gca,'xtick',[],'ytick',[]);
	xlabel('$t$','interpreter','latex','fontsize',34); 
	ylabel(['$x_' num2str(j) '$'],'interpreter','latex','fontsize',34);
end

%Basis functions plot
subplot(3,2,6); hold on; 
clrmap = lines(nbFct);
for i=1:nbFct
	plot(1:nbData, phi(:,i), '-','linewidth',3,'color',clrmap(i,:)); %axis tight; 
end
set(gca,'xtick',[],'ytick',[]);
xlabel('$t$','interpreter','latex','fontsize',34); ylabel('$\phi_k$','interpreter','latex','fontsize',34);

% print('-dpng','graphs/MPC_MP01.png');
pause;
close all;