function demo_MPC_MP02
% Batch LQR using sparse movement primitives with Fourier basis functions
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
nbDeriv = 1; %Number of static & dynamic features (D=2 for [x,dx])
nbVar = nbVarPos * nbDeriv; %Dimension of state vector
nbData = 500; %Number of datapoints in a trajectory

dt = 0.01; %Time step duration
rfactor = 1E-8;	%Control cost in LQR
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

%Construct Su and Sx matrices (transfer matrices in batch LQR)
Su = zeros(nbVar*nbData, nbVarPos*(nbData-1));
Sx = kron(ones(nbData,1),eye(nbVar)); 
M = B;
for n=2:nbData
	%Build Sx matrix
	id1 = (n-1)*nbVar+1:nbData*nbVar;
	Sx(id1,:) = Sx(id1,:) * A;
	%Build Su matrix
	id1 = (n-1)*nbVar+1:n*nbVar; 
	id2 = 1:(n-1)*nbVarPos;
	Su(id1,id2) = M;
	M = [A*M(:,1:nbVarPos), M];
end


%% Probabilistic movement primitives with Fourier basis functions 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% t = linspace(0, 4*pi, nbData);
% MuTmp = [fft(cos(t) + .4 * cos(t*2+pi/3)); fft(sin(t))];
% MuTmp = [ifft(MuTmp(1,:)); ifft(MuTmp(2,:))];
% MuQ = MuTmp(:);
% Q = eye(nbVar*nbData);

%Generate periodic data
nbSamples = 4; %Number of demonstrations
for n=1:nbSamples
	t = linspace(2*n/3, 4*pi+2*n/3, nbData);
	xtmp = (1+n.*1E-2) .* [cos(t); sin(t)] + (.7+n.*1E-2) .* [zeros(1,nbData); cos(t*2-pi/3)] + randn(nbVar,nbData) .* 1E-2;
	x(:,n) = xtmp(:);
end

%Compute basis functions Psi and activation weights w
k = -5:5;
% nbFct = length(k);
t = linspace(0, 1, nbData);
phi = exp(t' * k * 2 * pi * 1i) ./ nbData;
Psi = kron(phi, eye(nbVar)); 

% w = (Psi' * Psi + eye(nbFct).*1E-18) \ Psi' * x; 
w = pinv(Psi) * x; 

%Distribution in parameter space
Mu_R = mean(abs(w), 2); %Magnitude average
Mu_theta = mean_angle(angle(w), 2); %Phase average
Mu_w = Mu_R .* exp(1i * Mu_theta); %Reconstruction

Sigma_R = cov(abs(w')); %Magnitude spread
Sigma_theta = cov_angle(angle(w')); %Phase spread

% %Remove correlations
% Sigma_R = diag(diag(Sigma_R));
% Sigma_theta = diag(diag(Sigma_theta));

Sigma_w = Sigma_R .* exp(1i * Sigma_theta); %+ eye(size(Sigma_R)) * 1E-4; %Reconstruction

% % [V_R, D_R] = eig(Sigma_R);
% % [V_theta, D_theta] = eig(Sigma_theta);
% % U_R = V_R * D_R.^.5;
% % U_theta = V_theta * D_theta.^.5;
% U_R = sqrtm(Sigma_R);
% U_theta = sqrtm(Sigma_theta);
% U_w = U_R .* exp(1i * U_theta)
% Sigma_w = U_w * U_w';


MuQ = Psi * Mu_w;
Q = Psi / Sigma_w * Psi';


%% Reproduction
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% x0 = zeros(nbVarPos,1);
x0 = x(11:12,1); %[10; 0];
u = (Su' * Q * Su + R) \ (Su' * Q * (MuQ - Sx*x0)); 
rx = reshape(Sx*x0+Su*u, nbVar, nbData);

% uSigma = inv(Rq) .* 1E-1;
% xSigma = Su * uSigma * Su';


%% 2D Plots
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% rxi = imag(rx);
rx = real(rx);
figure('position',[10,10,2300,1200]); 
subplot(2,2,[1,3]); hold on; axis off;
for n=1:nbSamples
	plot(x(1:2:end,n), x(2:2:end,n), '-','linewidth',4,'color',[.7,.7,.7]);
end
plot(MuQ(1:2:end), MuQ(2:2:end), '-','linewidth',4,'color',[.8 0 0]);
plot(rx(1,:), rx(2,:), '-','linewidth',4,'color',[0,0,0]);
plot(rx(1,1), rx(2,1), '.','markersize',28,'color',[0,0,0]);
% plot(rxi(1,:), rxi(2,:), ':','linewidth',4,'color',[0,0,0]);
axis equal;

%Timeline Plots
for j=1:2
	subplot(2,2,2*(j-1)+2); hold on; %axis off;
	for n=1:nbSamples
		plot(1:nbData, x(j:2:end,n), '-','linewidth',3,'color',[.7,.7,.7]);
	end
% 	%Plot uncertainty
% 	id = j:nbVar:nbVar*nbData;
% 	S = diag(xSigma(id,id));
% 	patch([1:nbData nbData:-1:1], [rx(j,:)-S(:)' fliplr(rx(j,:)+S(:)')], [.8 0 0],'edgecolor',[.6 0 0],'facealpha', .2, 'edgealpha', .2);
% 	plot(1:nbData, rxi(j,:), ':','linewidth',4,'color',[0 0 0]);
	plot(1:nbData, rx(j,:), '-','linewidth',4,'color',[0 0 0]);
	plot(1:nbData, MuQ(j:2:end), '-','linewidth',4,'color',[.8 0 0]);

	set(gca,'xtick',[],'ytick',[]);
	xlabel('$t$','interpreter','latex','fontsize',34); 
	ylabel(['$x_' num2str(j) '$'],'interpreter','latex','fontsize',34);
end

% print('-dpng','graphs/MPC_MP02.png');
pause;
close all;
end

%%%%%%%%%%%%%%%%%%
function Mu = mean_angle(phi, dim)
	if nargin<2
		dim = 1;
	end
	Mu = angle(mean(exp(1i*phi), dim));
end

%%%%%%%%%%%%%%%%%%
function Sigma = cov_angle(phi)
	Mu = mean_angle(phi);
	e = phi - repmat(Mu, size(phi,1), 1);
	Sigma = cov(angle(exp(1i*e)));
end