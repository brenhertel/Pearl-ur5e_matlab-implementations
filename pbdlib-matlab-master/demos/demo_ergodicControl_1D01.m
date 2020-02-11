function demo_ergodicControl_1D01
% 1D ergodic control with a spatial distribution described as a GMM, inspired by G. Mathew and I. Mezic, 
% "Spectral Multiscale Coverage: A Uniform Coverage Algorithm for Mobile Sensor Networks", CDC'2009 
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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nbData = 4000; %Number of datapoints
nbFct = 50; %Number of basis functions along x and y
nbStates = 2; %Number of Gaussians to represent the spatial distribution
dt = 1E-2; %Time step
xlim = [0; 1]; %Domain limit for each dimension (considered to be 1 for each dimension in this implementation)
L = (xlim(2) - xlim(1)) * 2; %Size of [-xlim(2),xlim(2)]
om = 2 .* pi ./ L; %omega
u_max = 1E0; %Maximum speed allowed 
x = .1; %Initial position

%Desired spatial distribution represented as a mixture of Gaussians
Mu(:,1) = 0.7 *1;
Sigma(:,1) = 0.003; 
Mu(:,2) = 0.5;
Sigma(:,2) = 0.01;
Priors = ones(1,nbStates) ./ nbStates; %Mixing coefficients


%% Compute Fourier series coefficients phi_k of desired spatial distribution
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
rg = [0:nbFct-1]';
Lambda = (rg.^2 + 1).^-1; %Weighting vector (Eq.(15)

HK = L; %Rescaling term (as scalar)
% HK = [1; sqrt(.5)*ones(nbFct-1,1)]; %Rescaling term (as normalizing vector)

%Explicit description of phi_k by exploiting the Fourier transform properties of Gaussians (optimized version by exploiting symmetries)
w = rg .* om;
phi = zeros(nbFct,1);
for k=1:nbStates
	phi = phi + Priors(k) .* cos(w .* Mu(:,k)) .* exp(-.5 .* w.^2 .* Sigma(:,k)); %Eq.(21)
end
phi = phi ./ HK;


%% Ergodic control 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ckt = zeros(nbFct, 1);
for t=1:nbData
	r.x(:,t) = x; %Log data
	
	%Fourier basis functions and derivatives for each dimension (only cosine part on [0,L/2] is computed since the signal is even and real by construction) 
	fx = cos(x * rg .* om); %Eq.(18)
	gradf = -sin(x * rg .* om) .* rg .* om;
	
% 	%Fourier basis functions and derivatives for each dimension (only real part on [0,L/2] is computed since the signal is even and real by construction) 
% 	fx = real(exp(-i .* x * rg .* om)); %Eq.(18)
% 	gradf = real(-exp(-i .* x * rg .* om) .* i .* rg .* om);

	ckt = ckt + fx ./ HK;	%ckt./t are the Fourier series coefficients along trajectory (Eq.(17))

% 	%Controller with ridge regression formulation
% 	u = -gradf' * (Lambda .* (ckt./t - phi)) .* t .* 1E-1; %Velocity command
	
	%Controller with constrained velocity norm
	u = -gradf' * (Lambda .* (ckt./t - phi)); %Eq.(24)
	u = u .* u_max ./ (norm(u)+1E-2); %Velocity command
	
	x = x + u .* dt; %Update of position
end


%% Plot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('position',[10,10,1800,900],'color',[1 1 1]); 
%Plot signal
subplot(1,3,[1,2]); hold on; 
plot(1:nbData, r.x, '-','linewidth',3,'color',[.2 .2 .2]);
axis([1, nbData, xlim']);
xlabel('t','fontsize',18); ylabel('x','fontsize',18); 
set(gca,'xtick',[],'ytick',[]);
%Plot distribution
subplot(1,3,3); hold on; 
nbRes = 100;
xm = linspace(xlim(1),xlim(2),nbRes);
G = zeros(1,nbRes);
for k=1:nbStates
	G = G + Priors(k) .* mvnpdf(xm', Mu(:,k)', Sigma(:,k))'; %Spatial distribution
end
plot(G, xm, '-','linewidth',3,'color',[.8 0 0]);
xlabel('\phi(x)','fontsize',18); ylabel('x','fontsize',18); 
set(gca,'xtick',[],'ytick',[]);

% print('-dpng','graphs/ergodicControl_1D01.png'); 
pause;
close all;

