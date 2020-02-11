function demo_proMP01
% ProMP with several forms of basis functions, inspired by the original form with RBFs described in 
% Paraschos, A., Daniel, C., Peters, J. and Neumann, G., "Probabilistic Movement Primitives", NIPS'2013
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
nbStates = 8; %Number of basis functions
nbVar = 2; %Dimension of position data (here: x1,x2)
nbSamples = 5; %Number of demonstrations
nbData = 200; %Number of datapoints in a trajectory
nbRepros = 5; %Number of reproductions


%% Load handwriting data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
demos=[];
load('data/2Dletters/C.mat');
x=[];
for n=1:nbSamples
	s(n).x = spline(1:size(demos{n}.pos,2), demos{n}.pos, linspace(1,size(demos{n}.pos,2),nbData)); %Resampling
	x = [x, s(n).x(:)]; %Reorganize as trajectory datapoints 
end
t = linspace(0,1,nbData); %Time range


%% ProMP with radial basis functions 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Compute basis functions Psi and activation weights w
tMu = linspace(t(1), t(end), nbStates);
m(1).phi = zeros(nbData,nbStates);
for i=1:nbStates
	m(1).phi(:,i) = gaussPDF(t, tMu(i), 1E-2);
% 	m(1).phi(:,i) = mvnpdf(t', tMu(i), 1E-2); 
end
% m(1).phi = m(1).phi ./ repmat(sum(m(1).phi,2),1,nbStates); %Optional rescaling
m(1).Psi = kron(m(1).phi, eye(nbVar)); %Eq.(27)
m(1).w = (m(1).Psi' * m(1).Psi + eye(nbVar*nbStates).*1E-8) \ m(1).Psi' * x; %m(1).w = pinv(m(1).Psi) * x'; %Eq.(28)
%Distribution in parameter space
m(1).Mu_w = mean(m(1).w,2);
m(1).Sigma_w = cov(m(1).w') + eye(nbVar*nbStates) * 1E0; 
%Trajectory distribution
m(1).Mu = m(1).Psi * m(1).Mu_w; %Eq.(29)
m(1).Sigma = m(1).Psi * m(1).Sigma_w * m(1).Psi'; %Eq.(29)


%% ProMP with Bernstein basis functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Compute basis functions Psi and activation weights w
m(2).phi = zeros(nbData,nbStates);
for i=0:nbStates-1
	m(2).phi(:,i+1) = factorial(nbStates-1) ./ (factorial(i) .* factorial(nbStates-1-i)) .* (1-t).^(nbStates-1-i) .* t.^i; %Bernstein basis functions
end
m(2).Psi = kron(m(2).phi, eye(nbVar)); %Eq.(27)
m(2).w = (m(2).Psi' * m(2).Psi + eye(nbVar*nbStates).*1E-8) \ m(2).Psi' * x; %m(2).w = pinv(m(2).Psi) * x; %Eq.(28)
%Distribution in parameter space
m(2).Mu_w = mean(m(2).w,2);
m(2).Sigma_w = cov(m(2).w') + eye(nbVar*nbStates) * 1E0; 
%Trajectory distribution
m(2).Mu = m(2).Psi * m(2).Mu_w; %Eq.(29)
m(2).Sigma = m(2).Psi * m(2).Sigma_w * m(2).Psi'; %Eq.(29)


%% ProMP with Fourier basis functions (here, only DCT)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Compute basis functions Psi and activation weights w
m(3).phi = zeros(nbData,nbStates);
for i=1:nbStates
	xTmp = zeros(1,nbData);
	xTmp(i) = 1;
	m(3).phi(:,i) = idct(xTmp);
end	
m(3).Psi = kron(m(3).phi, eye(nbVar)); %Eq.(27)
m(3).w = (m(3).Psi' * m(3).Psi + eye(nbVar*nbStates).*1E-8) \ m(3).Psi' * x; %m(3).w = pinv(m(3).Psi) * x; %Eq.(28)
%Distribution in parameter space
m(3).Mu_w = mean(m(3).w,2);
m(3).Sigma_w = cov(m(3).w') + eye(nbVar*nbStates) * 1E0; 
%Trajectory distribution
m(3).Mu = m(3).Psi * m(3).Mu_w; %Eq.(29)
m(3).Sigma = m(3).Psi * m(3).Sigma_w * m(3).Psi'; %Eq.(29)


%% Conditioning with trajectory distribution (reconstruction from partial data)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
in0 = [1,nbData]; %Time steps input indices
out0 = 2:nbData-1; %Time steps output indices
in = [];
for i=1:length(in0)
	in = [in, (in0(i)-1)*nbVar+[1:nbVar]]; %Trajectory distribution input indices
end
out = [];
for i=1:length(out0)
	out = [out, (out0(i)-1)*nbVar+[1:nbVar]]; %Trajectory distribution output indices
end
%Reproduction by Gaussian conditioning
for k=1:3
	for n=1:nbRepros
		m(k).Mu2(in,n) = x(in,1) + repmat((rand(nbVar,1)-0.5)*2, length(in0), 1) ;

	% 	%Conditional distribution with trajectory distribution
	% 	m(k).Mu2(out,n) = m(k).Mu(out) + m(k).Sigma(out,in) / m(k).Sigma(in,in) * (m(k).Mu2(in,n) - m(k).Mu(in));

		%Efficient computation of conditional distribution by exploiting ProMP structure
		m(k).Mu2(out,n) = m(k).Psi(out,:) * ...
			(m(k).Mu_w + m(k).Sigma_w * m(k).Psi(in,:)' / (m(k).Psi(in,:) * m(k).Sigma_w * m(k).Psi(in,:)') * (m(k).Mu2(in,n) - m(k).Psi(in,:) * m(k).Mu_w));
	end
end


%% Plot 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('position',[10 10 1800 1300]); 
clrmap = lines(nbStates);
methods = {'ProMP with Radial basis functions','ProMP with Bernstein basis functions','ProMP with Fourier basis functions'};
for k=1:3
	%Plot signal
	subplot(3,3,k); hold on; axis off; title(methods{k},'fontsize',16);
	plot(x(1:2:end,:), x(2:2:end,:), '.','markersize',10,'color',[.7 .7 .7]);
	for n=1:nbRepros
		plot(m(k).Mu2(1:2:end,n), m(k).Mu2(2:2:end,n), '-','lineWidth',2,'color',[1 .6 .6]);
	end
	plot(m(k).Mu(1:2:end), m(k).Mu(2:2:end), '-','lineWidth',2,'color',[0 0 0]);
	axis tight; axis equal; 
	%Plot activation functions
	subplot(3,3,3+k); hold on; axis off; title('\phi_k','fontsize',16);
	for i=1:nbStates
		plot(1:nbData, m(k).phi(:,i),'linewidth',2,'color',clrmap(i,:));
	end
	axis([1, nbData, min(m(k).phi(:)), max(m(k).phi(:))]);
	%Plot Psi*Psi' matrix
	subplot(3,3,6+k); hold on; axis off; title('\Psi\Psi^T','fontsize',16);
	colormap(flipud(gray));
	imagesc(abs(m(k).Psi * m(k).Psi'));
	axis tight; axis square; axis ij;
end %k

% print('-dpng','graphs/demo_proMP01.png');
pause;
close all;