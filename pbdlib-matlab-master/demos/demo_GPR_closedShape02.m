function demo_GPR_closedShape02
% Gaussian process implicit surface (GPIS) representation with thin-plate covariance function in GPR
% 
% If this code is useful for your research, please cite the related publications:
% @incollection{Calinon19chapter,
% 	author="Calinon, S. and Lee, D.",
% 	title="Learning Control",
% 	booktitle="Humanoid Robotics: a Reference",
% 	publisher="Springer",
% 	editor="Vadakkepat, P. and Goswami, A.", 
% 	year="2019",
% 	doi="10.1007/978-94-007-7194-9_68-1",
% 	pages="1--52"
% }
% @inproceedings{Williams07,
% 	author = "Williams, O. and Fitzgibbon, A.",
% 	title = "Gaussian Process Implicit Surfaces",
% 	booktitle = "Gaussian Processes in Practice",
% 	year = "2007"
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
nbVarX = 2; %Dimension of x
nbVarY = 1; %Dimension of y 
nbDataRepro = 20.^2; %Number of datapoints in a reproduction
nbRepros = 6; %Number of randomly sampled reproductions
p(1)=1E0; p(2)=1E-5; %Thin-plate covariance function parameters 
SigmaY = eye(nbVarY);


%% Generate data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x = [-3 -2 -1  0  1  2  3  3  3  2  1  1  1  0 -1 -1 -1 -2 -3 -3;
		  2  2  2  2  2  2  2  1  0  0  0 -1 -2 -2 -2 -1  0  0  0  1];
x = [x, mean(x,2), [-4, -4, 4, 4; 4 -4 -4 4]] .*1E-1;
x = x(:,6:end); %Simulate missing datapoints
nbData = size(x,2);
y = [zeros(1,nbData-5), 1, -1, -1, -1, -1];
Data = [x; y];


%% Reproduction with GPR
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Mean computation
[Xm, Ym] = meshgrid(linspace(-.5,.5,nbDataRepro.^.5), linspace(-.5,.5,nbDataRepro.^.5));
xs = [Xm(:)'; Ym(:)'];
p(1) = max(max(pdist2(xs', xs'))); %Refine thin-plate covariance function parameter

rc = 4E-1;
S = eye(nbVarX) .* rc.^-2;
MuS = .5 .* rc .* diag(1 - xs' * S * xs)';
Mu = .5 .* rc .* diag(1 - x' * S * x)';

K = covFct(x, x, p, 1); %Inclusion of noise on the inputs for the computation of K
Ks = covFct(xs, x, p);
% r(1).Data = [xs; (Ks / K * y')']; %GPR with Mu=0
r(1).Data = [xs; MuS + (Ks / K * (y - Mu)')']; 

%Uncertainty evaluation
Kss = covFct(xs, xs, p); 
S = Kss - Ks / K * Ks';
r(1).SigmaOut = zeros(nbVarY, nbVarY, nbData);
for t=1:nbDataRepro
	r(1).SigmaOut(:,:,t) = SigmaY * S(t,t); 
end

%Generate stochastic samples from the prior 
[V,D] = eig(Kss);
for n=2:nbRepros/2
	yp = real(V*D^.5) * randn(nbDataRepro, nbVarY) * SigmaY .* 2E-1; 
	r(n).Data = [xs; yp'];
end

%Generate stochastic samples from the posterior 
[V,D] = eig(S);
for n=nbRepros/2+1:nbRepros
	ys = real(V*D^.5) * randn(nbDataRepro, nbVarY) * SigmaY .* 0.5 + r(1).Data(nbVarX+1:end,:)'; 
	r(n).Data = [xs; ys'];
end


%% Spatial plots 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('PaperPosition',[0 0 14 5],'position',[10 10 2500 900]); 
colormap([.7 .7 .7]);

%Plot center of GP (acting as prior if points on the contours are missing)
subplot(1,3,1); hold on; axis off; rotate3d on;
% for n=2:nbRepros/2
% 	coltmp = [.5 .5 .5] + [.5 .5 .5].*rand(1);
% 	mesh(Xm, Ym, reshape(r(n).Data(3,:), nbDataRepro.^.5, nbDataRepro.^.5), 'facealpha',.4,'edgealpha',.4,'facecolor',coltmp,'edgecolor',coltmp); %Prior samples
% end
mesh(Xm, Ym, reshape(MuS, nbDataRepro.^.5, nbDataRepro.^.5), 'facealpha',.8,'edgealpha',.8);
mesh(Xm, Ym, zeros(nbDataRepro.^.5, nbDataRepro.^.5), 'facealpha',.3,'edgealpha',.3,'facecolor',[0 0 0],'edgecolor',[0 0 0]);
tl = linspace(0,2*pi,100);
plot(cos(tl)*rc, sin(tl)*rc, 'k-','linewidth',2);
view(3); axis vis3d;

%Plot posterior distribution 
subplot(1,3,2); hold on; axis off; rotate3d on;
% for n=nbRepros/2+1:nbRepros
% 	coltmp = [.5 .5 .5] + [.5 .5 .5].*rand(1);
% 	mesh(Xm, Ym, reshape(r(n).Data(3,:), nbDataRepro.^.5, nbDataRepro.^.5), 'facealpha',.4,'edgealpha',.4,'facecolor',coltmp,'edgecolor',coltmp); %posterior samples
% end
mesh(Xm, Ym, reshape(r(1).Data(3,:), nbDataRepro.^.5, nbDataRepro.^.5), 'facealpha',.8,'edgealpha',.8);
mesh(Xm, Ym, zeros(nbDataRepro.^.5, nbDataRepro.^.5), 'facealpha',.3,'edgealpha',.3,'facecolor',[0 0 0],'edgecolor',[0 0 0]);
contour(Xm, Ym, reshape(r(1).Data(3,:), nbDataRepro.^.5, nbDataRepro.^.5), [0,0], 'linewidth',2,'color',[0 0 0]); 
plot3(Data(1,1:end-5), Data(2,1:end-5), Data(3,1:end-5), '.','markersize',18,'color',[.8 0 0]);
plot3(Data(1,end-4), Data(2,end-4), Data(3,end-4), '.','markersize',18,'color',[0 0 .8]);
plot3(Data(1,end-3:end), Data(2,end-3:end), Data(3,end-3:end), '.','markersize',18,'color',[0 .7 0]);
view(3); axis vis3d;

subplot(1,3,3); hold on; axis off; 
contour(Xm, Ym, reshape(r(1).Data(3,:), nbDataRepro.^.5, nbDataRepro.^.5), [0,0], 'linewidth',2,'color',[0 0 0]); 
contour(Xm, Ym, reshape(r(1).Data(3,:)+2E0.*diag(S).^.5', nbDataRepro.^.5, nbDataRepro.^.5), [0,0], 'linewidth',1,'color',[.6 .6 .6]); 
contour(Xm, Ym, reshape(r(1).Data(3,:)-2E0.*diag(S).^.5', nbDataRepro.^.5, nbDataRepro.^.5), [0,0], 'linewidth',1,'color',[.6 .6 .6]); 
plot(Data(1,1:end-5), Data(2,1:end-5), '.','markersize',18,'color',[1 0 0]);
plot(Data(1,end-4), Data(2,end-4), '.','markersize',18,'color',[0 0 .8]);
plot(Data(1,end-3:end), Data(2,end-3:end), '.','markersize',18,'color',[0 .7 0]);
axis equal;
% print('-dpng','graphs/GPR_GPIS01.png');

pause;
close all;
end


function K = covFct(x1, x2, p, flag_noiseObs)
	if nargin<4
		flag_noiseObs = 0;
	end
	
	%Thin plate covariance function (for 3D implicit shape)
	K = 12.^-1 .* (2 .* pdist2(x1',x2').^3 - 3 .* p(1) .* pdist2(x1',x2').^2 + p(1).^3);	

% 	%Thin plate covariance function (for 2D implicit shape -> does not seem to work)
% 	K = 2 .* pdist2(x1',x2').^2 .* log(pdist2(x1',x2')) - (1 + 2.*log(p(1))) .* pdist2(x1',x2').^2 + p(1).^2;
	
% 	%RBF covariance function
% 	K = 1E-1 .* exp(-p(1)^-1 .* pdist2(x1',x2').^2);
	
	if flag_noiseObs==1
		K = K + p(2) .* eye(size(x1,2),size(x2,2)); %Consideration of noisy observation y
	end
end