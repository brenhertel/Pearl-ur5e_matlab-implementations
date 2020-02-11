function demo_Riemannian_S3_interp01
% Interpolation of unit quaternions (orientation) by relying on Riemannian manifold, providing the same result as SLERP interpolation
%
% If this code is useful for your research, please cite the related publication:
% @article{Calinon19,
% 	author="Calinon, S. and Jaquier, N.",
% 	title="Gaussians on {R}iemannian Manifolds for Robot Learning and Adaptive Control",
% 	journal="arXiv:1909.05946",
% 	year="2019",
% 	pages="1--10"
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
model.nbVar = 3; %Dimension of the tangent space
model.nbVarMan = 4; %Dimension of the manifold
model.nbStates = 2; %Number of states
nbData = 100; %Number of interpolation steps
% nbIter = 20; %Number of iteration for the Gauss Newton algorithm

x = rand(model.nbVarMan,model.nbStates) - 0.5;
for i=1:model.nbStates
	x(:,i) = x(:,i) / norm(x(:,i));
end


%% Geodesic interpolation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
w = [linspace(1,0,nbData); linspace(0,1,nbData)];
xi = zeros(model.nbVarMan,nbData);
% xtmp = x(:,1);

for t=1:nbData
% 	%Interpolation between more than 2 points can be computed in an iterative form
% 	for n=1:nbIter
% 		utmp = zeros(model.nbVar,1);
% 		for i=1:model.nbStates
% 			utmp = utmp + w(i,t) * logmap(x(:,i), xtmp);
% 		end
% 		xtmp = expmap(utmp, xtmp);
% 	end
% 	xi(:,t) = xtmp;

	%Interpolation between two covariances can be computed in closed form
	xi(:,t) = expmap(w(2,t) * logmap(x(:,2),x(:,1)), x(:,1));
end


%% SLERP interpolation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for t=1:nbData
	xi2(:,t) = quatinterp(x(:,1)', x(:,2)', w(2,t), 'slerp');
end


%% Plots
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('position',[10,10,900,1250]);
for i=1:model.nbVarMan
	subplot(model.nbVarMan,1,i); hold on;
	for n=1:model.nbStates
		plot([1,nbData],[x(i,n),x(i,n)],'-','linewidth',2,'color',[0 0 0]);
		plot([1,nbData],[-x(i,n),-x(i,n)],'--','linewidth',2,'color',[0 0 0]);
	end
	h(1) = plot(xi(i,:),'-','linewidth',2,'color',[.8 0 0]);
	h(2) = plot(xi2(i,:),':','linewidth',2,'color',[0 .7 0]);
	if i==1
		legend(h,'geodesic','SLERP');
	end
	ylabel(['q_' num2str(i)]);
	axis([1, nbData -1 1]);
end
xlabel('t');

%print('-dpng','graphs/demo_Riemannian_S3_interp01.png');
pause;
close all;
end


%% Functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function x = expmap(u, mu)
	x = QuatMatrix(mu) * expfct(u);
end

function u = logmap(x, mu)
	if norm(mu-[1;0;0;0])<1e-6
		Q = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
	else
		Q = QuatMatrix(mu);
	end
	u = logfct(Q'*x);
end

function Exp = expfct(u)
	normv = sqrt(u(1,:).^2+u(2,:).^2+u(3,:).^2);
	Exp = real([cos(normv) ; u(1,:).*sin(normv)./normv ; u(2,:).*sin(normv)./normv ; u(3,:).*sin(normv)./normv]);
	Exp(:,normv < 1e-16) = repmat([1;0;0;0],1,sum(normv < 1e-16));
end

function Log = logfct(x)
% 	scale = acos(x(3,:)) ./ sqrt(1-x(3,:).^2);
	scale = acoslog(x(1,:)) ./ sqrt(1-x(1,:).^2);
	scale(isnan(scale)) = 1;
	Log = [x(2,:).*scale; x(3,:).*scale; x(4,:).*scale];
end

function Q = QuatMatrix(q)
	Q = [q(1) -q(2) -q(3) -q(4);
			 q(2)  q(1) -q(4)  q(3);
			 q(3)  q(4)  q(1) -q(2);
			 q(4) -q(3)  q(2)  q(1)];
end				 

% Arcosine re-defitinion to make sure the distance between antipodal quaternions is zero (2.50 from Dubbelman's Thesis)
function acosx = acoslog(x)
	for n=1:size(x,2)
		% sometimes abs(x) is not exactly 1.0
		if(x(n)>=1.0)
			x(n) = 1.0;
		end
		if(x(n)<=-1.0)
			x(n) = -1.0;
		end
		if(x(n)>=-1.0 && x(n)<0)
			acosx(n) = acos(x(n))-pi;
		else
			acosx(n) = acos(x(n));
		end
	end
end

function Ac = transp(g,h)
	E = [zeros(1,3); eye(3)];
	vm = QuatMatrix(g) * [0; logmap(h,g)];
	mn = norm(vm);
	if mn < 1e-10
		disp('Angle of rotation too small (<1e-10)');
		Ac = eye(3);
		return;
	end
	uv = vm / mn;
	Rpar = eye(4) - sin(mn)*(g*uv') - (1-cos(mn))*(uv*uv');	
	Ac = E' * QuatMatrix(h)' * Rpar * QuatMatrix(g) * E; %Transportation operator from g to h 
end