function demo_Riemannian_Sd_interp01
% Interpolation on d-sphere manifold
% (formulation with tangent space of the same dimension as the dimension of the manifold)
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
model.nbVar = 3; %Dimension for the embedding of the sphere
model.nbStates = 2; %Number of states
nbData = 50; %Number of interpolation steps
% nbIter = 20; %Number of iteration for the Gauss Newton algorithm

x = rand(model.nbVar,model.nbStates) - 0.5;
for i=1:model.nbStates
	x(:,i) = x(:,i) ./ norm(x(:,i));
end

p = rand(model.nbVar,1) - 0.5;
p = p ./ norm(p);
v = logmap(p, x(:,1)); %random vector in the tangent space of x(:,1) to be transported


%% Geodesic interpolation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
w = [linspace(1,0,nbData); linspace(0,1,nbData)];
xi = zeros(model.nbVar,nbData);
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

	%Interpolation between two points can be computed in closed form
	xi(:,t) = expmap(w(2,t) * logmap(x(:,2),x(:,1)), x(:,1));
	
	%Inner product between vector transported and direction of geodesic
	ptv = transp(x(:,1), xi(:,t)) * v;
% 	ptv = transp2(x(:,1), xi(:,t), v);
	dir = logmap(x(:,2), xi(:,t));
	if norm(dir) > 1E-5
		dir = dir ./ norm(dir);
		inprod(t) = dir' * ptv;
	end
		
end

inprod

% %% 2D plot
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% figure('position',[10,10,1300,650]); hold on; axis off; 
% plot(0,0,'k+');
% ht = plot(x(1,:),x(2,:),'.','markersize',20,'color',[.8 0 0]);
% u = logmap(x(:,2),x(:,1));
% plot2DArrow(x(:,1), u, [.8,0,0], 1, .04);
% plot(xi(1,:),xi(2,:),'-','color',[0 0 0]);
% axis equal; 


%% 3D plot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[X,Y,Z] = sphere(20);
figure('position',[10,10,1000,1000]); hold on; axis off; rotate3d on; 
colormap([.9 .9 .9]);
mesh(X,Y,Z);
ht = plot3(x(1,:), x(2,:), x(3,:), '.','markersize',20,'color',[.8 0 0]);
u = logmap(x(:,2), x(:,1));
mArrow3(x(:,1), x(:,1)+u, 'stemWidth',.004,'tipWidth',.016,'color',[.8 0 0]);
plot3(xi(1,:), xi(2,:), xi(3,:), '-','color',[0 0 0]);
view(0,70); axis equal; axis tight; axis vis3d;  

% pause;
% delete(ht)

% print('-dpng','graphs/demo_Riemannian_Sd_interp01.png');
pause;
close all;
end


%% Functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function x = expmap(u,x0)
	th = sqrt(sum(u.^2,1)); %norm(u,'fro')
	x = repmat(x0,[1,size(u,2)]) .* repmat(cos(th),[size(u,1),1]) + u .* repmat(sin(th)./th,[size(u,1),1]);
	x(:,th<1e-16) = repmat(x0,[1,sum(th<1e-16)]);		
end

function u = logmap(x,x0)
% 	%Version 0
% 	p = (x-x0) - trace(x0'*(x-x0)) * x0;
% 	if norm(p,'fro')<1e-16
% 		u = zeros(size(x));
% 	else
% 		u = acos(trace(x0'*x)) * p./(norm(p,'fro'));
% 	end

% 	%Version 1
% 	th = acos(x0'*x);	
% 	u = (x - repmat(x0, [1,size(x,2)]) .* repmat(cos(th), [size(x,1),1])) .* repmat(th./sin(th), [size(x,1),1]);

	%Version 2 (as in https://ronnybergmann.net/mvirt/manifolds/Sn.html)
	th = acos(x0'*x);
	u = x - x0' * x * x0; %also equals to (x - x0) - x0' * (x-x0) * x0 (as defined in Absil07), since x0'*x0=1 (unit hypershere)
	u = th .* u ./ norm(u); %norm(u) also equals to sqrt(1-(x0'*x).^2)
	u(:,th<1e-16) = 0;
end

function Ac = transp(x1,x2,t)
	if nargin==2
		t=1;
	end
	u = logmap(x2,x1);
	e = norm(u,'fro');
	u = u ./ (e+realmin);
	Ac = -x1 * sin(e*t) * u' + u * cos(e*t) * u' + eye(size(u,1)) - u * u';
end

%As in https://ronnybergmann.net/mvirt/manifolds/Sn.html (gives the same result as transp(x1,x2,t)*v)
function v = transp2(x,y,v)
	d = acos(x'*y);
	v = v - (logmap(y,x) + logmap(x,y)) .* (logmap(y,x)' * v) ./ d.^2;
end

function x = geodesic(u,x0,t)
	normu = norm(u,'fro');
	x = x0 * cos(normu*t) + u./normu * sin(normu*t);
end