function demo_Riemannian_Sd_MPC01
% Linear quadratic tracking on hypershpere S^d by relying on Riemannian manifold and 
% batch LQR recomputed in an online manner  (version with viapoints), 
% based on GMM encoding of movement, by using position and velocity data (-> acceleration commands)
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
nbSamples = 5; %Number of demonstrations
nbRepros = 1; %Number of reproductions
nbIter = 20; %Number of iteration for the Gauss Newton algorithm
nbIterEM = 20; %Number of iteration for the EM algorithm
nbData = 100; %Number of datapoints
nbD = 20; %Time window for LQR computation
nbDrawingSeg = 20; %Number of segments used to draw ellipsoids

model.nbStates = 6; %Number of states in the GMM
model.nbVarPos = 3; %Dimension of position data (here: x1,x2,x3)
model.nbDeriv = 2; %Number of static & dynamic features (D=2 for [x,dx])
model.nbVar = model.nbVarPos * model.nbDeriv; %Dimension of state vector in the tangent space
model.dt = 1E-2; %Time step duration
model.params_diagRegFact = 1E-4; %Regularization of covariance
model.rfactor = 1E-10; %Control cost in LQR 
e0 = [0; -1; 0]; %Origin on manifold 

%Control cost matrix
R = eye(model.nbVarPos) * model.rfactor;
R = kron(eye(nbD-1),R);


%% Discrete dynamical System settings (in tangent space)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
A1d = zeros(model.nbDeriv);
for i=0:model.nbDeriv-1
	A1d = A1d + diag(ones(model.nbDeriv-i,1),i) * model.dt^i * 1/factorial(i); %Discrete 1D
end
B1d = zeros(model.nbDeriv,1); 
for i=1:model.nbDeriv
	B1d(model.nbDeriv-i+1) = model.dt^i * 1/factorial(i); %Discrete 1D
end
A = kron(A1d, eye(model.nbVarPos)); %Discrete nD
B = kron(B1d, eye(model.nbVarPos)); %Discrete nD

%Build Sx and Su matrices for batch LQR
Su = zeros(model.nbVar*nbD, model.nbVarPos*(nbD-1));
Sx = kron(ones(nbD,1),eye(model.nbVar));
M = B;
for n=2:nbD
	id1 = (n-1)*model.nbVar+1:nbD*model.nbVar;
	Sx(id1,:) = Sx(id1,:) * A;
	id1 = (n-1)*model.nbVar+1:n*model.nbVar; 
	id2 = 1:(n-1)*model.nbVarPos;
	Su(id1,id2) = M;
	M = [A*M(:,1:model.nbVarPos), M]; %Also M = [A^(n-1)*B, M] or M = [Sx(id1,:)*B, M]
end


%% Generate data on a sphere from handwriting data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
demos = [];
load('data/2Dletters/S.mat');
u0 = [];
for n=1:nbSamples
 	s(n).u0 = []; 
	for m=1:model.nbDeriv
		if m==1
			dTmp = rotM(e0)' * [spline(1:size(demos{n}.pos,2), demos{n}.pos, linspace(1,size(demos{n}.pos,2),nbData)) .* 9E-2; zeros(1,nbData)]; %Data in tangent space e0
		else
			dTmp = gradient(dTmp) ./ model.dt; %Compute derivatives expressed in single tangent space e0
		end
		s(n).u0 = [s(n).u0; dTmp];
	end
	u0 = [u0, s(n).u0]; 
end
x0 = [expmap(u0(1:model.nbVarPos,:), e0); u0(model.nbVarPos+1:end,:)]; %x0 is on the manifold and the derivatives are expressed in the tangent space of e0


%% GMM parameters estimation (encoding of [x;dx] with x on sphere and dx in Euclidean space)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
model = init_GMM_kbins(u0, model, nbSamples);
model.MuMan = [expmap(model.Mu(1:model.nbVarPos,:), e0); model.Mu(model.nbVarPos+1:end,:)]; %Center on the manifold
model.Mu = zeros(model.nbVarPos,model.nbStates); %Center in the tangent plane at point MuMan of the manifold
u = [];
for nb=1:nbIterEM
	%E-step
	L = zeros(model.nbStates,size(x0,2));
	for i=1:model.nbStates
		L(i,:) = model.Priors(i) * gaussPDF(logmap(x0(1:model.nbVarPos,:), model.MuMan(1:model.nbVarPos,i)), model.Mu(1:model.nbVarPos,i), model.Sigma(1:model.nbVarPos,1:model.nbVarPos,i));
	end
	GAMMA = L ./ repmat(sum(L,1)+realmin, model.nbStates, 1);
	H = GAMMA ./ repmat(sum(GAMMA,2),1,nbData*nbSamples);
	%M-step
	for i=1:model.nbStates
		%Update Priors
		model.Priors(i) = sum(GAMMA(i,:)) / (nbData*nbSamples);
		%Update MuMan
		for n=1:nbIter
			xTar = model.MuMan(1:model.nbVarPos,i); %Position target
			uTmp = logmap(x0(1:model.nbVarPos,:), xTar);
			
% 			dTmp = gradient(uTmp) / model.dt;
			dTmp = x0(model.nbVarPos+1:end,:); %Derivatives are expressed in the tangent space of e0
			
			%Transportation of derivatives from e0 to xTar (MuMan)
			Ac = transp(e0, xTar);
			dTmp = Ac * dTmp; %Derivatives are now expressed in xTar (MuMan)
			
			u(:,:,i) = [uTmp; dTmp];
			model.MuMan(:,i) = [expmap(uTmp*H(i,:)', xTar); dTmp*H(i,:)'];
		end
		%Update Sigma
		model.Sigma(:,:,i) = u(:,:,i) * diag(H(i,:)) * u(:,:,i)' + eye(size(u,1)) * model.params_diagRegFact;
% 		model.Sigma(:,:,i) = blkdiag(eye(model.nbVarPos)*1E-1, eye(model.nbVarPos)*1E-5);
	end
end

%Precomputation of inverses (Does Q stay in the same tangent space as Sigma?)
for i=1:model.nbStates
	model.Q(:,:,i) = inv(model.Sigma(:,:,i));
end

%List of states with time stamps
[~,q] = max(H(:,1:nbData),[],1); 
qList = [];
qCurr = q(1);
t_old = 1; 
for t=1:nbData
	if qCurr~=q(t) || t==nbData
		tm = t_old + floor((t-t_old)/2);
		qList = [qList, [tm; qCurr]];	
		qCurr = q(t);
		t_old = t;
	end
end	


%% Batch LQR recomputed in an online manner (computation centered on x)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for n=1:nbRepros
	x = x0(1:model.nbVarPos,1); % + rand(model.nbVarPos,1)*8E-2; 
	x = x / norm(x);
	x_old = x;
	U = zeros(model.nbVar,1);
	for t=1:nbData
		r(n).x(:,t) = x; %Log data
		U(1:model.nbVarPos) = zeros(model.nbVarPos,1); %Set tangent space at x
		
		%Transportation of velocity vectors from x_old to x
		Ac = transp(x_old, x);
		U(model.nbVarPos+1:end) = Ac * U(model.nbVarPos+1:end); 

% 		%Version with stepwise reference
% 		%Set list of states for the next nbD time steps according to first demonstration (alternatively, an HSMM can be used)
% 		id = [t:min(t+nbD-1,nbData), repmat(nbData,1,t-nbData+nbD-1)];
% 		[~,q] = max(H(:,id),[],1); %works also for nbStates=1
% 		%Create single Gaussian N(MuQ,SigmaQ) based on optimal state sequence q
% 		MuQ = zeros(model.nbVar*nbD,1);
% 		SigmaQ = zeros(model.nbVar*nbD);
% 		for s=1:nbD
% 			id = (s-1)*model.nbVar+1:s*model.nbVar;
% 			%Transportation of Sigma and duCov from model.MuMan to x
% 			Ac = transp(model.MuMan(1:model.nbVarPos,q(s)), x);
% 			SigmaQ(id,id) = blkdiag(Ac,Ac) * model.Sigma(:,:,q(s)) * blkdiag(Ac,Ac)';	
% 			%Transportation of du from model.MuMan to x
% 			dxTmp = Ac * model.MuMan(model.nbVarPos+1:end,q(s));
% 			MuQ(id) = [logmap(model.MuMan(1:model.nbVarPos,q(s)), x); dxTmp];
% 		end

		%Version with viapoints
		id = t:t+nbD-1; %Time window
		qid = qList(1,:) > id(1) & qList(1,:) < id(end); 
		qTmp = qList(:,qid); %List only the states appearing within the time window
		MuQ = zeros(model.nbVar*nbD,1);
		Q = zeros(model.nbVar*nbD);
% 		SigmaQ = eye(model.nbVar*nbD) * 9E8;
		for i=1:size(qTmp,2)
			s = qTmp(1,i) - t; %Time step
			id = (s-1)*model.nbVar+1:s*model.nbVar;
			
			%Transportation of Q from model.MuMan to x
			Ac = transp(model.MuMan(1:model.nbVarPos,qTmp(2,i)), x);
% 			SigmaQ(id,id) = blkdiag(Ac,Ac) * model.Sigma(:,:,qTmp(2,i)) * blkdiag(Ac,Ac)';
			Q(id,id) = blkdiag(Ac,Ac) * model.Q(:,:,qTmp(2,i)) * blkdiag(Ac,Ac)'; %Does Q stay in the same tangent space as Sigma?
% 			Q(id,id) = blkdiag(Ac,eye(model.nbVarPos)) * model.Q(:,:,qTmp(2,i)) * blkdiag(Ac,eye(model.nbVarPos))';

			%Transportation of du from model.MuMan to x
			dxTmp = Ac * model.MuMan(model.nbVarPos+1:end,qTmp(2,i));
			MuQ(id) = [logmap(model.MuMan(1:model.nbVarPos,qTmp(2,i)), x); dxTmp];
% 			MuQ(id) = [logmap(model.MuMan(1:3,qTmp(2,i)), x); model.MuMan(4:5,qTmp(2,i))]; 

			%Log data (for 2D plots in the tangent space -> z=0)
			r(n).s(t).Mu(:,i) = rotM(x) * logmap(model.MuMan(1:model.nbVarPos,qTmp(2,i)), x);
			r(n).s(t).Sigma(:,:,i) = rotM(x) * Ac * model.Sigma(1:model.nbVarPos,1:model.nbVarPos,qTmp(2,i)) * Ac' * rotM(x)';
		end
		
		%Compute acceleration commands
		ddu = (Su' * Q * Su + R) \ Su' * Q * (MuQ - Sx * U); %Does Q stay in the same tangent space as Sigma?
% 		ddu = (Su' / SigmaQ * Su + R) \ (Su' / SigmaQ * (MuQ - Sx * U));
		
		%Log data (for plots)
		r(n).s(t).u = reshape(Sx*U+Su*ddu, model.nbVar, nbD);
		r(n).s(t).u2 = rotM(x) * r(n).s(t).u(1:3,:); %for 2D plots in the tangent space -> z=0
		r(n).s(t).q = qTmp;
		
		U = A * U + B * ddu(1:model.nbVarPos); %Update U with first control command
		x_old = x; %Keep x for next iteration
		x = expmap(U(1:model.nbVarPos), x); %Update x
	end
end


%% Plots
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clrmap = lines(model.nbStates);

%Display of covariance contours on the sphere
tl = linspace(-pi, pi, nbDrawingSeg);
Gdisp = zeros(model.nbVarPos, nbDrawingSeg, model.nbStates);
% Gvel = zeros(model.nbVarPos, model.nbStates);
for i=1:model.nbStates
	[V,D] = eig(model.Sigma(1:model.nbVarPos,1:model.nbVarPos,i));
	[d,id] = sort(diag(D),'descend');
	V = V(:,id);
	D = diag(d);
	Gdisp(:,:,i) = expmap(V*D.^.5*[cos(tl); sin(tl); zeros(1,nbDrawingSeg)], model.MuMan(1:model.nbVarPos,i));
% 	Gvel(:,i) = expmap(model.MuMan(model.nbVarPos+1:end,i)*model.dt*5, model.MuMan(1:model.nbVarPos,i));
end

%S2 manifold plot
figure('position',[10,10,2500,700]); 
subplot(2,3,[1,4]); hold on; axis off; grid off; rotate3d on; 
colormap([.8 .8 .8]);
[X,Y,Z] = sphere(20);
mesh(X,Y,Z);
% plot3(x0(1,:), x0(2,:), x0(3,:), '.','markersize',10,'color',[.5 .5 .5]);
for i=1:model.nbStates
	plot3(model.MuMan(1,i), model.MuMan(2,i), model.MuMan(3,i), '.','markersize',24,'color',clrmap(i,:));
	plot3(Gdisp(1,:,i), Gdisp(2,:,i), Gdisp(3,:,i), '-','linewidth',2,'color',clrmap(i,:));
% 	plot3(Gvel(1,i), Gvel(2,i), Gvel(3,i), 'o','markersize',6,'linewidth',2,'color',clrmap(i,:));
end
for n=1:nbRepros
	plot3(r(n).x(1,:), r(n).x(2,:), r(n).x(3,:), '-','linewidth',2,'color',[.6 .6 .6]);
	plot3(r(n).x(1,1), r(n).x(2,1), r(n).x(3,1), '.','markersize',24,'color',[.6 .6 .6]);
end
view(-20,15); axis equal; axis([-1 1 -1 1 -1 1].*1.8); axis vis3d;

%Tangent plane plot
subplot(2,3,[2,5]); hold on; axis off; 
plot(0,0,'+','markersize',40,'linewidth',2,'color',[.3 .3 .3]);
plot(0,0,'.','markersize',20,'color',[0 0 0]);
axis equal; axis([-1 1 -1 1].*1.1);
	
%Timeline plot
labList = {'$x_1$','$x_2$','$\dot{x}_1$','$\dot{x}_2$','$\ddot{x}_1$','$\ddot{x}_2$'}; 
for j=1:2
	v(j).limAxes = [1, nbData, min(r(1).x(j,:))-3E-1, max(r(1).x(j,:))+3E-1];
	subplot(2,3,j*3); hold on;
	%Plot viapoints reference
	for i=1:size(qList,2)
		errorbar(qList(1,i), model.MuMan(j,qList(2,i)), model.Sigma(j,j,qList(2,i)).^.5, 'linewidth',2,'color',clrmap(qList(2,i),:));
		plot(qList(1,i), model.MuMan(j,qList(2,i)), '.','markersize',20,'color',clrmap(qList(2,i),:));
	end
	for n=1:nbRepros
		plot(r(n).x(j,:), '-','linewidth',2,'color',[.6 .6 .6]);
	end
	if j<7
		ylabel(labList{j},'fontsize',24,'interpreter','latex');
	end
	axis(v(j).limAxes);
	set(gca,'xtick',[],'ytick',[],'linewidth',2);
	xlabel('$t$','fontsize',24,'interpreter','latex');
end


%% Anim plots
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for tt=1:nbData %58:58
	e0 = r(n).x(:,tt);
	xTmp = expmap(r(1).s(tt).u(1:model.nbVarPos,:), e0);
	%Tangent plane on S2 anim
	subplot(2,3,[1,4]); hold on;
	msh = repmat(e0,1,5) + rotM(e0)' * [1 1 -1 -1 1; 1 -1 -1 1 1; 0 0 0 0 0] * 1E0;
	h = patch(msh(1,:),msh(2,:),msh(3,:), [.8 .8 .8],'edgecolor',[.6 .6 .6],'facealpha',.3,'edgealpha',.3);
	h = [h, plot3(e0(1), e0(2), e0(3), '.','markersize',20,'color',[0 0 0])];
	%Referential on S2 anim
	msh = repmat(e0,1,2) + rotM(e0)' * [1 -1; 0 0; 0 0] * 2E-1;
	h = [h, plot3(msh(1,:),msh(2,:),msh(3,:), '-','linewidth',2,'color',[.3 .3 .3])];
	msh = repmat(e0,1,2) + rotM(e0)' * [0 0; 1 -1; 0 0] * 2E-1;
	h = [h, plot3(msh(1,:),msh(2,:),msh(3,:), '-','linewidth',2,'color',[.3 .3 .3])];
	%Path on S2 anim
	h = [h, plot3(xTmp(1,:),xTmp(2,:),xTmp(3,:), '-','linewidth',2,'color',[0 0 0])];
	
	%Tangent plane anim
	subplot(2,3,[2,5]); hold on; axis off; 
	for i=1:size(r(1).s(tt).Mu,2)
		h = [h, plotGMM(r(1).s(tt).Mu(1:2,i), r(1).s(tt).Sigma(1:2,1:2,i), clrmap(r(1).s(tt).q(2,i),:), .3)];
	end
	h = [h, plot(r(1).s(tt).u2(1,:), r(1).s(tt).u2(2,:), '-','linewidth',2,'color',[0 0 0])];
	
	%Time window anim
	for j=1:2
		subplot(2,3,j*3); hold on;
		msh = [tt tt+nbD-1 tt+nbD-1 tt tt; v(j).limAxes([3,3]) v(j).limAxes([4,4]) v(j).limAxes(3)];
		h = [h, plot(msh(1,:), msh(2,:), '-','linewidth',1,'color',[.6 .6 .6])];
		h = [h, plot(tt:tt+nbD-1, xTmp(j,:), '-','linewidth',2,'color',[0 0 0])];
		h = [h, plot(tt, xTmp(j,1), '.','markersize',20,'color',[0 0 0])];
	end

	drawnow;
% 	pause;
% 	print('-dpng','graphs/demo_Riemannian_Sd_MPC01.png');
	delete(h);
end

pause;
close all;
end


%% Functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function x = expmap(u,x0)
	theta = sqrt(sum(u.^2,1)); %norm(u,'fro')
	x = real(repmat(x0,[1,size(u,2)]) .* repmat(cos(theta),[size(u,1),1]) + u .* repmat(sin(theta)./theta,[size(u,1),1]));
	x(:,theta<1e-16) = repmat(x0,[1,sum(theta<1e-16)]);	
end

function u = logmap(x,x0)
	theta = acos(x0'*x); %acos(trace(x0'*x))
	u = (x - repmat(x0,[1,size(x,2)]) .* repmat(cos(theta),[size(x,1),1])) .* repmat(theta./sin(theta),[size(x,1),1]);
	u(:,theta<1e-16) = 0;
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