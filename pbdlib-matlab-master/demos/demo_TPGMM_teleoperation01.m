function demo_TPGMM_teleoperation01
% Time-invariant task-parameterized GMM applied to a teleoperation task (position and orientation).
%
% If this code is useful for your research, please cite the related publication:
% @article{Calinon16JIST,
%   author="Calinon, S.",
%   title="A Tutorial on Task-Parameterized Movement Learning and Retrieval",
%   journal="Intelligent Service Robotics",
%		publisher="Springer Berlin Heidelberg",
%		doi="10.1007/s11370-015-0187-9",
%		year="2016",
%		volume="9",
%		number="1",
%		pages="1--29"
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
disp('Use the mouse wheel to change the orientation, and move close or far from the line to see the change of behavior.');


%% Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nbData = 100;
model.nbStates = 2; %Number of Gaussians in the GMM
model.nbFrames = 1; %Number of candidate frames of reference
model.nbVarIn = 2; %Input dimension (position of object)
model.nbVarOut = 1; %Output dimension (orientation of robot end-effector)
model.nbVar = model.nbVarIn + model.nbVarOut;
model.dt = 0.01; %Time step duration
model.rfactor = 1E-2;	%Control cost in LQR
model.tfactor = 1E-2;	%Teleoperator cost

%Dynamical System settings 
A = kron([0 1; 0 0], eye(model.nbVarOut));
B = kron([0; 1], eye(model.nbVarOut));
%Control cost matrix
R = eye(model.nbVarOut) * model.rfactor;


%% Generate data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
posObj = [ones(1,nbData)*0; ones(1,nbData)*-0.3; mvnrnd(0,eye(model.nbVarOut)*1E-3,nbData)']; %Object position and orientation
Data0 = [repmat([0.2;-0.2],1,nbData/2)+randn(2,nbData/2)*1E-1, repmat([0.2;0.2],1,nbData/2)+randn(2,nbData/2)*1E-1]; %Path of robot/teleoperator
Data0 = [Data0; [posObj(end,1:nbData/2)-pi/2+randn(1,nbData/2)*1E-3, linspace(0,1,nbData/2)+randn(1,nbData/2)*1E-1]];

%Set task parameters
for t=1:nbData
	model.p(1,t).b = posObj(:,t); %param1 (object)
	for m=1:model.nbFrames
		%model.p(m,t).A = eye(model.nbVar);
		model.p(m,t).A = [cos(posObj(end,t)), -sin(posObj(end,t)), 0; sin(posObj(end,t)) cos(posObj(end,t)) 0; 0 0 1];
		model.p(m,t).invA = inv(model.p(m,t).A); %Precomputation of inverse
	end
end

%Observation of data from the perspective of the frames
for m=1:model.nbFrames
	for t=1:nbData
		Data(:,m,t) = model.p(m,t).invA * (Data0(:,t) - model.p(m,t).b); 
	end
end


%% TP-GMM learning
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fprintf('Parameters estimation of TP-GMM with EM');
model = init_tensorGMM_kmeans(Data, model); 
model = EM_tensorGMM(Data, model);


%% Reproduction
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fprintf('Reproduction\n');

%Simulate object and teleoperator trajectories
rPosObj = [ones(1,nbData)*0; ones(1,nbData)*-0.2; ones(1,nbData).*-0.2]; %Object position and orientation

aTmp = linspace(0,4*pi,nbData);
rPosTel = [ones(1,nbData).*aTmp*(0.02/4*pi)+sin(aTmp)*0.05+0.1; cos(aTmp)*0.4; ones(1,nbData).*0.2]; %Teleoperator position and orientation 

x = rPosTel(model.nbVarIn+1:model.nbVarIn+model.nbVarOut,1);
dx = zeros(model.nbVarOut,1);
for t=1:nbData
	%Frame1 (object)
	%pTmp(1).A = eye(model.nbVar);
	pTmp(1).A = [cos(rPosObj(end,t)), -sin(rPosObj(end,t)), 0; sin(rPosObj(end,t)) cos(rPosObj(end,t)) 0; 0 0 1];
	pTmp(1).b = rPosObj(:,t);

	%GMR with GMM adapted to the current situation
	mtmp.nbStates = model.nbStates;
	mtmp.Priors = model.Priors;
	for i=1:mtmp.nbStates
		mtmp.Mu(:,i) = pTmp(1).A * squeeze(model.Mu(:,1,i)) + pTmp(1).b;
		mtmp.Sigma(:,:,i) = pTmp(1).A * squeeze(model.Sigma(:,:,1,i)) * pTmp(1).A';
	end
	[MuOut(:,1), SigmaOut(:,:,1)] = GMR(mtmp, rPosTel(1:model.nbVarIn,t), 1:model.nbVarIn, model.nbVarIn+1:model.nbVarIn+model.nbVarOut);
	
	%Second Gaussian as teleoperator
	MuOut(:,2) = rPosTel(model.nbVarIn+1:model.nbVarIn+model.nbVarOut,t);
	SigmaOut(:,:,2) = eye(model.nbVarOut) * model.tfactor; 
	
	%Product of Gaussians
	SigmaTmp = zeros(model.nbVarOut);
	MuTmp = zeros(model.nbVarOut,1);
	for m=1:2
		SigmaTmp = SigmaTmp + inv(SigmaOut(:,:,m));
		MuTmp = MuTmp + SigmaOut(:,:,m) \ MuOut(:,m);
	end
	rMu(:,t) = SigmaTmp \ MuTmp;
	rSigma(:,:,t) = inv(SigmaTmp);
	
	%Linear quadratic tracking (infinite horizon)
	Q = zeros(model.nbVarOut*2);
	Q(1:model.nbVarOut,1:model.nbVarOut) = SigmaTmp;
	P = solveAlgebraicRiccati_eig(A, B/R*B', (Q+Q')/2); 
	L = R\B'*P; %Feedback term
	ddx = L * ([rMu(:,t); zeros(model.nbVarOut,1)] - [x; dx]); %Compute acceleration (with only feedback terms)
	dx = dx + ddx * model.dt;
	x = x + dx * model.dt;
	rData(:,t) = [rPosTel(1:model.nbVarIn,t); x];
end


% %% Plot
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% fprintf('Plots\n');
% 
% figure('position',[20,50,1300,500]);
% clrmap = lines(model.nbStates);
% ttl={'Frame 1 (in)','Frame 1 (out)'};
% 
% %DEMOS
% subplot(1,4,1); hold on; box on; title('Demonstrations');
% plotPlane(posObj, [0 0 0]);
% plotEE(Data0, [.7 0 .7]);
% %legend('object','robot','teleoperator');
% axis equal; %set(gca,'xtick',[],'ytick',[]);
% 
% %FRAME IN 
% subplot(1,4,2); hold on; grid on; box on; title(ttl{1});
% plot(squeeze(Data(1,1,:)), squeeze(Data(2,1,:)), '.','markersize',15,'color',[.7 0 .7]);
% for i=1:model.nbStates
% 	plotGMM(squeeze(model.Mu(1:2,1,i)), squeeze(model.Sigma(1:2,1:2,1,i)), clrmap(i,:), .4);
% end
% axis equal; %set(gca,'xtick',[0],'ytick',[0]);
% 
% %FRAME OUT 
% subplot(1,4,3); hold on; grid on; box on; title(ttl{2});
% for i=1:model.nbStates
% 	mtmp.nbStates = 1;
% 	mtmp.Priors = model.Priors(i);
% 	mtmp.Mu = model.Mu(3,1,i);
% 	mtmp.Sigma = model.Sigma(3,3,1,i);
% 	plotGMM1D(mtmp, clrmap(i,:), [-pi,0,2*pi,1], .3, 50);	
% end
% % for t=1:5:nbData
% % 	mtmp.nbStates = 1;
% % 	mtmp.Priors = 1;
% % 	mtmp.Mu = rMu(:,t);
% % 	mtmp.Sigma = rSigma(:,:,t);
% % 	plotGMM1D(mtmp, [.2 .2 .2], [-1,0,5,1], .3, 50);
% % end
% 
% %REPRO
% subplot(1,4,4); hold on; grid on; box on; title('Reproduction');
% axis equal; 
% %axis([min(rPosObj(1,:)) max(rPosObj(1,:)) min(rPosObj(2,:)) max(rPosObj(2,:))]); set(gca,'xtick',[0],'ytick',[0]);
% plotPlane(rPosObj, [0 0 0]);
% plotEE(rPosTel, [.5 .5 .5]);
% plotEE([rPosTel(1:model.nbVarIn,:); rMu], [0 .7 0]);
% plotEE(rData, [.7 0 .7]);
% 
% %print('-dpng','graphs/demo_TPGMM_teleoperation02.png');
% pause;
% close all;


% %% Plot
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% clrmap = lines(model.nbStates);
% figure('position',[20,50,1300,500]);
% %FRAME IN 
% subplot(1,2,1); hold on; grid on; box on; 
% %plot(squeeze(Data(1,1,:)), squeeze(Data(2,1,:)), '.','markersize',15,'color',[.7 0 .7]);
% for i=1:model.nbStates
% 	plotGMM(squeeze(model.Mu(1:2,1,i)), squeeze(model.Sigma(1:2,1:2,1,i)), clrmap(i,:), .4);
% end
% plot(0, 0, '+','markersize',15,'linewidth',2,'color',[0 0 0]);
% axis([-0.1,0.4,-0.1 0.8]); 
% set(gca,'xtick',[0],'ytick',[0],'fontsize',18);
% 
% %FRAME OUT 
% subplot(1,2,2); hold on; grid on; box on; 
% for i=1:model.nbStates
% 	mtmp.nbStates = 1;
% 	mtmp.Priors = model.Priors(i);
% 	mtmp.Mu = model.Mu(3,1,i);
% 	mtmp.Sigma = model.Sigma(3,3,1,i) + 1E-3;
% 	plotGMM1D(mtmp, clrmap(i,:), [-pi,0,2*pi,1], .3, 200);	
% end
% axis([-pi,pi,0,1.2]);
% set(gca,'xtick',[-pi,-pi/2,0,pi/2,pi],'xticklabel',{'-pi','-pi/2','0','pi/2','pi'},'ytick',[],'fontsize',18);
% 
% %print('-dpng','graphs/demo_TPGMM_teleoperation_model01.png');
% pause;
% close all;


%% Interactive plot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fig = figure('position',[10,10,1300,1300],'name','Move the robot with the mouse and wheel mouse'); hold on; box on;
set(fig,'WindowButtonMotionFcn',{@wbm});
set(fig,'WindowButtonDownFcn',{@wbd});
set(fig,'WindowScrollWheelFcn',{@wsw});
set(fig,'CloseRequestFcn',@crq);
setappdata(gcf,'mw',0);
setappdata(gcf,'cr',0);
set(gca,'Xtick',[]); set(gca,'Ytick',[]);
axis equal; axis([0 .3 -0.3 0.3]); 
clrmap = lines(model.nbStates);

rpo = rPosObj(:,1);
%xt = zeros(3,1);
xt = [zeros(2,1); -pi/2];
%x = zeros(model.nbVarOut,1);
x = -pi/2;
dx = zeros(model.nbVarOut,1);
h = [];
id = 1;
while getappdata(gcf,'cr')==0
	cur_point = get(gca,'Currentpoint');
	xt(1:2) = cur_point(1,1:2)';
	xt(3) = xt(3) + getappdata(gcf,'mw') * 0.1;
	setappdata(gcf,'mw',0);
	
	%Frame1 (object)
	offtmp = getappdata(gcf,'mb');
	if offtmp~=0
		rpo(end) = rpo(end) + offtmp * 0.1;
		setappdata(gcf,'mb',0);
	end
	pTmp(1).A = [cos(rpo(end)), -sin(rpo(end)), 0; sin(rpo(end)) cos(rpo(end)) 0; 0 0 1];
	pTmp(1).b = rpo;
	%GMM adapted to the current situation
	mtmp = [];
	mtmp.nbStates = model.nbStates;
	mtmp.Priors = model.Priors;
	for i=1:mtmp.nbStates
		mtmp.Mu(:,i) = pTmp(1).A * squeeze(model.Mu(:,1,i)) + pTmp(1).b;
		mtmp.Sigma(:,:,i) = pTmp(1).A * squeeze(model.Sigma(:,:,1,i)) * pTmp(1).A';
	end
		
	%GMR 
	[MuOut(:,1), SigmaOut(:,:,1)] = GMR(mtmp, xt(1:model.nbVarIn), 1:model.nbVarIn, model.nbVarIn+1:model.nbVarIn+model.nbVarOut);
	
	%Second Gaussian as teleoperator
	MuOut(:,2) = xt(model.nbVarIn+1:model.nbVarIn+model.nbVarOut);
	SigmaOut(:,:,2) = eye(model.nbVarOut) * model.tfactor; 
	
	%Product of Gaussians
	SigmaTmp = zeros(model.nbVarOut);
	MuTmp = zeros(model.nbVarOut,1);
	for m=1:2
		SigmaTmp = SigmaTmp + inv(SigmaOut(:,:,m));
		MuTmp = MuTmp + SigmaOut(:,:,m) \ MuOut(:,m);
	end
	rMu = SigmaTmp \ MuTmp;
	
	%rMu = MuOut(:,2); %Simulate demo phase
	
	%Linear quadratic tracking (infinite horizon)
	Q = zeros(model.nbVarOut*2);
	Q(1:model.nbVarOut,1:model.nbVarOut) = SigmaTmp;
	P = solveAlgebraicRiccati_eig(A, B/R*B', (Q+Q')/2); 
	L = R\B'*P; %Feedback term
	ddx = L * ([rMu; zeros(model.nbVarOut,1)] - [x; dx]); %Compute acceleration (with only feedback terms)
	dx = dx + ddx * model.dt;
	x = x + dx * model.dt;
	xr = [xt(1:model.nbVarIn); x];
	xh = [xt(1:model.nbVarIn); rMu];
	xh2 = [xt(1:model.nbVarIn); MuOut(:,1)];
	
	%Plot
	delete(h);
	h = plotPlane(rpo,[0 0 0]);
	%for i=1:model.nbStates
	%	h = [h plotGMM(mtmp.Mu(1:model.nbVarIn,i), mtmp.Sigma(1:model.nbVarIn,1:model.nbVarIn,i), clrmap(i,:), .4)];
	%end
	h = [h plotEE(xt,[0 0 0])];
	h = [h plotEE(xr,[.8 0 0])];
	%h = [h plotEE(xh,[0 .7 0])];
	%h = [h plotEE(xh2,[.5 1 .5])];
	drawnow;
	%print('-dpng',['graphs/anim/teleop_demo' num2str(id,'%.3d') '.png']);
	%print('-dpng',['graphs/anim/teleop_repro' num2str(id,'%.3d') '.png']);
	id = id+1;
end
delete(gcf);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function h = plotEE(x,col)
h = plot(x(1,:), x(2,:), '.','markersize',20,'color',col);
for t=1:size(x,2)
	msh = [x(1:2,t), x(1:2,t)+[cos(x(3,t)); sin(x(3,t))]*0.03]; 
	h = [h plot(msh(1,:), msh(2,:), '-','linewidth',3,'color',col)];
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function h = plotPlane(x,col)
h = []; %plot(x(1,:), x(2,:), '.','markersize',20,'color',col);
for t=1:size(x,2)
	msh = [x(1:2,t), x(1:2,t)+[cos(x(3,t)); sin(x(3,t))]*0.5]; 
	h = [h plot(msh(1,:), msh(2,:), '-','linewidth',3,'color',col)];
end

%% Mouse move
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function wbm(h,evd)

%% Mouse scroll wheel
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function wsw(h,evd) 
setappdata(gcf,'mw',evd.VerticalScrollCount);

%% Mouse button down
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function wbd(h,evd) % executes when the mouse button is pressed
muoseside = get(gcf,'SelectionType');
if strcmp(muoseside,'normal')==1
	setappdata(gcf,'mb',-1);
end
if strcmp(muoseside,'alt')==1
	setappdata(gcf,'mb',1);
end

%% Close request function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function crq(h,evd) 
setappdata(gcf,'cr',1);