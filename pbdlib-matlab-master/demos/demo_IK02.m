function demo_IK02
% Inverse kinematics with two arms and nullspace.
%
% This demo requires the robotics toolbox RTB10 (http://petercorke.com/wordpress/toolboxes/robotics-toolbox).
% First run 'startup_rvc' from the robotics toolbox.
%
% If this code is useful for your research, please cite the related publication:
% @article{Silverio19TRO,
% 	author="Silv\'erio, J. and Calinon, S. and Rozo, L. and Caldwell, D. G.",
% 	title="Learning Task Priorities from Demonstrations",
% 	journal="{IEEE} Trans. on Robotics",
% 	year="2019",
% 	volume="35",
% 	number="1",
% 	pages="78--94",
% 	doi="10.1109/TRO.2018.2878355"
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
disp('This demo requires the robotics toolbox RTB10 (http://petercorke.com/wordpress/toolboxes/robotics-toolbox).');


%% Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
dt = 0.01; %Time step
nbData = 500; %Numer of points in a trajectory


%% Create robot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Robot parameters
nbDOFs = 5; %Number of articulations
armLength = 0.6; %Length of each link
L1 = Link('d', 0, 'a', armLength, 'alpha', 0);
arm = SerialLink(repmat(L1,3,1));

%Initial pose
q0(:,1) = [pi/2 pi/2 pi/3 -pi/2 -pi/3];
q = q0;


%% IK with nullspace
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
r=[];
for t=1:nbData
	r.q(:,t)=q; %Log data
	
	Htmp = arm.fkine(q([1,4:5]));
	r.rx(:,t) = Htmp.t(1:2);
	Htmp = arm.fkine(q(1:3));
	r.lx(:,t) = Htmp.t(1:2);

	if t==1
		rxh = [linspace(r.rx(1,1),r.rx(1,1)+.3,nbData); linspace(r.rx(2,1),r.rx(2,1)+1.2,nbData)];
		lxh = [linspace(r.lx(1,1),r.lx(1,1)-.3,nbData); linspace(r.lx(2,1),r.lx(2,1)+1.2,nbData)];
	end

	rJ = arm.jacob0(q([1,4:5]),'trans');
	rJ = rJ(1:2,:);
	
	lJ = arm.jacob0(q(1:3),'trans');
	lJ = lJ(1:2,:);

	J = lJ; 
	J(3:4,[1,4:5]) = rJ;
	
	Ja = J(1:2,:);
	Jb = J(3:4,:);
	
% 	Na = eye(nbDOFs) - pinv(Ja) * Ja; %Nullspace projection matrix
	Nb = eye(nbDOFs) - pinv(Jb) * Jb; %Nullspace projection matrix
	
	ldx = 100 * (lxh(:,t) - r.lx(:,t));
	rdx = 100 * (rxh(:,t) - r.rx(:,t));
	
  %Prioritized bimanual tracking task
  Ja_mod = Ja * Nb;
  pinvJa_mod = Ja_mod' / (Ja_mod * Ja_mod' + eye(2) .* 1E-1); 
  dqb = pinv(Jb) * rdx;
  dq =  dqb + Nb * pinvJa_mod * (ldx - Ja * dqb);	
% 	dq =  dqb + Nb * pinv(Ja) * ldx; %Approximation
	
	q = q + dq*dt;
end


%% Plots
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('PaperPosition',[0 0 16 12],'position',[20,10,800,650],'color',[1 1 1]); hold on; axis off;
listT = round(linspace(1,nbData,10));
i=0;
for t=listT
	i=i+1;
	colTmp = [.9,.9,.9] - [.7,.7,.7] * i/length(listT);
	plotArm(r.q(1:3,t), ones(3,1)*armLength, [0;0;t/nbData], .03, colTmp); %left arm
	plotArm(r.q([1,4:5],t), ones(3,1)*armLength, [0;0;t/nbData+0.1], .03, colTmp); %right arm
end
plot3(rxh(1,:), rxh(2,:), ones(1,nbData)*2, 'r-','linewidth',2);
plot3(lxh(1,:), lxh(2,:), ones(1,nbData)*2, 'r-','linewidth',2);
%axis([-.2 1 -.2 1]); 
axis equal; axis tight;

%print('-dpng','graphs/demoIK02.png');
pause;
close all;