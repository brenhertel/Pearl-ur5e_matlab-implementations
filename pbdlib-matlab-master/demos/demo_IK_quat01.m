function demo_IK_quat01
% Inverse kinematics for orientation data represented as unit quaternions. 
%
% This demo requires the robotics toolbox RTB10 (http://petercorke.com/wordpress/toolboxes/robotics-toolbox).
% First run 'startup_rvc' from the robotics toolbox.
%
% Copyright (c) 2019 Idiap Research Institute, http://idiap.ch/
% Written by João Silvério and Sylvain Calinon
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
nbData = 100;


%% Create robot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Robot parameters (based on mdl_twolink.m model)
nbDOFs = 5; %Nb of degrees of freedom
armLength = 0.2;
L1 = Link('d', 0, 'a', armLength, 'alpha', 0);
robot = SerialLink(repmat(L1,nbDOFs,1));
%Initial pose
% q0(:,1) = [pi/2 pi/2 pi/3];
q0(:,1) = [pi/5 pi/5 pi/5 pi/8 pi/8];

%Set desired pose (pointing vertically)
% Htmp = robot.fkine([pi/2 pi/2 pi/2]);
Htmp = robot.fkine([pi/5 pi/5 pi/5 pi/5 pi/5]);

Qh = UnitQuaternion(Htmp); % Q denotes quaternion, q denotes joint position
% Qh = Quaternion(-pi/4, [0 0 1]); % -> Quaternion(angle,vector)
% Qh = Quaternion(-3*pi/4, [0 0 1]);
% Qh = Quaternion(-rand*pi/4, [0 0 1]);


%% IK
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
q = q0;
for t=1:nbData
	r.q(:,t) = q; %Log data
	Htmp = robot.fkine(q);
	Q = UnitQuaternion(Htmp);
	J = robot.jacob0(q,'rot');
	dx = angDiffFromQuat(Qh,Q) / dt;
	dq = pinv(J) * dx;
	q = q + dq * dt;
end


%% Plots
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('position',[20,10,1000,650]); hold on;
%plotopt = robot.plot({'noraise', 'nobase', 'noshadow', 'nowrist', 'nojaxes'});
for t=round(linspace(1,nbData,10))
	colTmp = [1,1,1] - [.7,.7,.7] * t/nbData;
	plotArm(r.q(:,t), ones(nbDOFs,1)*armLength, [0;0;t*0.1], .02, colTmp);
	%robot.plot(r.q(:,t)', plotopt);
end
axis([-.2 1 -0.4 1]); axis equal; set(gca,'xtick',[],'ytick',[]);

%print('-dpng','graphs/demoIK_quat02.png');
pause;
close all;