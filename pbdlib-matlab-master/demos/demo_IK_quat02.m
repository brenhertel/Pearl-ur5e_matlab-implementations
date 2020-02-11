function demo_IK_quat02
% Inverse kinematics for orientation tracking with unit quaternions
% (kinematic model of WAM 7-DOF arm or an iCub 7-DOF arm)
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
nbData = 100; %Number of datapoints
Kp = 2; %Tracking gain


%% Create robot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Robot parameters (built from function that contains WAM D-H parameters)
%[robot,L] = initWAMstructure('WAM');
[robot, link] = initiCubstructure('iCub');
nbDOFs = robot.n;

%Initial pose
q0(:,1) = zeros(1,nbDOFs);
%Set desired orientation
Qh = UnitQuaternion([0 0 1 0]); % Aligned with the basis frame


%% IK
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
q = q0;
for t=1:nbData
	r.q(:,t) = q; %Log data
	Htmp = robot.fkine(q);
	Q = UnitQuaternion(Htmp);
	Jw = robot.jacob0(q,'rot');
	
	%The new Jacobian is built from the geometric Jacobian to allow the
	%tracking of quaternion velocities in Cartesian space	
	J = 0.5 * QuatToMatrix(Q) * [zeros(1,nbDOFs);Jw];
	
	%Compute the quaternion derivative
	%-> The quaternion derivative at time t is given by:
	%dq(t) = (1/2) * q(t) * w(t), where w(t) is the angular velocity
	
	w = Kp*2*quaternionLog(Qh*Q.inv); % first compute angular velocity
	dQh = 0.5*Q*UnitQuaternion([0 w]); % quaternion derivative
	
	r.Q(t,:) = Q.double; %Log the quaternion at each instant
	
	dq = pinv(J) * dQh.double'; % now with a quaternion derivative reference
	%N = eye(nbDOFs) - pinv(J)*J; %Nullspace
	%dq = pinv(J) * dQh.double' + N * [1; 0; 0; 0; 0];
	%dq = N * [-1; -1; 0; 0; 1];
	
	q = q + dq * dt;
end


%% Plots
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('position',[20,10,1000,650]); hold on; rotate3d on;

% %Plot animation of the movement (with robotics toolbox functionalities)
% plotopt = robot.plot({'noraise', 'nobase', 'noshadow', 'nowrist', 'nojaxes'});
% robot.plot(r.q'); %'floorlevel',-0.5

%Plot animation of the movement (with standard plot functions)
posTmp(:,1) = zeros(3,1);
for t=round(linspace(1,nbData,10))
	colTmp = [.9,.9,.9] - [.7,.7,.7] * t/nbData;
	for i=1:nbDOFs
		T = link(i).fkine(r.q(1:i,t));
		posTmp(:,i+1) = T.t(1:3,end);
	end
	plot3(posTmp(1,:),posTmp(2,:),posTmp(3,:), '-','linewidth',2,'color',colTmp);
	plot3(posTmp(1,:),posTmp(2,:),posTmp(3,:), '.','markersize',18,'color',colTmp);
% 	plot3Dframe(T.t(1:3,1:3)*1E-1, T.t(1:3,end), min(eye(3)+colTmp(1,1),1));
end
plot3Dframe(eye(3)*2E-1, zeros(3,1));
set(gca,'xtick',[],'ytick',[],'ztick',[]);
xlabel('x_1'); ylabel('x_2'); zlabel('x_3');
view(3); axis equal; axis vis3d;

% %Plot quaternion trajectories
% figure; hold on;
% h1 = plot(dt*(1:nbData),repmat(Qh.double,nbData,1));
% h2 = plot(dt*(1:nbData),r.Q,'Linewidth',3);
% title('Quaternion elements (thin line = demonstration , bold line = reproduction)');
% xlabel('t');

pause;
close all;