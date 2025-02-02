% function to learn a RCP in batch mode using an arbitrary periodic trajectory as
% template

global rcps;

% general parameters
dt     = 0.01;
ym     = 0;
A      = 1;
n_rfs  = 50;
ID     = 1;
tau    = 1;

% initialize some arrays for data monitoring
T   = zeros(round(2*tau/dt+1),3);
P   = zeros(length(T),2);
Z   = zeros(length(T),2);
Y   = zeros(length(T),3);
PSI = zeros(length(T),n_rfs);
W   = zeros(length(T),n_rfs);

% this rcp will learn from the target data
rcp('init',ID,n_rfs,'learn_rcp_incremental');
rcp('reset_state',ID);
rcp('set_baseline',ID,ym);
rcp('set_amplitude',ID,A);

% create target data from Fourier series
omega = 2*pi/tau*4;
for i=0:2*tau/dt,
  t   = sin(omega*dt*i) + .25*cos(2*omega*dt*i + 0.77) + 0.1*sin(3*omega*dt*i+3);
  td  = omega*cos(omega*dt*i) - 2*omega*.25*sin(2*omega*dt*i + 0.77) + 3*omega*0.1*cos(3*omega*dt*i+3);
  tdd = -omega^2*sin(omega*dt*i) - (2*omega)^2*.25*cos(2*omega*dt*i + 0.77) - (3*omega)^2*0.1*sin(3*omega*dt*i+3);
  T(i+1,:)   = [t td tdd];
end;

% batch fitting
[Yp,Ypd,Ypdd]=rcp('batch_fit',ID,tau,dt,T(:,1),T(:,2),T(:,3));

% create predicted trajectory
rcp('reset_state',ID);
rcp('set_baseline',ID,ym);
rcp('set_amplitude',ID,A);

for i=0:2*tau/dt,
  [y,yd,ydd]=rcp('run',ID,tau,dt);
  P(i+1,:)   = [rcps(ID).p rcps(ID).pd];
  Z(i+1,:)   = [rcps(ID).z rcps(ID).zd];
  Y(i+1,:)   = [y yd ydd];
  PSI(i+1,:) = rcps(ID).psi';
  W(i+1,:,:) = rcps(ID).w;
end;

% plotting
time = (0:dt:tau*2)';

figure(1);
clf;

% plot position, velocity, acceleration vs. target
subplot(331);
plot(time,[Y(:,1) T(:,1)]) ;
title('y');
aa=axis;
axis([min(time) max(time) aa(3:4)]);

subplot(332);
plot(time,[Y(:,2) T(:,2)]);
title('yd');
aa=axis;
axis([min(time) max(time) aa(3:4)]);

subplot(333);
plot(time,[Y(:,3) T(:,3)]);
title('ydd');
aa=axis;
axis([min(time) max(time) aa(3:4)]);

% plot internal states
subplot(334);
plot(time,Z(:,1));
title('z');
aa=axis;
axis([min(time) max(time) aa(3:4)]);

subplot(335);
plot(time,Z(:,2));
title('zd');
aa=axis;
axis([min(time) max(time) aa(3:4)]);

subplot(336);
plot(time,PSI);
title('Weighting Kernels');
aa=axis;
axis([min(time) max(time) aa(3:4)]);

subplot(337);
plot(time,P(:,1));
title('p');
aa=axis;
axis([min(time) max(time) aa(3:4)]);

subplot(338);
plot(time,P(:,2));
title('pd');
aa=axis;
axis([min(time) max(time) aa(3:4)]);

subplot(339);
plot(W(end,:));
title('Weights');
xlabel(sprintf('tau=%f',tau));

drawnow;



