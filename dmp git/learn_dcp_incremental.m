% function to learn a DCP in incremental mode using a minimum jerk trajectory as
% template

global dcps;


% general parameters
dt        = 0.001;
goal      = 1;
tau       = 10;
n_rfs     = 10;
ID        = 1;

dcp('clear',ID);
dcp('init',ID,n_rfs,'minJerk_dcp',1);

% initialize some variables for plotting
Z=zeros(floor(tau/dt+1),2);
T=zeros(floor(tau/dt+1),3);
Y=T;
PSI=zeros(floor(tau/dt+1),n_rfs);
W=PSI;
X=Z;
V=Z;

% one sweep through the training data is all that is needed
n_fits = 1;

for r=1:n_fits+1

  dcp('reset_state',ID);
  dcp('set_goal',ID,goal,1);

  t=0;
  td=0;
  tdd=0;

  for i=0:tau/dt

    % the target trajectory computed by minimum jerk
    if (tau-i*dt > 0)
      [t,td,tdd]=min_jerk_step(t,td,tdd,goal,tau-i*dt,dt);
    end

    if r==n_fits+1 % on the last run, only prediciton is tested
      [y,yd,ydd]=dcp('run',ID,tau,dt);
    else % fit the desired trajectory
      [y,yd,ydd]=dcp('run_fit',ID,tau,dt,t,td,tdd);
    end

    Z(i+1,:)   = [dcps(ID).z dcps(ID).zd];
    T(i+1,:)   = [t td tdd];
    Y(i+1,:)   = [y yd ydd];
    V(i+1,:)   = [dcps(ID).v dcps(ID).vd];
    X(i+1,:)   = [dcps(ID).x dcps(ID).xd];
    PSI(i+1,:) = dcps(ID).psi';
    W(i+1,:)   = dcps(ID).w';

  end


  %plotting
  time = (0:dt:tau)';

  figure(1);
  clf;

  % plot position, velocity, acceleration vs. target
  subplot(431);
  plot(time,[Y(:,1) T(:,1)]);
  title('y vs.t');
  aa=axis;
  axis([min(time) max(time) aa(3:4)]);

  subplot(432);
  plot(time,[Y(:,2) T(:,2)]);
  title('yd vs. td');
  aa=axis;
  axis([min(time) max(time) aa(3:4)]);

  subplot(433);
  plot(time,[Y(:,3) T(:,3)]);
  title('ydd vs. tdd');
  aa=axis;
  axis([min(time) max(time) aa(3:4)]);

  % plot internal states
  subplot(434);
  plot(time,Z(:,1));
  title('z');
  aa=axis;
  axis([min(time) max(time) aa(3:4)]);

  subplot(435);
  plot(time,Z(:,2));
  title('zd');
  aa=axis;
  axis([min(time) max(time) aa(3:4)]);

  subplot(436);
  plot(time,PSI);
  title('Weighting Kernels');
  aa=axis;
  axis([min(time) max(time) aa(3:4)]);

  subplot(437);
  plot(time,V(:,1));
  title('v');
  aa=axis;
  axis([min(time) max(time) aa(3:4)]);

  subplot(438);
  plot(time,V(:,2));
  title('vd');
  aa=axis;
  axis([min(time) max(time) aa(3:4)]);

  subplot(439);
  plot(time,W);
  title('Linear Model Weights over Time');
  aa=axis;
  axis([min(time) max(time) aa(3:4)]);

  subplot(4,3,10);
  plot(time,X(:,1));
  title('x');
  aa=axis;
  axis([min(time) max(time) aa(3:4)]);

  subplot(4,3,11);
  plot(time,X(:,2));
  title('xd');
  aa=axis;
  axis([min(time) max(time) aa(3:4)]);

  subplot(4,3,12);
  plot(W(end,:));
  title('Weights');
  xlabel(sprintf('tau=%f',tau));

  drawnow;

end;
