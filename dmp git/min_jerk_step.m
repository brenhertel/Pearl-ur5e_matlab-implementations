function [x,xd,xdd] = min_jerk_step(x,xd,xdd,goal, tau, dt)
% function [x,xd,xdd] = min_jerk_step(x,xd,xdd,goal,tau, dt) computes
% the update of x,xd,xdd for the next time step dt given that we are
% currently at x,xd,xdd, and that we have tau until we want to reach
% the goal

if tau<dt,
	return;
end;

dist = goal - x;

a1   = 0;
a0   = xdd * tau^2;
v1   = 0;
v0   = xd * tau;

t1=dt;
t2=dt^2;
t3=dt^3;
t4=dt^4;
t5=dt^5;

c1 = (6.*dist + (a1 - a0)/2. - 3.*(v0 + v1))/tau^5;
c2 = (-15.*dist + (3.*a0 - 2.*a1)/2. + 8.*v0 + 7.*v1)/tau^4;
c3 = (10.*dist+ (a1 - 3.*a0)/2. - 6.*v0 - 4.*v1)/tau^3;
c4 = xdd/2.;
c5 = xd;
c6 = x;

x   = c1*t5 + c2*t4 + c3*t3 + c4*t2 + c5*t1 + c6;
xd  = 5.*c1*t4 + 4*c2*t3 + 3*c3*t2 + 2*c4*t1 + c5;
xdd = 20.*c1*t3 + 12.*c2*t2 + 6.*c3*t1 + 2.*c4;