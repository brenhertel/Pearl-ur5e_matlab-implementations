function dydt=MSDAccuracy_DE(t,y,lambda,funct,direction)
% MSDAccuracy_DE(...) repesents the differential equation for maximally smooth
% movements (jerk minimizing) and can be used with Matlab's bvc4c() to
% solve for the optimal genertion of a trajectory. 
%
%
%
% funct can by a function handle (accepting double, here t) 
% direction = +-1 % regular is +1
%
% y1'=y2
% y2'=y3
% y3'=y4
% y4'=y5
% y5'=y6
% y6'=2*(y1-t^2)
% 
%

%   old versions of Matlab used to run this:
%   dydt=jerk_acc(t,y,lambda,funct,ts,tt) % old matlab
%   dydt=[y(2),y(3), y(4), y(5), y(6), lambda^6*(y(1)-funct(t,ts,tt))]; % old Matalb

% Copyright (C) Yaron Meirovitch, 2012-2014

p=length(y);
target=funct(t); % if target is nan (disappears) move according to based functional, i.e. no accuracy is required
dydt=[y(2:end)', direction*lambda^p*(y(1)-target)]; % Matlab 2013

