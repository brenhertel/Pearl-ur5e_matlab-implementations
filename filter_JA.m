function varargout=filter_JA(trj,lambda,tt,endpoints,vel,acc,direction,method)
% filter_JA(...) applies optimal Jerk (third derivative of position) filter
% to the input trajectory trj.
% 'trj' is nxd array representing trajectory to be smoothed.
% 'tt' is nx1 array of time points corresponding to 'trj'. If tt=[] then
% filter_JA assumes uniform sampling and uses tt=linspace(
% data)
% 'l' is the accuracy demand (lambda)
% 'endpoints' (optional) is a 2xd vector with the two endpoints [xstart,ystart, ...; xend,yend, ...]
% 'vel' (optional) is a 2xd vector with the endpoint velocities
% [vxstart,xystart; vxend,vyend]
% 'acc' (optional) is  a vector with the endpoint accelerations [axstart,aystart,...; axend,ayend, ...]
% 'direction' ('optional') determines if the system is attracted to 'trj'
% (direction=1, default) or repelled by 'trj'.
% method (optional) is a string: 'slow' for a seond recomputation of optimal
% trajectory using a first solution as initial guess. 'fast' (default) for
% one solution based on initial guess only.
% varargout = [x, y,...  solx, soly,...]
%
% Copyright (C) Yaron Meirovitch, 2012-2014

%%%%%% DEFNS & INIT
if ~exist('lambda','var') || isempty(lambda)
    lambda = ceil(numel(trj)/20);
end

d = size(trj,2);
l = lambda * (size(trj,1) / 250) * (d^2/4);

if ~exist('endpoints','var')
    endpoints = [];
end

if ~exist('method','var')
    method = 'fast';
end

if size(endpoints,1) == 1
    endpoints = reshape(endpoints,d,2)';
end

varargout = cell(1,2*d);

if ~exist('direction','var')  || isempty(direction)
    direction=1;
end

if ~exist('tt','var') || isempty(tt)
    tt = linspace(0,1,size(trj,1));
end

%%%%%%%%%%%%%%%%%%%

for di=1:d
    
    F1 = griddedInterpolant(tt(~isnan(trj(:,di))),trj(~isnan(trj(:,di)),di),'cubic');
    
    %%%
    if isempty(endpoints)
        rx0=F1(tt(1)); rx1=F1(tt(end));
    else
        rx0=endpoints(1,di); rx1=endpoints(2,di);
    end
    
    if ~exist('vel','var')  || isempty(vel)
        [vx0,vx1]=deal(0,0);
    else
        vx0=vel(1,di);  vx1=vel(2,di);
    end
    
    if ~exist('acc','var')  || isempty(acc)
        [ax0,ax1]=deal(0,0);
    else
        ax0=acc(1,di);  ax1=acc(2,di);
    end
    
    
    
    %[ ax0, ay0, ax1,ay1]=deal(0,0,0,0); % old version did not accept
    %acceleration vecotor
    
    %guessx=bvpinit(tt,[rx0 0 0 0 0 0]); % erase the last two zeros for minimum acceleration
    
    %%%%
    bcx=@(y0,y1) bc(y0,y1,rx0, rx1, vx0, vx1,  ax0, ax1); % here and in the corresponding places inside the function you can control if it is minimum jerk
    % or minimum snap. E.g for minimum acceleration don't pass acceleration
    % values
    
    % snap
    % [vx0, vx1,  ax0, ax1, jx0, jx1]=deal(0,0,0,0,0,0);
    % [vy0, vy1,  ay0, ay1, jy0, jy1]=deal(0,0,0,0,0,0);
    % guessx=bvpinit(tt,[rx0 0 0 0 0 0 0 0]);
    % guessy=bvpinit(tt,[ry0 0 0 0 0 0 0 0]);
    % %%%%
    % bcx=@(y0,y1) jerk_bc(y0,y1,rx0, rx1, vx0, vx1,  ax0, ax1, jx0, jx1)
    % bcy=@(y0,y1) jerk_bc(y0,y1,ry0, ry1, vy0, vy1,  ay0, ay1, jy0, jy1)
    
    bvpx=@(t,y)  MSDAccuracy_DE(t,y,l,F1,direction);
    
    %syms t x0 xf t1 t2 real
    %syms a0 a1 a2 a3 a4 a5 real
    %opt=t.^(0:5)*[a0 a1 a2 a3 a4 a5]';
    %co=solve(subs(opt,t,t1)-x0,subs(diff(opt,1),t,t1),subs(diff(opt,2),t,t1),...
    %    subs(opt,t,t2)-xf,subs(diff(opt,1),t,t2),subs(diff(opt,2),t,t2),a0, a1, a2, a3, a4, a5);
    t1=tt(1); t2=tt(end);
    x0=rx0-F1(t1); xf=rx1-F1(t2);
    a0=x0 + (t2*(5*t1^4*x0 - 5*t1^4*xf) - t1^5*x0 + t1^5*xf - t2^2*(10*t1^3*x0 - 10*t1^3*xf))/(t1 - t2)^5;
    a1=(30*t1^2*t2^2*(x0 - xf))/(t1 - t2)^5;
    a2=-(30*t1*t2*(t1 + t2)*(x0 - xf))/(t1 - t2)^5;
    a3=(10*(x0 - xf)*(t1^2 + 4*t1*t2 + t2^2))/(t1 - t2)^5;
    a4=-(15*(t1 + t2)*(x0 - xf))/(t1 - t2)^5;
    a5=(6*(x0 - xf))/(t1 - t2)^5;
    
    guess_funx=@(t) [F1(t)+a5*t.^5 + a4*t.^4 + a3*t.^3 + a2*t.^2 + a1*t + a0;...
        5*a5*t^4 + 4*a4*t^3 + 3*a3*t^2 + 2*a2*t + a1;...
        20*a5*t^3 + 12*a4*t^2 + 6*a3*t + 2*a2;...
        60*a5*t^2 + 24*a4*t + 6*a3;...
        24*a4 + 120*a5*t;...
        120*a5];
    
    guessx2=bvpinit(tt,guess_funx);
    options = bvpset('stats','off','RelTol',2,'AbsTol',100);
    solx=bvp4c(bvpx,bcx,guessx2,options);
    if strcmp(method,'slow')
        solx=bvp4c(bvpx,bcx,solx);
    end
    varargout{di} = deval(solx,tt)';
    varargout{di + d} = solx;
end




