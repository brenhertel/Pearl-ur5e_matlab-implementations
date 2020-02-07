function [x, y, solx, soly]=filter_JA(trj,tt,l,endpoints,vel,acc,direction)
% filter_JA(...) applies optimal Jerk (third derivative of position) filter
% to the input trajectory trj. 
% 'trj' is nx2 array representing trajectory to be smoothed.   
% 'tt' is nx1 array of time points corresponding to 'trj'. If tt=[] then
% filter_JA assumes uniform sampling and uses tt=linspace(
% data)
% 'l' is the accuracy demand (lambda) 
% 'endpoints' (optional) is a vector with the two endpoints [xstart,ystart, xend,yend]
% 'vel' (optional) is a vector with the endpoitn velocities
% [vxstart,xystart, vxend,vyend] 
% 'acc' is  a vector with the endpoitn accelerations [axstart,aystart, axend,ayend] 
% 'direction' ('optional') determines if the system is attracted to 'trj'
% (directino=1, default) or repelled by 'trj'.  
%
% Copyright (C) Yaron Meirovitch, 2012-2014

%%%

if ~exist('tt','var') || isempty(tt)
    tt = linspace(0,1,size(trj,1));
end

F1 = griddedInterpolant(tt(~isnan(trj(:,1))),trj(~isnan(trj(:,1)),1),'cubic');
F2 = griddedInterpolant(tt(~isnan(trj(:,2))),trj(~isnan(trj(:,2)),2),'cubic');


%%%
if exist('endpoints','var') && ~isempty(endpoints)
    c=num2cell(endpoints); [rx0,ry0,rx1,ry1]=deal(c{:});
else
    rx0=F1(tt(1)); rx1=F1(tt(end));
    ry0=F2(tt(1)); ry1=F2(tt(end));
end
if exist('vel','var')  && ~isempty(vel)
    c=num2cell(vel); [vx0,vy0,vx1,vy1]=deal(c{:});
else
   [vx0,vy0,vx1,vy1]=deal(0,0,0,0);
end

if exist('acc','var')  && ~isempty(acc)
    c=num2cell(acc); [ax0,ay0,ax1,ay1]=deal(c{:});
else
   [ax0,ay0,ax1,ay1]=deal(0,0,0,0);
end

if ~exist('direction','var')
    direction=1;
end

%[ ax0, ay0, ax1,ay1]=deal(0,0,0,0); % old version did not accept
%acceleration vecotor

guessx=bvpinit(tt,[rx0 0 0 0 0 0]); % erase the last two zeros for minimum acceleration
guessy=bvpinit(tt,[ry0 0 0 0 0 0]);
%%%%
bcx=@(y0,y1) bc(y0,y1,rx0, rx1, vx0, vx1,  ax0, ax1); % here and in the corresponding places inside the function you can control if it is minimum jerk
% or minimum snap. E.g for minimum acceleration don't pass acceleration
% values
bcy=@(y0,y1) bc(y0,y1,ry0, ry1, vy0, vy1,  ay0, ay1);

% snap
% [vx0, vx1,  ax0, ax1, jx0, jx1]=deal(0,0,0,0,0,0);
% [vy0, vy1,  ay0, ay1, jy0, jy1]=deal(0,0,0,0,0,0);
% guessx=bvpinit(tt,[rx0 0 0 0 0 0 0 0]);
% guessy=bvpinit(tt,[ry0 0 0 0 0 0 0 0]);
% %%%%
% bcx=@(y0,y1) jerk_bc(y0,y1,rx0, rx1, vx0, vx1,  ax0, ax1, jx0, jx1)
% bcy=@(y0,y1) jerk_bc(y0,y1,ry0, ry1, vy0, vy1,  ay0, ay1, jy0, jy1)

bvpx=@(t,y)  MSDAccuracy_DE(t,y,l,F1,direction);
bvpy=@(t,y)  MSDAccuracy_DE(t,y,l,F2,direction);

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
y0=ry0-F2(t1); yf=ry1-F2(t2); 
b0=y0 + (t2*(5*t1^4*y0 - 5*t1^4*yf) - t1^5*y0 + t1^5*yf - t2^2*(10*t1^3*y0 - 10*t1^3*yf))/(t1 - t2)^5;
b1=(30*t1^2*t2^2*(y0 - yf))/(t1 - t2)^5;
b2=-(30*t1*t2*(t1 + t2)*(y0 - yf))/(t1 - t2)^5;
b3=(10*(y0 - yf)*(t1^2 + 4*t1*t2 + t2^2))/(t1 - t2)^5;
b4=-(15*(t1 + t2)*(y0 - yf))/(t1 - t2)^5;
b5=(6*(y0 - yf))/(t1 - t2)^5;
guess_funx=@(t) [F1(t)+a5*t.^5 + a4*t.^4 + a3*t.^3 + a2*t.^2 + a1*t + a0;...
                 5*a5*t^4 + 4*a4*t^3 + 3*a3*t^2 + 2*a2*t + a1;...
                 20*a5*t^3 + 12*a4*t^2 + 6*a3*t + 2*a2;...
                 60*a5*t^2 + 24*a4*t + 6*a3;...
                 24*a4 + 120*a5*t;...
                 120*a5];
guess_funy=@(t) [F2(t)+b5*t.^5 + b4*t.^4 + b3*t.^3 + b2*t.^2 + b1*t + b0;...
                 5*b5*t^4 + 4*b4*t^3 + 3*b3*t^2 + 2*b2*t + b1;...
                 20*b5*t^3 + 12*b4*t^2 + 6*b3*t + 2*b2;...
                 60*b5*t^2 + 24*b4*t + 6*b3;...
                 24*b4 + 120*b5*t;...
                 120*b5];            
guessx2=bvpinit(tt,guess_funx);
guessy2=bvpinit(tt,guess_funy);

options = bvpset('stats','off','RelTol',2,'AbsTol',100);

solx=bvp4c(bvpx,bcx,guessx2,options);
soly=bvp4c(bvpy,bcy,guessy2,options);
solx=bvp4c(bvpx,bcx,solx);
soly=bvp4c(bvpy,bcy,soly);


x=deval(solx,tt)'; y=deval(soly,tt)';




