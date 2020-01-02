function demoBoundaryCondition
%%% This example shows how the effect of lambda and the boundary conditions
%%% on the filter

 

% DEFS
lambda = 30; % lower this lambda to increase smoothness  
Left = 28;
Right = 29;
ax = [ -0.5666    1.3064   -0.8331    0.6441];
[Tx,Ty] =deal(-0.4,0.6);

%%%%%%%%% Example of simulating a drawing-like trajectory from points. 
% N=200;
% figure;
% tt=linspace(0,1,N);
% axis([-1 1 -1 1]);
% axis equal
% hold on;
% title('Task Space');
% pos = ginput;
% [trj, ~] = min_jerk_todorov(pos, N,  [], [], []); % Solve via-point
% minimum jerk (Fash and Hogan, 1985), numerically (E. Todorov, 1998)
% plot(trj(:,1),trj(:,2),'.','color',colors(1,:));
% save A-example trj tt
%%%%%%%%%%%%%


%%%%%%% Play with different boundary conitions 
figure('color','w'); 
template =getfield(load('A-example'),'trj');
tt =getfield(load('A-example'),'tt');

px = template([1 end],1);  py = template([1 end],2); 
title('Click on desired endpoints. Quit with Enter.','color','r');
button = -1;
while ~isempty(button)  
    text(Tx,Ty,['lambda = ' num2str(lambda)],'fontsize',14,'fontweight','bold'); hold all
    plot(template(:,1),template(:,2),'linewidth',2);  axis equal; 
    plot(px,py,'db'); drawnow; 
    lastBoundary = [px(1) py(1) px(2) py(2)];
    [x, y]=filter_JA(template,lambda,tt',lastBoundary);
    plot(x(:,1),y(:,1),'linewidth',2); 
    [px,py,button] = ginput(2);
    cla
    axis(ax)
end

%%%%%%% Play with different Lambda values
lambda = 10;
title('Change Lambda: Use only the left and right arrow. Quit with Enter','color','r')
button = -1;
while ~isempty(button)  
    
    axis(ax)
   
    lambda = lambda - 5*sum(button == Left);
    lambda = lambda + 5*sum(button == Right);
    
    [x, y]=filter_JA(template,lambda,tt',lastBoundary);
    cla
    plot(lastBoundary([1 3]),lastBoundary([2 4]),'db'); drawnow; hold all
    text(Tx,Ty,['lambda = ' num2str(lambda)],'fontsize',14,'fontweight','bold'); hold all
  
    plot(template(:,1),template(:,2),'linewidth',2);  axis equal; hold all
    plot(x(:,1),y(:,1),'linewidth',2); 
   
    [~,~,button] = ginput(1);
   
end

    
    
