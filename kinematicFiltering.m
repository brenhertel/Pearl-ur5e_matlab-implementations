function kinematicFiltering
% This is a demo showing how to call Jerk-Accuracy filtering on noisy
% scatter data in 2-D.
%
% The file wraps around the line: [x, y]=filter_JA(trj_ns,tt',l,[],[]);
%
% Yaron Meirovitch 

%%% DEFS
colors = [.7 .8 .1];
fontSize = 14;
folder = pwd; %#ok<NASGU>
filename='noise'; %#ok<NASGU>
greenReplace = [100 172 48]/255; % replace green 
redReplace = [255 72 165]/255; % replace red 
M=200; % number of samples per 5 via-points

%%% For demo use E. Todorov's algorithm to simulate Minimum Jerk trajectory
%%% (T. Flash, 1985).
f=figure;
axis([-1 1 -1 1]);
axis equal
hold on;
title('Task Space');
pos = ginput;
N = round(size(pos,1)*M/5);
[trj, ~] = min_jerk_todorov(pos, N,  [], [], []);
tt=linspace(0,1,N);
plot(trj(:,1),trj(:,2),'.','color',colors(1,:));
close(f);

% units are arbitrary:
if (0)
    xlabel('x [a.u.]'); %#ok<*UNRCH>
    ylabel('y [a.u.]');
end

%%% Add simulated noise to the Minimum Jerk trajectory
eD=.004;
noise=randn(size(trj,1),2)*eD;
wf=fir1(round(N/8),2/(N/2));
trj_ns=trj+noise;
trj_sm=filtfilt(wf,1,trj_ns);
l=25;


%%% This is the line that computes the optimal filter. 
[x, y]=filter_JA(trj_ns,l,tt');

%%% noise statistics
if (0)
    SNR0=norm(std(trj))/norm(std(noise));
    SNR1=norm(std(diff(trj)))/norm(std(diff(noise)));
    SNR2=norm(std(diff(trj,2)))/norm(std(diff(noise,2)));
    SNR3=norm(std(diff(trj,3)))/norm(std(diff(noise,3)));
    SNR4=norm(std(diff(trj,4)))/norm(std(diff(noise,4)));
    SNR5=norm(std(diff(trj,5)))/norm(std(diff(noise,5)));
end


%%% Display Results
f_noise=figure('color','w','position',[413    90   413   708]);
subplot(621); plot(trj(:,1),trj(:,2),'k'); axis equal; axis tight
ylabel('Position');
subplot(622); plot(trj_ns(:,1),trj_ns(:,2),'.'); hold on
subplot(622); plot(x(:,1),y(:,1),'g'); axis equal; axis tight
subplot(622); plot(trj_sm(:,1),trj_sm(:,2),'r','linewidth',1);

subplot(612); plot(N*sqrt(sum(diff(trj).^2,2)),'k','linewidth',1); hold on; ax1=axis;
subplot(612); plot(N*sqrt(sum(diff(trj_ns).^2,2))); hold on;
plot(N*sqrt(sum(diff(trj_sm).^2,2)),'r','linewidth',1);
subplot(612); plot(sqrt(x(:,2).^2+y(:,2).^2),'g','linewidth',1); hold on;  axis(ax1)
ylabel('Velocity','fontsize',fontSize);
set(gca,'xtick',[])

subplot(613); plot(N^2*sqrt(sum(diff(trj,2).^2,2)),'k','linewidth',1); hold on; ax2=axis;
subplot(613); plot(N^2*sqrt(sum(diff(trj_ns,2).^2,2))); hold on;
plot(N^2*sqrt(sum(diff(trj_sm,2).^2,2)),'r','linewidth',1);
subplot(613); plot(sqrt(x(:,3).^2+y(:,3).^2),'g','linewidth',1); hold on;  axis(ax2)
ylabel('Acceleration','fontsize',fontSize);
set(gca,'xtick',[])

subplot(614); plot(N^3*sqrt(sum(diff(trj,3).^2,2)),'k','linewidth',1); hold on; ax3=axis;
subplot(614); plot(N^3*sqrt(sum(diff(trj_ns,3).^2,2))); hold on;
plot(N^3*sqrt(sum(diff(trj_sm,3).^2,2)),'r','linewidth',1);
subplot(614); plot(sqrt(x(:,4).^2+y(:,4).^2),'g','linewidth',1); hold on;  axis(ax3)
ylabel('Jerk','fontsize',fontSize);
set(gca,'xtick',[])

subplot(615); plot(N^4*sqrt(sum(diff(trj,4).^2,2)),'k','linewidth',1); hold on; ax3=axis;
subplot(615); plot(N^4*sqrt(sum(diff(trj_ns,4).^2,2))); hold on;
plot(N^4*sqrt(sum(diff(trj_sm,4).^2,2)),'r','linewidth',1);
subplot(615); plot(sqrt(x(:,5).^2+y(:,5).^2),'g','linewidth',1); hold on;  axis(ax3)
ylabel('Snap','fontsize',fontSize);
set(gca,'xtick',[])
ylim([0 .25E5])


subplot(616); plot(N^5*sqrt(sum(diff(trj,5).^2,2)),'k','linewidth',1); hold on; ax3=axis;
subplot(616); plot(N^5*sqrt(sum(diff(trj_ns,5).^2,2))); hold on;
plot(N^5*sqrt(sum(diff(trj_sm,5).^2,2)),'r','linewidth',1);
subplot(616); plot(sqrt(x(:,6).^2+y(:,6).^2),'g','linewidth',1); hold on;  axis(ax3)
ylabel('Crackle','fontsize',fontSize);

set(findobj(gcf,'type','axes'),'box','off','tickdir','out','fontsize',12,...
    'ytick',[])

% units are arbitrary:
if (0)
    text(0,0,'a.u','rotation',90,'fontsize',32)
    text(0,0,'Time (a.u)','fontsize',20)
end

greenGraphics=findobj(f_noise,'color','g');
redGraphics=findobj(f_noise,'color','r');
blackGraphics=findobj(f_noise,'color','k','type','line');
set(greenGraphics,'color', greenReplace,'linewidth',3);
set(redGraphics,'color', redReplace,'linewidth',2);
set(blackGraphics,'color','k','linewidth',2);

if (0)
    print('-depsc2',  [folder filesep filename '.eps']);
    
    % other savings coud be... 
    
    %saveas(gcf,[folder filesep filename],'fig')
    %export_fig(fullfile(folder,'noise.eps'),'-eps',f_noise)
    %save noise 
end

disp('Press any key to continue with the demo.');
pause; 

% Simple Example: 

% prepare a template to be smoothly regenerated 
A = [0,0];
B = [1/2,1];
C = [1,0];


tt_lag = 0:1/200:1;
rr1 =  [1-tt_lag(1:end-1); tt_lag(1:end-1) ]' * [A;B];
rr2 =  [1-tt_lag; tt_lag ]' * [B;C];
rr = [rr1;rr2];

% define smoothness-accuracy paramtere 
lambda = 8;

% regenerate motion with new boundary condition 
[x, y]=filter_JA(rr,lambda,[],[A + [.2 0], C+[0 +.1]],[]);

% plot result 
figure; 
plot(rr(:,1),rr(:,2),'linewidth',2); hold all; 
plot(x(:,1),y(:,1),'linewidth',2)

pause;
%%% Example, adpatively changing boundary conditions and accuracy parameter
%%% lambda
demoBoundaryCondition;

% 3D example 
trj3 = [trj trj(:,1).*trj(:,2)-trj(:,1).^2];
[x, y, z]=filter_JA(trj3,lambda);
figure; plot3(trj3(:,1),trj3(:,2),trj3(:,3)); hold all;
plot3(x(:,1),y(:,1),z(:,1)); axis equal;
title('3D Example');


