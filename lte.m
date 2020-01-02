function [new_pos_x, new_pos_y] = lte(twoD_traj, fixed_points)
%
% Laplacian Trajectory Editing Demo (LTE) with hard constraints
% 
% Author: Thomas Nierhoff (2013)
% 
% The program deforms a predefined trajectory using LTE. 
% By clicking on the Matlab plot and dragging the cursor over the drawing
% area, additional fixed positional constraints can be imposed.
% The user has the option to specify the type of trajectory, the
% weighting factor of the fixed sampling points (omega) and 
% first derivative boundary conditions for the first and last 
% sampling points (not in any publication, just for fun).
% 
% This source code is given for free. Still it would be nice if you 
% could refer to any of my related publications, for example
% 
% @inproceedings{Nierhoff12,
%   author    = {Thomas Nierhoff and Sandra Hirche},
%   title     = {Fast trajectory replanning using Laplacian mesh optimization},
%   booktitle = {ICARCV},
%   year      = {2012},
%   pages     = {154-159},
% }
%

    global traj trajMod delta fixedPos fixedWeight boundCond;
    
    % -------------- options ----------------------------------------------
    
    % select type of trajectory
    traj = twoD_traj;
    
    % select fixed weight (omega)
    fixedWeight = 1e9;
    %fixedWeight = 0.1;
    
    % impose soft first derivative boundary condition (0: off // 1: on)
    % (not so important)
    boundCond   = 0;
    
    % -------------- start calculation ------------------------------------
    
    close all
    %fh = figure;
    
    trajMod  = traj;
    %fixedPos = [1            traj(1,:);
    %           size(traj,1) traj(end,:)];
    fixedPos = fixed_points;
    
    % generate the two sides of the equation system
    [L,delta] = generateLaplacian(traj,fixedPos,fixedWeight,boundCond);
       
    % plot trajectory and fixed sampling points
    %new_pos = traj';
    
    %plotTraj();
    calcDeformedTraj();
    new_pos_x = (trajMod(:,1))';
    new_pos_y = (trajMod(:,2))';
    %axis([-1 11 -6 6])
    %axis equal
    
    % feedback function
    %set(fh, 'windowbuttondownfcn', {@buttonDown, gca}, ...
    %        'windowbuttonupfcn',   {@buttonUp});
end



function buttonDown(obj, edata, axh)                                        % callback function for pressed down button
    set(obj, 'windowbuttonmotionfcn', {@buttonMotion, axh});    
    
    global trajMod fixedPos    

    % set new fixed position
    pt                = get(axh, 'currentpoint');                           
    [index,pos]       = findClosestPoint(trajMod,pt(1,1:2));
    fixedPos(end+1,:) = [index pos];
end



function buttonMotion(obj, edata, axh)                                      % callback function for moving button
    global fixedPos 
    
    % update fixed position   
    pt                = get(axh, 'currentpoint');
    fixedPos(end,2:3) = pt(1,1:2);
        
    calcDeformedTraj();  
    plotTraj();
end



function buttonUp(obj,edata)                                               % callback function for released button
    set(obj, 'windowbuttonmotionfcn', '');
end



function calcDeformedTraj()                                                 % calculate deformed trajectory with hard positional constraints
    global traj trajMod delta fixedPos fixedWeight boundCond

    [L,delta]    = generateLaplacian(traj,fixedPos,fixedWeight,boundCond);
    trajMod(:,1) = L\delta(:,1);
    trajMod(:,2) = L\delta(:,2);
end



function [L,delta] = generateLaplacian(traj,fixedPos,fixedWeight,boundCond) % generate the Laplacian matrix and the differential coordinates for LTE
    nbNodes = size(traj,1);
    nbDim   = size(traj,2);

    % create L matrix
    L            = diag(ones(nbNodes,1)*2)   - ...
                   diag(ones(nbNodes-1,1),1) - ... 
                   diag(ones(nbNodes-1,1),-1);
             
    L(1,2)       = -2; 
    L(end,end-1) = -2;
    L            = L/2;         

    % create Delta matrix
    delta = zeros(size(traj));
    for i=1:nbDim                                                           
        delta(:,i) = L*traj(:,i);
    end
    
    % boundary condition for first/last sampling point
    if boundCond == 0
        L(1,:)       = [];
        L(end,:)     = [];
        delta(1,:)   = [];
        delta(end,:) = [];
    end

    % append fixed positions
    for i=1:size(fixedPos,1)                                                                                
        L(end+1,fixedPos(i,1)) = fixedWeight;
        delta(end+1,:)         = fixedWeight*fixedPos(i,2:end);
    end
    
    L = sparse(L);
end



function plotTraj()                                                         % plot the trajectory and fixed sampling points
    global trajMod delta fixedPos

    v = axis;
    
    % plot trajectory
    plot(trajMod(:,1),trajMod(:,2),'LineWidth',2)                           
    hold on
    
    % plot fixed positions    
    for i=1:size(fixedPos,1)
        plot(trajMod(fixedPos(i,1),1),trajMod(fixedPos(i,1),2),'o',...
             'MarkerEdgeColor','k',            ...  
             'MarkerFaceColor',[1 0 0],        ...
             'MarkerSize',10)
    end
    hold off
        
    axis(v)
    set(gca,'XTickLabel',[])
    set(gca,'YTickLabel',[]) 
end



function [index,pos] = findClosestPoint(traj,pt)                            % find closest point in traj with respect to pt
    ptList      = repmat(pt,size(traj,1),1);
    dist        = sum((traj-ptList).^2,2);
    [val,index] = min(dist);
    pos         = traj(index,:);
end

function traj = calcSine()                                                  % generate sine wave
    x    = 0:0.04:10;
    y    = sin(5*x);
    traj = [x' y'];
end



function traj = calcSpiral()                                                % generate spiral
    t = linspace(0,20,250);
    x = cos(t).*(5-t*0.2);
    y = sin(t).*(5-t*0.2);
    traj = [x' y'];
end



function traj = calcLine()                                                  % generate straight line
    x    = 0:0.04:10;
    y    = zeros(size(x));
    traj = [x' y'];
end
