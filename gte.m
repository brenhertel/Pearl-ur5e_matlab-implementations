function [new_pos_x, new_pos_y] = gte(twoD_traj, fixed_points)


    global traj trajMod tangent fixedPos fixedWeight boundCond;
    
    % -------------- options ----------------------------------------------
    
    % select type of trajectory
    traj = twoD_traj;
    
    % select fixed weight (omega)
    fixedWeight = 1e9;
    
    % impose soft first derivative boundary condition (0: off // 1: on)
    % (not so important)
    boundCond   = 0;
    
    % -------------- start calculation ------------------------------------
    
    
    trajMod  = traj;
    fixedPos = fixed_points;
    
    % generate the two sides of the equation system
    [G,tangent] = generategraphincidence(traj,fixedPos,fixedWeight,boundCond);
    
    calcDeformedTraj();
    new_pos_x = (trajMod(:,1))';
    new_pos_y = (trajMod(:,2))';
end

function calcDeformedTraj()                                                 % calculate deformed trajectory with hard positional constraints
    global traj trajMod tangent fixedPos fixedWeight boundCond

    [G,tangent]    = generategraphincidence(traj,fixedPos,fixedWeight,boundCond);
    trajMod(:,1) = G\tangent(:,1);
    trajMod(:,2) = G\tangent(:,2);
end



function [G,tangent] = generategraphincidence(traj,fixedPos,fixedWeight,boundCond) % generate the Laplacian matrix and the differential coordinates for LTE
    nbNodes = size(traj,1);
    nbDim   = size(traj,2);

    % create L matrix
    G            = diag(ones(nbNodes,1)*-1)   + ...
                   diag(ones(nbNodes-1,1),1);
    disp(G)      

    % create Delta matrix
    tangent = zeros(size(traj));
    for i=1:nbDim                                                           
        tangent(:,i) = G*traj(:,i);
    end
    
    % boundary condition for first/last sampling point
    if boundCond == 0
        G(1,:)       = [];
        G(end,:)     = [];
        tangent(1,:)   = [];
        tangent(end,:) = [];
    end

    % append fixed positions
    for i=1:size(fixedPos,1)                                                                                
        G(end+1,fixedPos(i,1)) = fixedWeight;
        tangent(end+1,:)         = fixedWeight*fixedPos(i,2:end);
    end
    
    G = sparse(G);
end