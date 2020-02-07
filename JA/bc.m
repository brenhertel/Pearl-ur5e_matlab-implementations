function res=bc(varargin)
% bc() defines the boundary conditions of maximally smooth movement
% generation (jerk minimizing). It can be used with bcvc4c() and
% jerk_acc().
%
% BC: evaluates the residue of the boundary condition 

if length(varargin)==6 % min acceleration
    [y0,y1,r0, r1, v0, v1]=deal(varargin{:});
    res=[y0(1)-r0 y0(2)-v0 y1(1)-r1 y1(2)-v1];
elseif length(varargin)==8 % min jerk
    [y0,y1,r0, r1, v0, v1, a0, a1]=deal(varargin{:});
    res=[y0(1)-r0 y0(2)-v0 y0(3)-a0 y1(1)-r1 y1(2)-v1 y1(3)-a1];
elseif  length(varargin)==10 % min sanp
    [y0,y1,r0, r1, v0, v1, a0, a1 , j0, j1]=deal(varargin{:}); 
    res=[y0(1)-r0 y0(2)-v0 y0(3)-a0 y0(4)-j0 y1(1)-r1 y1(2)-v1 y1(3)-a1,y1(4)-j1];
end
