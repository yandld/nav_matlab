%%  given 5 Anchor Position and it's Ranging d with the target
%   calculate the target xyz
%   * = Anchor
%   o = Target
%
%     *     *      *
%           o
%     *            *    
%  @ in : all ranging (m)
%  @ out: 3d position triangulated
%%
function xyz = triangulate(rangings)
global dataset;

bSCoordinate = dataset.uwb.anchor;
constantValue = sum(abs(bSCoordinate).^2,2).^(1/2);

bSPcs = 5;
A = bSCoordinate(1:bSPcs-1,:)  - repmat(bSCoordinate(bSPcs,:),bSPcs-1,1);

M = -0.5*inv(A'*A)* A';
c = (constantValue(1:bSPcs-1,:).^2 -constantValue(bSPcs,:).^2);
xyz = M*(rangings(1:bSPcs-1,:).^2 - rangings(bSPcs,:).^2 - c ); 

end