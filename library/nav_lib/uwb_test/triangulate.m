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

% anchor position 
AnchorOne = [9.21;1.08;-0.17];%4.08
AnchorTwo = [0;0;-1.885];
AnchorThree = [0;6.281;-1.37];
AnchorFour = [1.705;12.88;-2.27];
AnchorFive = [9.31;11.59;-0.52];
bSCoordinate =   [AnchorOne, AnchorTwo,...
				  AnchorThree, AnchorFour,...
				  AnchorFive]'*30;
constantValue =  [norm(AnchorOne),norm(AnchorTwo),...
				  norm(AnchorThree),norm(AnchorFour),...
				  norm(AnchorFive)]'*30;
bSPcs = 5;
A = bSCoordinate(1:bSPcs-1,:)  - repmat(bSCoordinate(bSPcs,:),bSPcs-1,1);
M = -0.5*inv(A'*A)* A';
c = (constantValue(1:bSPcs-1,:).^2 -constantValue(bSPcs,:).^2);
xyz = M*(rangings(1:bSPcs-1,:).^2 - rangings(bSPcs,:).^2 - c ); 

end