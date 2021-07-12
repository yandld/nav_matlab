function delta_xyzb = comp_pos(curr_obs, rec_xyz)

% Broadcast Orbit
SPos    = curr_obs.data(:,curr_obs.col.XS:curr_obs.col.ZS);
numOfSatellites = size(SPos, 1);
A       = zeros(numOfSatellites, 4);

% Pseudorange of Each Satellite
obs = curr_obs.data(:,curr_obs.col.CorrP);

for i = 1:numOfSatellites
     b(i) = (obs(i,1) - norm(SPos(i,:) - rec_xyz, 'fro')); 
     A(i, :) =  [(-(SPos(i,1) - rec_xyz(1))) / obs(i) ,(-(SPos(i,2) - rec_xyz(2))) / obs(i) ,(-(SPos(i,3) - rec_xyz(3))) / obs(i) , 1 ];
end;

% These lines allow the code to exit gracefully in case of any errors
if rank(A) ~= 4
    delta_xyzb = zeros(4, 1);
    return
end

%--- Find position update ---------------------------------------------
delta_xyzb   = A \ b';

