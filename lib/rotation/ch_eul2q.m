function [Qb2n_312, Qb2n_321] = ch_eul2q(eul)
    
    [Cb2n_312, Cb2n_321] = ch_eul2m(eul);
     Qb2n_312 = ch_m2q(Cb2n_312);
     Qb2n_321 = ch_m2q(Cb2n_321);
%         r = eul(1);
%         p = eul(2);
%         y = eul(3);
%          cp = cos(p / 2);
%          sp = sin(p / 2);
%          cy = cos(y / 2);
%          sy = sin(y / 2);
%          cr = cos(r / 2);
%          sr = sin(r / 2);
%         
%         q(1) =  cr*cp*cy + sr*sp*sy;
%         q(2) = -cr*sp*sy + cp*cy*sr;
%         q(3) =  cr*cy*sp + sr*cp*sy;
%         q(4) =  cr*cp*sy - sr*cy*sp;
%         q = q';
% Ends