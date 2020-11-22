function [VDOP, HDOP, PDOP, GDOP] = ch_gpsdop(G, lat, lon)
% GPS原理及接收机设计 谢刚 DOP章节
S = [-sin(lon) cos(lon) 0; -sin(lat)*cos(lon) -sin(lat)*sin(lon) cos(lat); cos(lat)*cos(lon) cos(lat)*sin(lon) sin(lat)];
S = [S [0 0 0]'; [0 0 0 1]];
H = (G'*G)^(-1);
H = S*H*S';
VDOP = sqrt(H(3,3));
HDOP = sqrt(H(1,1) + H(2,2));
PDOP = sqrt (H(1,1) + H(2,2) + H(3,3));
GDOP = sqrt(trace(H));

end
