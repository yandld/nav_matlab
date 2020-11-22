% Measurement model function for the random sine signal demo

% Copyright (C) 2007 Jouni Hartikainen
%
% This software is distributed under the GNU General Public 
% Licence (version 2 or later); please refer to the file 
% Licence.txt, included with the software, for details.

function [Y,H] = ekf_err_uwb_h(x, uwb)

    position = x(1:3);	
    n = uwb.cnt;
    
	H = [];
	TM = repmat(position,1,n) - uwb.anchor(:,1:n);
	for ki=1:n
	       H = [H ;TM(:,ki)'/norm(TM(:,ki)),zeros(1,12)];
	end
	Y = [];
    
    
    