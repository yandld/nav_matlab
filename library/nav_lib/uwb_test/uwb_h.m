% Measurement model function for the random sine signal demo

% Copyright (C) 2007 Jouni Hartikainen
%
% This software is distributed under the GNU General Public 
% Licence (version 2 or later); please refer to the file 
% Licence.txt, included with the software, for details.

function [Y,H] = ekf_uwb_h(x, param)
    global UKF;
    position = x(1:3);	
    bSPcs = UKF.AnchorPcs;
	Zpred = [];
	TM = repmat(position,1,bSPcs) - UKF.AnchorPosition(:,1:bSPcs);
	for ki=1:bSPcs
	    Zpred = [Zpred ;norm(TM(:,ki))];
	end
	   Y = Zpred;
	

