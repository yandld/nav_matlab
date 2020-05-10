% Measurement model function for the random sine signal demo

% Copyright (C) 2007 Jouni Hartikainen
%
% This software is distributed under the GNU General Public 
% Licence (version 2 or later); please refer to the file 
% Licence.txt, included with the software, for details.

function [Y,H] = ekf_uwb_h(x, param)
    global dataset;
    
    position = x(1:3);	
    n = dataset.uwb.cnt;
	Zpred = [];
	TM = repmat(position,1,n) - dataset.uwb.anchor(:,1:n);
	for ki=1:n
	    Zpred = [Zpred ;norm(TM(:,ki))];
	end
	   Y = Zpred;
	

