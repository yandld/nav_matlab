function ch_plot_pos2d(pos, varargin)
%  pos: 位置： X, Y

defaultUnits = 'm';

param= inputParser;
addParameter(param,'units',defaultUnits,@isstring);

param.parse(varargin{:});
r = param.Results;


if(~isempty(r.units))
    defaultUnits = r.units;
end

plot(pos(:,1), pos(:,2));

title("2D位置" + "(" + defaultUnits + ")") ;
xlabel("X"); ylabel("Y");



end


