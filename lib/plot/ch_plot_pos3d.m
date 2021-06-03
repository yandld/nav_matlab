function ch_plot_pos3d(pos, varargin)
% pos: 位置： X, Y, Z

defaultUnits = 'm';

param= inputParser;
addParameter(param,'units',defaultUnits,@isstring);

param.parse(varargin{:});
r = param.Results;


if(~isempty(r.units))
    defaultUnits = r.units;
end

plot(pos(:,1), pos(:,2));
plot3(pos(:,1), pos(:,2), pos(:,3));

title("3D位置" + "(" + defaultUnits + ")") ;
 xlabel("X"); ylabel("Y"); zlabel("Z");

end


