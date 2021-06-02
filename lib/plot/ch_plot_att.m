function ch_plot_att(att, varargin)
%  plot 欧拉角
% att        欧拉角
% 可选参数: units: "rad" 或者 "deg", 默认 rad


defaultUnits = 'rad';

param= inputParser;
addParameter(param,'units',defaultUnits,@isstring);
   
param.parse(varargin{:});
r = param.Results;


if(~isempty(r.units))
    defaultUnits = r.units;
end


plot(att);
title("欧拉角");
legend("X", "Y", "Z");
xlabel("解算次数"); 
ylabel(defaultUnits); 



end

