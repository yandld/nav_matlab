function eul_plot(varargin)
%%  plot muiplte eul angles

	param= inputParser; 
	param.addOptional('time', []);
	param.addOptional('q1', []);
    param.addOptional('q2', []);
    param.addOptional('q3', []);
    
    param.parse(varargin{:});
    r = param.Results;

    if(r.time == 0 )
           error('no time data');
    end
    
    figure('Name', 'Eul Angles');
    
    if(~isempty(r.q1))
       hold on;
         eul = quat2eul((r.q1), 'ZYX') * (180/pi);	% use conjugate for sensor frame relative to Earth and convert to degrees.
        h(1) = plot(r.time, eul(:,1), 'r');
        h(2) = plot(r.time, eul(:,2), 'g');
        h(3) = plot(r.time, eul(:,3), 'b');
        legend(h, 'Z1','Y1', 'X1');
    end

    if(~isempty(r.q2))
         hold on;
         eul = quat2eul((r.q2), 'ZYX') * (180/pi);	% use conjugate for sensor frame relative to Earth and convert to degrees.
        h(1) = plot(r.time, eul(:,1), 'r.');
        h(2) =plot(r.time, eul(:,2), 'g.');
        h(3) =plot(r.time, eul(:,3), 'b.');
         ah=axes('position',get(gca,'position'), 'visible','off');
         legend(ah, h, 'Z2','Y2', 'X2');
    end

    
	xlabel('Time (s)');
	ylabel('Angle (deg)');
    
end
        