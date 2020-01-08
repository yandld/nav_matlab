
        function qo = ch_qintg(qi, gyr, dt)
            % v = norm(gyr);
             
%              q_temp(1) = cos(v * dt / 2);
%              q_temp(2) = sin(v * dt / 2) * gyr(1) / v;
%              q_temp(3) = sin(v * dt / 2) * gyr(2) / v;
%              q_temp(4) = sin(v * dt / 2) * gyr(3) / v;

             qsmall(1) = 1;
             qsmall(2) = gyr(1)*dt*0.5;
             qsmall(3) = gyr(2)*dt*0.5;
             qsmall(4) = gyr(3)*dt*0.5;
             
             qo = ch_qmul(qi, qsmall);
             qo = ch_qnormlz(qo);
        end

