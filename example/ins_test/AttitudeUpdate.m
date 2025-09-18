classdef AttitudeUpdate
    methods (Static)
        % 四元数乘法
        function q = quaternionMultiply(q1, q2)
            w1 = q1(1); x1 = q1(2); y1 = q1(3); z1 = q1(4);
            w2 = q2(1); x2 = q2(2); y2 = q2(3); z2 = q2(4);
            
            q = [w1*w2 - x1*x2 - y1*y2 - z1*z2;
                 w1*x2 + x1*w2 + y1*z2 - z1*y2;
                 w1*y2 - x1*z2 + y1*w2 + z1*x2;
                 w1*z2 + x1*y2 - y1*x2 + z1*w2];
        end
        
        % 向量转四元数
        function q = vectorToQuaternion(v)
            angle = norm(v);
            if angle < 1e-10
                q = [1; 0; 0; 0];
            else
                axis = v / angle;
                q = [cos(angle/2);
                     axis(1)*sin(angle/2);
                     axis(2)*sin(angle/2);
                     axis(3)*sin(angle/2)];
            end
        end
        
        % 单子样更新
        function q_next = singleSample(q_curr, omega, dt)
            theta = omega * dt;
            dq = AttitudeUpdate.vectorToQuaternion(theta);
            q_next = AttitudeUpdate.quaternionMultiply(q_curr, dq);
            q_next = q_next / norm(q_next);
        end
        
        % 双子样更新
        function q_next = twoSample(q_curr, omega1, omega2, dt)
            theta = dt * (omega1 + omega2) + ...
                   dt^2*(2/3) * cross(omega1, omega2);
            dq = AttitudeUpdate.vectorToQuaternion(theta);
            q_next = AttitudeUpdate.quaternionMultiply(q_curr, dq);
            q_next = q_next / norm(q_next);
        end
        % 四子样更新
        function q_next = fourSample(q_curr, omega1, omega2, omega3 , omega4, dt)
            theta = dt * (omega1 + omega2 + omega3 + omega4) + ...
                   dt^2*(214/105) * cross(omega1, omega4) + ...
                    dt^2*(92/105) * cross(omega2, omega4) + ...
                    dt^2*(54/105) * cross(omega3, omega4);
            dq = AttitudeUpdate.vectorToQuaternion(theta);
            q_next = AttitudeUpdate.quaternionMultiply(q_curr, dq);
            q_next = q_next / norm(q_next);
        end
        
        % 单子样加前一周期补偿(PARC)更新
        function q_next = singleSamplePARC(q_curr, omega_curr, omega_prev, dt)
            % 计算叉积补偿项
            cross_term = cross(omega_prev,omega_curr);
            
            % 计算修正后的等效旋转矢量
            theta = omega_curr * dt + dt^2/12 * cross_term;
            
            % 计算增量四元数
            dq = AttitudeUpdate.vectorToQuaternion(theta);
            
            % 更新四元数
            q_next = AttitudeUpdate.quaternionMultiply(q_curr, dq);
            
            % 归一化
            q_next = q_next / norm(q_next);
        end
    end
end