function [desired_state] = hexagon(t, qn)
% HEXAGON trajectory generator for a hexagon
% =================== Your code goes here ===================
time_tol = 12;
dt = 0.0001;

    function [pos, vel] = get_pos_vel(t)
        if t >= time_tol
            pos = [1;0;0];
            vel = [0;0;0];
        else
            % Define vertices of the hexagon (adjust radius & center)
            vertices = [
                cosd(0); sind(0)
                cosd(60); sind(60)
                cosd(120); sind(120)
                cosd(180); sind(180)
                cosd(240); sind(240)
                cosd(300); sind(300)
            ] * 0.5 + [0.5;0.5;0];
            
            % Divide path into 6 segments along hexagon sides
            segment_time = time_tol / 6;
            
            % Loop through segments and calculate position/velocity
            for i = 1:6
                if t >= (i-1)*segment_time & t < i*segment_time
                    % Calculate position & velocity for current segment
                    [pos, vel, ~] = tj_from_line(vertices(i,:), vertices(mod(i,6)+1,:), segment_time, t-(i-1)*segment_time);
                    break;
                end
            end
        end
    end

    if t >= time_tol
        pos = [1;0;0];
        vel = [0;0;0];
        acc = [0;0;0];
    else
        [pos, vel] = get_pos_vel(t);
        [~, vel1] = get_pos_vel(t+dt);
        acc = (vel1-vel)/dt;
    end
    
yaw = 0;
yawdot = 0;
% =================== Your code ends here ===================
desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;
end
