function [error_yaw_command, yaw_ref] = Yaw_Controllor(yaw_set, Dog_yaw, Vector)
    % Yaw Controller
    
    % Normalize yaw angles to [0, 360)
    normalize_yaw = @(angle) mod(angle, 360);

    if yaw_set == -1
        % No yaw control
        error_yaw_command = 0;
        yaw_ref = 0;
    elseif yaw_set == -2
        % Control yaw to motion direction
        angle_r = atan2(Vector(1), Vector(2));
        yaw_set_mode2 = rad2deg(angle_r);
        yaw_set_mode2 = normalize_yaw(yaw_set_mode2);
        
        error_yaw = yaw_set_mode2 - Dog_yaw;
        yaw_ref = yaw_set_mode2;
        
        % Use mod for angle normalization
        error_yaw_command = mod(error_yaw + 180, 360) - 180;
    elseif yaw_set >= 0 && yaw_set < 360
        % Control yaw
        error_yaw = normalize_yaw(yaw_set - Dog_yaw);
        yaw_ref = yaw_set;
        
        % Use mod for angle normalization
        error_yaw_command = mod(error_yaw + 180, 360) - 180;
    else
        error('Invalid yaw_set value');
    end
end