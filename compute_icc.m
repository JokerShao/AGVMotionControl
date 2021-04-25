%% params
% close all;
clc; clear;

chassis_w = 0.59727;
chassis_h = 0.59727;


chassis_theta = atan(chassis_h/chassis_w);
d = sqrt(chassis_w^2+chassis_h^2);

cmd_linear_xs = [-1 0 1];
cmd_linear_ys = [-1 0 1];
cmd_angular_zs = [-1 0 1];


% ENABLE_DRAW = 1;
ENABLE_DRAW = 0;

for idx_x=1:size(cmd_linear_xs, 2)
%     if ENABLE_DRAW
%         fg = figure('Name', 'omega '+string(omega_v(idx_h)));
%     end
    idx_x = 1
    idx_x

    cmd_linear_x = cmd_linear_xs(idx_x);
    for idx_y=1:size(cmd_linear_ys, 2)

        idx_y = 2
        cmd_linear_y = cmd_linear_ys(idx_y);
        for idx_z=1:size(cmd_angular_zs,2)

            idx_z = 1
            cmd_angular_z = cmd_angular_zs(idx_z);

            % convert control velocity to my format
            chassis_v = sqrt(cmd_linear_x^2+cmd_linear_y^2);
            chassis_alpha = atan2(cmd_linear_y, cmd_linear_x);
            chassis_omega = cmd_angular_z;

            raw_data = [cmd_linear_x cmd_linear_y cmd_angular_z chassis_v chassis_alpha]

            [w0, w1, w2, w3, icc, r0, r1, r2, r3] = AGV_inverse_kinematics( ...
                                                    d, chassis_w, chassis_h, chassis_theta, ...
                                                    chassis_v, chassis_omega, chassis_alpha);

            if (abs(w0(2))>pi) || (abs(w1(2))>pi) || ...
               (abs(w2(2))>pi) || (abs(w3(2))>pi)
                disp('11')
            end

            check_ik(chassis_w, chassis_h, chassis_omega, icc, r0, r1, r2, r3);

            if abs(chassis_omega) > 1e-9
                x1 = chassis_h/2+r3(1);
                y1 = -chassis_w/2+r3(2);
            else
                x1 = 0;
                y1 = 1e9;
            end
            if chassis_v < 1e-9
                chassis_alpha = 0;
            end

            ground_truth = [chassis_v chassis_omega chassis_alpha norm(icc) x1 y1 0];
            chassis_pred = AGV_forward_kinematics( ...
                chassis_w, chassis_h, w0, w1, w2, w3);

            
            vx = chassis_pred(1) * cos(chassis_pred(3));
            vy = chassis_pred(1) * sin(chassis_pred(3));
            vtheta = chassis_pred(2);
            calc_data = [vx vy vtheta chassis_pred(1) chassis_pred(3)]

            check_fk(ground_truth, chassis_pred)

            if ENABLE_DRAW
                draw(idx_y, chassis_w, chassis_h, chassis_v, chassis_alpha, ...
                        w0, w1, w2, w3, icc, r0, r1, r2, r3, chassis_pred(5:6));
            end
        end
    end
    if ENABLE_DRAW
        close all
    end
end

