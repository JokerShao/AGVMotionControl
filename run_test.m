%% params
% close all;
clc; clear;

chassis_w = 1;
chassis_h = 2;


chassis_theta = atan(chassis_h/chassis_w);
d = sqrt(chassis_w^2+chassis_h^2);

omega_v = -2*pi:pi/20:2*pi;
% (-pi, pi]
alpha_v = -pi+1e-8:pi/720:pi;
velocity_v = 0:4/10:2;


% ENABLE_DRAW = 1;
ENABLE_DRAW = 0;

for idx_h=1:size(omega_v, 2)
%     if ENABLE_DRAW
%         fg = figure('Name', 'omega '+string(omega_v(idx_h)));
%     end

    idx_h
%     idx_h = 1;
    chassis_omega = omega_v(idx_h); % rad/s
    for idx_w=1:size(alpha_v, 2)

%         idx_w
%         idx_w = 1;
        chassis_alpha = alpha_v(idx_w); %rad
        for idx_c=1:size(velocity_v,2)

%              idx_c
%             idx_c = 2;
            chassis_v = velocity_v(idx_c); % m/s
            [w0, w1, w2, w3, gt_r, r0, r1, r2, r3] = AGV_inverse_kinematics( ...
                                                    d, chassis_w, chassis_h, chassis_theta, ...
                                                    chassis_v, chassis_omega, chassis_alpha);

            if (abs(w0(2))>pi) || (abs(w1(2))>pi) || ...
               (abs(w2(2))>pi) || (abs(w3(2))>pi)
                disp('11')
            end

            check_ik(chassis_w, chassis_h, chassis_omega, gt_r, r0, r1, r2, r3);

            if abs(chassis_omega) > 1e-9
                x1 = chassis_h/2+r3(1);
                y1 = -chassis_w/2+r3(2);
            else
                x1 = 0;
                y1 = 0;
            end

            if chassis_v < 1e-9
                chassis_alpha = 0;
            end
            ground_truth = [chassis_v chassis_omega chassis_alpha norm(gt_r) x1 y1 0];
            chassis_pred = AGV_forward_kinematics( ...
                chassis_w, chassis_h, w0, w1, w2, w3);
            check_fk(ground_truth, chassis_pred);

            if ENABLE_DRAW
                draw(idx_w, chassis_w, chassis_h, chassis_v, chassis_alpha, ...
                        w0, w1, w2, w3, gt_r, r0, r1, r2, r3, chassis_pred(5:6));
            end
        end
    end
    if ENABLE_DRAW
        close all
    end
end

