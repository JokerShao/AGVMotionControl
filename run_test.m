%% params
% close all;
clc;
clear;

chassis_w = 1;
chassis_h = 2;


chassis_theta = atan(chassis_h/chassis_w);
d = sqrt(chassis_w^2+chassis_h^2);

omega_v = -2*pi:pi/360:2*pi;
alpha_v = -pi:pi/7:pi;

for idx_h=1:size(omega_v, 2)

    fg = figure('Name', 'omega '+string(omega_v(idx_h)));
    chassis_v = 2; % m/s
    chassis_omega = omega_v(idx_h) % rad/s

    for idx_w=1:size(alpha_v, 2)

%         idx_w = 1;
%         chassis_alpha = -5.1662;

        chassis_alpha = alpha_v(idx_w); %rad
        [w0, w1, w2, w3, r, r0, r1, r2, r3] = AGV_inverse_kinematics( ...
                                                d, chassis_w, chassis_h, chassis_theta, ...
                                                chassis_v, chassis_omega, chassis_alpha);
        draw_ik(idx_w, d, chassis_w, chassis_h, chassis_v, chassis_alpha, ...
                w0, w1, w2, w3, r, r0, r1, r2, r3);
        check_cross(chassis_w, chassis_h, chassis_omega, r, r0, r1, r2, r3);
    end

    close all
end

