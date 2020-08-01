%% params
% close all;
clc;
clear;

chassis_w = 1;
chassis_h = 2;


chassis_theta = atan(chassis_h/chassis_w);
d = sqrt(chassis_w^2+chassis_h^2);

omega_v = -2*pi:pi/20:2*pi;
alpha_v = -pi:pi/720:pi;


ENABLE_DRAW = 0;

for idx_h=1:size(omega_v, 2)

    if ENABLE_DRAW
        fg = figure('Name', 'omega '+string(omega_v(idx_h)));
    end

%     idx_h = 42;
    chassis_v = 2; % m/s
    chassis_omega = omega_v(idx_h) % rad/s

    for idx_w=1:size(alpha_v, 2)

%         idx_w = 1;
        chassis_alpha = alpha_v(idx_w); %rad

        [w0, w1, w2, w3, r, r0, r1, r2, r3] = AGV_inverse_kinematics( ...
                                                d, chassis_w, chassis_h, chassis_theta, ...
                                                chassis_v, chassis_omega, chassis_alpha);
                                            
%         if abs(w0(2)) > pi
%             disp('11')
%         end
%         
%         if abs(w1(2)) > pi
%             disp('11')
%         end
%         if abs(w2(2)) > pi
%             disp('11')
%         end
%         if abs(w3(2)) > pi
%             disp('11')
%         end
        
        check_cross(chassis_w, chassis_h, chassis_omega, r, r0, r1, r2, r3);

        if abs(chassis_omega) > 1e-9
        x1 = chassis_h/2+r3(1);
        y1 = -chassis_w/2+r3(2);
        ground_truth = [chassis_v chassis_omega chassis_alpha norm(r) x1 y1 0];
        else
            ground_truth = [chassis_v chassis_omega chassis_alpha norm(r) 0 0 0];
        end
            
%         corner_tl = [chassis_h/2 chassis_w/2];
%         corner_tr = [chassis_h/2 -chassis_w/2];
%         corner_bl = [-chassis_h/2 chassis_w/2];
%         corner_br = [-chassis_h/2 -chassis_w/2];
%         NN = [corner_tl; corner_tr; corner_bl; corner_br];
%         for i = 1:4
%             x2 = NN(i,1);
%             y2 = NN(i,2);
%             A = y2-y1;
%             B = x1-x2;
%             C = x2*y1-x1*y2;
% 
%             [A/A B/A C/A];
%         end

        chassis_pred = AGV_forward_kinematics( ...
            chassis_w, chassis_h, w0, w1, w2, w3);

        if ENABLE_DRAW
            draw_ik(idx_w, d, chassis_w, chassis_h, chassis_v, chassis_alpha, ...
                    w0, w1, w2, w3, r, r0, r1, r2, r3, chassis_pred(5:6));
        end

        check_fk(ground_truth, chassis_pred);
    end

    if ENABLE_DRAW
        close all
    end
end

