clc; clear;

real_data = csvread('encoder.csv');





chassis_w = 0.59727;
chassis_h = 0.59727;
time_now = 0;
vehicle_info = zeros(1,9);
last_vehicle_info = zeros(1,9);
d = sqrt(chassis_w^2+chassis_h^2);

% figure

for i=1:size(real_data, 1)


    data = real_data(i,:);

    vehicle_info = [time_now data(13) data(14) data(16) data(15) ...
                    data(20) data(18) data(17) data(19)];

    difference = [vehicle_info(1:5)-last_vehicle_info(1:5) last_vehicle_info(1,6:9)];

    last_vehicle_info=vehicle_info;

    dt = difference(1,1) / 1000;
    time_now = time_now+100;

    if abs(dt) < 1e-9 || abs(dt) > 1e3
        continue
    end

    v = (difference(1,2:5)/4096.0*0.6) / dt
    alpha = (difference(1,6:9) / 1638400)*pi*2

    for i=1:4
        if alpha(1,i) > pi
            alpha(1,i) = alpha(1,i)-pi*2;
        end
    end

    difference
    v;
    alpha;

    wheels = [v; alpha]'

    wheels = [ 0.78391413 -0.39082757; ...
               0.32472661  1.1671897; ...
               0.78391413  0.39082757; ...
               0.32472661 -1.1671897];
    vv = [1 0]';

    corner_tl = [chassis_h/2 chassis_w/2];
    corner_tr = [chassis_h/2 -chassis_w/2];
    corner_bl = [-chassis_h/2 chassis_w/2];
    corner_br = [-chassis_h/2 -chassis_w/2];

    alpha = wheels(1, 2)-pi/2;
    R = [cos(alpha) -sin(alpha);
         sin(alpha) cos(alpha)];
    r0 = R*vv;
    line([corner_tl(1) corner_tl(1)+r0(1)], [corner_tl(2) corner_tl(2)+r0(2)], 'Color', 'Red', 'LineStyle', '-.');

    alpha = wheels(2, 2)-pi/2;
    R = [cos(alpha) -sin(alpha);
         sin(alpha) cos(alpha)];
    r0 = R*vv;
    line([corner_br(1) corner_br(1)+r0(1)], [corner_br(2) corner_br(2)+r0(2)], 'Color', 'Red', 'LineStyle', '-.');


    alpha = wheels(3, 2)-pi/2;
    R0 = [cos(alpha) -sin(alpha);
         sin(alpha) cos(alpha)];
    r0 = R0*vv;
    line([corner_bl(1) corner_bl(1)+r0(1)], [corner_bl(2) corner_bl(2)+r0(2)], 'Color', 'Red', 'LineStyle', '-.');

    alpha = wheels(4, 2)-pi/2;
    R0 = [cos(alpha) -sin(alpha);
         sin(alpha) cos(alpha)];
    r0 = R0*vv;
    line([corner_tr(1) corner_tr(1)+r0(1)], [corner_tr(2) corner_tr(2)+r0(2)], 'Color', 'Red', 'LineStyle', '-.');

    

    chassis_state = AGV_forward_kinematics( ...
        chassis_w, chassis_h, wheels(1,:), wheels(2,:), wheels(3,:), wheels(4,:))

    chassis_v = chassis_state(1);
    chassis_omega = chassis_state(2);
    chassis_alpha = chassis_state(3);
    r = chassis_state(4);
    x0 = chassis_state(5);
    y0 = chassis_state(6);
    err = chassis_state(7);
    chassis_theta = atan(chassis_h/chassis_w);
    
    if err >= 0.02
        disp aa
    end
    
    close all
end





    chassis_state = AGV_forward_kinematics( ...
        chassis_w, chassis_h, wheels(1,:), wheels(2,:), wheels(3,:), wheels(4,:))

    chassis_v = chassis_state(1);
    chassis_omega = chassis_state(2);
    chassis_alpha = chassis_state(3);
    r = chassis_state(4);
    x0 = chassis_state(5);
    y0 = chassis_state(6);
    err = chassis_state(7);
    chassis_theta = atan(chassis_h/chassis_w);


    [w0, w1, w2, w3, gt_r, r0, r1, r2, r3] = AGV_inverse_kinematics( ...
                                        d, chassis_w, chassis_h, chassis_theta, ...
                                        chassis_v, chassis_omega, chassis_alpha);
    

    draw(1, chassis_w, chassis_h, chassis_v, chassis_alpha, ...
        w0, w1, w2, w3, gt_r, r0, r1, r2, r3, [x0 y0]);
    
    
