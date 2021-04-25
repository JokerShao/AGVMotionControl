%% params
% close all;
clc; clear;

chassis_w = 0.59727;
chassis_h = 0.59727;


chassis_theta = atan(chassis_h/chassis_w);
d = sqrt(chassis_w^2+chassis_h^2);

load('paired.mat');
sensor_array = paired(:,1:9);
ideal_twist = paired(:,10:end);

close all
figure;


for i=2593:size(sensor_array,1)
    i=5064
    w0 = sensor_array(i,[2 6]);
    w1 = sensor_array(i,[3 7]);
    w2 = sensor_array(i,[4 8]);
    w3 = sensor_array(i,[5 9]);

    chassis_pred = AGV_forward_kinematics( ...
        chassis_w, chassis_h, w0, w1, w2, w3);

    chassis_v = chassis_pred(1);
    chassis_omega = chassis_pred(2);
    chassis_alpha = chassis_pred(3);
    r_scale = chassis_pred(4);
    x = chassis_pred(5);
    y = chassis_pred(6);
    err = chassis_pred(7);


    draw(0, chassis_w, chassis_h, chassis_v, chassis_alpha, ...
                w0, w1, w2, w3, [0 0], [0 0], [0 0], [0 0], [0 0], chassis_pred(5:6));


    steer_radius = [5 0]';
    % corner_tl
    % corner_tr
    % corner_bl
    % corner_br
    corner = [chassis_h/2 chassis_w/2;
              -chassis_h/2 -chassis_w/2;
              -chassis_h/2 chassis_w/2;
              chassis_h/2 -chassis_w/2];
    for k=1:4
        alpha = sensor_array(i,k+5)-pi/2;
        R = [cos(alpha) -sin(alpha);
             sin(alpha) cos(alpha)];
        vv = R*steer_radius;
        plot([corner(k,1)-vv(1)/2 corner(k,1)+vv(1)/2], [corner(k,2)-vv(2)/2 corner(k,2)+vv(2)/2], '-.m');
    end

            
    %%
    % vehicle speed
%     dd = [cos(chassis_alpha), sin(chassis_alpha)];
%     dd_norm = dd/norm(dd);
%     vv = chassis_v*dd_norm * 0.5; % scale 0.5
    hold on;
    plot([0 ideal_twist(i,2)*0.5], [0 ideal_twist(i,3)*0.5], '-.b');
    hold on;
%     plot(0, 0, '-*b');

            
            
txt = {'i:', i, 't-offset:', abs(sensor_array(i,1)-ideal_twist(i,1))};
text(0.1,0.1,txt);
% xlim([-2 2])
% ylim([-3 3])
drawnow
    clf
end

