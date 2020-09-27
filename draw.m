function draw( ...
    idx_w, chassis_w, chassis_h, chassis_v, chassis_alpha, ...
    w0, w1, w2, w3, r, r0, r1, r2, r3, X)
%
%   2    0  Y
%           |_X
%   1    3


%%
figure;
hold on;
% subplot(4, 4, idx_w)

% x axis
hold on;
plot([chassis_h/2+0.1 chassis_h/2+0.2], [0 0], '-*r');
% y axis
hold on;
plot([chassis_h/2+0.1 chassis_h/2+0.1], [0 0.1], '-*g');

% car box
car_center = [0 0];
corner_tl = [chassis_h/2 chassis_w/2];
corner_tr = [chassis_h/2 -chassis_w/2];
corner_bl = [-chassis_h/2 chassis_w/2];
corner_br = [-chassis_h/2 -chassis_w/2];
corner = [corner_tl; corner_tr; corner_br; corner_bl; corner_tl];
hold on;
plot(corner(:,1), corner(:,2), '-*b');


%%
% vehicle speed
dd = [cos(chassis_alpha), sin(chassis_alpha)];
dd_norm = dd/norm(dd);
vv = chassis_v*dd_norm * 0.5; % scale 0.5
hold on;
plot([car_center(1) vv(1)], [car_center(2) vv(2)], '-*r');
hold on;
plot(car_center(1), car_center(2), '-*b');
% draw radius
hold on;
line([0, r(1)], [0, r(2)], 'Color', 'Red', 'LineStyle', '-.');

% top left wheel
d0 = [cos(w0(2)), sin(w0(2))];
d0_norm = d0 / norm(d0);
v0 = w0(1)*d0_norm*0.5;
hold on;
% wheel speed
plot([corner_tl(1) corner_tl(1)+v0(1)], [corner_tl(2) corner_tl(2)+v0(2)], '-*g');
% draw radius
hold on;
line([corner_tl(1) corner_tl(1)+r0(1)], [corner_tl(2) corner_tl(2)+r0(2)], 'Color', 'Red', 'LineStyle', '-.');

% bottom right wheel
d1 = [cos(w1(2)), sin(w1(2))];
d1_norm = d1 / norm(d1);
v1 = w1(1)*d1_norm*0.5;
hold on;
plot([corner_br(1) corner_br(1)+v1(1)], [corner_br(2) corner_br(2)+v1(2)], '-*g');
% draw radius
hold on;
line([corner_br(1) corner_br(1)+r1(1)], [corner_br(2) corner_br(2)+r1(2)], 'Color', 'Red', 'LineStyle', '-.');

% bottom left wheel
d2 = [cos(w2(2)), sin(w2(2))];
d2_norm = d2 / norm(d2);
v2 = w2(1)*d2_norm*0.5;
hold on;
plot([corner_bl(1) corner_bl(1)+v2(1)], [corner_bl(2) corner_bl(2)+v2(2)], '-*g');
% draw radius
hold on;
line([corner_bl(1) corner_bl(1)+r2(1)], [corner_bl(2) corner_bl(2)+r2(2)], 'Color', 'Red', 'LineStyle', '-.');

% top right wheel
d3 = [cos(w3(2)), sin(w3(2))];
d3_norm = d3 / norm(d3);
v3 = w3(1)*d3_norm*0.5;
hold on;
plot([corner_tr(1) corner_tr(1)+v3(1)], [corner_tr(2) corner_tr(2)+v3(2)], '-*g');
% draw radius
hold on;
line([corner_tr(1) corner_tr(1)+r3(1)], [corner_tr(2) corner_tr(2)+r3(2)], 'Color', 'Red', 'LineStyle', '-.');


% xlim([-1.5 1.5]);
% ylim([-1.5 1.5]);


% center and radius predict
scatter(X(1), X(2))
line([corner_tl(1) X(1)], [corner_tl(2) X(2)], 'Color', 'Black', 'LineStyle', '--');
line([corner_br(1) X(1)], [corner_br(2) X(2)], 'Color', 'Black', 'LineStyle', '--');
line([corner_bl(1) X(1)], [corner_bl(2) X(2)], 'Color', 'Black', 'LineStyle', '--');
line([corner_tr(1) X(1)], [corner_tr(2) X(2)], 'Color', 'Black', 'LineStyle', '--');

axis equal;

end










