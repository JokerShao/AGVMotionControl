function check_ik(chassis_w, chassis_h, chassis_omega, gt_r, r0, r1, r2, r3)

%   2    0  Y
%           |_X
%   1    3
corner_tl = [chassis_h/2 chassis_w/2];
corner_tr = [chassis_h/2 -chassis_w/2];
corner_bl = [-chassis_h/2 chassis_w/2];
corner_br = [-chassis_h/2 -chassis_w/2];

cross_x = [gt_r(1) corner_tl(1)+r0(1) corner_br(1)+r1(1) corner_bl(1)+r2(1) corner_tr(1)+r3(1)];
cross_y = [gt_r(2) corner_tl(2)+r0(2) corner_br(2)+r1(2) corner_bl(2)+r2(2) corner_tr(2)+r3(2)];

x_is_cross = (norm(cross_x - mean(cross_x)) < 1e-9);
y_is_cross = (norm(cross_y - mean(cross_y)) < 1e-9);
is_cross = x_is_cross && y_is_cross;

if ~is_cross && (abs(chassis_omega) > 1e-3)
    disp('not cross');
end

end
