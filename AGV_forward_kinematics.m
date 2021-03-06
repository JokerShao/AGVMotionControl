function chassis_state = AGV_forward_kinematics( ...
    chassis_w, chassis_h, wheel0, wheel1, wheel2, wheel3)
%%
%     [v0, alpha0]
%     [v1, alpha1]
%     [v2, alpha2]
%     [v3, alpha3]
%     chassis_state: v omega alpha r_scale icc_x icc_y error
    wheels = [wheel0; wheel1; wheel2; wheel3];


%% forward, backward, leftward, rightward, translate without rotation
if max(abs(wheels(:,2)-mean(wheels(:,2)))) < 0.0087 % 0.5 deg
    mean_v = mean(wheels(:,1));
    if abs(mean_v) < 1e-9
        chassis_state = [0 0 0 1e9 0 1e9 0];
    else
        chassis_state = [mean_v 0 mean(wheels(:,2)) 1e9 0 1e9 0];
    end
    return
end


%%
% spin around
lambda = atan(chassis_h/chassis_w);
d = (sqrt(chassis_w^2+chassis_h^2)/2);
alphas = [-lambda; -lambda; lambda; lambda];

if max(abs(wheels(:,2)-alphas)) < 1e-9
    % mean absolute velocity
    mav = mean(abs(wheels(:,1)));
    if max(abs(wheels(:,1)-[-mav; mav; -mav; mav])) < 1e-9
        chassis_state = [0 mav/d 0 0 0 0 0];
    return
    elseif max(abs(wheels(:,1)-[mav; -mav; mav; -mav])) < 1e-9
        chassis_state = [0 -mav/d 0 0 0 0 0];
    return
    end
end


%% normal case
% Ax+By+C=0
r0_ABC = [1 tan(wheels(1,2)) -chassis_h/2-chassis_w/(2/tan(wheels(1,2)))];
r1_ABC = [1 tan(wheels(2,2)) +chassis_h/2+chassis_w/(2/tan(wheels(2,2)))];
r2_ABC = [1 tan(wheels(3,2)) +chassis_h/2-chassis_w/(2/tan(wheels(3,2)))];
r3_ABC = [1 tan(wheels(4,2)) -chassis_h/2+chassis_w/(2/tan(wheels(4,2)))];

A = [r0_ABC; r1_ABC; r2_ABC; r3_ABC];
[~, D, V] = svd(A, 'econ');
X = V(:,3);
X = (X(1:2)/X(3))';
error = D(3,3);

car_center = [0 0];
corner_tl = [chassis_h/2 chassis_w/2];
corner_tr = [chassis_h/2 -chassis_w/2];
corner_bl = [-chassis_h/2 chassis_w/2];
corner_br = [-chassis_h/2 -chassis_w/2];

omegas = [wheels(1,1)/norm(X-corner_tl);
          wheels(2,1)/norm(X-corner_br);
          wheels(3,1)/norm(X-corner_bl);
          wheels(4,1)/norm(X-corner_tr)];
omega = mean(omegas);
r0 = X-corner_tl;
r0_normalized = r0/norm(r0);
phi0 = acos(dot([1 0], r0_normalized));
if r0_normalized(2)<0
    phi0=-phi0;
end
if fiseq(clamp(phi0-wheels(1,2)), -pi/2, 1e-9)
    omega=-omega;
end

r = X-car_center;
r_normalized = r/norm(r);
phi = acos(dot([1 0], r_normalized));
if r_normalized(2)<0
    phi=-phi;
end
if omega>0
    alpha=phi-pi/2;
else
    alpha=phi+pi/2;
end

% scala and must >0, because we already have alpha to describe orientation.
v = abs(omega)*norm(r);

chassis_state = [v omega clamp(alpha) norm(r) X error];

end












