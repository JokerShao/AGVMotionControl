function [wheel0, wheel1, wheel2, wheel3, ...
            r, r0, r1, r2, r3] = AGV_inverse_kinematics( ...
            d, chassis_w, chassis_h, theta, v, omega, alpha)
%% forward, backward, leftward, rightward, translation without rotate
if abs(omega)<1e-5 % 0.54 degree
    wheel0 = [v alpha];
    wheel1 = [v alpha];
    wheel2 = [v alpha];
    wheel3 = [v alpha];

    r = [0 0 0];
    r0 = [0 0 0];
    r1 = [0 0 0];
    r2 = [0 0 0];
    r3 = [0 0 0];
    return
end


%%
v0 = sqrt(v^2+d^2*omega^2/4-d*v*omega*cos(theta+alpha));
v1 = sqrt(v^2+d^2*omega^2/4+d*v*omega*cos(theta+alpha));
% v1 = sqrt(v^2+d^2*omega^2/4+d*abs(v*omega)*cos(theta-alpha));
% v0 = sqrt(v^2+d^2*omega^2/4-d*abs(v*omega)*cos(theta-alpha));

r_scale = abs(v/omega);
% r0_scale = sqrt((v/omega)^2+(d/2)^2-d*v/omega*cos(theta-alpha));
% r1_scale = sqrt((v/omega)^2+(d/2)^2+d*v/omega*cos(theta-alpha));
r0_scale = sqrt((v/omega)^2+(d/2)^2-d*v/omega*cos(theta+alpha));
r1_scale = sqrt((v/omega)^2+(d/2)^2+d*v/omega*cos(theta+alpha));

% spin around
if abs(v) < 1e-3
    lambda = atan(chassis_h/chassis_w);

    if omega > 0
        wheel0 = [v0 pi-lambda];
        wheel1 = [v1 -lambda];

        r = calculate_radius(r_scale, alpha+pi/2);
        r0 = calculate_radius(r0_scale, pi-lambda+pi/2);
        r1 = calculate_radius(r1_scale, -lambda+pi/2);
    else
        wheel0 = [v0 -lambda];
        wheel1 = [v1 pi-lambda];

        r = calculate_radius(r_scale, alpha-pi/2);
        r0 = calculate_radius(r0_scale, -lambda-pi/2);
        r1 = calculate_radius(r1_scale, pi-lambda-pi/2);
    end
    return
end


%%
if (alpha >= -theta) && (alpha <= pi-theta)
    if (omega < 0)
        beta = -(pi/2 - theta - alpha);
        beta0 = -atan( (d/2+r_scale*cos(pi/2+beta)) / (r_scale*sin(pi/2+beta)) );
        alpha0 = pi/2-theta+beta0;
        beta1 = atan( (d/2-r_scale*cos(pi/2+beta)) / (r_scale*sin(pi/2+beta)) );
        alpha1 = pi/2-theta+beta1;
    elseif (omega > 0)
        beta = -(pi/2 - theta - alpha);
        beta0 = atan( (d/2-r_scale*cos(pi/2+beta)) / (r_scale*sin(pi/2+beta)) );
        alpha0 = pi/2-theta+beta0;
        beta1 = -atan( (d/2+r_scale*sin(-beta)) / (r_scale*cos(-beta)) );
        alpha1 = pi/2-theta+beta1;
    end

else
    if (omega < 0)
        beta = -(pi/2 - theta - alpha);
        beta0 = -atan( (d/2+r_scale*cos(pi/2+beta)) / (r_scale*sin(pi/2+beta)) );
        if (alpha > pi-theta) && (alpha <= pi)% && (beta0 < 0)
            beta0 = beta0 + pi;
        elseif (alpha >= -pi) && (alpha < -theta) && (beta0 > 0)
            beta0 = beta0 + pi;
        end
        alpha0 = pi/2-theta+beta0;

        beta1 = atan( (d/2-r_scale*cos(pi/2+beta)) / (r_scale*sin(pi/2+beta)) );
        if (alpha > pi-theta) && (alpha <= pi) && (beta1 < 0)
            beta1 = beta1 + pi;
        elseif (alpha >= -pi) && (alpha < -theta)% && (beta1 < 0)
            beta1 = beta1 + pi;
        end
        alpha1 = pi/2-theta+beta1;
    elseif (omega > 0)
        beta = -(pi/2 - theta - alpha);
        beta0 = atan( (d/2-r_scale*cos(pi/2+beta)) / (r_scale*sin(pi/2+beta)) );
        if (alpha > pi-theta) && (alpha <= pi)% && (beta0 < 0)
            beta0 = beta0 + pi;
        elseif (alpha >= -pi) && (alpha < -theta)% && (beta0 > 0)
            beta0 = beta0 + pi;
        end
        alpha0 = pi/2-theta+beta0;

        beta1 = -atan( (d/2+r_scale*sin(-beta)) / (r_scale*cos(-beta)) );
        if (alpha > pi-theta) && (alpha <= pi)% && (beta1 < 0)
            beta1 = beta1 + pi;
        elseif (alpha >= -pi) && (alpha < -theta)% && (beta1 > 0)
            beta1 = beta1 + pi;
        end
        alpha1 = pi/2-theta+beta1;
    end
end

wheel0 = [v0 alpha0];
wheel1 = [v1 alpha1];
wheel2 = [0 0];
wheel3 = [0 0];


%%
if sign(omega) >= 0
    r = calculate_radius(r_scale, alpha+pi/2);
    r0 = calculate_radius(r0_scale, alpha0+pi/2);
    r1 = calculate_radius(r1_scale, alpha1+pi/2);
else
    r = calculate_radius(r_scale, alpha-pi/2);
    r0 = calculate_radius(r0_scale, alpha0-pi/2);
    r1 = calculate_radius(r1_scale, alpha1-pi/2);
end

r2 = [0 0 0];
r3 = [0 0 0];

end


% Angle between speed direction and vehicle head direction
function r = calculate_radius(r_scale, alpha)

    R = [cos(alpha) -sin(alpha);
         sin(alpha) cos(alpha)];

    x = [1 0]';
    direction = R*x;
    direction = direction / norm(direction);
    r = direction*r_scale;
end













