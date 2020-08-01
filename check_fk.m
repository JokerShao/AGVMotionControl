function check_fk(ground_truth, chassis_pred)

if (abs(ground_truth(3)-pi)<1e-9) && (abs(chassis_pred(3)+pi)<1e-9)
    chassis_pred(3) = pi;
elseif (abs(ground_truth(3)+pi)<1e-9) && (abs(chassis_pred(3)-pi)<1e-9)
    chassis_pred(3) = -pi;
end

differ = ground_truth-chassis_pred;

if sum(abs(differ)) > 1e-9
    disp('difference forward kinematics');
end






end