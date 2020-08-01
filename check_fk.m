function check_fk(ground_truth, chassis_pred)

% if fiseq(ground_truth(3), pi) && fiseq(chassis_pred(3), -pi)
%     chassis_pred(3) = pi;
% elseif fiseq(ground_truth(3), -pi) && fiseq(chassis_pred(3), pi)
%     chassis_pred(3) = -pi;
% end

differ = ground_truth-chassis_pred;
if sum(abs(differ)) > 1e-7
    disp('difference forward kinematics.');
end

end
