% clamp alpha to range -pi ~ pi
function alpha = clamp(alpha)
    if alpha > pi
        alpha = alpha-2*pi;
    elseif alpha < -pi
        alpha = alpha + 2*pi;
    end
end
