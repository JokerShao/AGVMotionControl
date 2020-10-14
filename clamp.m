% clamp alpha to range (-pi, pi]
function alpha = clamp(alpha)
    if fisgt(alpha, pi, 1e-9)
        alpha = alpha-2*pi;
    elseif fisltoeq(alpha, -pi, 1e-9)
        alpha = alpha + 2*pi;
    end
end
