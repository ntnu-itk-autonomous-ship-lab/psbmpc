function angle = wrap_to_02pi(angle)
[~, N] = size(angle);
if angle > 2*pi * ones(1, N)
    angle = angle - 2*pi;
elseif angle < zeros(1, N)
    angle = angle + 2*pi;
end
end

