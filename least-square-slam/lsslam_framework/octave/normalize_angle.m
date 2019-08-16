function norm_theta = normalize_angle(theta)
%Put angle theta in [-pi, pi]
norm_theta = theta;

while norm_theta < -pi
    norm_theta = norm_theta + 2 * pi;
end

while norm_theta > pi
    norm_theta = norm_theta - 2 * pi;
end

end