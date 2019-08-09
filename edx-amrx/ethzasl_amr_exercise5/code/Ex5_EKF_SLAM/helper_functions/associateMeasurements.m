function [v, H, R] = associateMeasurements(x, P, z, R, g)

% evaluate distances of measurements to the map (pp. 341)
nMeasurements = size(z,2);

%Extract landmarks from state vector
M = reshape(x(4:end),2, length(x(4:end))/2);
nMapEntries = size(M,2);

d = zeros(nMeasurements, nMapEntries);
v = zeros(2, nMeasurements * nMapEntries);
H = zeros(2, length(x), nMeasurements * nMapEntries);

for i = 1 : nMeasurements
    for j = 1 : nMapEntries
        [z_priori, H(:, :, j + (i-1) * nMapEntries)] = measurementFunction(x, j);
        v(:,j + (i-1) * nMapEntries) = z(:,i) - z_priori;        %TODO: add polar difference function here!
        W = H(:, :, j + (i-1) * nMapEntries) * P * H(:, :, j + (i-1) * nMapEntries)' + R(:,:,i);
        d(i,j) = v(:,j + (i-1) * nMapEntries)' * inv(W) * v(:,j + (i-1) * nMapEntries);
    end
end

% line feature matching (pp. 341)

% association of each measurement to the map point with minimal distance
[minima, map_index] = min(d');
[measurement_index] = find(minima < g^2);
map_index = map_index(measurement_index);

v = v(:, map_index + (measurement_index-1)*nMapEntries);
H = H(:, :, map_index + (measurement_index-1)*nMapEntries);
R = R(:, :, measurement_index);