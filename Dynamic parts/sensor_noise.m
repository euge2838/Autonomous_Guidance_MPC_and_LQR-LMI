function xD0 = sensor_noise(xD0)
% Sensor noise function

% GPS-IMU fusion sensor noise
% X and Y (position)
xD0(1:2) = 0.001*sqrt(12)*(rand(2,1) - 1/2) + xD0(1:2);

% THETA (orientation)
xD0(3) = 0.0001*sqrt(12)*(rand(1,1) - 1/2) + xD0(3);

% V (velocity)
xD0(4) = 0.005*sqrt(12)*(rand(1,1) - 1/2) + xD0(4);

% Alpha and Omega
xD0(5:6) = 0.0001*sqrt(12)*(rand(1,1) - 1/2) + xD0(5:6);

% figure(1)
% plot(u);
end

