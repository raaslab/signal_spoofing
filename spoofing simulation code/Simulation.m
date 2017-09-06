clear;
%generate true data
step = 15;
x_true=[0;0];
z_true=[0;0];
C=[0.9 0.2; 0.3 0.85];
u=[1;1];
H=eye(2);
for  i=1:step
    x_true(:,i+1) = C*x_true(:,i) + u + 0.5*randn(2,1);
    z_true(:,i+1) = H*x_true(:,i+1) + 0.5*randn(2,1)+[20;0];
end

%Kalman filter eatimation
Covatiance=eye(2);
x_estimate = x_true(:,1)+[0;0];
for i=2:step
    [x_estimate(:,i), Covatiance_update]=KalmanFilter(z_true(:,i), x_estimate(:,i-1), Covatiance,u);
    Covatiance = Covatiance_update;
end

plot(x_true(1,:),x_true(2,:))
hold on
plot(x_estimate(1,:),x_estimate(2,:))