clear;
%generate true data
step = 5;
x_true=[0;0];
z_true=[0;0];
F=[ 1 0; 0 1];
u=[1;1];
H=[1 -0; 0 1];
for  i=1:step
    x_true(:,i+1) = F*x_true(:,i) + u + (0.1)*randn(2,1);
    z_true(:,i+1) = H*x_true(:,i+1) + (0.1)*randn(2,1);
end

spoof = abs(2*zeros(2,6));
spoof(1,6) = 4.0441;
spoof(2,6) = 4.0441;

z_spoof = z_true + spoof;

%Kalman filter eatimation
Covatiance=eye(2);
Covatiance_spoof = Covatiance;
x_estimate = x_true(:,1)+[0;0];
x_estimate_spoof = x_estimate;
for i=2:step+1
    [x_estimate(:,i), Covatiance_update]=KalmanFilter(z_true(:,i), x_estimate(:,i-1), Covatiance,u);
    Covatiance = Covatiance_update;
end

for i=2:step+1
    [x_estimate_spoof(:,i), Covatiance_update_spoof] = KalmanFilter(z_spoof(:,i), x_estimate_spoof(:,i-1), Covatiance_spoof,u);
    Covatiance = Covatiance_update;
end

plot(x_true(1,:),x_true(2,:),'--gs','LineWidth',2,'MarkerSize',10)
hold on
plot(x_estimate(1,:),x_estimate(2,:),'-.rd','LineWidth',2,'MarkerSize',10)
plot(x_estimate_spoof(1,:),x_estimate_spoof(2,:),'-.bo','LineWidth',2,'MarkerSize',10)
hold off

distance = norm(x_estimate(:,6)-x_estimate_spoof(:,6))

