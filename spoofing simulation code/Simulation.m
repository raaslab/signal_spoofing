clear;
%generate true data
step_1 = 9;
step_2 = 10;
x_true=[0;0];
z_true=[0;0];
F=[ 1 0; 0 1];
u=[1;1];
H=[1 -0; 0 1];
for  i=1:step_2
    x_true(:,i+1) = F*x_true(:,i) + u + (0.1)*randn(2,1);
    z_true(:,i+1) = H*x_true(:,i+1) + (0.1)*randn(2,1);
end

spoof = abs(2*zeros(2,step_2+1));
spoof(1,step_1+1) =  0;
spoof(2,step_1+1) = 2*2.5666;

spoof(1,step_2+1) =  0;
spoof(2,step_2+1) = 2*1.8714;

z_spoof = z_true + spoof;

%Kalman filter eatimation
Covatiance=5*eye(2);
Covatiance_spoof = Covatiance;
x_estimate = x_true(:,1)+[0;0];
x_estimate_spoof = x_estimate;
for i=2:step_2+1
    [x_estimate(:,i), Covatiance_update]=KalmanFilter(z_true(:,i), x_estimate(:,i-1), Covatiance,u);
    Covatiance = Covatiance_update;
end


for i=2:step_2+1
    [x_estimate_spoof(:,i), Covatiance_update_spoof] = KalmanFilter(z_spoof(:,i), x_estimate_spoof(:,i-1), Covatiance_spoof,u);
    Covatiance_spoof = Covatiance_update;
end

plot(x_true(1,:),x_true(2,:),'--gs','LineWidth',2,'MarkerSize',10)
hold on
plot(x_estimate(1,:),x_estimate(2,:),'-.rd','LineWidth',2,'MarkerSize',10)
plot(x_estimate_spoof(1,:),x_estimate_spoof(2,:),'-.bo','LineWidth',2,'MarkerSize',10)
legend('True Position','Estimation without spoofing','Estimtion with spoofing')
hold off

distance1 = norm(x_estimate(:,step_1+1)-x_estimate_spoof(:,step_1+1),1)
distance2 = norm(x_estimate(:,step_2+1)-x_estimate_spoof(:,step_2+1),1)

