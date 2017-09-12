%clear;
%generate true data
% step_1 = 9;
% step_2 = 10;
%% Parameters
R=0.5*eye(2);
Q=0.5*eye(2);
F=[ 1 0; 0 1];
u=[1;1];
H=[1 -0; 0 1];

Covariance = eye(2);
P=Covariance;

% desired_separation_step = [4 7 10];
% desired_separation = 0.25*desired_separation_step*norm(u,2);
% T = max(desired_separation_step);
x_true=[0;0];
z_true=[0;0];
F=[ 1 0; 0 1];
u=[1;1];
H=[1 -0; 0 1];

%%
for  i=1:T
    x_true(:,i+1) = F*x_true(:,i) + u + (0.1)*randn(2,1);
    z_true(:,i) = H*x_true(:,i) + (0.1)*randn(2,1);
end

spoof = abs(2*zeros(2,T));
spoof(2,:) = 2*spoof_add;

z_spoof = z_true + spoof;

%Kalman filter eatimation
Covatiance=5*eye(2);
Covatiance_spoof = Covatiance;
x_estimate = x_true(:,1)+[0;0];
x_estimate_spoof = x_estimate;
for i=2:T
    [x_estimate(:,i), Covatiance_update]=KalmanFilter(z_true(:,i), x_estimate(:,i-1), Covatiance,u);
    Covatiance = Covatiance_update;
end


for i=2:T
    [x_estimate_spoof(:,i), Covatiance_update_spoof] = KalmanFilter(z_spoof(:,i), x_estimate_spoof(:,i-1), Covatiance_spoof,u);
    Covatiance_spoof = Covatiance_update;
end

plot(x_true(1,:),x_true(2,:),'--gs','LineWidth',2,'MarkerSize',10)
hold on
plot(x_estimate(1,:),x_estimate(2,:),'-.rd','LineWidth',2,'MarkerSize',10)
plot(x_estimate_spoof(1,:),x_estimate_spoof(2,:),'-bo','LineWidth',2,'MarkerSize',10)
legend('True Position','Estimation without spoofing','Estimtion with spoofing')
hold off

for i = 1:length(desired_separation)
    distance(i) = norm(x_estimate(:,desired_separation_step(i))-x_estimate_spoof(:,desired_separation_step(i)),1);
end

