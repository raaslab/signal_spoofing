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

Covariance = [1 0;0 1];
P=Covariance;

% desired_separation_step = [4 7 10];
% desired_separation = 0.25*desired_separation_step*norm(u,2);
% T = max(desired_separation_step);
x_true=[0.;0];
z_true=[0;0];
F=[ 1 0; 0 1];
u=[1;1];
H=[1 -0; 0 1];

%%
for  i=1:T
    x_true(:,i+1) = F*x_true(:,i) + u + (0.1)*randn(2,1);
    z_true(:,i) = H*x_true(:,i) + (0.2)*randn(2,1);
end

spoof = abs(2*zeros(2,T));
spoof(1,:) = 0.1*spoof_add;
spoof(2,:) = 1.9*spoof_add;

z_spoof = z_true + spoof;

%Kalman filter eatimation
Covatiance=5*eye(2);
Covatiance_spoof = Covatiance;
x_estimate = x_true(:,1)+[0.5*randn;0.5*randn];
x_estimate_spoof = x_estimate;
for i=2:T
    [x_estimate(:,i), Covatiance_update]=KalmanFilter(z_true(:,i), x_estimate(:,i-1), Covatiance,u);
    Covatiance = Covatiance_update;
end


for i=2:T
    [x_estimate_spoof(:,i), Covatiance_update_spoof] = KalmanFilter(z_spoof(:,i), x_estimate_spoof(:,i-1), Covatiance_spoof,u);
    Covatiance_spoof = Covatiance_update;
end

true_distance = (x_estimate - x_estimate_spoof);
for iii = 1:length(true_distance)
    true_separation(iii) = norm(true_distance(:,iii),1);
end
 x_true(:,T+1)=[];
plot(x_true(1,:),x_true(2,:),'--gs','LineWidth',2,'MarkerSize',10)
hold on
plot(x_estimate(1,:),x_estimate(2,:),'-.rd','LineWidth',2,'MarkerSize',10)
plot(x_estimate_spoof(1,:),x_estimate_spoof(2,:),'-bo','LineWidth',2,'MarkerSize',10)
legend('True Position','Estimation without spoofing','Estimation with spoofing')
% hold off

 true_separation(2)
for i = 1:T
    distance(i) = norm(x_estimate(:,i)-x_estimate_spoof(:,i),1);
end

for i = 1:T
    distance2(i) = norm(x_estimate(:,i)-x_estimate_spoof_offline(:,i),1);
end

distance_online = [0,0];
for i = 3:T
    distance_online(i)  = abs(distance(i)- desired_separation(i-2));
end
hold on
distance_offline = [0,0];
for i = 3:T
    distance_offline(i)  = abs(distance2(i)- desired_separation(i-2));
end

