Total_alarm = 0;
Threshold = 0.5;
drift_term = 0.;

for j = 1:1000
    
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
    

    g_k = 0;
    
    %%
    for  i=1:T
        x_true(:,i+1) = F*x_true(:,i) + u + (0.2)*randn(2,1);
        z_true(:,i) = H*x_true(:,i) + (0.3)*randn(2,1);
    end
    %
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
        [x_estimate_spoof(:,i), Covatiance_update, difference]=KalmanFilter(z_spoof(:,i), x_estimate_spoof(:,i-1), Covatiance,u);
        [g_k, Alarm] = SPRT_Detector(g_k,Threshold, drift_term, difference);
        
        if Alarm == 1
            break;
        end
        Covatiance = Covatiance_update;
    end
    
    if Alarm == 1
        Total_alarm = Total_alarm + 1;
    end
    
end


% hold off


