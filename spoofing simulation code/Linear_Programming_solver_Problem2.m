clear all;

%% Parameters
R=0.1*eye(2);
Q=0.1*eye(2);
F=[ 1 0; 0 1];
u=[1;1];
H=[1 -0; 0 1];

%difference between m_0 and \tilde{m}_0?
% m_uncertain = [1;1];

Covariance = eye(2);
P=Covariance;

desired_separation_step = [3:15];
% desired_separation_step = [ 3 5 9 11 13 15 ];
% desired_separation_step = [5  10 15];
% desired_separation_step = [2 6];

desired_separation = 0.25*desired_separation_step*norm(u,2);
% desired_separation = [1.48    0];
% desired_separation = 0.25*ones(1:11);
%  desired_separation = [2 0];

T = max(desired_separation_step);

%% Calculate Kalman Gains
for i=1:T
    P=Covariance(:,:,i);
    P=F*P*F'+Q;
    Kalman_gain(:,:,i)=P*H*(H*P*H'+R)^(-1);
    P=(eye(2)-Kalman_gain(:,:,i)*H)*P;
    Covariance(:,:,i+1) = P;
end

%% Linear programming formulation
for i=1:T
    A(:,:,i) = F - Kalman_gain(:,:,i) *H*F;
end

Q = zeros(length(desired_separation_step) + 2*T, 2*T);

% for each desired step
for t = 1 : length(desired_separation_step)
    Prod_A{t}(:,:,desired_separation_step(t)) = Kalman_gain(:,:,desired_separation_step(t));
    for i = 1 : desired_separation_step(t) - 1
        Prod_A{t}(:,:,i) =  Kalman_gain(:,:,i);
        for j = i : desired_separation_step(t) - 1
            Prod_A{t}(:,:,i) = A(:,:,j)*Prod_A{t}(:,:,i);
        end
    end
    
    for i=1:2*desired_separation_step(t)
        if mod(i,2)
            Prod{t} = Prod_A{t}(:,:,(i+1)/2);
            L{t}(i) = sum(Prod{t}(:,1));
        else
            Prod{t} = Prod_A{t}(:,:,(i)/2);
            L{t}(i) = sum(Prod{t}(:,2));
        end
    end
    
    Q(t,1:length(L{t})) = -abs(L{t});
end

% rest of the rows are an identity matrix
Q(size(Q,1)-2*T + 1:end,:) = -eye(2*T);

f = ones(1,2*T);
b = zeros(length(desired_separation_step) + 2*T, 1);
b(1:length(desired_separation)) = -desired_separation;
spoof = linprog(f,Q,b)
spoof_add(1:T)=spoof(2*(1:T));
spoof_add = spoof_add';



