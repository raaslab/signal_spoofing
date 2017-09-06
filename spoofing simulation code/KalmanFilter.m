function [Target_estimate_potition_update, Covatiance_update]=KalmanFilter(Ture_measurement, Target_estimate_potition, Covatiance,u_o)
 x_o=Target_estimate_potition;
 z=Ture_measurement;
 P=Covatiance;
 F=[0.9 0.2; 0.3 0.85];
 H=eye(2);
 Q=0.5*eye(2);
 R=0.5*eye(2);
 %Predicate
 
 x_o=F*x_o+u_o;
 P=F*P*F'+Q;
 
 %Update
 Kk=P*H*(H*P*H'+R)^(-1);
 x_o=x_o+Kk*(z-H*x_o);
 P=(eye(2)-Kk*H)*P;
 
 Target_estimate_potition_update=x_o;
 Covatiance_update = P;
end