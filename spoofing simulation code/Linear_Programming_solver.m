clear;
R=0.5*eye(2);
Q=0.5*eye(2);
F=[0.9 0.2; 0.3 0.85];
u=[1;1];
H=eye(2);
step=4;
%Riccati equation generate 
Covariance = eye(2);
Kalman_gain = (F*Covariance*F' + R)*H'*(H*Covariance*H' + Q)^-1;
for i=1:step
    Covariance(:,:,i+1) = F*Covariance(:,:,i)*F' - F*Covariance(:,:,i)*H'*(H*Covariance(:,:,i)*H'+Q)^(-1)*H*Covariance(:,:,i)*F' + R;
    Kalman_gain(:,:,i+1) = (F*Covariance(:,:,i+1)*F' + R)*H'*(H*Covariance(:,:,i+1)*H' + Q)^-1;
end
%Linear programming matrix
for i=1:step
    A(:,:,i) = F - Kalman_gain(:,:,i) *H*F;
end

L=eye(step);
for i=1:step
   for j=i:step
       
   end
    
end


L= [0 0 1;0 0.8 1;0.4 0.8 1;0 0 -1;0 -1 0; -1 0 0];
f= [-0. -0. -1];
b=[1 2 3.0 0 0 0];
x = linprog(f,A,b)