 clear;
R=0.1*eye(2);
Q=0.1*eye(2);
F=[ 1 0; 0 1];
u=[1;1];
H=[1 -0; 0 1];
step=5;
%Riccati equation generate 
Covariance = 10*eye(2);
 P=Covariance;
% Kalman_gain = (F*Covariance*F' + R)*H'*(H*Covariance*H' + Q)^-1;
% for i=1:step
%     Covariance(:,:,i+1) = F*Covariance(:,:,i)*F' - F*Covariance(:,:,i)*H'*(H*Covariance(:,:,i)*H'+Q)^(-1)*H*Covariance(:,:,i)*F' + R;
%     Kalman_gain(:,:,i+1) = (F*Covariance(:,:,i+1)*F' + R)*H'*(H*Covariance(:,:,i+1)*H' + Q)^-1;
% end

for i=1:step
    P=Covariance(:,:,i);
    P=F*P*F'+Q;
    Kalman_gain(:,:,i)=P*H*(H*P*H'+R)^(-1);
    P=(eye(2)-Kalman_gain(:,:,i)*H)*P;
    Covariance(:,:,i+1) = P;
end

%Linear programming matrix
for i=1:step
    A(:,:,i) = F - Kalman_gain(:,:,i) *H*F;
end


Prod_A(:,:,step) = Kalman_gain(:,:,step);
for i=1:step-1
     Prod_A(:,:,i) =  Kalman_gain(:,:,i);
   for j=i:step-1
       Prod_A(:,:,i) =  A(:,:,j)*Prod_A(:,:,i);
   end   
end

for i=1:2*step
    if mod(i,2)
        Prod = Prod_A(:,:,(i+1)/2);
        L(i) = sum(Prod(:,1));
    else
        Prod = Prod_A(:,:,(i)/2);
        L(i) = sum(Prod(:,2));
    end
end

L=-abs(L);
%  A(:,:,6)* A(:,:,5)*A(:,:,4)* A(:,:,3)*A(:,:,2)*Kalman_gain(:,:,1)
%   A(:,:,6)* A(:,:,5)*A(:,:,4)* A(:,:,3)*Kalman_gain(:,:,2)
%   
%   syms e_x1 e_y1 e_x2 e_y2 e_x3 e_y3 e_x4 e_y4 e_x5 e_y5 
%   
%   Prod_A(:,:,1+1)*[e_x1;e_y1] + Prod_A(:,:,2+1)*[e_x2;e_y2] + Prod_A(:,:,3+1)*[e_x3;e_y3] + Prod_A(:,:,4+1)*[e_x4;e_y4] + Prod_A(:,:,5+1)*[e_x5;e_y5] 
  
  

% L= [0 0 1;0 0.8 1;0.4 0.8 1;0 0 -1;0 -1 0; -1 0 0];
L1 = -eye(10);
Q = [L ; L1]
f= [1. 1. 1 1. 1. 1 1. 1. 1 1];
b=[-5; zeros(10,1)];
x = linprog(f,Q,b)