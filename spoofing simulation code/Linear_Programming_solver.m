 clear;
R=0.1*eye(2);
Q=0.1;
F=[1, 0; 0.1 1];
u=[1;1];
H=[1 -0.3;1 0];
step=5;
%Riccati equation generate 
Covariance = 1*eye(2);
Kalman_gain = (F*Covariance*F' + R)*H'*(H*Covariance*H' + Q)^-1;
for i=1:step
    Covariance(:,:,i+1) = F*Covariance(:,:,i)*F' - F*Covariance(:,:,i)*H'*(H*Covariance(:,:,i)*H'+Q)^(-1)*H*Covariance(:,:,i)*F' + R;
    Kalman_gain(:,:,i+1) = (F*Covariance(:,:,i+1)*F' + R)*H'*(H*Covariance(:,:,i+1)*H' + Q)^-1;
end
%Linear programming matrix
for i=1:step+1
    A(:,:,i) = F - Kalman_gain(:,:,i) *H*F;
end

L=eye(step);
Prod_A = eye(2);
for i=2:step+1
     Prod_A(:,:,i) =  Kalman_gain(:,:,i-1);
   for j=i:step+1
       Prod_A(:,:,i) =  A(:,:,j)*Prod_A(:,:,i);
   end   
end


 A(:,:,6)* A(:,:,5)*A(:,:,4)* A(:,:,3)*A(:,:,2)*Kalman_gain(:,:,1)
  A(:,:,6)* A(:,:,5)*A(:,:,4)* A(:,:,3)*Kalman_gain(:,:,2)

L= [0 0 1;0 0.8 1;0.4 0.8 1;0 0 -1;0 -1 0; -1 0 0];
f= [-0. -0. -1];
b=[1 2 3.0 0 0 0];
x = linprog(f,A,b)