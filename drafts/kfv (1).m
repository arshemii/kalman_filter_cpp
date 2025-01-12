%{
Equations for KF.
Linear state space format:
xn=F*xp+G*up+w (F: system matrix  G: Input matrix  w: process noise)
yn=H*xn+vk (H: Observation matrix  vk: measurement noise)
Example: Voltage
equations:
predict:
xn(estimate)=A*xp(true)+B*up(true)
Pn(estimate)=A*Pp(true)*A'+Q
correction:
k=Pn(estimate)*H'*(H*Pn(estimate)*H'+R)^-1
xn(true)=xn(estimate)+k(z(measurement)-H*xn(estimate))
P(new true)=(I-k*H)*Pn(estimate)
(P: initial as identity, R: covariance of measurand-here only position
uncertainity, Q: 
%}

clc
close all
clear

n=100;
sp=0.1;
t=linspace(0, (n-1)*sp, n);
sens_unc=0.6;
V_rms=5*ones(1,100);
noise=normrnd(0, sens_unc, 1, n);
V_meas=5+noise;
V=zeros(1,100);
V(1)=V_rms(1);

R=sens_unc^2;
P=0.1*eye(1);
A=eye(1);
Q=normrnd(0, 0.04, 1, 1);
H=eye(1); %Vriable of estimation is the same with variable of measurement

for i=2:n
    V(i)=A*V(i-1);
    P=A*P*A'+Q;
    K=P*H'*inv(H*P*H'+R);
    V(i)=V(i)+K*(V_meas(i)-H*V(i));
    P=(eye(1)-K*H)*P;
end

figure;
plot(t, V_rms, 'b');
hold on;
plot(t, V, 'g');
plot(t, V_meas, 'r');
hold off;


