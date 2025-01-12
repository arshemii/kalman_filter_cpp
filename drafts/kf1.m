%{
Equations for KF.
Linear state space format:
xn=F*xp+G*up+w (F: system matrix  G: Input matrix  w: process noise)
yn=H*xn+vk (H: Observation matrix  vk: measurement noise)
Example: Pos and vel estimation with constant velocity and external
position measurement. Sensor each 10 update provides measurands.
equations:
predict:
xn(estimate)=A*xp(true)+B*up(true)
Pn(estimate)=A*Pp(true)*A'+Q
correction:
k=Pn(estimate)*H'*(H*Pn(estimate)*H'+R)^-1
xn(true)=xn(estimate)+k(z(measurement)-H*xn(estimate))
P(new true)=(I+k*H)*Pn(estimate)
(P: initial as identity, R: covariance of measurand-here only position
uncertainity, Q: 
%}

clc
close all
clear

var=0.6;
sp=0.1;
n=200;
t=linspace(0, (n-1)*sp, n);
acc=ones(1,100);
x=nan(2, n);
x(1,1)=0;
x(2,1)=0;

x_proc=nan(2,n);
x_proc(1,1)=0;
x_proc(2,1)=0;

p_meas=nan(1,n);
for i=10:10:100
    p_meas(i)=0.5*acc(i)*(sp*i)^2+x(2,1)*(sp*i)+x(1,1)+normrnd(0, var, 1);
end


A=[1, sp;
    0, 1];
B=[0.5*(sp^2); sp];
P=0.1*eye(2);
Q=normrnd(0, 0.01, 2, 2);
R=var^2;
H=[1, 0];

for i=2:100
    x(:,i)=A*x(:,i-1)+B*acc(i-1);
    x_proc(:, i)=x(:,i);
    P=A*P*A'+Q;
    if ~isnan(p_meas(i))
        K=P*H'*inv(H*P*H'+R);
        x(:,i)=x(:,i)+K*(p_meas(i)-H*x(:,i));
        P=(eye(2)+K*H)*P;
    end
end

figure;

subplot(1,2,1)
plot(t, x(1,:), 'r');
hold on;
plot(t, x_proc(1,:), 'g');
hold off;

subplot(1,2,2)
plot(t, x(2,:), 'r');
hold on;
plot(t, x_proc(2,:), 'g');
hold off;