clear
close all
clc

n=100;
sp=0.1;
t=linspace(0, (n-1)*sp, n);
sigma=0.9;
V_mean=3;

A=[1, sp;
    0, 1];
%B=0;
P=0.2*eye(2);
Q=normrnd(0, 0.02, 2, 2);
H=[1, 0];
R=sigma^2;

X=nan(2, n);
X(1,1)=0;
X(2,1)=V_mean;

Xp=nan(2, n);
Xp(1,1)=0;
Xp(2,1)=V_mean;

X_meas=nan(1,n);
X_meas(1)=0.1;

for m=2:n
    X_meas(m)=V_mean*(m*sp)+X_meas(m-1)+normrnd(0, sigma, 1, 1);
end

figure;

subplot(1,2,1);
p1=plot(t, X(1,:), 'r');
hold on;
p2=plot(t, X_meas, 'g');
p3=plot(t, Xp(1,:), 'y');
hold off;
legend('Pos')

subplot(1,2,2);
v1=plot(t, X(2,:), 'r');
hold on;
v2=plot(t, Xp(2,:), 'g');
hold off;
legend('vel')


for i=2:n
    X(:, i)=A*X(:, i-1);
    P=A*P*A'+Q;
    Xp(:, i)=X(:, i);

    K=P*H'*inv(H*P*H'+R);
    X(:, i)=X(:, i)+K*(X_meas(i)-H*X(:, i));
    P=(eye(2)-K*H)*P;

    p1.YData=X(1,:);
    p2.YData=X_meas;
    p3.YData=Xp(1,:);
    v1.YData=X(2,:);
    v2.YData=Xp(2,:);
    pause(sp);
end

