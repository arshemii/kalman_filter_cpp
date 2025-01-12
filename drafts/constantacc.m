clear
close all
clc

n=80;
sp=0.4; %s
A=[1, sp; 0, 1];
B=[0.5*(sp^2); sp];
u=1.5; %m/s^2
sigma=0.1;
t=linspace(0, (n-1)*sp, n);

P=0.1*eye(2);

X=nan(2, n);
X(1,1)=0;
X(2,1)=0;

X_proc=nan(2, n);
X_proc(1,1)=0;
X_proc(2,1)=0;

X_meas=nan(1, n);
for i=3:3:n
    X_meas(i)=normrnd(0, sigma, 1)+0.5*u*((i*sp)^2)+X(2,1)*(i*sp)+X(1,1);
end

R=sigma^2;

Q=normrnd(0, 0.1, 2, 2);

H=[1,0];

figure;
ax=plot(t, X(1,:), 'r');
hold on;
ac=plot(t, X_proc(1,:), 'g');
hold off;

for m=2:n
    %prediction
    X(:, m)=A*X(:, m-1)+B*u;
    X_proc(:, m)=X(:, m);
    P=A*P*A'+Q;
    
    %correction
    if ~isnan(X_meas(m))
        K=P*H'*inv(H*P*H'+R);
        X(:, m)=X(:, m)+K*(X_meas(m)-H*X(:, m));
        P=(eye(2)-K*H)*P;
    end
    ax.YData=X(1,:);
    ac.YData=X_proc(1,:);
    pause(sp);
end
