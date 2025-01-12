
%to make fake data normal guassian
data=normrnd(5, 3, 1000, 6);
%a linear array for time
t=linspace(0, 50, 1000);

figure(1)
subplot(2, 3, 1)
plot(t, data(:, 1))
xlabel('Time')
ylabel('Acceleration in X axis')
legend('Acc in Ax')   % to attach a sticker on plot
grid % to make grids on plot

subplot(2,3,3)
plot(t, data(:,3))
xlabel('Time')
ylabel('Acc in z')
legend('acc in az')
grid