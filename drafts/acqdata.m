clear
close all
clc

s=serialport("COM3", 921600);
sp=0.01;
ns=3000;

x_sens=read(s, 4, "uint8");
g_sens=read(s, 4, "uint8");

sensitivity_x=typecast(x_sens(1:4), 'double');
sensitivity_g=typecast(g_sens(1:4), 'double');

IMU=nan(6, ns);
t=linspace(0, (ns-1)*sp, sp);
for kk=1:ns
    for j=1:3
        datax=read(s, 2, "uint8");
        IMU(j, kk)=typecast(datax(1:2), 'single');
        datag=read(s, 2, "uint8");
        IMU(j+3, kk)=typecast(datag(1:2), 'single');
    end
end

figure(1)

subplot(2,3,1)
plot(t, IMU(1,:))
xlabel('Time')
ylabe('acc_x')
grid

subplot(2,3,2)
plot(t, IMU(2,:))
xlabel('Time')
ylabe('acc_y')
grid

subplot(2,3,3)
plot(t, IMU(3,:))
xlabel('Time')
ylabe('acc_z')
grid

subplot(2,3,4)
plot(t, IMU(4,:))
xlabel('Time')
ylabe('ang_x')
grid

subplot(2,3,5)
plot(t, IMU(5,:))
xlabel('Time')
ylabe('ang_y')
grid

subplot(2,3,6)
plot(t, IMU(6,:))
xlabel('Time')
ylabe('ang_z')
grid