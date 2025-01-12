clear
close all
clc

s=serialport("COM3", 921600);
sx=read(s, 4, "int8");

g=9.80665;
n=2000;
sp=0.01;
t=linspace(0, (n-1)*sp, n);
accx_up=nan(1, n);
accx_dn=nan(1, n);
tic;
for k=1:n
    accx_up(k)=0.000061*read(s, 1, "int16");
end

disp('you have 10 senonds to change the orientation of your sensor');
pause(10);

for k=1:n
    accx_dn(k)=0.000061*read(s, 1, "int16");
end
mytime=toc;
mean_up=mean(accx_up);
mean_dn=mean(accx_dn);

constant_bias=0.5*(mean_dn+mean_up)-g;

