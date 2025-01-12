%clear
close all
clc

s=serialport("COM3", 921600);
sx=read(s, 4, "int8");


n=2000;
sp=0.01;
t=linspace(0, (n-1)*sp, n);
accx=nan(1, n);

figure;
ax=plot(t, accx);

tic;
for k=1:n
    accx(k)=0.000061*read(s, 1, "int16");
    ax.YData=accx;
end
mytime=toc;
mu_acx=mean(accx);

standard_deviation=std(accx);
uncertainity_A=standard_deviation/sqrt(n);

