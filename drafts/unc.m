clear
close all
clc

s=serialport("COM3", 921600);

sensitivity=read(s, 4, "int8");

n=100;
sp=0.01;
t=linspace(0, (n-1)*sp, n);
accx=nan(1, 100);

figure;
ax=plot(t, accx);
xlabel('time');
ylabel('z acceleration');
grid;


for k=1:n
    %char=read(s, 1, "char");
    %if char=='a'
    accx(k)=sensitivity*read(s, 1, "int16");
    ax.YData=accx(k);
    %end
end

mu_acx=mean(accx);

std=std(accx);
uncertainity_A=std/sqrt(n);

