clear
close all
clc

s=serialport("COM3", 921600);

n=100;
sp=0.1;
t=linspace(0, (n-1)*sp, n);
sensitivity=0.000061;
g=9.80665;

data1=nan(1,n);

%%figure;
%%acc=plot(t, data1);
%%legend('acc in z');

tic;
for i=1:n
    data1(i)=read(s, 1, "int16");
    data1(i)=g*data1(i)*sensitivity;
    %acc.YData=data1;
end

t1=toc;

disp('Please upside down sensor. You have 5 seconds to do that')
pause(5);

data2=nan(1,n);
for i=1:n
    data2(i)=read(s, 1, "int16");
    data2(i)=g*data2(i)*sensitivity;
end

mean1=mean(data1);
mean2=mean(data2);

Bias=0.5*(mean1-mean2)-g;