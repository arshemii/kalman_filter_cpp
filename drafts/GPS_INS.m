clear
close all
clc

%% Initializations
stm=serialport("COM8", 921600);
nSample=1000;
sr=100;          %sampling rate
xcl=nan(3, nSample);
gyro=nan(3, nSample);
t=nan(1,nSample);

%% Useful constant
d2r=pi/180;
r2d=1/d2r;
costG=9.80217887910626;  % Magnitude in m/s^2

%% GPS Measures; ZUPT correction data does not change (Ideally) due to the standing condition
GpsMeas.Lat=(40 + 48/60 + 58/3600)*d2r;
GpsMeas.Lon=(12 + 3/60 + 45/3600)*d2r;
GpsMeas.Alt=14;
GpsMeas.VeN=0.01;
GpsMeas.VeE=-0.02;
GpsMeas.VeD=0.01;

%% Geometrical Parameters
params.R0=6378137;                   % m earth average radius
params.eccentricity=0.0818191908425; % Earth eccentricity
params.wie=15/3600*d2r;              % earth angular rate in rad/s
params.gam=9.7803267714;             %m/s^2
params.kgamma=0.00193185138639;
% kgamma and gam are in 2.134

%% Data aquisition and mechanization
flush(stm)
write(stm, 'k', 'char');
cntIMU=1;
cntGPS=1;
while(cntIMU<=nSample)
    data=read(stm, 1, 'char');
    switch data
        case 'T'
            t(cntIMU)=read(stm, 1, 'single');
        case 'A'
            data=read(stm, 3, 'int16');
            xcl(:, cntIMU)=data*0.000061;
        case 'W'
            data=read(stm, 3, 'int16');
            gyro(:, cntIMU)=data*0.004375;
            cntIMU=cntIMU+1;
        case '$'
            nmea=readline(stm);
            app=char(nmea);
            GpsMeas.Lat=(str2double(app(18:19)) + str2double(app(20:21))/60 + str2double(app(22:27))/3600)*d2r;
            GpsMeas.Lon=(str2double(app(31:33)) + str2double(app(34:35))/60 + str2double(app(36:41))/3600)*d2r;
            disp(nmea);

    end
end

disp('End of initialization')

%% Mechanization example: Initialization
nSample=10000;
AVP= zeros(9, nSample);
AVP(:, 1)= [0; 0; 0; 0; 0; 0; GpsMeas.Lat; GpsMeas.Lon; GpsMeas.Alt];

A_med=mean(xcl, 2, 'omitnan'); % output is a 3*1 vector
AVP(1,1)=0;
AVP(2,1)=atan(-A_med(1)/sqrt(A_med(2)^2+A_med(3)^2));
AVP(3,1)=atan2(-A_med(2), -A_med(3));
% based on 5.101
% The projhection of gravity vector

%u_z=-A_med/norm(A_med);
%Cbn(3,1:3)=u_z;
%if abs(Cbn(3,1))<0.85
%    Cbn(2,1)=0;
%    Cbn(2,2)=Cbn(3,3)/sqrt(Cbn(3,2)^2+Cbn(3,3)^2);
%    Cbn(2,3)=Cbn(3,2)/sqrt(Cbn(3,2)^2+Cbn(3,3)^2);
%else
%    Cbn(2,3)=0;
%    Cbn(2,1)=Cbn(3,2)/sqrt(Cbn(3,1)^2+Cbn(3,2)^2);
%    Cbn(2,2)=-Cbn(3,1)/sqrt(Cbn(3,1)^2+Cbn(3,2)^2);
%end
%Cbn(1,1)=Cbn(2,2)*Cbn(3,3)-Cbn(2,3)*Cbn(3,2);
%Cbn(1,2)=Cbn(2,3)*Cbn(3,1)-Cbn(2,1)*Cbn(3,3);
%Cbn(1,3)=Cbn(2,1)*Cbn(3,2)-Cbn(2,2)*Cbn(3,1);

%AVP(1:3,1)=dcm2angle(Cbn);

%Cbn=eul2rotm(AVP(1:3), 'ZYX');

W_med=mean(gyro,2,'omitnan')*d2r;
sr=100;
xcl=nan(3, nSample);
gyro=nan(3, nSample);
t=nan(1, nSample);
cntIMU=2;


dX=zeros(15, nSample);   % all the values of correction
dX(13:15, 1) = -W_med;   % add all the bias of the gyroscope (Mean of all values of gyro when the system is standing
                         % System is without motion, for first 10 second
                         % mean of all value is bias
P = eye(15)* 0.0001;     % first guess for P is always identity matrix and we multiply
                         % to avoid divergence
Q = zeros(15);
Q(1:3, 1:3) = ((0.09*(pi/180)/60))^2*eye(3); %1e-3^2*sr*eye(3);
Q(4:6, 4:6) = (0.008/60)^2*eye(3); %1e_3^2*sr*eye(3);

Q(10:12, 10:12) = ((3.2e-6*costG)/sqrt(100))*2^eye(3); %1e-3^2/100*eye(3);
Q(13:15, 13:15) = ((0.8*(pi/180)/3600)/sqrt(300))^2*eye(3); %1e-3^2/100*eye(3);

%Q(1:3, 1:3) = ((0.09*(pi/180)/60))^2*eye(3); %1e-3^2*sr*eye(3);
%Q(4:6, 4:6) = (0.008/60)^2*eye(3); %1e_3^2*sr*eye(3);

%Q(10:12, 10:12) = ((3.2e-6*costG)/sqrt(100))*2^eye(3); %1e-3^2/100*eye(3);
%Q(13:15, 13:15) = ((0.8*(pi/180)/3600)/sqrt(300))^2*eye(3); %1e-3^2/100*eye(3);

H = zeros(6,15);
H(1:3, 7:9) = -eye(3);
H(4:6, 4:6) = -eye(3);


R = zeros(6);  % Do I need to change it to 6*6???
R(1,1) = atan(1/6478000)^2;  % 1meter is converted to degree
R(2,2) = atan(1/6478000)^2;
R(3,3) = 1;
% Uncertainities on each direction x, y, z
R(4:6, 4:6) = eye(3)*0.0001;

while cntIMU<=nSample
    data=read(stm, 1, 'char');
    switch data
        case 'T'
            t(cntIMU)=read(stm, 1, 'single');
        case 'A'
            data=read(stm, 3, 'int16');
            xcl(:, cntIMU)=data*0.000061*G;
        case 'W'
            data=read(stm, 3, 'int16');
            gyro(:, cntIMU)=data*0.004375*d2r;
              %tic
            [AVP(:, cntIMU), Cbn, Omegaien, Omegaenn] = Mech2NED(AVP(:, cntIMU-1), gyro(:, cntIMU) + dX(13:15, cntIMU-1), xcl(:, cntIMU) + dX(10:12, cntIMU-1), params, 1/sr);
            [dX(:, cntIMU), P] = KFT(xcl(:, cntIMU) + dX(10:12, cntIMU-1), Cbn, params, AVP(:, cntIMU), dX(:, cntIMU-1), P, Q, 1/sr, Omegaien, Omegaenn);
              %toc
            cntIMU=cntIMU+1;
        case '$'
            cntIMU = cntIMU - 1;
            nmea=readline(stm);
            GpsMeas.Lat=(str2double(app(18:19)) + str2double(app(20:21))/60 + str2double(app(22:27))/3600)*d2r;
            GpsMeas.Lon=(str2double(app(31:33)) + str2double(app(34:35))/60 + str2double(app(36:41))/3600)*d2r;
            GpsMeas.Alt=(str2double(app(54:59)));
            token = split(nmea, ',');
            if token(7) ~= '0'   %Check the position for my message
                [dX(:, cntIMU), P, AVP(:, cntIMU)] = KFC(GpsMeas, AVP(:, cntIMU), Cbn, dX(:, cntIMU), P, H, R);
            else
                disp('no correction');
            end
            disp(nmea)
            cntIMU = cntIMU + 1;
    end
end

figure
plot(AVP(1:3,:)'*r2d)
figure
plot(AVP(4:6,:)'*r2d)
figure
plot(deg2km((AVP(7,:)-AVP(7,1))*r2d)*1000, deg2km((AVP(8,:)-AVP(8,1))*r2d)*1000, AVP(9,:))