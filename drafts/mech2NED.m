function [AVP, Cbn, Omegaien, Omegaenn] = mech2NED(AVP, wibb, fibb, params, Dt)
%MECH2NED Summary of this function goes here
%   Detailed explanation goes here

%% State vector components
Heading = AVP(1);
Pitch = AVP(2);
Roll = AVP(3);
V = AVP (4:6);
Lat = AVP(7);
Lon = AVP(8);
Alt = AVP(9);

%% Attitude update
Cbn = eul2rotm([Heading, Pitch, Roll], 'ZYX');
Omegaibb = rotVec2Mat(wibb);    %wibb is what the gyro measures
wien = [cos(Lat)*params.wie, 0, -sin(Lat)*params.wie];  %Formula 2.150 multiplied by wiee
Omegaien = rotVec2Mat(wien);
Rn = params.R0*(1-params.eccentricity^2)/(1-params.eccentricity^2*sin(Lat)^2)^1.5; % 2.105
Re = params.RO/sqrt(1-params.eccentricity^2*sin(Lat)^2);     % 2.106
Lat_dot = V(1)/(Rn+Alt);    % p 15 mechanization
Lon_dot = V(2)/(cos (Lat) * (Re+Alt));    % p 15 mechanization
wenn = [cos(Lat)*Lon_dot, -Lat_dot, -sin(Lat)*Lon_dot];   % p 15 mechanization
% Time derivative of curvilinear positions formula 2.111
Omegaenn = rotVec2Mat(wenn);
Cbn = Cbn*(eye (3) +Omegaibb*Dt) - (Omegaien + Omegaenn) *Cbn*Dt;


%% Specific force transformation and gravity model

fibn = Cbn*fibb;
gamma0 = params.gam*(1+params.kgamma*sin(Lat)*sin(Lat))/(sqrt(1-params.eccentricity^2*sin(Lat)*sin(Lat))); % m/s^2 & 2.134
gamma = gamma0*(sqrt(Rn*Re)/(sqrt(Rn*Re)+Alt))^2; % m/s^2  from where???? Maybe 2.133 to 2.138
gn = [0 0 gamma]'-params.wie^2*(sqrt(Rn*Re)+Alt)*[sin(Lat)*cos(Lat) 0 cos(Lat)*cos(Lat)]'; % m/s^2 - Vettore gravità del filo a p


%% Velocity update
V = V+(fibn+gn-(Omegaenn+2*Omegaien)*V)*Dt;


%% Position update
Alt = Alt - V(3)*Dt;
Lat = Lat + V(1)/(Rn+Alt)*Dt;
Re = params.RO/sqrt(1-params.eccentricity^2*sin(Lat)^2);
Lon = Lon + V(2)/(cos(Lat)*(Re+Alt))*Dt;
% Why we don't have Re-Alt???
AVP(1:3) = rotm2eul(Cbn,'ZYX');
AVP(4:6) = V;
AVP(7) = Lat;
AVP(8) = Lon;
AVP(9) = Alt;
end

