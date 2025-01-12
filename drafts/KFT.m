function [dX, P] = KFT(fibb, Cbn, params, AVP, dX, P, Q, ts, Omegaie, Omegaen)
% Time update equations
% Error state
Lat = AVP(7);
Alt = AVP(9);
V = AVP(4:6);
fn = Cbn* fibb;

% % % M = params.R0*(1-params.eccentricity^2)/(1-params.eccentricity^2*sin(Lat)^2)^1.5;
% % % N = params.R0/sqrt(1-params.eccentricity^2*sin(Lat)^2);
% % % SquEarthEcc = params.eccentricity^2;
% % % EarthRate = params.wie;
% % % gamma0 = params.gam*(1+params.kgamma*sin(Lat)*sin(Lat))/(sqrt(1-params.eccentricity^2*sin(Lat)*sin(Lat)));
% % % F =zeros(15);
% % %   F(7:9, 7:9) = [0              0          -V(1)/((M+Alt)^2);
% % %            V(2)*sin(Lat)/((N+Alt)*cos(Lat)*cos(Lat)   0  V(2)/(cos(Lat)*(N+Alt)^2);
% % %            0                           0                      0];
% % %   F(7:9, 4:6) = [1/(M+Alt)              0          0;
% % %            0   1/((N+Alt)*cos(Lat))  0);
% % %            0                           0                      -1];
% % %   F(4:6, 7:9) = [-(V(2)^2*sec(Lat)^2)/(N+Alt)-2*V(2)*EarthRate*cos(Lat) 0  -V(1)*V(3)/(M+Alt)^2+V(2)^2*tan(Lat)/(N+Alt)^2;
% % %            0   1/((N+Alt)*cos(Lat))  0);
% % %            0                           0                      -1];
% % %   F(4:6,4:6)=[V(3)/(M+Alt)   -2*V(2)*tan(Lat)/(N+Alt)-2*EarthRate*sin(lat)   V(1)/(M+Alt);
% % %            v(2)*tan(Lat)/(N+Alt)+2*EarthRate*sin(Lat)  (V(l)*tan (Lat)+V(3))/(N+Alt)   V(2)/(N+Alt) + 2*EarthRate*cos(Lat);
% % %            -2*V(1)/(M+A1t)             -2*V(2)/(N+Alt) -2*EarthRate*cos (Lat)                           0];
% % %   F(4:6,1:3)=[0      fn(3,1)      -fn(2,1);
% % %            -fn(3,1)    O    fn(1,1);
% % %            fn(2,1) -fn(1,1)     0];
% % %   F(1:3,7:9)=[EarthRate*sin(Lat)   0     V(2)/(N+Alt)^2;
% % %            O   0     -V(1)/(M+Alt)^2;
% % %            v(2)/(N+Alt)*cos (Lat)^2)+EarthRate*cos(Lat) 0    -V(2)*tan (Lat)/(N+Alt)^2];
% % %   F(1:3,4:6)=[0     -1/(N + Alt)    o;
% % %            1/(M+Alt)            0        0;
% % %            0         tan(Lat)/(N+Alt)    0];
% % %   F(1:3, 1:3)=-(Omegaie + Omegaen);
% % %   F(4:6, 10:12)= Cbn;
% % %   F(1:3, 13:15)= Cbn;

F21 = rotVec2Mat(-Cbn*fibb);
Rn = params.RO* (1-params.eccentricity^2)/(1-params.eccentricity^2*sin(Lat)^2)^1.5;
Re = params.R0/sqrt(1-params.eccentricity^2*sin(Lat)^2);
F32 = zeros(3);
F32(1,1) = 1/(Rn + AVP(9));
F32(2,2) = 1/((Re + AVP(9)*cos (Lat)));
F32(3,3) = -1;
gamma0 = params.gam*(1+params.kgamma*sin(Lat)*sin(Lat))/(sqrt (1-params.eccentricity^2*sin(Lat)*sin(Lat))); % m/s^2
reSe = Re*sqrt(cos(Lat)^2+(1-params.eccentricity^2)^2*sin(Lat)^2);
F23 = zeros(3);
F23(3,3) = -2*gamma0/reSe;

F = zeros(15);
F(4:6,1:3) = F21;
F(4:6,7:9) = F23;
F(7:9,4:6) = F32;
F(1:3,13:15) = Cbn;
F(4:6,10:12) = Cbn;

PHI = eye(15) +F*ts;   % 14.72 formula of the book
dX = PHI * dX;
G = eye(15);
G(4:6, 1:3) = Cbn;
G(7:9, 4:6) = -Cbn;
% G(10:12, 10:12) = eye(3);
% G(13:15, 13:15) = eye(3);
P = PHI*P*PHI' + G*Q*G'*ts;
end

