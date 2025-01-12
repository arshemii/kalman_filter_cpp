function [dX,P,AVP] = KFC(GpsMeas, AVP, Cbn, dX, P, H, R)
% KFTCorrection Function for the evaluation of the correction equations of
% serror state Kalman filter
K = P*H'/(H*P*H'+R);

P = (eye(15)-K*H)*P;
P = 0.5*(P+P');
dZ = [GpsMeas.Lat-AVP(7); GpsMeas.Lon-AVP(8); GpsMeas.Alt-AVP(9); GpsMeas.VeN-AVP(4); GpsMeas.VeE-AVP(5); GpsMeas.VeD-AVP(6)];

dX = dX - K*dZ;

AVP(4:9) = AVP(4:9) + dX(4:9);
dPsi_vect = rotVec2Mat(dX(1:3));
Cbn = Cbn*(eye(3) + dPsi_vect);
[AVP(1), AVP(2), AVP(3)] = dcm2angle(Cbn,'ZYX');
dX(1:9) = zeros(9,1);
end

