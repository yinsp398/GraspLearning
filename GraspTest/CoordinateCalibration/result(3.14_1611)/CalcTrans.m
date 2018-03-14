load coordinate.txt
RobotMat = coordinate(:,4:6)';
CameraMat = [coordinate(:,1:3)';ones(1,18)];



TransMat = RobotMat * CameraMat' * inv(CameraMat*CameraMat');
RobotMat_2 = TransMat * CameraMat;
Aerr = RobotMat_2-RobotMat;
Rerr = Aerr./RobotMat;
TransMat
Aerr
Rerr