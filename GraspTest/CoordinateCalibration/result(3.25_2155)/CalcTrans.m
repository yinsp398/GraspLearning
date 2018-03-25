load coordinate.txt
[row,low]=size(coordinate);
RobotMat = coordinate(:,4:6)';

CameraMat = [coordinate(:,1:3)';ones(1,row)];



TransMat = RobotMat * CameraMat' * inv(CameraMat*CameraMat');
RobotMat_2 = TransMat * CameraMat;
Aerr = RobotMat_2-RobotMat;
ERR = Aerr*Aerr';
TransMat
Aerr