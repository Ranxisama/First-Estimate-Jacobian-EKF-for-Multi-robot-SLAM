function odo = odometryGeneration(XrTrue,XphiTrue,PosNum,addOdoNoise,Q)
if addOdoNoise == 0
    Q(1,1) = 0;
    Q(2,2) = 0;
    Q(3,3) = 0;
end

odo_x = [XrTrue(1:2:(end-3),1),XrTrue(3:2:(end-1),1), ...
    (XrTrue(3:2:(end-1),2)-XrTrue(1:2:(end-3),2)+sqrt(Q(1,1))*Randn(PosNum,1)).*cos(XphiTrue(1:(end-1),2)) + ...
    (XrTrue(4:2:end,2)-XrTrue(2:2:(end-2),2)+sqrt(Q(2,2))*Randn(PosNum,1)).*sin(XphiTrue(1:(end-1),2))];

odo_y = [XrTrue(1:2:(end-3),1),XrTrue(3:2:(end-1),1), ...
    -(XrTrue(3:2:(end-1),2)-XrTrue(1:2:(end-3),2)+sqrt(Q(1,1))*Randn(PosNum,1)).*sin(XphiTrue(1:(end-1),2)) + ...
    (XrTrue(4:2:end,2)-XrTrue(2:2:(end-2),2)+sqrt(Q(2,2))*Randn(PosNum,1)).*cos(XphiTrue(1:(end-1),2))];

odo_phi = [XrTrue(1:2:(end-3),1),XrTrue(3:2:(end-1),1), ...
    XphiTrue(2:end,2)-XphiTrue(1:(end-1),2)+sqrt(Q(3,3))*Randn(PosNum,1)];

odo = zeros((PosNum)*3,3);

odo(1:3:(end-2),:) = odo_x;
odo(2:3:(end-1),:) = odo_y;
odo(3:3:end,:) = odo_phi;