function odo = odometryGeneration(XrTrue,XphiTrue,PosNum,addOdoNoise,Q)
if addOdoNoise == 0
    Q(1,1) = 0;
    Q(2,2) = 0;
    Q(3,3) = 0;
end

odo_x = [XrTrue(1:2:end-3,1),XrTrue(3:2:end-1,1), ...
    (XrTrue(3:2:end-1,2)-XrTrue(1:2:end-3,2)+sqrt(Q(1,1))*randn(PosNum-1,1)).*cos(XphiTrue(1:end-1,1)) + ...
    (XrTrue(4:2:end,2)-XrTrue(2:2:end-2,2)+sqrt(Q(2,2))*randn(PosNum-1,1)).*sin(XphiTrue(1:end-1,1))];

odo_y = [XrTrue(1:2:end-3,1),XrTrue(3:2:end-1,1), ...
    -(XrTrue(3:2:end-1,2)-XrTrue(1:2:end-3,2)+sqrt(Q(1,1))*randn(PosNum-1,1)).*sin(XphiTrue(1:end-1,1)) + ...
    (XrTrue(4:2:end,2)-XrTrue(2:2:end-2,2)+sqrt(Q(2,2))*randn(PosNum-1,1)).*cos(XphiTrue(1:end-1,1))];

odo_phi = [XrTrue(1:2:end-3,1),XrTrue(3:2:end-1,1), ...
    XphiTrue(2:end,1)-XphiTrue(1:end-1,1)+sqrt(Q(3,3))*randn(PosNum-1,1)];

odo = zeros((PosNum-1)*3,3);

odo(1:3:end-2,:) = odo_x(1:1:end,:);
odo(2:3:end-1,:) = odo_y(1:1:end,:);
odo(3:3:end,:) = odo_phi(1:1:end,:);