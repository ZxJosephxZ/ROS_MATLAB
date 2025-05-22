% === Configuración ROS ===
rosshutdown;
setenv('ROS_MASTER_URI','http://192.168.0.24:11311');  
setenv('ROS_IP','192.168.0.22'); 
rosinit();

% === Añadir Publisher de velocidad ===
velPub = rospublisher('/robot0/cmd_vel', 'geometry_msgs/Twist');
velMsg = rosmessage(velPub);

% === Parámetros de navegación ===
safeDist = 0.7;   % Distancia de seguridad al frente
forwardSpeed = 0.2;
turnSpeed = 0.3;



% === Parámetros del mapa ===
resolution = 0.01;
mapSize = 15 / resolution;
logOddsMap = zeros(mapSize, mapSize);

% === Log-odds ===
l_occ = log(9);
l_min = -5;
l_max = 5;

% === Parámetros del LIDAR ===
laserTopic = '/robot0/laser_1';  % Ajusta si tu tópico es distinto
minRange = 0.15;
maxRange = 8.0;

% === Suscripciones ===
odomSub = rossubscriber('/robot0/odom');
laserSub = rossubscriber(laserTopic);

% === Loop principal ===
for i = 1:100
    odomMsg = receive(odomSub, 3);
    pose = odomMsg.Pose.Pose;

    x0 = pose.Position.X;
    y0 = pose.Position.Y;
    quat = pose.Orientation;
    yaw = quat2eul([quat.W quat.X quat.Y quat.Z]);
    theta = yaw(1);

    % Recibir mensaje láser
    scanMsg = receive(laserSub, 3);
    ranges = scanMsg.Ranges;
    angleMin = scanMsg.AngleMin;
    angleInc = scanMsg.AngleIncrement;

    % Visualizar mapa actual
    imagesc(flipud(logOddsMap)); 
    colormap('gray'); colorbar;
    set(gca, 'YDir', 'normal');
    axis equal tight;
    hold on;
    plot((x0/resolution)+1, mapSize - (y0/resolution), 'bo', 'MarkerSize', 5, 'LineWidth', 2);

    % Procesar cada rayo del láser
    for r = 1:length(ranges)
        range = ranges(r);
        if range < minRange || range > maxRange
            continue;
        end

        angle = angleMin + (r-1) * angleInc;
        worldTheta = theta + angle;

        % Posición de colisión estimada
        x1 = x0 + range * cos(worldTheta);
        y1 = y0 + range * sin(worldTheta);

        % Dibujar rayo
        plot([x0 x1]/resolution + 1, mapSize - [y0 y1]/resolution, 'r-');

        % Marcar punto de colisión
        [ix1, iy1] = world2map(x1, y1, resolution, mapSize);
        if ix1 > 0 && ix1 <= mapSize && iy1 > 0 && iy1 <= mapSize
            logOddsMap(iy1, ix1) = min(l_max, logOddsMap(iy1, ix1) + l_occ);
        end
    end

    hold off;
    title(['Mapa con láser (iteración ' num2str(i) ')']);
    drawnow;
    % === NAVEGACIÓN BÁSICA ===
    frontAngles = [-0.2 0.2];  % ± aprox. 11 grados
    idxs = find((angleMin + (0:length(ranges)-1)*angleInc) >= frontAngles(1) & ...
                (angleMin + (0:length(ranges)-1)*angleInc) <= frontAngles(2));
    frontRanges = ranges(idxs);
    frontClear = all(frontRanges > safeDist);

    if frontClear
        % Avanza
        velMsg.Linear.X = forwardSpeed;
        velMsg.Angular.Z = 0;
    else
        % Obstruido: girar hacia el lado más despejado
        leftClear = mean(ranges(1:round(end/3))) > safeDist;
        rightClear = mean(ranges(round(end*2/3):end)) > safeDist;

        if leftClear && ~rightClear
            velMsg.Linear.X = 0;
            velMsg.Angular.Z = turnSpeed;
        elseif ~leftClear && rightClear
            velMsg.Linear.X = 0;
            velMsg.Angular.Z = -turnSpeed;
        else
            velMsg.Linear.X = 0;
            velMsg.Angular.Z = turnSpeed; % Gira por defecto
        end
    end

    send(velPub, velMsg);
end

% Detener el robot al final
velMsg.Linear.X = 0;
velMsg.Angular.Z = 0;
send(velPub, velMsg);


% === Crear mapa final ===
probMap = 1 - (1 ./ (1 + exp(logOddsMap)));
binaryMap = logOddsMap > 0;

mapObj = binaryOccupancyMap(binaryMap, 1/resolution);

figure;
show(mapObj);
title('Mapa final basado en colisiones de láser');
save('mapa_final_laser.mat', 'mapObj');

% === Función auxiliar ===
function [ix, iy] = world2map(x, y, resolution, mapHeight)
    ix = round(x / resolution) + 1;
    iy = mapHeight - round(y / resolution);
end

