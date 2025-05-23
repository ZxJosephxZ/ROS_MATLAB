% === CONFIGURACIÓN ROS ===
rosshutdown;
setenv('ROS_MASTER_URI','http://192.168.0.24:11311');  
setenv('ROS_IP','192.168.0.22'); 
rosinit();

%% MAIN
% === PARÁMETROS INICIALES ===
safeDist = 0.7;
forwardSpeed = 0.2;
turnSpeed = 0.3;
robotRadius = 0.25;

resolution = 0.06;
mapSize = 15 / resolution;
logOddsMap = zeros(mapSize, mapSize);
visitedMap = false(mapSize, mapSize);

l_occ = log(9);
l_min = -5;
l_max = 5;
l_free = log(1/9);

laserTopic = '/robot0/laser_1';
odomSub = rossubscriber('/robot0/odom');
laserSub = rossubscriber(laserTopic);
velPub = rospublisher('/robot0/cmd_vel', 'geometry_msgs/Twist');
velMsg = rosmessage(velPub);

% === LOOP PRINCIPAL ===
for i = 1:100
    % Obtener datos del robot
    [x0, y0, theta] = obtener_posicion(odomSub);
    [ranges, angleMin, angleInc] = obtener_laser(laserSub);

    % Actualizar mapa
    [logOddsMap, visitedMap] = actualizar_mapa_completo(x0, y0, theta, ranges, angleMin, angleInc, ...
        resolution, mapSize, logOddsMap, visitedMap, l_occ, l_free, l_min, l_max);

    visualizar_mapa(logOddsMap, visitedMap, x0, y0, resolution, mapSize, i);

    % Navegación evasiva
    velMsg = navegacion_evasion(ranges, angleMin, angleInc, safeDist, robotRadius, ...
        forwardSpeed, turnSpeed, velMsg);

    send(velPub, velMsg);
end

% === DETENER ROBOT ===
detener_robot(velPub, velMsg);

% === MOSTRAR MAPA FINAL ===
guardar_mapa_final(logOddsMap, visitedMap, resolution, mapSize);

function [x, y, theta] = obtener_posicion(odomSub)
    msg = receive(odomSub, 3);
    pose = msg.Pose.Pose;
    x = pose.Position.X;
    y = pose.Position.Y;
    quat = pose.Orientation;
    yaw = quat2eul([quat.W quat.X quat.Y quat.Z]);
    theta = yaw(1);
end

function [ranges, angleMin, angleInc] = obtener_laser(laserSub)
    msg = receive(laserSub, 3);
    ranges = msg.Ranges;
    angleMin = msg.AngleMin;
    angleInc = msg.AngleIncrement;
end

function [ix, iy] = world2map(x, y, resolution, mapHeight)
    ix = round(x / resolution) + 1;
    iy = mapHeight - round(y / resolution);
end

function [logOddsMap, visitedMap] = actualizar_mapa_completo(x0, y0, theta, ranges, angleMin, angleInc, ...
    resolution, mapSize, logOddsMap, visitedMap, l_occ, l_free, l_min, l_max)

    for r = 1:length(ranges)
        range = ranges(r);
        if range < 0.15 || range > 8.0
            continue;
        end

        angle = angleMin + (r-1) * angleInc;
        worldTheta = theta + angle;

        x1 = x0 + range * cos(worldTheta);
        y1 = y0 + range * sin(worldTheta);

        nSteps = ceil(range / resolution);
        for s = 1:nSteps-1
            xStep = x0 + s * resolution * cos(worldTheta);
            yStep = y0 + s * resolution * sin(worldTheta);
            [ixStep, iyStep] = world2map(xStep, yStep, resolution, mapSize);
            if ixStep > 0 && ixStep <= mapSize && iyStep > 0 && iyStep <= mapSize
                logOddsMap(iyStep, ixStep) = max(l_min, logOddsMap(iyStep, ixStep) + l_free);
                visitedMap(iyStep, ixStep) = true;
            end
        end

        [ix1, iy1] = world2map(x1, y1, resolution, mapSize);
        if ix1 > 0 && ix1 <= mapSize && iy1 > 0 && iy1 <= mapSize
            logOddsMap(iy1, ix1) = min(l_max, logOddsMap(iy1, ix1) + l_occ);
            visitedMap(iy1, ix1) = true;
        end
    end
end

function visualizar_mapa(logOddsMap, visitedMap ,x0, y0, resolution, mapSize, i)
    % Convertir log-odds a probabilidades
    probMap = 1 - (1 ./ (1 + exp(logOddsMap)));  % Convertir log-odds a probabilidad

    % Mostrar el mapa
    imagesc(flipud(probMap));                   % Flip para orientación correcta
    colormap(flipud(gray));                     % Libre = blanco, Ocupado = negro
    colorbar;
    set(gca, 'YDir', 'normal');
    axis equal tight;
    hold on;

    % Dibujar posición del robot
    plot((x0/resolution)+1, (y0/resolution)+1, 'bo', 'MarkerSize', 5, 'LineWidth', 2);

    % Obtener datos láser nuevamente solo para visualización de rayos
    laserSub = rossubscriber('/robot0/laser_1');
    scanMsg = receive(laserSub, 3);
    ranges = scanMsg.Ranges;
    angleMin = scanMsg.AngleMin;
    angleInc = scanMsg.AngleIncrement;

    quat = [1 0 0 0];  % Valor por defecto en caso de fallo
    odomSub = rossubscriber('/robot0/odom');
    odomMsg = receive(odomSub, 3);
    quat = odomMsg.Pose.Pose.Orientation;
    yaw = quat2eul([quat.W quat.X quat.Y quat.Z]);
    theta = yaw(1);

    minRange = 0.15;
    maxRange = 8.0;

    % Dibujar cada rayo válido
    for r = 1:length(ranges)
        range = ranges(r);
        if range < minRange || range > maxRange
            continue;
        end

        angle = angleMin + (r-1) * angleInc;
        worldTheta = theta + angle;

        x1 = x0 + range * cos(worldTheta);
        y1 = y0 + range * sin(worldTheta);

        plot([x0 x1]/resolution + 1, [y0 y1]/resolution + 1, 'r-');
    end

    title(sprintf('Mapa con Rayos LIDAR - Iteración %d', i));
    drawnow;
    hold off;
end


function velMsg = navegacion_evasion(ranges, angleMin, angleInc, safeDist, robotRadius, ...
    forwardSpeed, turnSpeed, velMsg)

    angles = angleMin + (0:length(ranges)-1) * angleInc;
    frontIdxs = find(angles >= -0.2 & angles <= 0.2);
    leftIdxs  = find(angles >= pi/4 & angles <= pi/2);
    rightIdxs = find(angles >= -pi/2 & angles <= -pi/4);

    frontRanges = ranges(frontIdxs);
    leftRanges  = ranges(leftIdxs);
    rightRanges = ranges(rightIdxs);

    frontClear = all(frontRanges > safeDist);
    leftTooClose = any(leftRanges < robotRadius);
    rightTooClose = any(rightRanges < robotRadius);

    if frontClear && ~leftTooClose && ~rightTooClose
        velMsg.Linear.X = forwardSpeed;
        velMsg.Angular.Z = 0;
    else
        if leftTooClose && ~rightTooClose
            velMsg.Linear.X = 0;
            velMsg.Angular.Z = -turnSpeed;
        elseif rightTooClose && ~leftTooClose
            velMsg.Linear.X = 0;
            velMsg.Angular.Z = turnSpeed;
        elseif ~frontClear
            if mean(leftRanges) > mean(rightRanges)
                velMsg.Linear.X = 0;
                velMsg.Angular.Z = turnSpeed;
            else
                velMsg.Linear.X = 0;
                velMsg.Angular.Z = -turnSpeed;
            end
        else
            velMsg.Linear.X = 0;
            velMsg.Angular.Z = turnSpeed;
        end
    end
end

function detener_robot(velPub, velMsg)
    velMsg.Linear.X = 0;
    velMsg.Angular.Z = 0;
    send(velPub, velMsg);
end

function guardar_mapa_final(logOddsMap, visitedMap, resolution, mapSize)
    probMap = 1 - (1 ./ (1 + exp(logOddsMap)));
    finalMap = 0.5 * ones(mapSize, mapSize);
    finalMap(visitedMap & probMap < 0.3) = 0;
    finalMap(visitedMap & probMap > 0.7) = 1;
    figure;
    imagesc(finalMap);
    colormap([1 1 1; 0.5 0.5 0.5; 0 0 0]);
    caxis([0 1]); 
    axis equal tight;
    title('Mapa final (0: libre, 0.5: desconocido, 1: ocupado)');
    colorbar('Ticks',[0 0.5 1], 'TickLabels', {'Libre','Desconocido','Ocupado'});

    binaryMap = logOddsMap > 0;
    mapObj = binaryOccupancyMap(binaryMap, 1/resolution);
    figure;
    show(mapObj);
    title('Mapa final basado en colisiones de láser');
    save('mapa_final_laser.mat', 'mapObj');

    [ixGrid, iyGrid] = find(~visitedMap);
    xCoords = (ixGrid - 1) * resolution;
    yCoords = (mapSize - iyGrid) * resolution;
    unvisitedWorldCoords = [xCoords, yCoords];
    save('celdas_no_visitadas.mat', 'unvisitedWorldCoords');

    figure;
    scatter(xCoords, yCoords, 'k.');
    title('Celdas no visitadas en coordenadas del mundo');
    xlabel('X (m)');
    ylabel('Y (m)');
    axis equal tight;
end

