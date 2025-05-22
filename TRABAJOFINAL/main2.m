% === Configuración ROS ===
rosshutdown;
setenv('ROS_MASTER_URI','http://192.168.0.24:11311');  
setenv('ROS_IP','192.168.0.22'); 
rosinit();

% === Parámetros del mapa y navegación ===
resolution = 0.01;
mapSize = 15 / resolution;
logOddsMap = zeros(mapSize, mapSize);

l_occ = log(9);
l_min = -5;
l_max = 5;

% === Suscripciones ROS ===
odomSub = rossubscriber('/robot0/odom');
laserSub = rossubscriber('/robot0/laser_1');


    % Obtener posición del robot
    odomMsg = receive(odomSub, 3);
    pose = odomMsg.Pose.Pose;
    x0 = pose.Position.X;
    y0 = pose.Position.Y;
    quat = pose.Orientation;
    yaw = quat2eul([quat.W quat.X quat.Y quat.Z]);
    theta = yaw(1);

    % Recibir datos del láser
    scanMsg = receive(laserSub, 3);

    % Actualizar mapa
    logOddsMap = actualizarMapaLaser(logOddsMap, x0, y0, theta, scanMsg, resolution, l_occ, l_max, mapSize);

    % Visualizar mapa (ACLARACION LA i ES EL NUMERO DE ITERACIONES QUE ESTAS HACIENDO AUNQUE ESTA FUNCION NO ES NECESARIA IMPLEMENTARLA PORQUE SOLO TE HACE VER LA VENTANITA ESA)
    visualizarMapa(logOddsMap, x0, y0, resolution, mapSize, i);


guardarMapaFinal(logOddsMap, resolution, 'mapa_final_laser.mat', 'Mapa final basado en colisiones de láser');

