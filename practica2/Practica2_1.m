%% INICIALIZACIÓN DE ROS
setenv('ROS_MASTER_URI','http://192.168.991.991:11311');  
setenv('ROS_IP','192.168.199.199'); 
rosinit()

%% PARÁMETROS DEL CONTROLADOR
D_deseado = 0.5;             % Distancia deseada a la pared
Kp_dist = 1.0;               % Ganancia proporcional del error de distancia
Kp_ang = 1.5;                % Ganancia para corrección angular
Kd_curv = 1.0;               % Nueva: Ganancia derivativa para curvatura local
V_lineal = 0.3;              % Velocidad lineal constante

MAX_TIME = 1000;
medidas = zeros(6, MAX_TIME);  % Matriz para almacenar datos

%% SUBSCRIBERS Y PUBLISHERS
odom = rossubscriber('/robot0/odom');
sonar0 = rossubscriber('/robot0/sonar_0', rostype.sensor_msgs_Range);
pub = rospublisher('/robot0/cmd_vel', 'geometry_msgs/Twist');
msg_vel = rosmessage(pub);

%% BUCLE DE CONTROL
r = robotics.Rate(10);
waitfor(r);
pause(3);

i = 0;
dist_hist = [0 0];  % Historial de distancias para estimar curvatura
pos_hist = [0 0];   % Posiciones anteriores para desplazamiento

while i < MAX_TIME
    i = i + 1;

    %% Datos de sensores
    pos = odom.LatestMessage.Pose.Pose.Position;
    ori = odom.LatestMessage.Pose.Pose.Orientation;
    yaw = quat2eul([ori.W ori.X ori.Y ori.Z]);
    yaw = yaw(1);

    msg_sonar0 = receive(sonar0);
    dist = min(msg_sonar0.Range, 5);  % Límite del sensor

    %% Estimación de desplazamiento avanzado
    dx = pos.X - pos_hist(1);
    dy = pos.Y - pos_hist(2);
    dist_avance = hypot(dx, dy);

    %% Estimación de curvatura local
    delta_dist = dist - dist_hist(1);
    curvatura_local = atan2(delta_dist, dist_avance + 1e-6);  % Evita división por cero

    %% Errores
    Edist = dist - D_deseado;
    Eori = curvatura_local;  % Error de orientación basado en cambio de distancias

    %% Control adaptativo
    consigna_vel_linear = V_lineal;
    consigna_vel_ang = Kp_dist * Edist + Kp_ang * Eori + Kd_curv * (dist_hist(1) - dist_hist(2));

    %% Aplicar velocidades
    msg_vel.Linear.X = consigna_vel_linear;
    msg_vel.Angular.Z = consigna_vel_ang;
    send(pub, msg_vel);

    %% Guardar datos
    medidas(:, i) = [dist; dist_hist(1); dist_avance; Eori; Edist; consigna_vel_ang];

    %% Actualización de historial
    dist_hist = [dist dist_hist(1)];
    pos_hist = [pos.X pos.Y];

    %% Condición de parada si está estable
    if abs(Edist) < 0.01 && abs(Eori) < 0.01 && dist_avance < 0.001
        break;
    end

    waitfor(r);
end

%% DETENER ROBOT
msg_vel.Linear.X = 0;
msg_vel.Angular.Z = 0;
send(pub, msg_vel);

save('medidas_curvas.mat', 'medidas');
rosshutdown;
