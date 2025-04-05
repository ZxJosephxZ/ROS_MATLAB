
clear;
clc;
init_ros(); % Inicialización de ROS

% Definir parámetros
Kp_dist = 0.5;
Kp_ang = 1.0;
vel_linear_max = 0.5;
vel_angular_max = 0.3;
umbral_distancia = 0.1;

% Suscriptores y publicadores
odom = rossubscriber('/robot0/odom');
pub = rospublisher('/robot0/cmd_vel', 'geometry_msgs/Twist');
msg_vel = rosmessage(pub);

% Pedir coordenadas de destino
x_destino = input('Ingrese la coordenada X del destino: ');
y_destino = input('Ingrese la coordenada Y del destino: ');

r = robotics.Rate(10);
waitfor(r);
pause(3); % Esperar datos de odometría

distancia_anterior = Inf;
se_acerca = true;

while true
    [pos, yaw] = get_position_orientation(odom);
    [Edist, Eori] = compute_errors(x_destino, y_destino, pos, yaw);
    
    if Edist > distancia_anterior + 0.05
        se_acerca = false;
    end
    distancia_anterior = Edist;
    
    [v_lin, v_ang] = compute_control(Edist, Eori, Kp_dist, Kp_ang, vel_linear_max, vel_angular_max);
    
    if (Edist < umbral_distancia) || ~se_acerca
        stop_robot(pub, msg_vel);
        break;
    end
    
    send_velocity(pub, msg_vel, v_lin, v_ang);
    fprintf('Dist: %.2f m, Error ang: %.2f rad, Vel Lin: %.2f, Vel Ang: %.2f\n', ...
        Edist, Eori, v_lin, v_ang);
    
    waitfor(r);
end

function init_ros()
    setenv('ROS_MASTER_URI', 'http://192.168.990.901:11311');
    setenv('ROS_IP', '192.168.990.900');
    rosinit(); % Inicialización de ROS
end
function [pos, yaw] = get_position_orientation(odom)
    pose = odom.LatestMessage.Pose.Pose;
    pos = pose.Position;
    ori = pose.Orientation;
    yaw = quat2eul([ori.W ori.X ori.Y ori.Z]);
    yaw = yaw(1); % Extraer solo el ángulo en radianes
end
function [Edist, Eori] = compute_errors(x_destino, y_destino, pos, yaw)
    Edist = sqrt((x_destino - pos.X)^2 + (y_destino - pos.Y)^2);
    ang_deseado = atan2(y_destino - pos.Y, x_destino - pos.X);
    Eori = ang_deseado - yaw;
    Eori = adjust_orientation_error(Eori); % Ajuste de error
end
function Eori = adjust_orientation_error(Eori)
    while Eori > pi
        Eori = Eori - 2*pi;
    end
    while Eori < -pi
        Eori = Eori + 2*pi;
    end
end
function [v_lin, v_ang] = compute_control(Edist, Eori, Kp_dist, Kp_ang, vel_linear_max, vel_angular_max)
    v_lin = min(Kp_dist * Edist, vel_linear_max);
    v_ang = min(Kp_ang * Eori, vel_angular_max);
    
    % Reducción de velocidad al acercarse al objetivo
    if Edist < 0.2
        v_lin = v_lin * 0.5;
        v_ang = v_ang * 0.7;
    end
end
function send_velocity(pub, msg_vel, v_lin, v_ang)
    msg_vel.Linear.X = v_lin;
    msg_vel.Angular.Z = v_ang;
    send(pub, msg_vel);
end
function stop_robot(pub, msg_vel)
    msg_vel.Linear.X = 0;
    msg_vel.Angular.Z = 0;
    send(pub, msg_vel);
    fprintf('Robot detenido.\n');
    rosshutdown;
end