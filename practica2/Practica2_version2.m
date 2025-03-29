% Posicion actual del robot (1,1)
%% INICIALIZACIÓN DE ROS (COMPLETAR ESPACIOS CON LAS DIRECCIONES IP)
setenv('ROS_MASTER_URI','http://192.168.990.901:11311');
setenv('ROS_IP','192.168.990.900');
rosinit() % Inicialización de ROS en la IP correspondiente
%% DECLARACIÓN DE VARIABLES NECESARIAS PARA EL CONTROL
umbral_distancia = 0.1; % Margen de error en metros
umbral_angulo = 0.05; % Margen de error en radianes
Kp_dist = 0.5; % Ganancia proporcional para control de distancia
Kp_ang = 1.0; % Ganancia proporcional para control de orientación
vel_linear_max = 0.5; % Velocidad lineal máxima reducida para mayor
% precision
vel_angular_max = 0.3; % Velocidad angular máxima reducida
%% DECLARACIÓN DE SUBSCRIBERS Y PUBLISHERS
odom = rossubscriber('/robot0/odom'); % Subscripción a la odometría
pub = rospublisher('/robot0/cmd_vel', 'geometry_msgs/Twist'); %
Publicador de velocidades
msg_vel = rosmessage(pub); % Mensaje de velocidad
%% DEFINICIÓN DE PUNTO DESTINO (INGRESADO POR USUARIO)
x_destino = input('Ingrese la coordenada X del destino: ');
y_destino = input('Ingrese la coordenada Y del destino: ');
%% Definimos la periodicidad del bucle (10 Hz)
r = robotics.Rate(10);
waitfor(r);
pause(3); % Esperamos para recibir datos de odometría
%% Bucle de control
distancia_anterior = Inf; % Inicializamos con distancia infinita
se_acerca = true; % Flag para indicar si se está acercando al objetivo
while true
% Obtener la posición y orientación actuales
pos = odom.LatestMessage.Pose.Pose.Position;
ori = odom.LatestMessage.Pose.Pose.Orientation;
yaw = quat2eul([ori.W ori.X ori.Y ori.Z]);
yaw = yaw(1); % Ángulo en radianes
% Cálculo del error de distancia
Edist = sqrt((x_destino - pos.X)^2 + (y_destino - pos.Y)^2);
% Verificar si estamos empezando a alejarnos del objetivo
if Edist > distancia_anterior + 0.05 % Si la distancia aumenta
significativamente
se_acerca = false;
end
distancia_anterior = Edist;
% Cálculo del error de orientación
ang_deseado = atan2(y_destino - pos.Y, x_destino - pos.X);
Eori = ang_deseado - yaw;

% Ajuste del error de orientación para mantenerse en el rango [-pi,pi]
while Eori > pi
Eori = Eori - 2*pi;
end
while Eori < -pi
Eori = Eori + 2*pi;
end
% Control proporcional de velocidad lineal y angular con saturación
consigna_vel_linear = min(Kp_dist * Edist, vel_linear_max);
consigna_vel_ang = min(Kp_ang * Eori, vel_angular_max);
% Reducir velocidad cuando esté cerca del objetivo
if Edist < 2*umbral_distancia
consigna_vel_linear = consigna_vel_linear * 0.5;
consigna_vel_ang = consigna_vel_ang * 0.7;
end
% Condiciones de parada mejoradas
if (Edist < umbral_distancia) || ~se_acerca
% Si estamos dentro del umbral o nos estamos alejando del
objetivo
msg_vel.Linear.X = 0;
msg_vel.Angular.Z = 0;
send(pub, msg_vel);
break;
end
% Aplicar consignas de control
msg_vel.Linear.X = consigna_vel_linear;
msg_vel.Angular.Z = consigna_vel_ang;
% Enviar comando de velocidad
send(pub, msg_vel);
% Mostrar información de diagnóstico
fprintf('Distancia: %.2f m, Error ang: %.2f rad, Vel Lin: %.2f, Vel Ang: %.2f\n', ...
Edist, Eori, msg_vel.Linear.X, msg_vel.Angular.Z);
% Esperar al siguiente ciclo
waitfor(r);
end
%% DETENER EL ROBOT Y DESCONEXIÓN DE ROS
msg_vel.Linear.X = 0;
msg_vel.Angular.Z = 0;
send(pub, msg_vel);
% Mostrar mensaje de finalización
fprintf('Robot ha llegado al destino o se ha detenido cerca de él.\n');
fprintf('Posición final: X=%.2f, Y=%.2f\n', pos.X, pos.Y);
% rosshutdown;