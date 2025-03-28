%% INICIALIZACIÓN DE ROS (COMPLETAR ESPACIOS CON LAS DIRECCIONES IP)
setenv('ROS_MASTER_URI','http://192.168.991.991:11311');  
setenv('ROS_IP','192.168.199.199'); 

rosinit() % Inicialización de ROS en la IP correspondiente

%% DECLARACIÓN DE VARIABLES NECESARIAS PARA EL CONTROL
D_deseado = 0.5; % Distancia deseada a la pared (en metros)
Kp_dist = 1.0; % Ganancia proporcional para el error de distancia
Kp_ang = 1.5; % Ganancia proporcional para el error de orientación
V_lineal = 0.3; % Velocidad lineal constante

MAX_TIME = 1000; % Máximo número de iteraciones
medidas = zeros(5, MAX_TIME); % Matriz para almacenar datos

%% DECLARACIÓN DE SUBSCRIBERS Y PUBLISHERS
odom = rossubscriber('/robot0/odom'); % Subscripción a la odometría
sonar0 = rossubscriber('/robot0/sonar_0', rostype.sensor_msgs_Range); % Subscripción al sensor ultrasónico

pub = rospublisher('/robot0/cmd_vel', 'geometry_msgs/Twist'); % Publicador de velocidad
msg_vel = rosmessage(pub); % Mensaje de velocidad

%% Definimos la periodicidad del bucle (10 Hz)
r = robotics.Rate(10);
waitfor(r);

pause(3); % Esperamos para recibir datos de odometría

%% Bucle de control
i = 0;  
lastdist = 0;
lastdistav = 0;

while true
    i = i + 1; 
    
    %% Obtener la posición y medidas del sonar
    pos = odom.LatestMessage.Pose.Pose.Position;  
    ori = odom.LatestMessage.Pose.Pose.Orientation;     
    yaw = quat2eul([ori.W ori.X ori.Y ori.Z]); 
    yaw = yaw(1); % Ángulo en radianes
    
    msg_sonar0 = receive(sonar0);
    dist = msg_sonar0.Range; % Medida del sensor
    
    % Limitamos la distancia máxima medida por el sensor
    if dist > 5
        dist = 5; 
    end
    
    %% Calcular la distancia avanzada (delta entre iteraciones)
    distav = sqrt((pos.X - lastdistav)^2 + (pos.Y - lastdist)^2);
    
    %% Calcular errores de control
    Eori = atan2(dist - lastdist, distav); % Error de orientación
    Edist = dist - D_deseado; % Error de distancia a la pared
    
    %% Almacenar datos para análisis
    medidas(1, i) = dist; 
    medidas(2, i) = lastdist; 
    medidas(3, i) = distav; 
    medidas(4, i) = Eori; 
    medidas(5, i) = Edist; 
    
    %% Calcular consignas de velocidad
    consigna_vel_linear = V_lineal; % Velocidad lineal fija
    consigna_vel_ang = Kp_dist * Edist + Kp_ang * Eori; % Control proporcional
    
    %% Condición de parada (cuando errores son pequeños)
    if (abs(Edist) < 0.01) && (abs(Eori) < 0.01)
        break;
    end
    
    %% Aplicar consignas de control
    msg_vel.Linear.X = consigna_vel_linear;
    msg_vel.Angular.Z = consigna_vel_ang;
    
    % Enviar comando de velocidad
    send(pub, msg_vel);
    
    %% Guardar valores actuales para la siguiente iteración
    lastdist = dist;
    lastdistav = distav;
    
    % Esperar al siguiente ciclo
    waitfor(r);
    
    %% Salir si se alcanzó el número máximo de iteraciones
    if i == MAX_TIME
        break;
    end
end

%% DETENER EL ROBOT Y DESCONEXIÓN DE ROS
msg_vel.Linear.X = 0;
msg_vel.Angular.Z = 0;
send(pub, msg_vel);

save('medidas.mat', 'medidas'); % Guardar datos de simulación

rosshutdown;
