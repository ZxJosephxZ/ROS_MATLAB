%%  

rosshutdown; 

%% 

setenv('ROS_MASTER_URI','http://192.168.0.101:11311'); 

setenv('ROS_IP','192.168.0.100'); 

rosinit();

%% DECLARACIÓN DE SUBSCRIBER 

% suscriber a ODOM 

odom = rossubscriber('/robot0/odom'); % Suscribirse a la odometría 

pause(2) % Espera unos segundos para recibir datos 

% suscriber a SONAR 

sonar0 = rossubscriber('/robot0/sonar_0'); % Suscribirse a la sonar 

pause(2) % Espera unos segundos para recibir datos 

sonar1 = rossubscriber('/robot0/sonar_1'); % Suscribirse a la sonar 

pause(2) % Espera unos segundos para recibir datos 

sonar2 = rossubscriber('/robot0/sonar_2'); % Suscribirse a la sonar 

pause(2) % Espera unos segundos para recibir datos 

sonar3 = rossubscriber('/robot0/sonar_3'); % Suscribirse a la sonar 

pause(2) % Espera unos segundos para recibir datos 

sonar4 = rossubscriber('/robot0/sonar_4'); % Suscribirse a la sonar 

pause(2) % Espera unos segundos para recibir datos 

sonar5 = rossubscriber('/robot0/sonar_5'); % Suscribirse a la sonar 

pause(2) % Espera unos segundos para recibir datos 

sonar6 = rossubscriber('/robot0/sonar_6'); % Suscribirse a la sonar 

pause(2) % Espera unos segundos para recibir datos 

sonar7 = rossubscriber('/robot0/sonar_7'); % Suscribirse a la sonar 

pause(2) % Espera unos segundos para recibir datos 

%% Suscribirse al sensor LIDAR 

laser = rossubscriber('/robot0/laser_1');  

pause(2); % Espera a que lleguen datos 

%% DECLARACIÓN DE PUBLISHERS 

pub = rospublisher('/robot0/cmd_vel', 'geometry_msgs/Twist'); 

%% Obtener la posición actual del robot 

pos = odom.LatestMessage.Pose.Pose.Position; 

disp(['Posición actual: x=', num2str(pos.X), ', y=', num2str(pos.Y), ', z=', num2str(pos.Z)]); 

%msg = odom.LatestMessage; % Obtener el último mensaje recibido 

%disp(msg); % Mostrar el mensaje en la consola 

%% GENERACIÓN DE MENSAJE 

msg=rosmessage(pub); %% Creamos un mensaje del tipo declarado en "pub" (geometry_msgs/Twist) 

% Rellenamos los campos del mensaje para que el robot avance a 0.2 m/s 

% Velocidades lineales en x,y y z (velocidades en y o z no se usan en robots diferenciales y entornos 2D) 

msg.Linear.X=0; 

msg.Linear.Y=0; 

msg.Linear.Z=0; 

% Velocidades angulares (en robots diferenciales y entornos 2D solo se utilizará el valor Z) 

msg.Angular.X=0; 

msg.Angular.Y=0; 

msg.Angular.Z=0; 

% Definimos la perodicidad del bucle (10 hz) 

r = robotics.Rate(10); 

% Bucle de control infinito 

% Capturar posición inicial 

msg1 = odom.LatestMessage; 

x1 = msg1.Pose.Pose.Position.X; 

y1 = msg1.Pose.Pose.Position.Y; 

yaw1 = msg1.Pose.Pose.Orientation.Z; 

for i = 1:10 % Lo ejecutamos 10 veces, para 1 segundos a 10 Hz 

send(pub, msg); 

waitfor(r); % Esperar hasta el siguiente ciclo 

end 

% Capturar posición final 

msg2 = odom.LatestMessage; 

x2 = msg2.Pose.Pose.Position.X; 

y2 = msg2.Pose.Pose.Position.Y; 

yaw2 = msg2.Pose.Pose.Orientation.Z; 

% Calcular resolución como diferencia mínima detectada 

dx = abs(x2 - x1); 

dy = abs(y2 - y1); 

dyaw = abs(yaw2 - yaw1); 

% Guardar valores en las listas 

q_lineal = sqrt(dx^2 + dy^2); 

q_angular = dyaw; 

% Detener el robot antes de la siguiente prueba 

msg.Linear.X = 0; 

msg.Angular.Z = 0; 

send(pub, msg); 

disp(['q_lineal = ', num2str(q_lineal), ', q_angular = ', num2str(q_angular)]); 

%% Captura de datos de sonar a dos metros 

num_medidas = 1000; % Número de mediciones 

medidas = zeros(1, num_medidas); % Vector para almacenar mediciones 

medidas_filtradas = zeros(1, num_medidas); % Vector para la media móvil 

for i = 1:num_medidas 

msg = sonar5.LatestMessage; % Obtener la medición más reciente 

medidas(i) = msg.Range_; % Guardar la distancia medida 

pause(0.01); % Pequeña pausa para evitar saturación 

% Aplicar filtro de media móvil (promedio de los últimos 5 valores) 

if i >= 5 

medidas_filtradas(i) = mean(medidas(i-4:i)); 

else 

medidas_filtradas(i) = medidas(i); % No hay suficientes valores aún 

end 

pause(0.01); % Pequeña pausa para evitar saturación 

end 

%% Graficar las mediciones originales y filtradas del sonar 

figure; 

plot(medidas, 'r', 'DisplayName', 'Medición Original'); hold on; 

plot(medidas_filtradas, 'b', 'DisplayName', 'Media Móvil (5 valores)'); 

xlabel('Número de medición'); 

ylabel('Distancia medida (m)'); 

title('Filtro de Media Móvil en Sensor Sonar'); 

legend; 

grid on; 

hold off; 

%% Análisis estadístico de las medidas del sonar 

max_valor = max(medidas); 

media_valor = mean(medidas); 

varianza_original = var(medidas); 

varianza_filtrada = var(medidas_filtradas); 

disp(['Varianza sin filtrar: ', num2str(varianza_original)]); 

disp(['Varianza filtrada: ', num2str(varianza_filtrada)]); 

disp(['Valor máximo: ', num2str(max_valor)]); 

disp(['Valor medio: ', num2str(media_valor)]); 

%% Captura de 1000 medidas desde un ángulo específico del LIDAR a 2 metros 

num_medidas = 1000; % Número de mediciones 

angulo_interes = 2; % Índice del ángulo en el array de rangos (ajustar según sea necesario) 

medidas = zeros(1, num_medidas); % Vector para almacenar mediciones 

medidas_filtradas = zeros(1, num_medidas); % Vector para almacenar valores filtrados 

for i = 1:num_medidas 

msg = laser.LatestMessage; % Obtener medición actual 

medidas(i) = msg.Ranges(angulo_interes); % Obtener la distancia en el ángulo seleccionado 

pause(0.01); % Esperar 10ms para evitar saturación 

% Aplicar filtro de media móvil (promedio de los últimos 5 valores) 

if i >= 5 

medidas_filtradas(i) = mean(medidas(i-4:i)); 

else 

medidas_filtradas(i) = medidas(i); % No hay suficientes valores aún 

end 

end 

%% Graficar las mediciones originales y filtradas del LIDAR 

figure; 

plot(medidas, 'r', 'DisplayName', 'Medición Original'); hold on; 

plot(medidas_filtradas, 'b', 'DisplayName', 'Media Móvil (5 valores)'); 

xlabel('Número de medición'); 

ylabel('Distancia medida (m)'); 

title('Filtro de Media Móvil en Sensor LIDAR'); 

legend; 

grid on; 

hold off; 

%% Análisis estadístico de las medidas del LIDAR 

max_valor = max(medidas); 

media_valor = mean(medidas); 

varianza_original = var(medidas); 

varianza_filtrada = var(medidas_filtradas); 

disp(['Varianza sin filtrar: ', num2str(varianza_original)]); 

disp(['Varianza filtrada: ', num2str(varianza_filtrada)]); 

disp(['Valor máximo: ', num2str(max_valor)]); 

disp(['Valor medio: ', num2str(media_valor)]); 

%% Obtener datos de relevantes para detectar paredes LIDAR 

angulo_del1 = 195; % Índice del ángulo en el array de rangos (ajustar según sea necesario) 

angulo_del2 = 205; 

angulo_tra1 = 5; 

angulo_tra2 = 395; 

angulo_izq1=95; 

angulo_izq2=105; 

angulo_der1 = 295; 

angulo_der2 = 305; 

%msg = laser.LatestMessage; 
msg = laser.LatestMessage;
if isempty(msg) || isempty(msg.Ranges)
    disp('Error: No se están recibiendo datos del LIDAR.');
    return;
end
disp(['Número total de lecturas LIDAR: ', num2str(length(msg.Ranges))]);
%
%dist_frente1 = msg.Ranges(angulo_del1); 
dist_frente1 = msg.Ranges(angulo_del1);
if isnan(dist_frente1) || isinf(dist_frente1)
    dist_frente1 = 10; % Valor por defecto si no hay detección
end
%
dist_frente2 = msg.Ranges(angulo_del2); 

dist_atras1 = msg.Ranges(angulo_tra1); 

dist_atras2 = msg.Ranges(angulo_tra2); 

dist_izq1 = msg.Ranges(angulo_izq1); 

dist_izq2 = msg.Ranges(angulo_izq2); 

dist_der1 = msg.Ranges(angulo_der1); 

dist_der2 = msg.Ranges(angulo_der2); 

angulo_del1 = 175; % Índice del ángulo en el array de rangos (ajustar según sea necesario) 

angulo_del2 = 185; 

angulo_tra1 = 5; 

angulo_tra2 = 355; 

angulo_izq1 = 85; 

angulo_izq2 = 95; 

angulo_der1 = 265; 

angulo_der2 = 275; 

%% Calcular confianza LASER 

m_pared_del = detectar_pendiente(dist_frente1, dist_frente2, angulo_del1, angulo_del2); 

m_pared_tras = detectar_pendiente(dist_atras1, dist_atras2, angulo_tra1, angulo_tra2); 

m_pared_izq = detectar_pendiente(dist_izq1, dist_izq2, angulo_izq1, angulo_izq2); 

m_pared_der = detectar_pendiente(dist_der1, dist_der2, angulo_der1, angulo_der2); 

Q1 = calidad_paralelas(m_pared_del, m_pared_tras); 

Q2 = calidad_paralelas(m_pared_izq, m_pared_der); 

disp(['Confianza en paralelidad de paredes delantera y trasera: ', num2str(Q1*100), '%']); 

disp(['Confianza en paralelidad de paredes izquierda y derecha: ', num2str(Q2*100), '%']); 

%% Obtener codigo de paredes con sensor LIDAR 

cod=detectar_paredes_lidar(dist_frente1, dist_frente2,dist_atras1, dist_atras2, dist_izq1, dist_izq2, dist_der1, dist_der2, angulo_del1, angulo_del2, angulo_tra1, angulo_tra2, angulo_izq1, angulo_izq2, angulo_der1, angulo_der2); 

disp(['Codigo del numero de paredes identificadas con LIDAR: ', num2str(cod)]); 

%% Obtener datos de relevantes para detectar paredes SONAR 

dist_izq = sonar0.LatestMessage.Range_; % 90° 

dist_der = sonar5.LatestMessage.Range_; % 180º 

dist_frente1 = sonar2.LatestMessage.Range_; % 10° 

dist_frente2 = sonar3.LatestMessage.Range_; % 350° 

dist_atras1 = sonar6.LatestMessage.Range_; % 225° 

dist_atras2 = sonar7.LatestMessage.Range_; % 135° 

angulo1=10; 

angulo2=350; 

angulo3=225; 

angulo4=135; 

%% Detectar codigo de paredes con SONAR 

cod=detectar_paredes(dist_izq, dist_der, dist_frente1, dist_frente2, dist_atras1, dist_atras2, angulo1, angulo2, angulo3, angulo4); 

disp(['Codigo del numero de paredes identificadascon SONAR: ', num2str(cod)]); 

%% Calcular confianza SONAR 

m_pared_del = detectar_pendiente(dist_frente1, dist_frente2, angulo1, angulo2); 

m_pared_tras = detectar_pendiente(dist_frente1, dist_frente2, angulo1, angulo2); 

Q = calidad_paralelas(m_pared_del, m_pared_tras); 

disp(['Confianza en paralelidad de paredes: ', num2str(Q*100), '%']); 

%% Recorrido programado del amibot; 

% Capturar posición inicial 

msg_odom = odom.LatestMessage; 

x_inicial = msg_odom.Pose.Pose.Position.X; 

y_inicial = msg_odom.Pose.Pose.Position.Y; 

% Capturar la orientación inicial 

msg_odom = odom.LatestMessage; 

yaw_inicial = obtener_yaw(msg_odom); % Obtener ángulo en grados 

% Ejecución del recorrido 

avanzar(2); % 1. Avanzar 2 metros 

girar(90); % 2. Girar 90º izquierda 

avanzar(1); % 3. Avanzar 1 metro 

girar(-90); % 4. Girar -90º derecha 

avanzar(1); % 5. Avanzar 1 metro 

% Movimiento 1: Avanzar 2 metros 

x1 = x_inicial + 2 * cos(yaw_inicial); 

y1 = y_inicial + 2 * sin(yaw_inicial); 

% Movimiento 2: Girar 90º izquierda (yaw aumenta en pi/2 radianes) 

yaw2 = yaw_inicial + 90; 

% Movimiento 3: Avanzar 1 metro 

x2 = x1 + 1 * cos(yaw2); 

y2 = y1 + 1 * sin(yaw2); 

% Movimiento 4: Girar -90º derecha (yaw disminuye en pi/2 radianes) 

yaw3 = yaw2 - 90; 

% Movimiento 5: Avanzar 1 metro 

x_esperado = x2 + 1 * cos(yaw3); 

y_esperado = y2 + 1 * sin(yaw3); 

% Capturar posición final 

msg_odom = odom.LatestMessage; 

x_final = msg_odom.Pose.Pose.Position.X; 

y_final = msg_odom.Pose.Pose.Position.Y; 

% Cálculo del error global de odometría 

error_x = abs(x_final - x_esperado); 

error_y = abs(y_final - y_esperado); 

error_total = sqrt(error_x^2 + error_y^2); 

% Mostrar resultados 

disp(['Error en X: ', num2str(error_x), ' m']); 

disp(['Error en Y: ', num2str(error_y), ' m']); 

disp(['Error total de odometría: ', num2str(error_total), ' m']); 

%% FUNCION AVANZAR 

function avanzar(distancia) 

disp('avanzando'); 

odom = rossubscriber('/robot0/odom'); 

pub = rospublisher('/robot0/cmd_vel', 'geometry_msgs/Twist'); 

% Esperar para recibir los primeros datos 

pause(2); 

% Capturar la posición inicial 

tic;
while toc < 2
    msg_odom = odom.LatestMessage;
    x_inicial = msg_odom.Pose.Pose.Position.X;
    y_inicial = msg_odom.Pose.Pose.Position.Y;
    pause(0.1);
end

% Crear el mensaje de velocidad 

msg = rosmessage(pub); 

msg.Linear.X = 0.2; % Velocidad de avance (ajustable) 

% Definir la tasa de actualización (10 Hz) 

r = robotics.Rate(10); 

% Bucle de control para avanzar hasta la distancia deseada 

while true 

% Enviar el comando de movimiento 

send(pub, msg); 

% Obtener la posición actual 

msg_odom = odom.LatestMessage; 

x_actual = msg_odom.Pose.Pose.Position.X; 

y_actual = msg_odom.Pose.Pose.Position.Y; 

% Calcular la distancia recorrida 

distancia_recorrida = sqrt((x_actual - x_inicial)^2 + (y_actual - y_inicial)^2); 

% Comprobar si ha alcanzado la distancia deseada 

if distancia_recorrida >= distancia 

break; 

end 

% Esperar hasta la siguiente iteración 

waitfor(r); 

end 

% Detener el robot 

msg.Linear.X = 0; 

send(pub, msg); 

disp(['Robot ha avanzado ', num2str(distancia), ' metros.']); 

end 

%% FUNCION GIRAR 

function girar(angulo) 

% Función para girar el AmigoBot un cierto ángulo en grados 

% Suscribirse a la odometría y crear el publicador de velocidad 

odom = rossubscriber('/robot0/odom'); 

pub = rospublisher('/robot0/cmd_vel', 'geometry_msgs/Twist'); 

% Esperar a recibir datos iniciales 

pause(2); 

% Capturar la orientación inicial 

tic;
while toc < 2
    msg_odom = odom.LatestMessage;
    yaw_inicial = obtener_yaw(msg_odom);
    pause(0.1);
end

% Crear mensaje de giro 

msg = rosmessage(pub); 

% Determinar dirección del giro 

if angulo >= 0 

msg.Angular.Z = 0.5; % Giro a la izquierda 

else 

msg.Angular.Z = -0.5; % Giro a la derecha 

end 

% Definir tasa de actualización 

r = robotics.Rate(10); 

while true 

% Enviar comando de giro 

send(pub, msg); 

% Obtener la orientación actual 

msg_odom = odom.LatestMessage; 

yaw_actual = obtener_yaw(msg_odom); 

% Calcular diferencia de ángulo 

diferencia_yaw = abs(yaw_actual - yaw_inicial); 

% Comprobar si se ha alcanzado el ángulo deseado 

if diferencia_yaw >= abs(angulo) 

break; 

end 

waitfor(r); 

end 

% Detener el robot 

msg.Angular.Z = 0; 

send(pub, msg); 

disp(['Robot ha girado ', num2str(angulo), ' grados.']); 

end 

% FUNCION obtener_yaw 

function yaw = obtener_yaw(msg_odom) 

% Extrae el ángulo Yaw desde la odometría en un entorno 2D 

q = msg_odom.Pose.Pose.Orientation; 

yaw = atan2(2.0 * (q.W * q.Z + q.X * q.Y), 1.0 - 2.0 * (q.Y^2 + q.Z^2)); 

yaw = rad2deg(yaw); % Convertir a grados 

% Normalizar yaw entre 0 y 360 

if yaw < 0 

yaw = yaw + 360; 

end 

end 

%% FUNCION OBTENER Q 

function Q = calidad_paralelas(m1, m2) 

% Convertir NaN a infinito 

if isnan(m1) 

m1 = Inf; 

end 

if isnan(m2) 

m2 = Inf; 

end 

% Manejar caso especial: 

if isinf(m1) && isinf(m2) 

error_paralelo = 0; % Son paralelas perfectamente (ambas verticales) 

elseif isinf(m1) || isinf(m2) 

error_paralelo = Inf; % Si una es vertical y la otra no, son completamente diferentes 

else 

error_paralelo = abs(abs(m1) - abs(m2)); % Diferencia normal entre pendientes 

end 

% Definir función de calidad 

Q = 1 / (1 + error_paralelo); 

end 

%% FUNCION DETECTAR PARED 

function existe = detectar_pared(dist1,dist2,angulo1,angulo2) 

[x1, y1] = calcular_coordenadas(dist1, angulo1); 

[x2, y2] = calcular_coordenadas(dist2, angulo2); 

% Tomar el valor absoluto de las coordenadas 

x1 = abs(x1); 

y1 = abs(y1); 

x2 = abs(x2); 

y2 = abs(y2); 

% Calcular la pendiente en valor absoluto 

m = abs((y2 - y1) / (x2 - x1)); 

% Comprobar si la recta es horizontal o vertical 

if abs(m) < 1 % Pendiente muy pequeña ≈ 0 

existe=0; 

elseif abs(x2 - x1) <= 1 % Diferencia en X ≈ 0 

existe=1;  

else 

existe=0; 

end 

end 

% Función auxiliar para calcular coordenadas a partir de distancia y ángulo 

function [x, y] = calcular_coordenadas(d, theta) 

theta_rad = deg2rad(theta); 

x = d * cos(theta_rad); 

y = d * sin(theta_rad); 

end 

%% FUNCION DETECTAR PENDIENTE 

function m = detectar_pendiente(dist1,dist2,angulo1,angulo2) 

[x1, y1] = calcular_coordenadas(dist1, angulo1); 

[x2, y2] = calcular_coordenadas(dist2, angulo2); 

% Tomar el valor absoluto de las coordenadas 

x1 = abs(x1); 

y1 = abs(y1); 

x2 = abs(x2); 

y2 = abs(y2); 

% Calcular la pendiente en valor absoluto 

m = abs((y2 - y1) / (x2 - x1)); 

end 

%% FUNCION DETECTAR PAREDES 

function codigo = detectar_paredes(dist_izq, dist_der, dist_frente1, dist_frente2, dist_atras1, dist_atras2, angulo1, angulo2, angulo3, angulo4) 

% Definir un umbral para determinar si hay pared (en metros) 

umbral = 1; 

% Determinar si hay pared en cada dirección 

% 1 si hay pared, 0 si no hay pared 

pared_delante = (detectar_pared(dist_frente1,dist_frente2,angulo1,angulo2) && dist_frente1 < 1.02); 

pared_atras = (detectar_pared(dist_atras1,dist_atras2,angulo3,angulo4) && dist_atras1 < 1.42); 

pared_izquierda = dist_izq < umbral; 

pared_derecha = dist_der < umbral; 

% Crear el código binario 

codigo_paredes = bin2dec([num2str(pared_delante), num2str(pared_izquierda), num2str(pared_derecha), num2str(pared_atras)]);  

disp(bin2dec([num2str(pared_delante), num2str(pared_izquierda), num2str(pared_derecha), num2str(pared_atras)])); 

switch codigo_paredes 

case 0 

codigo = 0; 

case 1 

codigo = 3; 

case 2 

codigo = 2; 

case 3 

codigo = 8; 

case 4 

codigo = 4; 

case 5 

codigo = 10; 

case 6 

codigo = 9; 

case 7 

codigo = 12; 

case 8 

codigo = 1; 

case 9 

codigo = 6; 

case 10 

codigo = 5; 

case 11 

codigo = 11; 

case 12 

codigo = 7; 

case 13 

codigo = 13; 

case 14 

codigo = 14; 

case 15 

codigo = 15; 

otherwise 

codigo = 999; 

end 

end 

%% FUNCION DETECTAR PAREDES LIDAR 

function codigo = detectar_paredes_lidar(dist_frente1, dist_frente2,dist_atras1, dist_atras2, dist_izq1, dist_izq2, dist_der1, dist_der2, angulo_del1, angulo_del2, angulo_tra1, angulo_tra2, angulo_izq1, angulo_izq2, angulo_der1, angulo_der2) 

% Definir un umbral para determinar si hay pared (en metros) 

umbral = 1.02; 

% Determinar si hay pared en cada dirección 

% 1 si hay pared, 0 si no hay pared 

pared_delante = (detectar_pared(dist_frente1,dist_frente2,angulo_del1,angulo_del2) && dist_frente1 < umbral); 

pared_atras = (detectar_pared(dist_atras1,dist_atras2,angulo_tra1,angulo_tra2)&& dist_atras1 < umbral); 

pared_izquierda = (detectar_pared(dist_izq1,dist_izq2,angulo_izq1,angulo_izq2)&& dist_izq1 < umbral); 

pared_derecha = (detectar_pared(dist_der1,dist_der2,angulo_der1,angulo_der2)&& dist_der1 < umbral); 

% Crear el código binario 

codigo_paredes = bin2dec([num2str(pared_delante), num2str(pared_izquierda), num2str(pared_derecha), num2str(pared_atras)]);  

switch codigo_paredes 

case 0 

codigo = 0; 

case 1 

codigo = 3; 

case 2 

codigo = 2; 

case 3 

codigo = 8; 

case 4 

codigo = 4; 

case 5 

codigo = 10; 

case 6 

codigo = 9; 

case 7 

codigo = 12; 

case 8 

codigo = 1; 

case 9 

codigo = 6; 

case 10 

codigo = 5; 

case 11 

codigo = 11; 

case 12 

codigo = 7; 

case 13 

codigo = 13; 

case 14 

codigo = 14; 

case 15 

codigo = 15; 

otherwise 

codigo = 999; 

end 

end 