rosshutdown;
%posicion del robot para primera prueba 0.91 8.17
setenv('ROS_MASTER_URI','http://192.168.0.101:11311')
setenv('ROS_IP','192.168.0.100')
rosinit;
sonar5 = rossubscriber('/robot0/sonar_5');
sonar0 = rossubscriber('/robot0/sonar_0');

sonar7 = rossubscriber('/robot0/sonar_7');
sonar4 = rossubscriber('/robot0/sonar_4');

pause(2);
datos5 = sonar5.LatestMessage.Range_;
datos0 = sonar0.LatestMessage.Range_;
datos7 = sonar7.LatestMessage.Range_;
datos4 = sonar4.LatestMessage.Range_;
disp(['Distancia medida por el sensor: ', num2str(datos5), ' m']);
disp(['Distancia medida por el sensor: ', num2str(datos0), ' m']);
disp(['Distancia medida por el sensor: ', num2str(datos7), ' m']);
disp(['Distancia medida por el sensor: ', num2str(datos4), ' m']);
if abs(datos5 - datos0) <= 0.1 % Diferencia pequeña -> Paredes paralelas
    disp('El robot está alineado con paredes paralelas izquierda y derecha.');
else
    disp('Las paredes detectadas no son paralelas izquierda y derecha.');
end
if abs(datos4 - datos7) < 0.1 % Diferencia pequeña -> Paredes paralelas
    disp('El robot está alineado con paredes paralelas arriba y abajo.');
else
    disp('Las paredes detectadas no son paralelas arriba y abajo.');
end