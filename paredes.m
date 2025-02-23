rosshutdown;
%posicion del robot para primera prueba 0.91 8.17
setenv('ROS_MASTER_URI','http://192.168.0.101:11311')
setenv('ROS_IP','192.168.0.100')
rosinit;
%robot mirando al eje y+
%eje x 5+ 0-
sonar5 = rossubscriber('/robot0/sonar_5');
sonar0 = rossubscriber('/robot0/sonar_0');
% eje y 7- 4+
sonar7 = rossubscriber('/robot0/sonar_7');
sonar4 = rossubscriber('/robot0/sonar_4');

pause(2);
datos5 = sonar5.LatestMessage.Range_;
datos0 = sonar0.LatestMessage.Range_;
datos7 = sonar7.LatestMessage.Range_;
datos4 = sonar4.LatestMessage.Range_;
disp(['Distancia medida por el sensor5: ', num2str(datos5), ' m']);
disp(['Distancia medida por el sensor0: ', num2str(datos0), ' m']);
disp(['Distancia medida por el sensor7: ', num2str(datos7), ' m']);
disp(['Distancia medida por el sensor4: ', num2str(datos4), ' m']);
distancia=1.1;
sensores = [datos5, datos0, datos7, datos4];
numero_paredes = sum(sensores < distancia);
disp(['Número de paredes detectadas: ', num2str(numero_paredes)]);
%if datos5 < distancia
%   disp('pared en derecha'); 
%end
rosshutdown;