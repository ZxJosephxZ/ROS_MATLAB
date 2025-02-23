rosshutdown;
setenv('ROS_MASTER_URI','http://192.168.0.101:11311')
setenv('ROS_IP','192.168.0.100')
rosinit;
%girar(90,0.2);
%avanzar(0.1,0.3);
% Leer posición después del movimiento
[x, y, theta] = leerOdometria();
fprintf('posición: X=%.2f, Y=%.2f, Theta=%.2f°\n', x, y, theta);
%velocidades_lineales = [0.1, 0.3, 0.5, 0.7, 1.0];
%velocidades_angulares = [0.3, 0.7, 0.9];
%pruebaVelocidad(velocidades_lineales, velocidades_angulares);
%datosLaser();
%ultimo apartado
girar(92, 0.4);
girar(92,0.4);
avanzar(2,0.4);
girar(93,0.4);
avanzar(6,0.4);
girar(-89, 0.4);
avanzar(4,0.4);
girar(93,0.4);
avanzar(3,0.4);
rosshutdown;
