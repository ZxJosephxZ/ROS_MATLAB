rosshutdown;
%posicion del robot para primera prueba 0.91 8.17
setenv('ROS_MASTER_URI','http://192.168.0.101:11311')
setenv('ROS_IP','192.168.0.100')
rosinit;
sonar = rossubscriber('/robot0/sonar_7');
%Foto apartado sonares
pause(2);
%datos = sonar.LatestMessage;
%disp(datos);
%datos = sonar.LatestMessage.Range_;
%disp(['Distancia medida por el sensor: ', num2str(datos), ' m']);
n_medidas = 1000;
datos = zeros(1, n_medidas);

for i = 1:n_medidas
    datos(i) = sonar.LatestMessage.Range_; % Guardar medida actual
    disp(['Distancia medida por el sensor: ', num2str(datos(i)), ' m']);
    pause(0.01); % Esperar un pequeño intervalo
end

% Graficar las mediciones
figure;
plot(datos);
xlabel('Muestra');
ylabel('Distancia (m)');
title('Mediciones del Sensor Sonar');
grid on;

% Filtro de media móvil con ventana de 5
ventana = 5;
datos_filtrados = filter(ones(1, ventana)/ventana, 1, datos);

% Graficar datos filtrados
figure;
plot(datos, 'r'); hold on;
plot(datos_filtrados, 'b', 'LineWidth', 2);
xlabel('Muestra');
ylabel('Distancia (m)');
title('Comparación de Datos Originales y Filtrados');
legend('Datos Originales', 'Datos Filtrados');
grid on;


rosshutdown;