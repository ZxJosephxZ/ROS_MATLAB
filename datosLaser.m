function datosLaser()
laser1=rossubscriber('robot0/laser_1');
pause(1);
%Obtener la matriz de los datos del rango que ocupa el laser
datos = laser1.LatestMessage;
%disp(datos);
%distancias = datos.Ranges;
%angulos=linspace(-pi,pi,length(distancias));
%Descomentar en caso de querer ver las señales disp(angulos);
%Cambiar los datos a angulos para su interpretacion: y=dsen(r) x=dcos(r)
%r=angulo
%x=distancias .* cos(angulos);
%y=distancias .* sin(angulos);
%Representacion en el plano del laser
%figure;
%plot(x,y,'r.');
%xlabel('X(m)'); ylabel('Y(m)');
%title('Medidas del sensor laser');
%axis equal;
%grid on;
%Apartado 2-4 6-8 
%medimos la distancia al frente del robot
distancia_media = datos.Ranges(ceil(end/2));
disp(['distancia del lase en: ', num2str(distancia_media), 'm']);
n_medidas = 1000;
datos = zeros(1, n_medidas);

for i = 1:n_medidas
    datos(i) = laser.LatestMessage.Ranges(ceil(end/2));
    pause(0.01);
end

% Graficar los datos
figure;
plot(datos);
xlabel('Muestra');
ylabel('Distancia (m)');
title('Mediciones del Sensor Láser');
grid on;

media_distancia = mean(datos);
varianza_ruido = var(datos);
max_ruido = max(datos) - min(datos);

disp(['Media: ', num2str(media_distancia)]);
disp(['Varianza: ', num2str(varianza_ruido)]);
disp(['Máximo ruido: ', num2str(max_ruido)]);
%robot posicionado en 3 2
end