function pruebaVelocidad(velocidades, angulares)
%la coordenada es 1 3 del robot
fprintf('Pruebas de velocidades lineales');
for i=1:length(velocidades)
    [xi, yi, thetai] = leerOdometria();
    avanzar(1,velocidades(i));
    [x, y, theta] = leerOdometria();
fprintf('Nueva posición: X=%.2f, Y=%.2f, Theta=%.2f°\n', (x-xi), (y-yi), (theta-thetai));    
end
fprintf('Pruebas de velocidades angulares');
for t=1:length(angulares)
    girar(0,angulares(t));
    [x, y, theta] = leerOdometria();
fprintf('Nueva posición: X=%.2f, Y=%.2f, Theta=%.2f°\n', x, y, theta);
end
end