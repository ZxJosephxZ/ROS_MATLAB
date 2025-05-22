function guardarMapaFinal(logOddsMap, resolution, nombreArchivo, titulo)
    % Convertir log-odds a probabilidades
    probMap = 1 - (1 ./ (1 + exp(logOddsMap)));
    binaryMap = logOddsMap > 0;

    % Crear y mostrar mapa binario
    mapObj = binaryOccupancyMap(binaryMap, 1/resolution);

    fig = figure;
    show(mapObj);
    title(titulo);

    % Guardar archivo .mat
    save(nombreArchivo, 'mapObj');

    % Guardar imagen como .png
    [folder, baseName, ~] = fileparts(nombreArchivo);  % Separar nombre base
    if isempty(folder)
        folder = '.';  % Guardar en directorio actual si no se especifica
    end
    pngFile = fullfile(folder, [baseName '.png']);
    saveas(fig, pngFile);

    close(fig);  % Opcional: cerrar figura después de guardar
end
