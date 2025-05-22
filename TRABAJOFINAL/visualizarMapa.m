function visualizarMapa(logOddsMap, x0, y0, resolution, mapSize, iter)
    imagesc(flipud(logOddsMap));
    colormap('gray'); colorbar;
    set(gca, 'YDir', 'normal');
    axis equal tight;
    hold on;

    plot((x0/resolution)+1, mapSize - (y0/resolution), 'bo', 'MarkerSize', 5, 'LineWidth', 2);
    title(['Mapa (iteración ' num2str(iter) ')']);
    drawnow;
    hold off;
end
