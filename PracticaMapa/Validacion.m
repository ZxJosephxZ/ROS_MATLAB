function flag = dentro_mapa(ix, iy, grid_size)
    flag = ix > 0 && iy > 0 && ix <= grid_size && iy <= grid_size;
end

