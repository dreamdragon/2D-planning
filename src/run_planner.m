function run_planner(file)
    command = ['./run_planner ',file];
    system(command); 
    result = importdata('output.txt');
    
    c_space = (result == 0);
    search = (result == 15);
    path = (result == 30);
    
    
    r = zeros(size(result));
    g = zeros(size(result));
    b = zeros(size(result));
    
    r(c_space) = 255;
    g(c_space) = 255;
    b(c_space) = 255;
    
    r(search) = 255;
    g(search) = 255;
    b(search) = 0;
    
    r(path) = 0;
    g(path) = 255;
    b(path) = 0;
    
    rgb = zeros(size(result,1),size(result,2),3);
    rgb(:,:,1) = r;
    rgb(:,:,2) = g;
    rgb(:,:,3) = b;
    
    J = figure(1);
    imagesc(uint8(rgb));
    print(J,'-djpeg',[file,'.jpg']);
end