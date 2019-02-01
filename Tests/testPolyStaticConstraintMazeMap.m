%% Maze structure development

mazeMap=[1,1,1,1,1;
         1,6,0,0,1;
         1,1,1,0,1;
         1,5,0,0,1;
         1,1,1,1,1;];
     
     
maze= MazeMatrix(mazeMap);
constraints=maze.generateMazeObstacles;
figure(1)
maze.plot;



