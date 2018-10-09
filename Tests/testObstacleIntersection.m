%Mission control object must exist
[intersections,collisions]=getIntersectionCollisionCandidates(obj);
obj.avoidanceGrid.resetGrid

ag=obj.avoidanceGrid;
r=intersectObstaclesWithGrid(obj,ag,intersections);

ag.recalculate
obj.plotGridSlice(ag,StatisticType.Reachability,2)
obj.plotGridSlice(ag,StatisticType.Obstacle,3)
obj.plotGridSlice(ag,StatisticType.Visibility,4)

