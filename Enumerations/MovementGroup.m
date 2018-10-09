%% This class is used for Separation avoidance trees build, there is no reason to use it in full reach set
% The purpose of Separation avoidance is to show inferiority of the other
% separated approaches, 
classdef MovementGroup<uint32    
    
    enumeration
        Horizontal(1),  % left right straigth
        Vertical(2),
        Backslash(3),
        Slash(4),
        Combined(5) 
    end
    
    methods(Static)
        function r=toString(member)
            r=char(member);
        end
    
        function r=getMovementGroupMembers(group)
            if group == MovementGroup.Vertical
                r=[MovementType.Up,MovementType.Down,MovementType.Straight];
            end
            
            if group == MovementGroup.Horizontal
                r=[MovementType.Left,MovementType.Right,MovementType.Straight];
            end
            
            if group == MovementGroup.Slash
                r=[MovementType.UpLeft, MovementType.DownRight,MovementType.Straight];
            end
            
            if group == MovementGroup.Backslash
                r=[MovementType.UpRight, MovementType.DownLeft, MovementType.Straight];
            end
        end
        
        function r=getTrajectoryGroups(predictorNode)
            r=[];
            movements = predictorNode.collectMovements;
            for k=1:4
                compSet = MovementGroup.getMovementGroupMembers(k);
                test = ismember(movements,compSet);
                if sum(test) == length(movements)
                    r=[r,MovementGroup(k)];
                end
            end
        end
    end
end

