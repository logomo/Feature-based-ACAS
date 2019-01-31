classdef MovementGroup<uint32    

    % MovementGroup This class is used for Separation avoidance trees (ACAS-X like) build, there is no reason to use it in full reach set
    %   The purpose of Separation avoidance is to show inferiority of the other separated approaches, 
    enumeration
        Horizontal(1),  % Movements on horizontal plane GCF
        Vertical(2),    % Movements on vertical plane  GCF
        Backslash(3),   % Movements on plane with - 45 deg angle with horizontal
        Slash(4),       % Movements on plane with + 45 deg angle and +90 angle to Backslash plane
        Combined(5)     % All separation planes, horizontal, vertical, slash and backslash
    end
    
    methods(Static)
        function r=toString(member)
            r=char(member);
        end
    
        function r=getMovementGroupMembers(group)
            %Gets a list of Movement Types supported in given movement group
            %   grou - MovementGroup enumeration member
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
            %For Given trajectory (PredictorNode instance) the list of viable Movement Groups are returned
            %   predictorNode - the tip of the trajectory
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

