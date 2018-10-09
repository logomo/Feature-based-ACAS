classdef MovementType < uint32
    %MOVEMENTTYPE Summary of this class goes here
    %   Detailed explanation goes here
    enumeration
        Straight(0);
        Down(1);
        Up(2);
        Left(3);
        Right(4);
        DownLeft(5);
        DownRight(6);
        UpLeft(7);
        UpRight(8);
    end
    
    methods(Static)
        function r=toString(member)
            r=char(member);
        end
        % Static getter for planar movements
        %   ismember(MovementType.Straight,MovementType.getPlanarMovementSet())
        %   ismember(MovementType.Up,MovementType.getPlanarMovementSet())
        function r=getPlanarMovementSet()
           r= [MovementType.Straight,MovementType.Right, MovementType.Left];
        end
    end
   
end

