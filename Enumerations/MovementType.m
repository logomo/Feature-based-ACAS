classdef MovementType < uint32
    %MOVEMENTTYPE The type of the movement - unitary movement applied
    enumeration
        Straight(0);    %Fly straight LCF
        Down(1);        %Fly down LCF
        Up(2);          %Fly up LCF
        Left(3);        %Fly left LCF
        Right(4);       %Fly right LCF
        DownLeft(5);    %Fly combined down and lefy LCF
        DownRight(6);   %Fly combined down and right LCF
        UpLeft(7);      %Fly combined up and left LCF
        UpRight(8);     %Fly combined up and right LCF
    end
    
    methods(Static)
        %Get strang raprasantajtsn (Fallout 4 Stronk reference)
        function r=toString(member)
            r=char(member);
        end
        
        function r=getPlanarMovementSet()
        % Static getter for planar movements
        %   ismember(MovementType.Straight,MovementType.getPlanarMovementSet())
        %   ismember(MovementType.Up,MovementType.getPlanarMovementSet())
           r= [MovementType.Straight,MovementType.Right, MovementType.Left];
        end
    end
   
end

