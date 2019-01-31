classdef ExamplePolygonType<uint32
    %Example polygons collection for polyconstraint

    
    enumeration
        Unusual(1),             % see poly static constraint
        PentaTrap(2),           % Pentagon hollow trype
        Hospital(3),            % H-shaped buildng
        Square(4),              % Square with even side
        Poly5(5),               % 5 vertex convex polygon
    end
    
    methods(Static)
        function r=toString(member)
            r=char(member);
        end
        
        
        function r=getPolygonData(member)
            %gets polygon data scaled to 1, with centering at 0.0
            
            
            %unusual building r=5
            if member == ExamplePolygonType.Unusual
                r=[-4,3;4,3;4,-3;2,-3;2,0;0,0;0,-3;-4,-3;-4,-1;-2,-1;-2,2;-4,2;-4,3]'./5;
            end
            %pg1=[-4,3;4,3;4,-3;2,-3;,2,0;0,0;0,-3;-4,-3;-4,-1;-2,-1;-2,2;-4,2;-4,3]';
            %plot(pg1(1,:),pg1(2,:))

            %penta trap r=5
            if member == ExamplePolygonType.PentaTrap
                r= [-4,-3;-4,3;0,5;4,3;4,-3;3,-3;3,2;0,3;-3,2;-3,-3;-4,-3]'./5;
            end
            %pg2=[-4,-3;-4,3;0,5;4,3;4,-3;3,-3;3,2;0,3;-3,2;-3,-3;-4,-3]';
            %plot(pg2(1,:),pg2(2,:))

            %H building r=5
            if member == ExamplePolygonType.Hospital
                r= [-4,3; -4,-3; -2,-3; -2,-1; 2,-1; 2,-3;4,-3; 4,3; 2,3; 2,1; -2,1; -2,3;-4,3]'./5;
            end
            %pg3=[-4,3; -4,-3; -2,-3; -2,-1; 2,-1; 2,-3;4,-3; 4,3; 2,3; 2,1; -2,1; -2,3;-4,3]';
            %plot(pg3(1,:),pg3(2,:))

            % square r=5
            if member == ExamplePolygonType.Square
                r= [2.5,2.5;2.5,-2.5;-2.5,-2.5;-2.5,2.5;2.5,2.5]'./2.5;
            end
            %pg4=[2.5,2.5;2.5,-2.5;-2.5,-2.5;-2.5,2.5;2.5,2.5]';
            %plot(pg4(1,:),pg4(2,:))

            % poly5 r=5 Poly5
            if member == ExamplePolygonType.Poly5
                r= [-3.99,3.02;0.88,4.92;4.92,0.89;1.9,-4.63;-4.87,-1.11;-3.99,3.02]'./5;
            end
            %pg5=[-3.99,3.02;0.88,4.92;4.92,0.89;1.9,-4.63;-4.87,-1.11;-3.99,3.02]';
            %plot(pg5(1,:),pg5(2,:))
        end
    end
end

