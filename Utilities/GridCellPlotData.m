classdef GridCellPlotData<handle
    %GRIDCELLPLOTDATA Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        horizontalCell;
        verticalCell;
        probabilities;
    end
    
    methods
        function obj=GridCellPlotData(hc,vc,prob)
            obj.horizontalCell=hc;
            obj.verticalCell=vc;
            obj.probabilities=prob;
        end
        
        function r=getColorVectorGS(obj,probId)
            %0-black,1-white
            sca=1-obj.probabilities(probId);
            r=[sca,sca,sca];
        end
        
        function r=getcolorVectorRGB(obj,probId)
           if probId==6
               sca=obj.probabilities(3);
           else
               sca=obj.probabilities(probId);
           end
           %biased rating overflow fix
           if sca > 1
               sca=1;
           end
           if sca < 0
               sca=0;
           end
           %red Green Blue
           r = [0,0,0];
           %pReachability
           if (probId==1)
               r=[1-sca,sca,0];
           end
           %pVisibility
           if (probId==2)
               r=[0,sca,sca];
           end
           %pObstacle
           if (probId==3)
               r=[1,1-sca,1-sca];
           end
           %pDecision
           if (probId==4)
               r=[sca,0,sca];
           end
           %pFeasibility
           if (probId==5)
               r=[sca,sca,sca];
           end
           %pObstacleGraph
           if (probId==6)
               r=[sca,1-sca,1-sca];
           end
        end
    end
    
end

