classdef GridLayer<handle
    %GRIDLAYER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        cells;
        layerIndex;
        parrentGrid;
        verticalCellCount = 0;
        horizontalCellCount = 0;
        thetaStart
        thetaEnd
        phiStart
        phiEnd
        dStart
        dEnd
        stepsTheta
        avgStepTheta
        stepsPhi
        avgStepPhi
        minimalEntryTime=0;
        maximalLeaveTime=1;
        toleratedDeviation = 0;
    end
    
    methods
        function obj = GridLayer(ds,de,ts,te,ps,pe,hc,vc)
            if (nargin ~=0)
                obj.thetaStart = ts;
                obj.thetaEnd = te;
                obj.phiStart = ps;
                obj.phiEnd = pe;
                obj.dStart = ds;
                obj.dEnd = de;
                obj.verticalCellCount = vc;
                obj.horizontalCellCount = hc;
                obj.stepsTheta = linspace(ts,te,hc+1);
                obj.avgStepTheta = (te-ts)/hc;
                obj.stepsPhi = linspace(ps,pe,vc+1);
                obj.avgStepPhi = (pe-ps)/vc;
                b(hc,vc) = GridCell();
                for k=1:hc;
                    for l=1:vc;
                        b(k,l)=GridCell(ds,de,obj.stepsTheta(k),obj.stepsTheta(k+1),obj.stepsPhi(l),obj.stepsPhi(l+1));
                    end
                end
                obj.cells = b;
            end
        end
        
        function linkIt(obj)
            for k=1:obj.horizontalCellCount;
                for l=1:obj.verticalCellCount;
                   cell = obj.cells(k,l);
                   cell.ijkIndex=[obj.layerIndex;k;l];
                   cell.layerIndex=obj.layerIndex;
                   cell.horizontalIndex=k;
                   cell.verticalIndex=l;
                   cell.parrentGrid=obj.parrentGrid;
                   cell.parrentLayer=obj;
                   cell.calculateSurfaceInnerOuter;
                end
            end
        end
        function resetLayer(obj)
            for k=1:obj.horizontalCellCount
                for l=1:obj.verticalCellCount
                        obj.cells(k,l).resetCell;
                end
            end
        end
        
        function r=getCellIndexes(obj,theta,phi)
            if  obj.thetaStart > theta || obj.thetaEnd < theta || obj.phiStart > phi || obj.phiEnd < phi
                r=0;
            else
                h_index = floor((theta + abs(obj.thetaStart))/obj.avgStepTheta)+1;
                v_index = floor((phi + abs(obj.phiStart))/obj.avgStepPhi)+1;
                if h_index == (obj.horizontalCellCount +1)
                    h_index=h_index-1;
                end
                if v_index == (obj.verticalCellCount +1)
                    v_index=v_index-1;
                end
                r= [h_index,v_index];
            end
        end
        
        function r=calculateArrivalTime(obj)
            wLayer=obj;
            cellCount = wLayer.horizontalCellCount*wLayer.verticalCellCount;
            passingTimes= zeros(2,cellCount);
            for k=1:cellCount
                cell = wLayer.cells(k);
                et=cell.minimalEntryTime;
                lt=cell.maximalLeaveTime;
                passingTimes(:,k)=[et;lt];
            end
            for k=1:cellCount
                if passingTimes(1,k) == 0
                    passingTimes(1,k) = inf;
                end
            end
            for k=1:cellCount
                if passingTimes(2,k) == 0
                    passingTimes(2,k) = -inf;
                end
            end
            l_entry = min(passingTimes(1,:));
            l_leave = max(passingTimes(2,:));
            if l_entry ~= inf
                obj.minimalEntryTime=l_entry;
            end
            if l_leave ~= - inf
                obj.maximalLeaveTime=l_leave;
            end
            r = passingTimes;
        end
        function r=calculateToleratedDistanceDeviation(obj)
            cell = obj.cells(1);
            center = cell.center;
            maxPoint=Cmnf.plan2euc(cell.dEnd,cell.thetaStart,cell.phiStart);
            maxDistnace=norm(center-maxPoint,2);
            %now propagate the fact trough all cells
            obj.toleratedDeviation = maxDistnace;
            for k=1:(obj.verticalCellCount*obj.horizontalCellCount)
                obj.cells(k).toleratedDeviation = maxDistnace;
            end
            r=maxDistnace;
        end
        function r=expand(obj,farCount,nearCount)
            [m,n] = size(obj.cells);
            r=[];
            for k=1:m
                for l=1:n
                    r=[r,obj.cells(k,l).expand(farCount,nearCount)];
                end
            end
        end
        
        function r=expandRapid(obj)
            [m,n] = size(obj.cells);
            r=[];
            for k=1:m
                for l=1:n
                    r=[r,obj.cells(k,l).expandRapid];
                end
            end 
        end
        
        function r=expandACAS(obj,lm,separations)
            [m,n] = size(obj.cells);
            r=[];
            for k=1:m
                for l=1:n
                    r=[r,obj.cells(k,l).expandACAS(lm,separations)];
                end
            end
        end
    end
    
end

