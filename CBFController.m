classdef CBFController < matlab.DiscreteEventSystem & matlab.system.mixin.Propagates & ...
        matlab.system.mixin.CustomIcon
    %This SimEvents module forms a control zone for traffic merging problems and each vehicle is controlled by a control barrier function controller that guarantees all the constraints.
    %CBF Controller implementation
    properties (Nontunable)
        capacity = 100; %Control zone capacity
        Length = 400; % Control zone length
        step = 0.1; % Simulation step
        Vd = 30; % Desired speed (m/s)
        Vmax = 30;%Max speed (m/s)
        Vmin = 0;%Min speed (m/s)dynamics
        Umax = 0.4;%Max acc. (g)
        Umin = 0.6;%Min acc. (-g)
    end

    properties(DiscreteState)
        numVehicles;
        numVehiclesDeparted;
        AverageFuelConsumption;
        AverageTravelTime;
        maintainFirstVehicle;
        firstID;  %record the id of the first CAV in the CZ queue
        formerLane; %record the lane id of the i-1 CAV in the CZ queue
        formerSpeed;%record the speed of the i-1 CAV in the CZ queue
        formerAcc;  %record the acceleration of the i-1 CAV in the CZ queue
        distance;   %record the last vehicle position in the CZ queue
        
        main_pos; %record the last vehicle position in the main lane queue
        main_speed;%record the last vehicle speed in the main lane queue
        main_acc;  %record the last vehicle acc. in the main lane queue
        main_exist;%sign for the existence of ip in the main lane queue
        merg_pos; %record the last vehicle position in the merging lane queue
        merg_speed;%record the last vehicle speed in the merging lane queue
        merg_acc; %record the last vehicle acc. in the merging lane queue
        merg_exist;%sign for the existence of ip in the merging lane queue
        two_events; % to detect two events (happens at the same time instant)       
    end

    % Pre-computed constants
    properties(Access = private)
        pen;
    end

    methods(Access = protected)
        function num = getNumInputsImpl(~)
            % Define number of inputs for system with optional inputs
            num = 2;
        end
        
        function num = getNumOutputsImpl(~)
            % Define number of outputs for system with optional outputs
            num = 2;
        end
        
        function entityTypes = getEntityTypesImpl(obj)
            % Define entity types being used in this model
            entityTypes(1) = obj.entityType('CAV', 'CAV', 1, false);
            entityTypes(2) = obj.entityType('INFO', 'INFO', 1, false);
            
        end
        
        function [input, output] = getEntityPortsImpl(~)
            % Define data types for entity ports
            input = {'CAV','INFO'};
            output = {'CAV','INFO'};
        end
        
       function [storageSpec, I, O] = getEntityStorageImpl(obj)
            % Input queue for control zone
            storageSpec(1) = obj.queueFIFO('CAV', obj.capacity);
            % Input queue after merging point
            storageSpec(2) = obj.queueFIFO('CAV', obj.capacity);
            % Input queue beyond mission space 
            storageSpec(3) = obj.queueFIFO('CAV', obj.capacity); %reserved
            % Input queue for INFO
            storageSpec(4) = obj.queueFIFO('INFO', obj.capacity);%reserved
            
            I = [1 4];
            O = [2 4];
        end
        
        function sz = getOutputSizeImpl(~)
            % Return size for each output port
            sz(1) = 1;
            sz(2) = 1;
        end
        function dt = getOutputDataTypeImpl(~)
            % Return data type for each output port
            dt(1) = 'CAV';
            dt(2) = 'INFO';
        end
        
        function cp = isOutputComplexImpl(~)
            % Return true for each output port with complex data
            cp(1) = false;
            cp(2) = false;
        end
        
        function [name1, name2] = getInputNamesImpl(~)
            % Return input port names for System block
            name1 = 'IN';
            name2 = 'INFO';
            
        end
        
        function [name1, name2] = getOutputNamesImpl(~)
            % Return input port names for System block
            name1 = 'OUT';
            name2 = 'INFO';
            
        end
        function icon = getIconImpl(~)
            icon = sprintf('CBF Controller');
        end
        
        function [sz, dt, cp] = getDiscreteStateSpecificationImpl(~, ~)
            sz = [1, 1];
            dt = 'double';
            cp = false;
        end
        
        function setupImpl(obj)
            obj.AverageFuelConsumption = 0;
            obj.AverageTravelTime = 0;
            obj.numVehicles = 0;
            obj.numVehiclesDeparted = 0;
            obj.maintainFirstVehicle = 0;
            obj.firstID = 1;
            obj.formerSpeed = 0;
            obj.formerAcc = 0;
            obj.formerLane = 0;
            obj.distance = 0;
            obj.main_pos = 0;
            obj.main_speed = 0;
            obj.main_acc = 0;
            obj.main_exist = 0;
            obj.merg_pos = 0;
            obj.merg_speed = 0;
            obj.merg_acc = 0;
            obj.merg_exist = 0;
            obj.two_events = 0;
            obj.pen = plotBGIMAGE();
        end
        
        
        function [entity, events] = INFOEntryImpl(obj, storage, entity, tag)
            if storage == 4
                obj.newArrival = entity.data.VehicleID;
                events = obj.eventDestroy();
            end
        end
        
        function [entity, events] = INFOGenerateImpl(obj, storage, entity, tag)
            switch tag
                case 'arrival'
                    entity.data.VehicleID = 0;
                    events = obj.eventForward('output', 2, 0);
            end
        end
        
        function [entity, events] = CAVGenerateImpl(obj, storage, entity, tag)
            switch tag
                case 're-generate'
                    events = obj.eventForward('output', 1, 0);
            end
        end
        
        function [entity, events] = CAVEntryImpl(obj, storage, entity, ~)
            events = [];
            if storage == 1 % apply CBF control
                obj.numVehicles = obj.numVehicles + 1;
                entity.data.distance = obj.distance;
                if(obj.two_events == 1) % two events at the same time, i.e., two CAVs arrive at the same time
                    entity.data.distance = 0.1;
                    entity.data.Position = -0.1;
                end                
                if entity.data.ID == 1
                    events = obj.eventTimer('CZ',obj.step);
                end
                obj.two_events = 1; % events count
                
            elseif storage == 2
                if entity.data.ID == 1
                    events = obj.eventTimer('ACZ',obj.step);
                end
            else
                if entity.data.ID == 1
                    events = obj.eventTimer('default',obj.step);
                end
            end
        end
        
        function [entity, events] = CAVTimerImpl(obj, storage, entity, tag)
            events = [];
            
            switch tag
                case 'CZ'  % control zone
                    events = [ obj.eventIterate(1, 'cbf', 1), obj.eventTimer('CZ',obj.step) ];
                    
                case 'ACZ' % after control zone
                    events =  [obj.eventIterate(2, 'cruise', 1),  obj.eventTimer('ACZ',obj.step)] ;
                    
                case 'default'
                    events = obj.eventTimer('default',obj.step);
            end
        end
        
        function [entity, events, next] = CAVIterateImpl(obj, storage, entity, tag, status)
            events = [];
            switch tag
                case 'cbf'
                    x1 = []; %i_p vehicle state
                    obj.two_events = 0; % events reset, no two events
                    if(entity.data.ID == obj.firstID) % first CAV in the CZ queue
                        x0 = [entity.data.Position,entity.data.Speed,entity.data.distance,0,0]; 
                        [entity.data.Position,entity.data.Speed,entity.data.distance,entity.data.Acceleration] ...
                            = CBF_Control(x0,x1,obj.Vd,obj.Vmax,obj.Vmin,obj.Umax,obj.Umin,obj.step,1,1, entity.data.ID);
                    else                        
                        x0 = [entity.data.Position,entity.data.Speed,entity.data.distance,obj.formerSpeed,obj.formerAcc];
                        if (entity.data.Lane == obj.formerLane) % front vehicle in the same lane
                            [entity.data.Position,entity.data.Speed,entity.data.distance,entity.data.Acceleration] ...
                                = CBF_Control(x0,x1,obj.Vd,obj.Vmax,obj.Vmin,obj.Umax,obj.Umin,obj.step,1,2, entity.data.ID);%2 -> type
                        else  %front vehicle in the different lane
                            x1 = []; % vehicle ip state
                            if(entity.data.Lane == 1)  %main lane                         
                                if(obj.main_exist == 1) % ip exists
                                    x1 = [obj.main_pos,obj.main_speed,obj.main_acc];
                                end
                            else %merging lane
                                if(obj.merg_exist == 1)% ip exists
                                    x1 = [obj.merg_pos,obj.merg_speed,obj.merg_acc];
                                end
                            end
                            [entity.data.Position,entity.data.Speed,entity.data.distance,entity.data.Acceleration] ...
                                = CBF_Control(x0,x1,obj.Vd,obj.Vmax,obj.Vmin,obj.Umax,obj.Umin,obj.step,1,3, entity.data.ID);%3->type
                        end
                    end
                    if(entity.data.Acceleration >= 0)  %calculate fuel consumption
                        entity.data.FuelConsumption = fuel_consumption(entity.data.FuelConsumption,entity.data.Acceleration,entity.data.Speed);
                    end
                    entity.data.TravelTime = entity.data.TravelTime + obj.step; % calculte travel time
                    
                    obj.pen.p_color = plotCAV(entity.data.Position, entity.data.Lane, entity.data.ID,obj.pen.p, obj.pen.p_color);
                    obj.formerLane = entity.data.Lane; % update i-1(i_p) for the CZ queue
                    obj.formerSpeed = entity.data.Speed;
                    obj.formerAcc = entity.data.Acceleration;
                    if(entity.data.Lane == 1) % update ip for the main lane queue
                        obj.main_pos = entity.data.Position;
                        obj.main_speed = entity.data.Speed;
                        obj.main_acc = entity.data.Acceleration;
                        obj.main_exist = 1;
                    else % update ip for the merging lane queue
                        obj.merg_pos = entity.data.Position;
                        obj.merg_speed = entity.data.Speed;
                        obj.merg_acc = entity.data.Acceleration;
                        obj.merg_exist = 1;
                    end
                    if(obj.numVehicles == entity.data.ID)
                        obj.distance = entity.data.Position;  % the last CAV position is the distance of new arrival CAV
                        obj.main_exist = 0; % next loop, starting from the first CAV in the CZ queue, reset
                        obj.merg_exist = 0; 
                    end
                    if (entity.data.Position > obj.Length)
                        events = obj.eventForward('storage', 2, 0);
                        obj.firstID = obj.firstID + 1;
                        obj.numVehiclesDeparted = obj.numVehiclesDeparted + 1;
                        obj.AverageFuelConsumption = (obj.AverageFuelConsumption*(obj.numVehiclesDeparted - 1) + entity.data.FuelConsumption)/obj.numVehiclesDeparted;
                        obj.AverageTravelTime = (obj.AverageTravelTime*(obj.numVehiclesDeparted - 1) + entity.data.TravelTime)/obj.numVehiclesDeparted;
                        plotPerformanceMetrics(obj.AverageFuelConsumption,obj.AverageTravelTime,obj.numVehiclesDeparted,obj.pen.fuel, obj.pen.time, obj.pen.through);
                        
                        %record performance add by lc
                        performanceItem = [obj.numVehiclesDeparted, obj.AverageTravelTime, obj.AverageFuelConsumption ];
                        savePerformance(performanceItem, 2);
                    end
                    next = true;                   
                case 'cruise'
                    x0 = [entity.data.Position,entity.data.Speed,entity.data.distance,0,0]; x1 = [];
                    [entity.data.Position,entity.data.Speed,entity.data.Acceleration] ...
                        = CBF_Control(x0,x1,obj.Vd,obj.Vmax,obj.Vmin,obj.Umax,obj.Umin,obj.step, 0, 1, entity.data.ID); %constant speed
                    obj.pen.p_color = plotCAV(entity.data.Position, entity.data.Lane, entity.data.ID, obj.pen.p, obj.pen.p_color);
                    if (entity.data.Position > obj.Length + 110) % 110 = lane length considered after the merging point
                        if entity.data.ID == 1
                            if  obj.maintainFirstVehicle == 0
                                events = obj.eventGenerate(2, 're-generate', 0, 1);
                                obj.maintainFirstVehicle = 1;
                            else
                                events = [];
                            end
                        else
                            events = obj.eventForward('output', 1, 0);
                        end
                    end
                    next = true;
            end
        end
    end
end
