classdef Controller < matlab.DiscreteEventSystem & ...
        matlab.system.mixin.Propagates & ...
        matlab.system.mixin.CustomIcon
    %This SimEvents module forms a control zone for traffic merging problems and each vehicle is under optimal control
    
    properties (Nontunable)
        capacity = 100; % Control zone capacity
        step = 0.1; % Simulation step
        L = 400; % Length of control zone
    end
    properties (DiscreteState)
        % network status
        numVehicles;
        numVehiclesDeparted;
        
        % performance metric
        AverageFuelConsumption;
        OptimalFuelConsumption;
        AverageTravelTime;
        
        % info waiting to be exchanged with the coordinator
        newArrival;
        CurrentFinalTime; % enter MZ
        CurrentLane;
        maintainFirstVehicle;
        
        %performance record lc
        %performanceItem ;
        
    end
     % Pre-computed constants
    properties(Access = private)
        pen;
    end
    
    methods (Access=protected)
        
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
            % Input queue for entities / current lane -> control implemented
            storageSpec(1) = obj.queueFIFO('CAV', obj.capacity);
            % Input queue for pause/continue messages
            storageSpec(2) = obj.queueFIFO('INFO', obj.capacity);
            % Input queue for entities / current lane -> waiting for the info from the
            % coordinator
            storageSpec(3) = obj.queueFIFO('CAV', obj.capacity);
            % Input queue for entities / after merging point
            storageSpec(4) = obj.queueFIFO('CAV', obj.capacity);
            
            I = [3 2];
            O = [4,2];
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
            icon = sprintf('OPTIMAL CONTROLLER');
        end
        
        function [sz, dt, cp] = getDiscreteStateSpecificationImpl(~, ~)
            sz = 1;
            dt = 'double';
            cp = false;
        end
        
        function setupImpl(obj)
            obj.numVehicles = 0;
            obj.numVehiclesDeparted = 0;
            obj.AverageFuelConsumption = 0;
            obj.AverageTravelTime = 0;
            obj.CurrentFinalTime = 15;
            obj.maintainFirstVehicle = 0;
            obj.pen = plotBGIMAGE();
        end
        
        
        function [entity, events] = INFOEntryImpl(obj, storage, entity, tag)
            % Called when a pause message enters the block
            if storage == 2
                obj.newArrival = entity.data.VehicleID;
                events = obj.eventDestroy();
            end
        end
        
        
        function [entity, events] = INFOGenerateImpl(obj, storage, entity, tag)
            switch tag     
                case 'arrival'
%                     entity.data.VehicleID = 0;
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
            %% Entering the Control Zone - waiting for info
             if storage == 3 % input port storage
                [entity.data.Position, entity.data.Speed, entity.data.Acceleration] = ...
                    dynamics(entity.data.coe, entity.data.Position, entity.data.Speed, 1, entity.data.ID);
                plotCAV(entity.data.Position, entity.data.Lane, entity.data.ID, obj.pen.p, obj.pen.p_color);
                
                % send info packet to the coordinator
                events = [ obj.eventGenerate(2, 'arrival', 0, 1), obj.eventTimer('delay',obj.step)];
             end
            
              %% Receiving the info, start to implement control
            if storage == 1
                % storage where the optimal control applied
%                  entity.data.ID = obj.newArrival;
                obj.numVehicles = obj.numVehicles + 1;
                
                %% Vehicle Coordination Structure
                if (entity.data.ID == 1) % first vehicle entering the network
                    entity.data.FinalTime = obj.CurrentFinalTime;
                    obj.CurrentLane = entity.data.Lane;
                    obj.CurrentFinalTime = entity.data.FinalTime;
                else
                    % different sources
                    tc1 = entity.data.ArrivalTime + obj.L/30 + (30-entity.data.Speed)^2/(2*0.4*9.81*30);
                    vm = sqrt(2*obj.L*0.4*9.81 + entity.data.Speed^2);
                    tc2 = entity.data.ArrivalTime + (vm - entity.data.Speed)/(0.4*9.81);
                    tc = max(tc1,tc2);
                    tf1 = obj.CurrentFinalTime + 1.8;
                    entity.data.FinalTime = max(tc,tf1);

                    obj.CurrentLane = entity.data.Lane;
                    obj.CurrentFinalTime = entity.data.FinalTime;
                end
                %%
                
                entity.data.coe = updateAcceleration(entity.data.Speed, entity.data.FinalSpeed,...
                    entity.data.ArrivalTime, entity.data.FinalTime, entity.data.Position);
                if entity.data.ID == 1
                    events = obj.eventTimer('CZ',obj.step);
                else
                    events = [];
                end
            end
            
            %% After the Merging Point
            if storage == 4
                if entity.data.ID == 1
                    events = obj.eventTimer('MZ',obj.step);
                else
                    events = [];
                end
                
                % Calculate performance metric in real time:
                % Average Fuel Consumption and Average Travel Time over the control zone for the whole network
                obj.AverageFuelConsumption = obj.AverageFuelConsumption * obj.numVehiclesDeparted + entity.data.FuelConsumption;
                obj.AverageTravelTime = obj.AverageTravelTime * obj.numVehiclesDeparted + entity.data.FinalTime - entity.data.ArrivalTime;
                obj.numVehiclesDeparted = obj.numVehiclesDeparted + 1;
                obj.AverageFuelConsumption = obj.AverageFuelConsumption / (obj.numVehiclesDeparted);
                obj.AverageTravelTime = obj.AverageTravelTime / (obj.numVehiclesDeparted);
                
                % Plot the performance metric in real time
                plotPerformanceMetrics(obj.AverageFuelConsumption, obj.AverageTravelTime,obj.numVehiclesDeparted,obj.pen.fuel, obj.pen.time, obj.pen.through);
               
                %record performance info --lc
                %  Depart number ---  Travel Time --- Fuel consumption
                performanceItem = [obj.numVehiclesDeparted, obj.AverageTravelTime, obj.AverageFuelConsumption ];
                savePerformance(performanceItem, 1);
            end
        end
        function [entity, events] = CAVTimerImpl(obj, storage, entity, tag)
            events = [];
            
            switch tag
                case 'CZ' % control zone - control
                    events = [ obj.eventIterate(1, 'optimal', 1), obj.eventTimer('CZ',obj.step) ];
                    
                case 'delay' % control zone - delay
                    if obj.newArrival == entity.data.ID
                        events = [ obj.eventIterate(3, 'optimal', 1), obj.eventForward('storage',1, 0) ];
                    else
                        events = [ obj.eventIterate(3, 'optimal', 1), obj.eventTimer('delay',obj.step)];
                    end
                    
                case 'MZ' % merging zone
                    events =  [obj.eventIterate(4, 'mergingzone', 1),  obj.eventTimer('MZ',obj.step)] ;
            end           
        end
        
        function [entity, events, next] = CAVIterateImpl(obj, storage, entity, tag, status)
            events = [];
            switch tag
                case 'optimal'
                    if storage == 3
                        % compute the dynamics based on the latest control
                        % cruise
                        [entity.data.Position, entity.data.Speed, entity.data.Acceleration] = ...
                            dynamics(entity.data.coe, entity.data.Position, entity.data.Speed, 1, entity.data.ID);
                    else
                        % compute the dynamics based on the latest control
                        % control
                        [entity.data.Position, entity.data.Speed, entity.data.Acceleration] = ...
                            dynamics(entity.data.coe, entity.data.Position, entity.data.Speed, 0, entity.data.ID);
                    end
                    obj.pen.p_color = plotCAV(entity.data.Position, entity.data.Lane, entity.data.ID, obj.pen.p, obj.pen.p_color);
                    
                    %% compute the real fuel consumption
                    entity.data.FuelConsumption = fuel_consumption(entity.data.FuelConsumption, ...
                        entity.data.Acceleration, entity.data.Speed);
                    % entering the merging zone
                    if entity.data.Position >= obj.L
                        events = obj.eventForward('storage', 4, 0);
                    else
                        events = [];
                    end
                    next = true;
                        
                case 'mergingzone'
                    [entity.data.Position, entity.data.Speed, entity.data.Acceleration] ...
                        = dynamics(entity.data.coe, entity.data.Position, entity.data.Speed, 1, entity.data.ID);
                    obj.pen.p_color = plotCAV(entity.data.Position, entity.data.Lane, entity.data.ID, obj.pen.p, obj.pen.p_color);
                    
                    if entity.data.Position > obj.L + 110
                        if entity.data.ID == 1
                            if  obj.maintainFirstVehicle == 0
                                events = obj.eventGenerate(4, 're-generate', 0, 1);
                                obj.maintainFirstVehicle = 1;
                            else
                                events = [];
                            end
                            %
                        else
                            events = obj.eventForward('output', 1, 0);
                            % events = obj.eventDestroy();
                        end
                    end
                    next = true;
            end
        end
    end
end
