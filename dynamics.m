function [p, v, u] = dynamics(coe, pos, speed, flag, vehicleID)
% NOTICE: modify the model name to reflect the current simulation time
    t = get_param('SingleMerging', 'SimulationTime'); %get simulation time

    persistent vehStateRec_OC; %define persistent variable to record the veh state of optimal controller
    if isempty(  vehStateRec_OC)
        vehStateRec_OC = [];
    end

    if flag == 0
        % flag: 1 (cruise); 0 (control)
        v = 1 / 2 * coe(1) * t ^ 2 + coe(2) * t + coe(3);
        p = 1 / 6 * coe(1) * t ^ 3 + 1 / 2 * coe(2) * t^2 + coe(3) * t + coe(4);

        u = coe(1) * t + coe(2);
    else
        p = pos + speed * 0.1;
        v = speed;
        u = 0;
    end
    
    vehStateRec_OC = [vehStateRec_OC; t, vehicleID, p, v, u];
    vehStateRec_OC = unique( vehStateRec_OC, 'rows' );
    [vehStateRec_OC, ~] = sortrows( vehStateRec_OC, 2 ); %sort by vehId
    save vehStateRec_OC vehStateRec_OC; %save as  vehStateRec_OC.mat
end