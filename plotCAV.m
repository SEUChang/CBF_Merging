function p_color = plotCAV(pos, lane, id, p, p_color)
%% lc test 0114
% veh_info = [id, lane, pos];
% save veh_info veh_info;
%% origin
switch lane
    case 1
        if p_color(id) == 0
            p(id).MarkerEdgeColor = 'blue';
            p(id).MarkerFaceColor = 'blue';
            p(id).Marker = '>';
            p_color(id) = 1;
        end
        set(p(id),'XData',pos,'YData',0);
    case 2
        if p_color(id) == 0
            p(id).MarkerEdgeColor = 'red';
            p(id).MarkerFaceColor = 'red';
            p_color(id) = 1;
        end
        if(pos <= 400)
            set(p(id),'XData',400 - (400 - pos)*cosd(30),'YData',-0.5*(400 - pos));
        else
            set(p(id),'XData',pos,'YData',0);
        end
        
end
drawnow
axis([-20 500 -250 50]);
end