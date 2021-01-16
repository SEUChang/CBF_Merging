function [pos, vel, dis, a] = CBF_Control(x1,y1,vd,vmax,vmin,ca,cd,dt,flag, type, vehicleID) % for vehicles in the different lane
%record motion info
SimTime = get_param('SingleMerging', 'SimulationTime');
persistent vehStateRec_CBF;
if isempty( vehStateRec_CBF  )
    vehStateRec_CBF = [];
end

x0 = [x1(1),x1(2),x1(3)]; %vehicle i state
v0 = x1(4); dv0 = x1(5);  %i-1 vehicle state speed and acc
% y1 ip vehicle [pos, speed, acc]
if(flag == 1)
    g = 9.81; m = 1650;
    f0 = 0.1; f1 = 5; f2 = 0.25;   
    eps = 10; psc = 1;   
    L = 400;   
    Fr = f0 + f1*x0(2) + f2*x0(2)^2; %Fr = 0;
    
    A_vmax = 1;  %CBF for max and min speed, eq(12) LgB1
    b_vmax = m*(vmax - x0(2))^3 + Fr; % eq(12)
    A_vmin = -1; %eq(13) LgB2
    b_vmin = m*(x0(2) - vmin)^3 - Fr;%eq(13)   LfB1
    
    phi0 = -2*(x0(2) - vd)*Fr/m + eps*(x0(2) - vd)^2; %CLF for desired speed, eq(17) LfV  + eps * V
    phi1 = 2*(x0(2) - vd)/m; % eq(17)  LgV
    
    switch type
        case 1  % first vehicle
            A = [phi1 -1; 1 0; -1 0; A_vmax 0; A_vmin 0];
            b = [-phi0; ca*m*g; cd*m*g; b_vmax; b_vmin];
        case 2  % i-1 is in the same lane
            h = x0(3) - 1.8*x0(2) - 0.5*(v0 - x0(2))^2/(cd*g); %CBF for safety  eq(27)
            Bf = 1/h; %eq28 B6
            LfBf = -(m*(v0 - x0(2)) - m*(v0 - x0(2))*dv0/(cd*g) - Fr*((v0 - x0(2))/(cd*g) - 1.8))*Bf^2/m;%eq28 Lfb6
            LgBf = (1.8 - (v0 - x0(2))/(cd*g))*Bf^2/m;%eq28 Lgb6
            A = [phi1 -1; LgBf 0; 1 0; -1 0; A_vmax 0; A_vmin 0];     %A_clf (1) -> A_cbfSafe -> umax ->umin ->vmax ->vmin
            b = [-phi0; -LfBf + 1/Bf; ca*m*g; cd*m*g; b_vmax; b_vmin]; 
        case 3  % i-1 is in the different lane
            cdg = cd*g;   %CBF for safe merging between i and i-1
            h = x0(3) - 0.5*(v0 - x0(2))^2/cdg - 1.8*x0(2)*(x0(1) + 0.5*(x0(2)^2 - v0^2)/cdg)/L; %eq25,   h5
            Bf = 1/h; %B5
            constant = x0(2) - v0 + 1.8*v0*x0(2)/L + Fr*((v0 - x0(2))/cdg - 1.8*v0*x0(2)/(cdg*L) - 1.8*x0(1)/L - (1.8*x0(2)^2 - 1.8*v0^2)/(cdg*L))/m;
            constant1 = ( (v0-x0(2))*L - 1.8*v0^2 )/(cdg*L);
            LfBf = (constant + constant1*dv0)*Bf^2;%LfB5
            LgBf = Bf^2*(1.8*v0*x0(2) + 1.8*x0(1)*cdg + (1.8*x0(2)^2 - 1.8*v0^2) - (v0 - x0(2))*L)/(cdg*L*m);%Lgb5
            A_bf = [LgBf 0];
            b_bf = -LfBf+1/Bf;
            if(numel(y1)~= 0) % if ip vehicle exists
                h = y1(1) - x0(1) - 1.8*x0(2) - 0.5*(y1(2) - x0(2))^2/(cd*g); %CBF for safety between i and ip
                Bf = 1/h;
                LfBf_ip = -(m*(y1(2) - x0(2)) - m*(y1(2) - x0(2))*y1(3)/(cd*g) - Fr*((y1(2) - x0(2))/(cd*g) - 1.8))*Bf^2/m;
                LgBf_ip = (1.8 - (y1(2) - x0(2))/(cd*g))*Bf^2/m;
                b_ip = -LfBf_ip + 1/Bf;
                A_ip = [LgBf_ip, 0];        
            else
                b_ip = [];
                A_ip = [];
            end
            A = [phi1 -1;A_bf;A_ip; 1 0; -1 0; A_vmax 0; A_vmin 0];
            b = [-phi0;b_bf; b_ip; ca*m*g; cd*m*g; b_vmax; b_vmin];
    end
    H = [2/(m^2) 0; 0 2*psc];
    F = [-2*Fr/(m^2); 0];

    options = optimoptions('quadprog',...
        'Algorithm','interior-point-convex','Display','off');
    [u,~,~,~,~] = quadprog(H,F,A,b,[],[],[],[],[],options);
    if (numel(u) == 0)  % if no solution found (usually happens in very heavy traffic, then use minimum deceleration or 0 deceleration)
       u = [-cd*m*g 0]; % minimum deceleration
    end
    a = (u(1) - Fr)/m;
    t=[0 dt];
    [~,xx]=ode45('acc',t,[x0, u(1), v0]);
    pos = xx(end, 1);
    vel = xx(end, 2);
    dis = xx(end, 3);
else
    pos = x0(1) + x0(2)*dt;
    vel = x0(2);
    dis = 1.8; % unused
    a = 0;
end

%record vehicle info add by lc
vehSteteRecItem = [SimTime, vehicleID, pos, vel, a];
vehStateRec_CBF = [vehStateRec_CBF; vehSteteRecItem];
vehStateRec_CBF = sortrows(vehStateRec_CBF, 2 );
save vehStateRec_CBF vehStateRec_CBF; %save info
end
