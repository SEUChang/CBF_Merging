function [] = savePerformance( performanceItem, type )
%the helper to record OC & CBF controller performance
%type 1 OC   type 2 CBF
    persistent OCperformanceRec;  % define persistent variable for OC
    persistent CBFperformanceRec; %  define persistent variable for CBF
    if type == 1 
        if isempty( OCperformanceRec )
            OCperformanceRec = [];
        end 
        file_name = "OCperformanceRec";
        OCperformanceRec = [OCperformanceRec; performanceItem];
        save (file_name,"OCperformanceRec");        % save  performanceMetrics
        
    else%type == 2
        if isempty( CBFperformanceRec )
            CBFperformanceRec = [];
        end
        file_name = "CBFperformanceRec";
        CBFperformanceRec = [CBFperformanceRec; performanceItem];
        save (file_name,"CBFperformanceRec");           % save  performanceMetrics
    end
  

end

    %% get system time and append it to file name
%     T = fix(clock);
%     T_length = length(T);
%     T_now = [];
%     for i = 2:T_length
%         if  T(i)<10   %²¹Áã
%             t0 = num2str(0);
%             t1 = num2str(T(i));
%             t = [t0 t1];
%             T_now = [T_now t];       
%         else
%             t = num2str(T(i));
%             T_now = [T_now t];     
%         end
%     end
%     T_NOW = [num2str(T(1)) T_now];
%     file_name = "perfomanceRec_" + T_NOW; 