function plotPerformanceMetrics(fuel, time, car,fuel_legend,time_legend,through_legend)

f_val = num2str(fuel);
fuel_legend.String = f_val;

t_val = num2str(time);
time_legend.String = t_val;

c_val = num2str(car);
through_legend.String = c_val;
end