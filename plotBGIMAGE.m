function pen = plotBGIMAGE()

plot([-20,500],[10,10],'b',[-20,500],[0,0],'b--',...
    [-20,362.68],[-10,-10],'b',[402.68,500],[-10,-10],'b',...
    [24.53,362.28],[-205,-10],'r',[36.27,400],[-210,0],'r--',...
    [42.88,402.28],[-217.5,-10],'r');
hold on;
t = -50;
y=zeros(100,1);

p = plot(t,y,'o','MarkerSize',10);
p_color = zeros(1, 100); % check if a CAV has determined its color
axis([-20 500 -250 50]);
hold on


text(300, 40, 'Ave. Fuel/mL:');
fuel_legend = text(410, 40, '');
text(300, 20, 'Ave. Time/s:');
time_legend = text(410, 20, '');
text(380, -30, 'Throughput:');
through_legend = text(480, -30, '');

pen.p = p; 
pen.p_color = p_color; 
pen.fuel = fuel_legend; 
pen.time = time_legend;
pen.through = through_legend;

end