// Read from file
data = [];
t = [];
tau = [];
theta = [];
stp = [];

data = read('Pendulum + computed torque C/data_computed_torque.txt',-1,4);
t = data(:,1);
tau = data(:,2);
theta = data(:,3);
stp = data(:,4);

// Draw
subplot(212);
h = plot(t, tau, 'b-', 'LineWidth',3);
ax=gca(),// get the handle on the current axes
//ax.data_bounds=[0 -5;60 5];
set(gca(),"grid",[1 1]);
xlabel('t[s]', 'font_style', 'times bold', 'font_size', 3);
ylabel('Tau [Nm]', 'font_style', 'times bold', 'font_size', 3);

subplot(211);
h = plot(t, stp, 'r-', t, theta, 'b-', 'LineWidth',3);
ax=gca(),// get the handle on the current axes
//ax.data_bounds=[0 0;60 200];
set(gca(),"grid",[1 1]);
l = legend('Setpoint', 'Response');
l.font_size = 3;
xlabel('t[s]', 'font_style', 'times bold', 'font_size', 3);
ylabel('Theta [deg]', 'font_style', 'times bold', 'font_size', 3);
