%[text] # Lab 0x09: Improved Motor Control with Velocity Observation
%[text] Work through this example from the augmented continuous motor model to the discrete observer. Change the marked design selections for your robot. The .mlx version is the student live script; this .m file is its readable authoring equivalent. Run one version only, with the .sltx template here. Requires MATLAB, Control System Toolbox, and Simulink. No Python setup.
%[text] State order: \[wheel velocity (rad/s); displacement (rad); disturbance (rad/s^2)\]. Observer input order: \[limited motor voltage (V); encoder displacement (rad)\]. Reference: [https://charlierefvem.github.io/ME4305/case-studies/case\_motor\_observer.html](https://charlierefvem.github.io/ME4305/case-studies/case_motor_observer.html)
%[text] ## 1. Select physical parameters and the firmware sample period
%[text] STUDENT: replace these nominal example values with your documented motor model and sample period. A shared model for both wheels is a starting point; use separate designs if your evidence supports different motor parameters.
Km = 3.5188;          % Steady wheel speed per volt, (rad/s)/V from lab 0x02
tau = 0.0472;        % Motor time constant, s from lab 0x02
Ts = 0.03;         % Observer AND controller period, s
Vdc = 7.2;         % Example voltage limit / supply reference, V
assert(all(isfinite([Km tau Ts Vdc])) && all([Km tau Ts Vdc] > 0));
%%
%[text] ## 2. Build the continuous open-loop model, including the disturbance state
%[text] omega\_dot = -omega/tau + Km\*u/tau + d theta\_dot = omega; d\_dot = 0 (slowly varying disturbance approximation). C selects measured position. It does NOT select the velocity for feedback.
A = [-1/tau 0 1; 1 0 0; 0 0 0];
B = [Km/tau; 0; 0];
C = [0 1 0];
D = 0;
assert(rank(obsv(A,C)) == 3, 'Continuous model is not observable.');
sys = ss(A,B,C,D);
%%
%[text] ## 3. Discretize the augmented model using a zero-order hold
%[text] Augment first, then discretize. The command and disturbance are treated as constant over one sample interval. Do not invert A: it is singular here.
sysd = c2d(sys,Ts,'zoh');
[Ad,Bd,Cd,Dd] = ssdata(sysd);
assert(rank(obsv(Ad,Cd)) == 3, 'Discrete model is not observable.');
disp('Discrete plant matrices:'); %[output:84375ff7]
%Ad is the s=discrete state transition matrix
%Bd is the discrete input matrix
disp(Ad); disp(Bd); disp(Cd); disp(Dd); %[output:7b631f10]
%%
%[text] ## 4. Select observer error dynamics explicitly
%[text] STUDENT: choose these parameters directly, then compare a second choice. The settling-time relation is an approximate dominant-pair design guide, not a guaranteed motor-response specification. Faster poles can amplify encoder quantization. Keep Ts fixed while comparing observer tuning.
t_set = 0.30;      % Approximate 2-percent error settling time, s
zeta = 0.60;       % Damping ratio of the chosen dominant pair (0 < zeta < 1)
s3 = -10;          % Third continuous error pole, 1/s, explicitly selected
assert(isfinite(t_set) && t_set > 0);
assert(isfinite(zeta) && zeta > 0 && zeta < 1);
assert(isfinite(s3) && isreal(s3) && s3 < 0);
wn = 4/(zeta*t_set);
poles_s = [-zeta*wn + 1i*wn*sqrt(1-zeta^2); ...
           -zeta*wn - 1i*wn*sqrt(1-zeta^2); s3];
poles_z = exp(poles_s*Ts);
L = place(Ad',Cd',poles_z)';
%%
%[text] ## 5. Assemble Ao, Bo, Co, Do and check the error poles
%[text] xhat\_next = Ao\*xhat + Bo\*\[u; theta\] yhat = Co\*xhat + Do\*\[u; theta\] All three estimates are output so the unchanged three-output observer subsystem can display them. Velocity is yhat(1), position is yhat(2).
Ao = Ad - L*Cd;
Bo = [Bd-L*Dd L];
Co = eye(3);
Do = zeros(3,2);
assert(isequal(size(Ao),[3 3]) && isequal(size(Bo),[3 2]));
assert(isequal(size(Co),[3 3]) && isequal(size(Do),[3 2]));
assert(all(abs(eig(Ao)) < 1), 'Observer error poles must be inside unit circle.');
% Compare pole sets without depending on eigenvalue ordering.
assert(max(min(abs(eig(Ao)-poles_z.'),[],2)) < 1e-8);
disp(table(poles_s,poles_z,'VariableNames',{'Selected_s','Selected_z'})); %[output:442e4c17]
disp('Achieved observer error poles:'); disp(eig(Ao)); %[output:5a23e868]
% The supplied Simulink block retains the original underscore variable names.
A_o = Ao; B_o = Bo; C_o = Co; D_o = Do;


%%
%[text] ## 6. Print paste-ready arrays using MATLAB alone
%[text] Copy the full-precision output; do not copy rounded Workspace displays. No mat2numpystr, Python interpreter, or Python package is needed in MATLAB.
fprintf('\nfrom ulab import numpy as np\n'); %[output:711fd7e6]
printArray('Ao',Ao); printArray('Bo',Bo); %[output:67d05f0b]
printArray('Co',Co); printArray('Do',Do); %[output:0a8729f8]
% A small nonzero check catches input order, state order, and indexing errors.
% STUDENT: reproduce this update on the board before connecting the observer
% to hardware. Small floating-point differences are expected.
x_check = [1; 0.2; -0.5];
w_check = [2; 0.21];
y_check = Co*x_check + Do*w_check;
x_next_check = Ao*x_check + Bo*w_check;
disp('Check: x = [1; 0.2; -0.5], w = [2; 0.21]'); %[output:888446b5]
format long g
disp('Current output y_check:'); disp(y_check); %[output:3741dca5]
disp('Next state x_next_check:'); disp(x_next_check); %[output:59cf516d]
%%
%[text] ## 7. Select PI gains and create a working model from the template
%[text] STUDENT: select these gains explicitly. These are example starting values, not verified settings for your robot. Derivative and feedforward are absent. Simulink's PI blocks use the continuous integral gain (UseKiTs = off). Firmware uses I\_next = I + error and Ki\_sample = Ki\_cont\*Ts instead.
Kp = 0.6;       % V/(rad/s)
Ki_cont = 3.75;    % V/rad, for Simulink / a time-weighted integral
Ki_sample = Ki_cont*Ts; % V/(rad/s), for the firmware's per-sample sum
omega_set = 1.5;   % Example velocity step, rad/s
disp('SECTION 7 EXECUTED SUCCESSFULLY') %[output:9a75fef0]
disp(omega_set) %[output:4ee8b675]
t_stop = 1.0;      % Simulation duration, s
fprintf('Firmware gains: Kp = %.17g, Ki_sample = %.17g\n',Kp,Ki_sample); %[output:0383e4fe]
% The model compares finite difference, a five-sample average, and observation.
% Its three PI blocks all run at Ts, limit voltage to +/-Vdc, and use clamping.
% It includes encoder and PWM quantization, but does not model static friction.
% STUDENT: inspect the observer velocity against the known simulated velocity.
% Change observer tuning and compare tracking/noise; a smoother line alone
% does not establish a more accurate estimate. Retain your selected settings.
% Create a working model on the first run. Later runs reuse the open model
% so changes made while exploring it are preserved. Save your own .slx copy
% if desired; the distributed .sltx remains the reusable starting template.
template_file = fullfile(fileparts(mfilename('fullpath')), ...
    'lab09_observer_controller.sltx');
model_name = 'lab09_observer_controller';
if ~bdIsLoaded(model_name) %[output:group:8a71df21]
    Simulink.createFromTemplate(template_file,'Name',model_name); %[output:57fcd579]
end %[output:group:8a71df21]
out = sim(model_name,'StopTime',num2str(t_stop));
v_true = out.logsout.getElement('lab09_true_velocity').Values;
v_obs = out.logsout.getElement('lab09_observed_velocity').Values;
v_command = out.logsout.getElement('lab09_voltage').Values;
figure('Name','Lab 9 simulated observer check');
subplot(2,1,1);
plot(v_true.Time,v_true.Data,'DisplayName','Simulated true velocity');
hold on
stairs(v_obs.Time,v_obs.Data,'DisplayName','Observer velocity');
yline(omega_set,'--','DisplayName','Setpoint');
ylabel('Wheel velocity (rad/s)'); grid on; legend('Location','best');
subplot(2,1,2);
stairs(v_command.Time,v_command.Data);
ylabel('Limited command (V)'); xlabel('Time (s)'); grid on;
%%
%[text] ## 8. Record your design choices
%[text] STUDENT: add brief notes to your live script: source of Km/tau, selected Ts, two observer tuning choices and the observed tradeoff, selected design, and the board/MATLAB numeric-check result. The lab memo will summarize these. The model is an estimation demonstration, not proof of loaded-wheel behavior. Keep the .mlx and .sltx in a writable folder; generated caches are local.
function printArray(name,M)
% Print a two-dimensional floating-point array without external dependencies.
    fprintf('%s = np.array([\n',name);
    for row = 1:size(M,1)
        fprintf('    [');
        fprintf('%.17g',M(row,1));
        fprintf(', %.17g',M(row,2:end));
        fprintf('],\n');
    end
    fprintf('])\n');
end

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":8.4}
%---
%[output:84375ff7]
%   data: {"dataType":"text","outputData":{"text":"Discrete plant matrices:\n","truncated":false}}
%---
%[output:7b631f10]
%   data: {"dataType":"text","outputData":{"text":"         0.529621213034011                         0        0.0222018787447947\n        0.0222018787447947                         1      0.000368071323245691\n                         0                         0                         1\n\n          1.65516887557592\n        0.0274400290728165\n                         0\n\n     0     1     0\n\n     0\n\n","truncated":false}}
%---
%[output:442e4c17]
%   data: {"dataType":"text","outputData":{"text":"                <strong>Selected_s<\/strong>                             <strong>Selected_z<\/strong>             \n    <strong>___________________________________<\/strong>    <strong>___________________________________<\/strong>\n\n    -13.3333333333333+17.7777777777778i    0.577224100453174+0.34079510262509i\n    -13.3333333333333-17.7777777777778i    0.577224100453174-0.34079510262509i\n                  -10+0i                   0.740818220681718+0i               \n\n","truncated":false}}
%---
%[output:5a23e868]
%   data: {"dataType":"text","outputData":{"text":"Achieved observer error poles:\n          0.577224100453181 +     0.340795102625102i\n          0.577224100453181 -     0.340795102625102i\n          0.740818220681733 +                     0i\n\n","truncated":false}}
%---
%[output:711fd7e6]
%   data: {"dataType":"text","outputData":{"text":"\nfrom ulab import numpy as np\n","truncated":false}}
%---
%[output:67d05f0b]
%   data: {"dataType":"text","outputData":{"text":"Ao = np.array([\n    [0.529621213034011, -7.810618411905101, 0.022201878744794675],\n    [0.022201878744794675, 0.36564520855408578, 0.00036807132324569121],\n    [0, -114.74662688721376, 1],\n])\nBo = np.array([\n    [1.6551688755759217, 7.810618411905101],\n    [0.027440029072816494, 0.63435479144591422],\n    [0, 114.74662688721376],\n])\n","truncated":false}}
%---
%[output:0a8729f8]
%   data: {"dataType":"text","outputData":{"text":"Co = np.array([\n    [1, 0, 0],\n    [0, 1, 0],\n    [0, 0, 1],\n])\nDo = np.array([\n    [0, 0],\n    [0, 0],\n    [0, 0],\n])\n","truncated":false}}
%---
%[output:888446b5]
%   data: {"dataType":"text","outputData":{"text":"Check: x = [1; 0.2; -0.5], w = [2; 0.21]\n","truncated":false}}
%---
%[output:3741dca5]
%   data: {"dataType":"text","outputData":{"text":"Current output y_check:\n                         1\n                       0.2\n                      -0.5\n\n","truncated":false}}
%---
%[output:59cf516d]
%   data: {"dataType":"text","outputData":{"text":"Next state x_next_check:\n          3.90696420893251\n         0.283241449143264\n         0.647466268872137\n\n","truncated":false}}
%---
%[output:9a75fef0]
%   data: {"dataType":"text","outputData":{"text":"SECTION 7 EXECUTED SUCCESSFULLY\n","truncated":false}}
%---
%[output:4ee8b675]
%   data: {"dataType":"text","outputData":{"text":"                       1.5\n\n","truncated":false}}
%---
%[output:0383e4fe]
%   data: {"dataType":"text","outputData":{"text":"Firmware gains: Kp = 0.59999999999999998, Ki_sample = 0.11249999999999999\n","truncated":false}}
%---
%[output:57fcd579]
%   data: {"dataType":"error","outputData":{"errorType":"runtime","text":"Error using sltemplate.internal.findMatchingTemplate\nName 'C:\\Users\\jorda\\AppData\\Local\\Temp\\Editor_iwqcr\\lab09_observer_controller.sltx' does not match any template\n\nError in <a href=\"matlab:matlab.lang.internal.introspective.errorDocCallback('Simulink.createFromTemplate', 'C:\\Program Files\\MATLAB\\R2026b\\toolbox\\simulink\\sltemplate\\+Simulink\\createFromTemplate.m', 43)\" style=\"font-weight:bold\">Simulink.createFromTemplate<\/a> (<a href=\"matlab: opentoline('C:\\Program Files\\MATLAB\\R2026b\\toolbox\\simulink\\sltemplate\\+Simulink\\createFromTemplate.m',43,0)\">line 43<\/a>)\n        [filename,info] = sltemplate.internal.findMatchingTemplate(templatename);"}}
%---
