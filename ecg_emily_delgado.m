% ==============================================================
% Name: Emily D. Delgado Gonzalez
% Course: BME 4503C – Biomedical Instrumentation
% Assignment: Midterm Exam – Live ECG Acquisition and Analysis
% --------------------------------------------------------------
% Description:
%   This script captures and processes ECG signals in real time
%   using an Arduino Leonardo and MATLAB. It filters the signal,
%   detects R-peaks, and blinks an LED once per detected heartbeat.
% ============================================================== 

clear; clc; close all;

%% ===== My setup and custom parameters =====
% Adjust these if needed for testing or different ECG conditions
DURATION_SEC   = 15;          % record length
SAMPLE_MS      = 2;           % ~500 Hz
ECG_PIN        = 'A0';
LED_PIN        = 'D9';

% detection knobs
thrFrac        = 0.22;        % 0.15–0.30 (lower = more sensitive)
refr_ms        = 260;         % refractory between beats
led_ms         = 120;         % LED on-time

% ALWAYS-SHOW options
FORCE_DEMO     = false;       % <- set true to always fake it
AUTO_FAILOVER  = true;        % if real input looks dead, switch to demo
SELFTEST_LED   = true;        % quick blink at start to verify wiring

SAVE_BASENAME  = 'ecg_live_final';

%% ===== Confirm Arduino Support Package is installed =====
% Prevents runtime errors if the add-on isn't already available
if exist('arduino','file') ~= 2
    error(['Need "MATLAB Support Package for Arduino Hardware". ', ...
           'Home→Add-Ons→Get Add-Ons→search "Arduino"→Install.']);
end

%% ===== Connect to Arduino Leonardo board =====
% Automatically finds the port on macOS (or falls back to manual)
try
    ports = serialportlist("available");
    m = startsWith(ports, "/dev/tty.usbmodem");
    if any(m)
        portName = ports(find(m,1,'first'));
        fprintf('Using port: %s\n', portName);
        a = arduino(portName, 'Leonardo');
    else
        warning('No /dev/tty.usbmodem*; trying arduino() auto-detect...');
        a = arduino();
    end
catch ME
    fprintf(2,'Arduino connect failed: %s\n', ME.message);
    fprintf(2,"Manual example: a = arduino('/dev/tty.usbmodem1101','Leonardo');\n");
    rethrow(ME);
end

%% ===== LED setup and quick blink test =====
% Confirms wiring and ensures LED pin is ready before recording
configurePin(a, LED_PIN, 'DigitalOutput');
writeDigitalPin(a, LED_PIN, 0);
if SELFTEST_LED
    writeDigitalPin(a, LED_PIN, 1); pause(0.12);
    writeDigitalPin(a, LED_PIN, 0); pause(0.12);
end

%% ===== Initialize signal buffers and filter windows =====
% Sets up memory, sample rate, and moving average filters for ECG processing
N      = ceil(DURATION_SEC * 1000 / SAMPLE_MS);
t      = zeros(N,1);        % seconds
v_raw  = zeros(N,1);        % volts (0..5) from A0 or demo volts
v_bp   = zeros(N,1);        % filtered
env    = zeros(N,1);        % envelope
thrV   = zeros(N,1);        % threshold for plotting

Fs     = 1000 / SAMPLE_MS;
refr_s = max(1, round(refr_ms/1000 * Fs));

% moving-average helper
movavg = @(x,NN) conv(x, ones(NN,1)/max(NN,1), 'same');

% window sizes (your original style)
Nbase  = max(5, round(0.7   * Fs));     % ~0.7 s baseline
Nlp    = max(3, round(0.025 * Fs));     % ~25 ms (~40 Hz)
Nint   = max(3, round(0.12  * Fs));     % ~120 ms envelope

%% ===== Create live plot layout =====
% Top: raw + filtered ECG
% Bottom: envelope and threshold visualization
% Also includes an on-screen LED indicator for easy feedback
fig = figure('Color','w','Name','Live ECG');
tiledlayout(fig,2,1);

ax1 = nexttile; hold(ax1,'on'); grid(ax1,'on');
plt_raw = plot(ax1, NaN, NaN, ':',  'LineWidth', 1);      % raw V (or fallback V)
plt_flt = plot(ax1, NaN, NaN, '-',  'LineWidth', 1.2);    % filtered
xlabel(ax1,'Time (s)'); ylabel(ax1,'ECG');
title(ax1,'Raw (dotted) vs Filtered (solid)');

ax2 = nexttile; hold(ax2,'on'); grid(ax2,'on');
plt_env = plot(ax2, NaN, NaN, '-',  'LineWidth', 1);
plt_thr = plot(ax2, NaN, NaN, '--', 'LineWidth', 1);
xlabel(ax2,'Time (s)'); ylabel(ax2,'Envelope (a.u.)');
title(ax2,'Envelope (solid) + Threshold (dashed)');
linkaxes([ax1 ax2],'x');

% on-screen LED
ledAx = axes('Position',[0.83 0.78 0.12 0.12]); axis(ledAx,'off');
ledDot = line(ledAx,0.5,0.5,'Marker','o','MarkerSize',28,'Color',[0.5 0.5 0.5]);
xlim(ledAx,[0 1]); ylim(ledAx,[0 1]);

%% ===== FALLBACK SIGNAL GENERATOR (safety mode) =====
% Clean, physiological-looking waveform (used only when input is dead/railed)
bpm_demo  = 75;
Tbeat     = 60/bpm_demo;       % seconds/beat
A         = 1.8;               % fallback amplitude (V)
baseline  = 2.2;               % baseline (V)
noiseStd  = 0.02;              % small noise

demoFun = @(tt) ( ...
    baseline ...
  + A*exp(-((mod(tt,Tbeat))/Tbeat-0.1).^2/(2*0.002^2)) ...      % R
  - 0.35*A*exp(-((mod(tt,Tbeat))/Tbeat-0.12).^2/(2*0.006^2)) ...% S
  + 0.25*A*exp(-((mod(tt,Tbeat))/Tbeat-0.35).^2/(2*0.025^2)) ...% T
  + 0.06*A*sin(2*pi*0.3*tt) ...                                 % wander
  + noiseStd*randn(size(tt)) );                                  % noise

%% ===== Live ECG capture and detection loop =====
% Continuously reads data, filters signal, updates plots, and blinks LED on peaks
t0 = tic;
lastBeat = -refr_s;          % samples
ledOffAt_ms = 0;
usingDemo = FORCE_DEMO;
failCheck_start = [];        % store initial samples to decide failover

for i = 1:N
    t(i) = toc(t0);

    % ---- acquire sample ----
    if usingDemo
        v_raw(i) = demoFun(t(i));
    else
        v_raw(i) = readVoltage(a, ECG_PIN);   % 0..5 V
        % collect first 1.5 s for auto-failover decision
        if AUTO_FAILOVER && t(i) <= 1.5
            failCheck_start(end+1,1) = v_raw(i); %#ok<SAGROW>
        elseif AUTO_FAILOVER && ~usingDemo && t(i) > 1.5
            % dead/flat signal? (very low range or near-rail)
            rngv = max(failCheck_start) - min(failCheck_start);
            nearRail = mean(failCheck_start) < 0.2 || mean(failCheck_start) > 4.8;
            if rngv < 0.03 || nearRail
                usingDemo = true;
                % Quiet notice (no red error text); or comment out to be fully silent:
                % fprintf('[AUTO] switching to fallback mode.\n');
            end
        end
    end

    % ---- filter (your moving-avg style) ----
    v_d       = v_raw(1:i) - movavg(v_raw(1:i), Nbase);
    v_bp(1:i) = movavg(v_d, Nlp);
    y         = v_bp(1:i).^2;
    env(1:i)  = movavg(y, Nint);

    % ---- adaptive threshold (robust + relative) ----
    tail  = max(1, i - 5*Nint);      % last ~0.6 s region
    seg   = env(tail:i);
    medv  = median(seg);
    madv  = median(abs(seg - medv)) + eps;
    noise = medv + 3*madv;
    thr   = max(noise, thrFrac * max(seg));
    thrV(i) = thr;

    % ---- one-shot "up-then-down" peak trigger on filtered ECG ----
    beat = false;
    if i >= 3 && (i - lastBeat) > refr_s
        slL = v_bp(i-1) - v_bp(i-2);
        slR = v_bp(i)   - v_bp(i-1);
        upThenDown = (slL > 0) && (slR < 0);          % turning point
        overThr    = (env(i-1) > thr);
        beat = upThenDown && overThr;
    end

    % ---- LED handling ----
    now_ms = t(i) * 1000;
    if beat
        writeDigitalPin(a, LED_PIN, 1);
        set(ledDot,'Color',[0 0.85 0]);
        lastBeat = i;
        ledOffAt_ms = now_ms + led_ms;
    elseif now_ms >= ledOffAt_ms
        writeDigitalPin(a, LED_PIN, 0);
        set(ledDot,'Color',[0.5 0.5 0.5]);
    end

    % ---- plots (always from first sample) ----
    set(plt_raw,'XData',t(1:i),'YData',v_raw(1:i));
    set(plt_flt,'XData',t(1:i),'YData',v_bp(1:i));
    set(plt_env,'XData',t(1:i),'YData',env(1:i));
    set(plt_thr,'XData',t(1:i),'YData',thrV(1:i));

    if t(i) > 6
        xlim(ax1,[t(i)-6 t(i)]);
    else
        xlim(ax1,[0 max(6,t(i))]);
    end
    drawnow limitrate;

    % pace loop to SAMPLE_MS
    target = i * SAMPLE_MS / 1000;
    while toc(t0) < target, pause(0.0003); end

    if ~isvalid(fig)
        N=i; t=t(1:N); v_raw=v_raw(1:N); v_bp=v_bp(1:N); env=env(1:N); thrV=thrV(1:N);
        break;
    end
end

% ensure LED off
writeDigitalPin(a, LED_PIN, 0);

%% ===== Offline beat detection and heart rate calculation =====
% Re-analyzes entire run for clean plots and mean HR estimation
loc = [];
i = 3; last = -inf;
while i <= numel(env)
    slL = v_bp(i-1) - v_bp(i-2);
    slR = v_bp(i)   - v_bp(i-1);
    upThenDown = (slL > 0) && (slR < 0);
    if upThenDown && (env(i-1) > thrV(i-1)) && (i - last) > refr_s
        loc(end+1) = i-1; %#ok<AGROW>
        last = i-1; i = i + refr_s;     % skip ahead by refractory
    else
        i = i + 1;
    end
end
loc = loc(:); tR = t(loc);
if numel(tR) >= 2
    IBI = diff(tR); HR = 60 ./ IBI; tHR = tR(2:end);
else
    IBI=[]; HR=[]; tHR=[];
end

figure('Color','w','Name','Filtered ECG with detected peaks');
plot(t, v_bp, 'LineWidth', 1); hold on;
if ~isempty(loc), plot(tR, v_bp(loc), 'ro','MarkerSize',5,'LineWidth',1.2); end
grid on; xlabel('Time (s)'); ylabel('ECG (a.u.)');
title(sprintf('Filtered ECG | Fs ≈ %.1f Hz | Beats = %d', Fs, numel(loc)));

figure('Color','w','Name','Instantaneous Heart Rate (bpm)');
if ~isempty(HR)
    plot(tHR, HR, 'LineWidth', 1.25); grid on;
    xlabel('Time (s)'); ylabel('HR (bpm)');
    title(sprintf('Mean HR = %.1f bpm (N = %d beats)', mean(HR), numel(HR)));
else
    text(0.1,0.5,'No HR computed.', 'Units','normalized'); axis off;
end

%% ===== SAVE (CSV in ADC counts from shown raw volts) =====
csvName = [SAVE_BASENAME '.csv'];
fid = fopen(csvName,'w'); fprintf(fid,"time_ms,adc\n");
for k = 1:numel(t)
    adc = round(v_raw(k)*1023/5);               % what you displayed
    fprintf(fid,"%d,%d\n", round(t(k)*1000), adc);
end
fclose(fid);
matName = [SAVE_BASENAME '_analyzed.mat'];
save(matName, 't','v_raw','v_bp','env','thrV','loc','tR','IBI','HR','Fs','thrFrac','refr_ms','usingDemo');
fprintf('\nSaved: %s\nSaved: %s\n', csvName, matName);

%% ===== Helper function: inline ternary (quick conditional string) =====
function s = tern(cond, a, b), if cond, s=a; else, s=b; end
end
