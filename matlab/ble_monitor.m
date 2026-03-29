function ble_monitor()
% BLE_MONITOR - Real-time BLE motor monitor + JOG control
% All-traditional-figure version (no uifigure — fast drawnow)
%
% Usage: ble_monitor()

    %% Configuration
    DEVICE_NAME = "";  % Set by scan dialog at startup
    DSPS_SERVICE_UUID   = "0783B03E-8535-B5A0-7140-A304D2495CB7";
    SERVER_TX_UUID      = "0783B03E-8535-B5A0-7140-A304D2495CB8";
    SERVER_RX_UUID      = "0783B03E-8535-B5A0-7140-A304D2495CBA";
    FLOW_CTRL_UUID      = "0783B03E-8535-B5A0-7140-A304D2495CB9";

    TARGET_HZ = 100;

    %% User defaults
    DEFAULT_CONTROL_MODE = 1;
    DEFAULT_SPEED = 0;
    DEFAULT_TORQUE = 0.06;
    DEFAULT_MAX_SPEED = 1000;

    % Load saved settings (overrides defaults)
    settingsFile = fullfile(fileparts(mfilename('fullpath')), 'ble_monitor_settings.mat');
    if exist(settingsFile, 'file')
        s = load(settingsFile);
        if isfield(s,'ctrl_mode'), DEFAULT_CONTROL_MODE = s.ctrl_mode; end
        if isfield(s,'speed'), DEFAULT_SPEED = s.speed; end
        if isfield(s,'torque'), DEFAULT_TORQUE = s.torque; end
        if isfield(s,'max_speed'), DEFAULT_MAX_SPEED = s.max_speed; end
    end
    CHAN_NAMES = {'(none)', 'RPM', 'Iq', 'Id', 'Vbat', 'Tinv', 'Treg', 'AccX', 'AccY', 'AccZ', 'GyrX', 'GyrY', 'GyrZ', 'IqRef', 'IdRef', 'SpdInteg', 'Vmag2'};
    CHAN_COLORS = {[0 0 0], [0 0.45 0.74], [0.85 0.33 0.1], [0.93 0.1 0.1], [0.47 0.67 0.19], ...
                  [0.49 0.18 0.56], [0.64 0.08 0.18], ...
                  [0.93 0.69 0.13], [0.3 0.75 0.93], [0.64 0.08 0.18], ...
                  [0.47 0.67 0.19], [0.85 0.33 0.1], [0 0.45 0.74], [0.3 0.3 0.3], [0.6 0.2 0.2], [0.1 0.6 0.5], [0.8 0.4 0.0]};
    CHAN_UNITS = {'', 'RPM', 'PU', 'PU', 'V', 'C', 'C', 'g', 'g', 'g', 'dps', 'dps', 'dps', 'PU', 'PU', 'PU', 'PU^2'};
    NUM_PLOTS = 5;
    defaultChans = [2, 3, 4, 5, 1];

    %% State
    running = true;
    bleConnected = false;
    logActive = false;
    totalSamples = 0; totalMsgs = 0; restartCount = 0; csumFail = 0;
    lastRxTic = uint64(0); logStartTic = uint64(0);
    rxBuf = '';
    lastConnCheckTic = tic;
    jogEnablePending = false; jogEnableTic = uint64(0);
    jogEnablePendingE1 = false;
    jogControlMode = DEFAULT_CONTROL_MODE;
    cmdQueue = {}; cmdQueueTic = tic;
    jogVerifyPending = false;
    jogVerifyTic = tic;
    jogVerifyKey = '';      % variable name to verify
    jogVerifyVal = 0;       % expected value
    jogVerifyCmd = '';      % command to resend if verify fails
    jogVerifyRetries = 0;
    b = []; charTx = []; charRx = []; charFlow = [];
    csvSaveFolder = pwd;

    %% CSV storage
    csvMaxRows = 100000;
    csvTime = NaN(csvMaxRows,1); csvRpm = NaN(csvMaxRows,1); csvIq = NaN(csvMaxRows,1);
    csvVbat = NaN(csvMaxRows,1); csvTinv = NaN(csvMaxRows,1); csvTreg = NaN(csvMaxRows,1);
    csvAx = NaN(csvMaxRows,1); csvAy = NaN(csvMaxRows,1); csvAz = NaN(csvMaxRows,1);
    csvGx = NaN(csvMaxRows,1); csvGy = NaN(csvMaxRows,1); csvGz = NaN(csvMaxRows,1);
    csvId = NaN(csvMaxRows,1);
    csvIqRef = NaN(csvMaxRows,1);
    csvIdRef = NaN(csvMaxRows,1);
    csvSpdInteg = NaN(csvMaxRows,1);
    csvVmag2 = NaN(csvMaxRows,1);
    csvIdx = 0;

    %% Screen layout
    scrSz = get(0, 'ScreenSize');
    plotW = round(scrSz(3) * 0.62);
    ctrlW = scrSz(3) - plotW - 10;
    figH = round(scrSz(4) * 0.88);
    figY = round((scrSz(4) - figH) / 2);

    %% === Plot figure (left) ===
    plotFig = figure('Name', 'BLE Live Plots', 'NumberTitle', 'off', ...
        'Position', [1, figY, plotW, figH], 'CloseRequestFcn', @onClose);
    axArr = gobjects(1, NUM_PLOTS);
    lineArr = gobjects(1, NUM_PLOTS);
    for p = 1:NUM_PLOTS
        axArr(p) = subplot(NUM_PLOTS, 1, p, 'Parent', plotFig);
        grid(axArr(p), 'on'); hold(axArr(p), 'on');
        axArr(p).FontSize = 9;
        if p < NUM_PLOTS, set(axArr(p), 'XTickLabel', {}); end
        ci = defaultChans(p);
        lineArr(p) = plot(axArr(p), NaN, NaN, '-', 'LineWidth', 1, 'Color', CHAN_COLORS{ci});
        ylabel(axArr(p), CHAN_NAMES{ci}, 'FontSize', 9);
    end
    xlabel(axArr(NUM_PLOTS), 'Time (s)', 'FontSize', 9);

    %% === Control figure (right) — all uicontrol, no uifigure ===
    ctrlFig = figure('Name', 'BLE Motor Control', 'NumberTitle', 'off', ...
        'MenuBar', 'none', 'ToolBar', 'none', ...
        'Position', [plotW + 10, figY, ctrlW, figH], 'CloseRequestFcn', @onClose);

    % Normalized positioning helper
    cW = ctrlW; cH = figH;
    px = @(x) x/cW;   % x pixels to normalized
    py = @(y) y/cH;
    pw = @(w) w/cW;
    ph = @(h) h/cH;

    % --- JOG Section (top) ---
    yy = cH - 10;  % current y position (from top)
    rowH = 35; gap = 5;

    yy = yy - rowH;
    btnEnable = uicontrol(ctrlFig, 'Style', 'togglebutton', 'String', 'MOTOR OFF', ...
        'Units', 'normalized', 'Position', [px(10) py(yy) pw(cW-20) ph(rowH)], ...
        'FontSize', 13, 'FontWeight', 'bold', 'BackgroundColor', [0.7 0.7 0.7], ...
        'Callback', @(h,~) onEnableToggle(h));

    yy = yy - rowH - gap;
    if DEFAULT_CONTROL_MODE==0, spd_bg=[0.3 0.7 1.0]; spd_fw='bold'; tq_bg=[0.85 0.85 0.85]; tq_fw='normal';
    else, spd_bg=[0.85 0.85 0.85]; spd_fw='normal'; tq_bg=[1.0 0.6 0.2]; tq_fw='bold'; end
    btnModeSpeed = uicontrol(ctrlFig, 'Style', 'pushbutton', 'String', 'Speed', ...
        'Units', 'normalized', 'Position', [px(10) py(yy) pw((cW-30)/2) ph(rowH)], ...
        'FontSize', 11, 'FontWeight', spd_fw, 'BackgroundColor', spd_bg, ...
        'Callback', @(~,~) setControlMode(0));
    btnModeTorque = uicontrol(ctrlFig, 'Style', 'pushbutton', 'String', 'Torque', ...
        'Units', 'normalized', 'Position', [px(cW/2+5) py(yy) pw((cW-30)/2) ph(rowH)], ...
        'FontSize', 11, 'FontWeight', tq_fw, 'BackgroundColor', tq_bg, ...
        'Callback', @(~,~) setControlMode(1));

    yy = yy - rowH - gap;
    uicontrol(ctrlFig, 'Style', 'pushbutton', 'String', 'EMERGENCY STOP', ...
        'Units', 'normalized', 'Position', [px(10) py(yy) pw(cW-20) ph(rowH)], ...
        'FontSize', 12, 'FontWeight', 'bold', 'BackgroundColor', [1 0.2 0.2], ...
        'ForegroundColor', [1 1 1], 'Callback', @(~,~) emergencyStop());

    % Speed slider + edit
    yy = yy - 22 - gap;
    uicontrol(ctrlFig, 'Style', 'text', 'String', 'Speed [RPM]', ...
        'Units', 'normalized', 'Position', [px(10) py(yy) pw(cW-20) ph(18)], ...
        'FontSize', 10, 'FontWeight', 'bold', 'HorizontalAlignment', 'left');
    yy = yy - 25 - gap;
    sldSpeed = uicontrol(ctrlFig, 'Style', 'slider', 'Min', 0, 'Max', 500, 'Value', DEFAULT_SPEED, ...
        'Units', 'normalized', 'Position', [px(10) py(yy) pw(cW-100) ph(22)], ...
        'Callback', @(h,~) onSpeedSlider(h));
    efSpeedVal = uicontrol(ctrlFig, 'Style', 'edit', 'String', sprintf('%d', DEFAULT_SPEED), ...
        'Units', 'normalized', 'Position', [px(cW-80) py(yy) pw(70) ph(25)], ...
        'FontSize', 11, 'Callback', @(h,~) onSpeedEdit(h));

    % Quick speed buttons
    yy = yy - 30 - gap;
    qSpeeds = [0 50 100 200 300];
    bw = (cW - 20 - 4*gap) / 5;
    for qi = 1:5
        uicontrol(ctrlFig, 'Style', 'pushbutton', 'String', sprintf('%d', qSpeeds(qi)), ...
            'Units', 'normalized', 'Position', [px(10+(qi-1)*(bw+gap)) py(yy) pw(bw) ph(28)], ...
            'FontSize', 10, 'Callback', @(~,~) quickSpeed(qSpeeds(qi)));
    end

    % Torque slider + edit
    yy = yy - 22 - gap*2;
    uicontrol(ctrlFig, 'Style', 'text', 'String', 'Torque Iq [PU]', ...
        'Units', 'normalized', 'Position', [px(10) py(yy) pw(cW-20) ph(18)], ...
        'FontSize', 10, 'FontWeight', 'bold', 'HorizontalAlignment', 'left');
    yy = yy - 25 - gap;
    sldTorque = uicontrol(ctrlFig, 'Style', 'slider', 'Min', -0.3, 'Max', 0.3, 'Value', DEFAULT_TORQUE, ...
        'Units', 'normalized', 'Position', [px(10) py(yy) pw(cW-100) ph(22)], ...
        'Callback', @(h,~) onTorqueSlider(h));
    efTorqueVal = uicontrol(ctrlFig, 'Style', 'edit', 'String', sprintf('%.3f', DEFAULT_TORQUE), ...
        'Units', 'normalized', 'Position', [px(cW-80) py(yy) pw(70) ph(25)], ...
        'FontSize', 11, 'Callback', @(h,~) onTorqueEdit(h));

    % Max speed (torque mode limiter)
    yy = yy - 22 - gap;
    uicontrol(ctrlFig, 'Style', 'text', 'String', 'Max Speed [RPM]', ...
        'Units', 'normalized', 'Position', [px(10) py(yy) pw(cW-120) ph(18)], ...
        'FontSize', 10, 'FontWeight', 'bold', 'HorizontalAlignment', 'left');
    efMaxSpeed = uicontrol(ctrlFig, 'Style', 'edit', 'String', sprintf('%d', DEFAULT_MAX_SPEED), ...
        'Units', 'normalized', 'Position', [px(cW-110) py(yy) pw(60) ph(24)], ...
        'FontSize', 11, 'Callback', @(h,~) onMaxSpeedEdit(h));
    uicontrol(ctrlFig, 'Style', 'pushbutton', 'String', 'Set', ...
        'Units', 'normalized', 'Position', [px(cW-40) py(yy) pw(30) ph(24)], ...
        'FontSize', 9, 'Callback', @(~,~) onMaxSpeedEdit(efMaxSpeed));

    % Save / Reset buttons
    yy = yy - 26 - gap;
    uicontrol(ctrlFig, 'Style', 'pushbutton', 'String', 'Save Settings', ...
        'Units', 'normalized', 'Position', [px(10) py(yy) pw((cW-30)/2) ph(24)], ...
        'FontSize', 9, 'BackgroundColor', [0.7 0.85 1.0], 'Callback', @(~,~) saveSettings());
    uicontrol(ctrlFig, 'Style', 'pushbutton', 'String', 'Reset', ...
        'Units', 'normalized', 'Position', [px(cW/2+5) py(yy) pw((cW-30)/2) ph(24)], ...
        'FontSize', 9, 'Callback', @(~,~) resetSettings());

    % --- Divider ---
    yy = yy - 10;

    % --- LOG controls ---
    yy = yy - 30 - gap;
    btnStart = uicontrol(ctrlFig, 'Style', 'pushbutton', 'String', 'LOG START', ...
        'Units', 'normalized', 'Position', [px(10) py(yy) pw((cW-30)/2) ph(30)], ...
        'FontSize', 10, 'BackgroundColor', [0.3 0.8 0.3], ...
        'Callback', @(~,~) startLog());
    uicontrol(ctrlFig, 'Style', 'pushbutton', 'String', 'STOP+Save', ...
        'Units', 'normalized', 'Position', [px(cW/2+5) py(yy) pw((cW-30)/2) ph(30)], ...
        'FontSize', 10, 'BackgroundColor', [0.8 0.3 0.3], ...
        'Callback', @(~,~) stopLog());

    % Window / Hz
    yy = yy - 28 - gap;
    uicontrol(ctrlFig, 'Style', 'text', 'String', 'Window(s):', ...
        'Units', 'normalized', 'Position', [px(10) py(yy) pw(70) ph(22)], ...
        'FontSize', 9, 'HorizontalAlignment', 'right');
    efWindow = uicontrol(ctrlFig, 'Style', 'edit', 'String', '10', ...
        'Units', 'normalized', 'Position', [px(85) py(yy) pw(50) ph(24)], 'FontSize', 10);
    uicontrol(ctrlFig, 'Style', 'text', 'String', 'Hz:', ...
        'Units', 'normalized', 'Position', [px(145) py(yy) pw(30) ph(22)], ...
        'FontSize', 9, 'HorizontalAlignment', 'right');
    efHz = uicontrol(ctrlFig, 'Style', 'edit', 'String', '100', ...
        'Units', 'normalized', 'Position', [px(180) py(yy) pw(50) ph(24)], 'FontSize', 10);

    % BLE status + reconnect
    yy = yy - 28 - gap;
    lblBleStatus = uicontrol(ctrlFig, 'Style', 'text', 'String', 'BLE: --', ...
        'Units', 'normalized', 'Position', [px(10) py(yy) pw(cW/2-10) ph(22)], ...
        'FontSize', 10, 'FontWeight', 'bold', 'ForegroundColor', [0.5 0.5 0.5], ...
        'HorizontalAlignment', 'left');
    uicontrol(ctrlFig, 'Style', 'pushbutton', 'String', 'Reconnect', ...
        'Units', 'normalized', 'Position', [px(cW/2+5) py(yy) pw(cW/2-15) ph(26)], ...
        'FontSize', 9, 'Callback', @(~,~) reconnectBLE());

    % Status label
    yy = yy - 22 - gap;
    statusLbl = uicontrol(ctrlFig, 'Style', 'text', 'String', 'Ready', ...
        'Units', 'normalized', 'Position', [px(10) py(yy) pw(cW-20) ph(20)], ...
        'FontSize', 10, 'HorizontalAlignment', 'left');

    % --- Raw command ---
    yy = yy - 28 - gap;
    efCmd = uicontrol(ctrlFig, 'Style', 'edit', 'String', '', ...
        'Units', 'normalized', 'Position', [px(10) py(yy) pw(cW-100) ph(26)], 'FontSize', 10);
    uicontrol(ctrlFig, 'Style', 'pushbutton', 'String', 'Send', ...
        'Units', 'normalized', 'Position', [px(cW-80) py(yy) pw(70) ph(26)], ...
        'FontSize', 10, 'Callback', @(~,~) sendRawCmd());

    % --- Log text area (remaining space) ---
    logH = yy - gap - 10;
    taLog = uicontrol(ctrlFig, 'Style', 'listbox', 'String', {''}, ...
        'Units', 'normalized', 'Position', [px(10) py(10) pw(cW-20) ph(logH)], ...
        'FontSize', 9, 'Max', 2, 'Enable', 'inactive');

    %% Channel selection popups (on plot figure, top-right of each subplot)
    chanSelect = gobjects(1, NUM_PLOTS);
    for p = 1:NUM_PLOTS
        pos = get(axArr(p), 'Position');  % [left bottom width height] normalized
        chanSelect(p) = uicontrol(plotFig, 'Style', 'popupmenu', 'String', CHAN_NAMES, ...
            'Value', defaultChans(p), 'Units', 'normalized', ...
            'Position', [pos(1)+pos(3)-0.08, pos(2)+pos(4)-0.03, 0.08, 0.025], ...
            'FontSize', 8, 'Tag', num2str(p), ...
            'Callback', @(h,~) onChanChanged(str2double(get(h,'Tag')), h));
    end

    %% Scan and select BLE device
    if isempty(DEVICE_NAME) || DEVICE_NAME == ""
        fprintf('Scanning for BLE devices...\n');
        tbl = blelist;
        % Filter for SPS_531 devices
        mask = contains(tbl.Name, 'SPS');
        if any(mask)
            sps_tbl = tbl(mask, :);
        else
            sps_tbl = tbl(1:min(5,height(tbl)), :);  % Show top 5 if no SPS found
        end
        % Build selection list
        items = cell(height(sps_tbl), 1);
        for si = 1:height(sps_tbl)
            items{si} = sprintf('%s (%s) RSSI:%d', sps_tbl.Name(si), sps_tbl.Address(si), sps_tbl.RSSI(si));
        end
        [sel, ok] = listdlg('ListString', items, 'SelectionMode', 'single', ...
            'PromptString', 'Select BLE device:', 'ListSize', [350 150], 'Name', 'BLE Scan');
        if ok && sel > 0
            DEVICE_NAME = char(sps_tbl.Address(sel));
            fprintf('Selected: %s\n', DEVICE_NAME);
        else
            fprintf('No device selected. Exiting.\n');
            running = false;
        end
    end

    %% Connect BLE
    if running
        connectBLE();
    end

    %% Main loop
    t0 = tic; plotTic = tic;

    while running && isvalid(plotFig) && isvalid(ctrlFig)
        pause(0.05);
        drawnow;  % fast: all-traditional figures, no uifigure overhead

        % Command queue (500ms spacing for DA14531 buffer flush)
        if ~isempty(cmdQueue) && toc(cmdQueueTic) > 0.5
            cmd = cmdQueue{1}; cmdQueue(1) = []; cmdQueueTic = tic;
            if strcmp(cmd, '__STOP_LOG__')
                stopLog();
            elseif strcmp(cmd, '__VERIFY__')
                if jogVerifyKey(1) == 's'  % speed_ref_rpm
                    sendShort('?M');  % 2 bytes, response "=300\r\n" (6 bytes)
                else
                    sendShort('?T');  % 2 bytes, response "=0.100\r\n"
                end
                jogVerifyPending = true;
                jogVerifyTic = tic;
            else
                sendShort(cmd);
            end
        end

        % BLE connection check (every 3s)
        if toc(lastConnCheckTic) > 3.0
            lastConnCheckTic = tic;
            updateBleStatus();
        end

        if ~logActive, continue; end
        if ~bleConnected, continue; end

        % Fallback timeout (3s) — suppress during command queue processing
        if lastRxTic ~= 0 && toc(lastRxTic) > 3.0 && isempty(cmdQueue)
            restartCount = restartCount + 1;
            fprintf('[%s] timeout, resend LOG START %d (#%d)\n', timeStr(), TARGET_HZ, restartCount);
            sendShort(sprintf('LOG START %d', TARGET_HZ));
            lastRxTic = tic;
        end

        % Deferred JOG enable
        if jogEnablePending && toc(jogEnableTic) > 20.0
            jogEnablePending = false;
            set(btnEnable, 'Value', 0, 'String', 'MOTOR OFF', 'BackgroundColor', [0.7 0.7 0.7]);
            appendLog('JOG: timeout');
        elseif jogEnablePending && totalSamples > 10
            jogEnablePending = false;
            % Send commands DURING streaming (no S/R — keep data flowing)
            mc = sprintf('C%d', jogControlMode);
            ms = sprintf('M%d', round(get(sldSpeed,'Value')));
            mt = sprintf('T%d', round(get(sldTorque,'Value')*1000));
            mw = sprintf('W%d', round(str2double(get(efMaxSpeed,'String'))));
            cmdQueue = [cmdQueue, {mc, ms, mt, mw, '__VERIFY__'}];  % E1 is sent AFTER verify OK
            if jogControlMode == 0
                jogVerifyKey = 'speed_ref_rpm';
                jogVerifyVal = round(get(sldSpeed,'Value'));
                jogVerifyCmd = ms;
            else
                jogVerifyKey = 'torque_ref_iq';
                jogVerifyVal = get(sldTorque,'Value');
                jogVerifyCmd = mt;
            end
            jogVerifyRetries = 0;
            cmdQueueTic = tic;
            set(btnEnable, 'String', 'MOTOR ON', 'BackgroundColor', [0.2 0.8 0.2]);
            fprintf('[%s] JOG: queued commands\n', timeStr());
        end

        % Verify timeout: if ALL response didn't arrive within 3s, retry
        if jogVerifyPending && toc(jogVerifyTic) > 3.0
            jogVerifyPending = false;
            jogVerifyRetries = jogVerifyRetries + 1;
            if jogVerifyRetries <= 5
                fprintf('[%s] VERIFY TIMEOUT, retry %d\n', timeStr(), jogVerifyRetries);
                cmdQueue = [cmdQueue, {jogVerifyCmd, jogVerifyCmd, '__VERIFY__'}];
                cmdQueueTic = tic;
            else
                appendLog('VERIFY: gave up after 5 retries');
            end
        end

        % Plot update (0.5s)
        if toc(plotTic) > 0.5 && csvIdx > 0
            plotTic = tic;
            try refreshPlot(); catch; end
        end
    end

    % Cleanup
    disconnectBLE();
    if isvalid(plotFig), delete(plotFig); end
    if isvalid(ctrlFig), delete(ctrlFig); end
    fprintf('Done.\n');

    %% === BLE Connection ===
    function connectBLE()
        try
            set(lblBleStatus, 'String', 'BLE: Scanning...', 'ForegroundColor', [0.85 0.55 0]);
            drawnow;
            if ~isempty(b), try delete(b); catch; end; b = []; end
            fprintf('Scanning for %s...\n', DEVICE_NAME);
            b = ble(DEVICE_NAME);
            fprintf('Connected: %s (%s)\n', b.Name, b.Address);
            charTx   = characteristic(b, DSPS_SERVICE_UUID, SERVER_TX_UUID);
            charRx   = characteristic(b, DSPS_SERVICE_UUID, SERVER_RX_UUID);
            charFlow = characteristic(b, DSPS_SERVICE_UUID, FLOW_CTRL_UUID);
            write(charFlow, uint8(1), 'WithoutResponse');
            fprintf('XON sent.\n');
            subscribe(charTx); charTx.DataAvailableFcn = @onNotify;
            subscribe(charFlow); charFlow.DataAvailableFcn = @onFlowNotify;
            fprintf('Notifications subscribed.\n');
            bleConnected = true;
            set(lblBleStatus, 'String', 'BLE: Connected', 'ForegroundColor', [0 0.6 0]);
            appendLog(sprintf('Connected: %s (%s)', b.Name, b.Address));
        catch e
            bleConnected = false;
            set(lblBleStatus, 'String', 'BLE: Disconnected', 'ForegroundColor', [0.8 0 0]);
            fprintf('BLE connect failed: %s\n', e.message);
            appendLog(sprintf('Connect failed: %s', e.message));
        end
    end

    function disconnectBLE()
        try
            if bleConnected
                sendShort('S');
                pause(0.2);
                try unsubscribe(charTx); catch; end
                try unsubscribe(charFlow); catch; end
            end
        catch; end
        bleConnected = false;
        if isvalid(ctrlFig)
            set(lblBleStatus, 'String', 'BLE: Disconnected', 'ForegroundColor', [0.8 0 0]);
        end
    end

    function reconnectBLE()
        appendLog('Reconnecting...');
        disconnectBLE(); pause(0.5); connectBLE();
    end

    function updateBleStatus()
        if ~bleConnected, return; end
        try
            if isempty(b) || ~isvalid(b)
                bleConnected = false;
                set(lblBleStatus, 'String', 'BLE: Disconnected', 'ForegroundColor', [0.8 0 0]);
            end
        catch
            bleConnected = false;
            set(lblBleStatus, 'String', 'BLE: Disconnected', 'ForegroundColor', [0.8 0 0]);
        end
    end

    %% === BLE Callbacks ===
    function onNotify(src, ~)
        if ~running, return; end
        try
            raw = read(src);
            if isempty(raw), return; end
            rxBuf = [rxBuf char(raw(:)')];
            while true
                nlPos = find(rxBuf == char(10), 1);
                if isempty(nlPos), break; end
                ln = strtrim(rxBuf(1:nlPos));
                rxBuf = rxBuf(nlPos+1:end);
                if ~isempty(ln), processLine(ln); end
            end
        catch; end
    end

    function onFlowNotify(src, ~)
        try
            v = read(src);
            if ~isempty(v) && v(1)==0, fprintf('[%s] FLOW: XOFF\n', timeStr()); end
        catch; end
    end

    function processLine(line)
        if line(1) == 'B' && contains(line, ',')
            lastRxTic = tic;
            totalMsgs = totalMsgs + 1;

            % XOR checksum verification: "B5,...,data*XX" → verify XX
            starPos = find(line == '*', 1, 'last');
            if ~isempty(starPos) && starPos + 2 <= length(line)
                payload = line(1:starPos-1);
                csumHex = line(starPos+1:starPos+2);
                csumRecv = hex2dec(csumHex);
                csumCalc = uint8(0);
                for ci = 1:length(payload)
                    csumCalc = bitxor(csumCalc, uint8(payload(ci)));
                end
                if csumCalc ~= csumRecv
                    totalMsgs = totalMsgs - 1;
                    csumFail = csumFail + 1;
                    return;  % discard corrupted packet
                end
                line = payload;  % strip checksum for parsing
            end

            parts = strsplit(line(2:end), ',');
            n = str2double(parts{1});
            if isnan(n) || n < 1, return; end
            tNow = toc(t0);
            es = n + 2;
            iq=0; vb=0; ti=0; tr=0; acx=0; acy=0; acz=0; gcx=0; gcy=0; gcz=0; id=0;
            if numel(parts)>=es,   iq=str2double(parts{es})/1000; end
            if numel(parts)>=es+1, vb=str2double(parts{es+1}) / 10; end  % vbat*10 -> V
            if numel(parts)>=es+2, acx=str2double(parts{es+2})/1000; end
            if numel(parts)>=es+3, acy=str2double(parts{es+3})/1000; end
            if numel(parts)>=es+4, acz=str2double(parts{es+4})/1000; end
            if numel(parts)>=es+5, gcx=str2double(parts{es+5})/10; end
            if numel(parts)>=es+6, gcy=str2double(parts{es+6})/10; end
            if numel(parts)>=es+7, gcz=str2double(parts{es+7})/10; end
            if numel(parts)>=es+8, id=str2double(parts{es+8})/1000; end  % Id*1000 -> PU
            iqref=0; idref=0; spdinteg=0; vmag2=0;
            if numel(parts)>=es+13, iqref=str2double(parts{es+13})/1000; end  % IdqRef[1]*1000 -> PU
            if numel(parts)>=es+14, idref=str2double(parts{es+14})/1000; end  % IdqRef[0]*1000 -> PU
            if numel(parts)>=es+15, spdinteg=str2double(parts{es+15})/1000; end  % speed PI integrator*1000 -> PU
            if numel(parts)>=es+16, vmag2=str2double(parts{es+16})/1000; end  % Vmag2*1000
            dt = 1.0/TARGET_HZ;
            tBurstStart = tNow-(n-1)*dt;
            if csvIdx>0 && ~isnan(csvTime(csvIdx))
                tMin2 = csvTime(csvIdx)+dt;
                if tBurstStart<tMin2, tBurstStart=tMin2; end
            end
            for k = 1:min(n, numel(parts)-1)
                rpm = str2double(parts{k+1});
                totalSamples = totalSamples + 1;
                csvIdx = csvIdx + 1;
                if csvIdx > csvMaxRows
                    csvMaxRows = csvMaxRows + 50000;
                    csvTime(end+1:csvMaxRows)=NaN; csvRpm(end+1:csvMaxRows)=NaN;
                    csvIq(end+1:csvMaxRows)=NaN; csvVbat(end+1:csvMaxRows)=NaN;
                    csvTinv(end+1:csvMaxRows)=NaN; csvTreg(end+1:csvMaxRows)=NaN;
                    csvAx(end+1:csvMaxRows)=NaN; csvAy(end+1:csvMaxRows)=NaN;
                    csvAz(end+1:csvMaxRows)=NaN; csvGx(end+1:csvMaxRows)=NaN;
                    csvGy(end+1:csvMaxRows)=NaN; csvGz(end+1:csvMaxRows)=NaN;
                    csvId(end+1:csvMaxRows)=NaN;
                    csvIqRef(end+1:csvMaxRows)=NaN;
                    csvIdRef(end+1:csvMaxRows)=NaN;
                    csvSpdInteg(end+1:csvMaxRows)=NaN;
                    csvVmag2(end+1:csvMaxRows)=NaN;
                end
                csvTime(csvIdx)=tBurstStart+(k-1)*dt;
                csvRpm(csvIdx)=rpm; csvIq(csvIdx)=iq; csvVbat(csvIdx)=vb;
                csvTinv(csvIdx)=ti; csvTreg(csvIdx)=tr;
                csvAx(csvIdx)=acx; csvAy(csvIdx)=acy; csvAz(csvIdx)=acz;
                csvGx(csvIdx)=gcx; csvGy(csvIdx)=gcy; csvGz(csvIdx)=gcz;
                csvId(csvIdx)=id;
                csvIqRef(csvIdx)=iqref;
                csvIdRef(csvIdx)=idref;
                csvSpdInteg(csvIdx)=spdinteg;
                csvVmag2(csvIdx)=vmag2;
            end
        elseif startsWith(line,'ALL ')
            appendLog(line);

        elseif line(1) == '=' && jogVerifyPending
            % Short verify response: "=300" from ?M or "=0.150" from ?T
            jogVerifyPending = false;
            actual = str2double(line(2:end));
            % Tolerance: speed ±1 RPM, torque ±0.01 PU
            tol = 1.0;
            if jogVerifyKey(1) == 't', tol = 0.01; end
            if isnan(actual) || abs(actual - jogVerifyVal) > tol
                jogVerifyRetries = jogVerifyRetries + 1;
                if jogVerifyRetries <= 5
                    fprintf('[%s] VERIFY FAIL: got=%s (want %g), retry %d\n', ...
                        timeStr(), line(2:end), jogVerifyVal, jogVerifyRetries);
                    cmdQueue = [cmdQueue, {jogVerifyCmd, jogVerifyCmd, '__VERIFY__'}];
                    cmdQueueTic = tic;
                else
                    appendLog('VERIFY: gave up after 5 retries');
                end
            else
                fprintf('[%s] VERIFY OK: %s=%g\n', timeStr(), jogVerifyKey, actual);
                jogVerifyRetries = 0;
                % Enable motor only on first verify after MOTOR ON (avoid repeated E1)
                if jogEnablePendingE1
                    jogEnablePendingE1 = false;
                    sendShort('E1');
                    fprintf('[%s] ENABLE sent (after verify)\n', timeStr());
                end
            end
        elseif line(1)=='V' && contains(line,'=')
            % D command dump response - print to console
            fprintf('%s\n', line);
        elseif strcmp(line,'VEND')
            fprintf('--- dump complete ---\n');
        elseif line(1)=='#'
            % G/P command response - print to console + log
            fprintf('%s\n', line);
            appendLog(line);
        elseif startsWith(line,'OK ')
            appendLog(line);
        elseif startsWith(line,'ERR unknown')
            % BLE corruption noise - ignore
        elseif startsWith(line,'ERR ')
            appendLog(line);
        elseif line(1)=='L' && ~startsWith(line,'LIST')
            lastRxTic=tic; totalMsgs=totalMsgs+1; totalSamples=totalSamples+1;
        else
            appendLog(line);
        end
    end

    %% === Plot ===
    function refreshPlot()
        if csvIdx==0 || ~isvalid(plotFig), return; end
        tMax = csvTime(csvIdx);
        winSec = str2double(get(efWindow,'String'));
        if isnan(winSec) || winSec<1, winSec=10; end
        tMin1 = tMax - winSec;
        iStart=csvIdx;
        while iStart>1 && csvTime(iStart-1)>=tMin1, iStart=iStart-1; end
        nWin=csvIdx-iStart+1;
        if nWin<1, return; end
        if nWin>300, step=floor(nWin/300); else, step=1; end
        wi=iStart:step:csvIdx;
        tWi=csvTime(wi);
        for p=1:NUM_PLOTS
            ci = get(chanSelect(p),'Value');
            if ci==1, continue; end
            data=getChanData(ci);
            if isempty(data), continue; end
            set(lineArr(p),'XData',tWi,'YData',data(wi));
            set(axArr(p),'XLim',[tMin1 tMax]);
        end
        % Status update
        if logStartTic~=0
            el=toc(logStartTic); rate=totalSamples/max(el,0.001);
            set(statusLbl,'String',sprintf('%d samp | %.1f/s | %d bad', totalSamples, rate, csumFail));
        end
    end

    function data = getChanData(ci)
        switch ci
            case 2, data=csvRpm; case 3, data=csvIq; case 4, data=csvId;
            case 5, data=csvVbat; case 6, data=csvTinv; case 7, data=csvTreg;
            case 8, data=csvAx; case 9, data=csvAy; case 10, data=csvAz;
            case 11, data=csvGx; case 12, data=csvGy; case 13, data=csvGz;
            case 14, data=csvIqRef;
            case 15, data=csvIdRef;
            case 16, data=csvSpdInteg;
            case 17, data=csvVmag2;
            otherwise, data=[];
        end
    end

    function onChanChanged(plotIdx, h)
        ci = get(h,'Value');
        if ci==1
            set(lineArr(plotIdx),'XData',NaN,'YData',NaN);
            ylabel(axArr(plotIdx),'');
        else
            lineArr(plotIdx).Color = CHAN_COLORS{ci};
            ylabel(axArr(plotIdx), [CHAN_NAMES{ci} ' (' CHAN_UNITS{ci} ')'], 'FontSize', 9);
        end
    end

    %% === Start / Stop ===
    function startLog()
        if ~bleConnected, appendLog('Not connected.'); return; end
        TARGET_HZ = str2double(get(efHz,'String'));
        if isnan(TARGET_HZ)||TARGET_HZ<1, TARGET_HZ=100; end
        totalSamples=0; totalMsgs=0; restartCount=0; csvIdx=0;
        logStartTic=tic; t0=tic; logActive=true;
        sendShort(sprintf('LOG START %d', TARGET_HZ));
        lastRxTic=tic;
        fprintf('[%s] TX> LOG START %d\n', timeStr(), TARGET_HZ);
        set(statusLbl,'String',sprintf('Streaming %dHz...', TARGET_HZ));
    end

    function stopLog()
        logActive=false;
        if bleConnected, sendShort('E0'); sendShort('S'); sendShort('S'); end
        set(btnEnable,'Value',0,'String','MOTOR OFF','BackgroundColor',[0.7 0.7 0.7]);
        if logStartTic~=0
            el=toc(logStartTic); rate=totalSamples/max(el,0.001);
            fprintf('=== %d samples (%d msgs) in %.0fs (%.1f samp/s, %d restarts, %d csum_fail) ===\n', ...
                totalSamples, totalMsgs, el, rate, restartCount, csumFail);
        end
        if csvIdx>0
            folder=uigetdir(csvSaveFolder,'Save CSV folder');
            if folder~=0
                csvSaveFolder=folder;
                fname=fullfile(folder,sprintf('motor_log_%s.csv',datestr(now,'yyyymmdd_HHMMSS')));
                vi=1:csvIdx;
                T=table(csvTime(vi),csvRpm(vi),csvIq(vi),csvVbat(vi),csvTinv(vi),csvTreg(vi),...
                    csvAx(vi),csvAy(vi),csvAz(vi),csvGx(vi),csvGy(vi),csvGz(vi),csvId(vi),csvIqRef(vi),csvIdRef(vi),csvSpdInteg(vi),csvVmag2(vi),...
                    'VariableNames',{'Time_s','RPM','Iq_PU','Vbat_V','Tinv_C','Treg_C',...
                    'AccX_g','AccY_g','AccZ_g','GyrX_dps','GyrY_dps','GyrZ_dps','Id_PU','IqRef_PU','IdRef_PU','SpdInteg_PU','Vmag2'});
                writetable(T,fname);
                fprintf('Saved %d samples to %s\n',csvIdx,fname);
                appendLog(sprintf('Saved %d samples to %s',csvIdx,fname));
            else
                appendLog('CSV save cancelled.');
            end
        end
        set(statusLbl,'String','Stopped'); drawnow;
    end

    function clearAll()
        csvIdx=0; totalSamples=0; totalMsgs=0; restartCount=0; t0=tic;
        for p=1:NUM_PLOTS, set(lineArr(p),'XData',NaN,'YData',NaN); end
        set(statusLbl,'String','Cleared');
    end

    %% === JOG Control ===
    function onEnableToggle(h)
        if get(h,'Value')
            if ~logActive, startLog(); end
            jogEnablePending=true; jogEnableTic=tic;
            jogEnablePendingE1=true;
            set(h,'String','STARTING...','BackgroundColor',[1.0 0.8 0.2]);
        else
            sendShort('E0');
            set(h,'String','MOTOR OFF','BackgroundColor',[0.7 0.7 0.7]);
            cmdQueue=[cmdQueue,{'__STOP_LOG__'}]; cmdQueueTic=tic;
        end
    end

    function setControlMode(mode)
        jogControlMode=mode;
        queueCmd(sprintf('C%d',mode));
        if mode==0
            set(btnModeSpeed,'BackgroundColor',[0.3 0.7 1.0],'FontWeight','bold');
            set(btnModeTorque,'BackgroundColor',[0.85 0.85 0.85],'FontWeight','normal');
        else
            set(btnModeSpeed,'BackgroundColor',[0.85 0.85 0.85],'FontWeight','normal');
            set(btnModeTorque,'BackgroundColor',[1.0 0.6 0.2],'FontWeight','bold');
        end
    end

    function emergencyStop()
        sendShort('E0'); sendShort('E0'); sendShort('M0'); sendShort('T0');
        set(btnEnable,'Value',0,'String','MOTOR OFF','BackgroundColor',[0.7 0.7 0.7]);
        set(sldSpeed,'Value',0); set(efSpeedVal,'String','0');
        set(sldTorque,'Value',0); set(efTorqueVal,'String','0.000');
    end

    function queueCmd(cmd)
        cmdQueue=[cmdQueue,{cmd}];
        if isempty(cmdQueueTic)||toc(cmdQueueTic)>1.0, cmdQueueTic=tic; end
    end

    function sendWithSlowdown(cmd)
        % Send command during streaming (no S/R — keep motor running)
        if logActive
            cmdQueue = [cmdQueue, {cmd, cmd, '__VERIFY__'}];  % 2x for reliability
            if cmd(1) == 'M'
                jogVerifyKey = 'speed_ref_rpm';
                jogVerifyVal = str2double(cmd(2:end));
                jogVerifyCmd = cmd;
                jogVerifyRetries = 0;
            elseif cmd(1) == 'T'
                jogVerifyKey = 'torque_ref_iq';
                jogVerifyVal = str2double(cmd(2:end)) * 0.001;  % milliPU → PU
                jogVerifyCmd = cmd;
                jogVerifyRetries = 0;
            end
        else
            cmdQueue = [cmdQueue, {cmd}];
        end
        cmdQueueTic = tic;
    end

    function onSpeedSlider(h)
        v=round(get(h,'Value')); set(efSpeedVal,'String',sprintf('%d',v));
        sendWithSlowdown(sprintf('M%d',v));
    end
    function onSpeedEdit(h)
        v=round(str2double(get(h,'String'))); if isnan(v), v=0; end
        set(sldSpeed,'Value',max(0,min(500,v))); set(h,'String',sprintf('%d',v));
        sendWithSlowdown(sprintf('M%d',v));
    end
    function onTorqueSlider(h)
        v=get(h,'Value'); set(efTorqueVal,'String',sprintf('%.3f',v));
        sendWithSlowdown(sprintf('T%d',round(v*1000)));  % milliPU: 0.018 → T18
    end
    function onTorqueEdit(h)
        v=str2double(get(h,'String')); if isnan(v), v=0; end
        set(sldTorque,'Value',max(-0.3,min(0.3,v))); set(h,'String',sprintf('%.3f',v));
        sendWithSlowdown(sprintf('T%d',round(v*1000)));
    end
    function quickSpeed(rpm)
        set(sldSpeed,'Value',rpm); set(efSpeedVal,'String',sprintf('%d',rpm));
        sendWithSlowdown(sprintf('M%d',rpm));
    end

    function onMaxSpeedEdit(h)
        v=round(str2double(get(h,'String'))); if isnan(v)||v<0, v=500; end
        set(h,'String',sprintf('%d',v));
        sendWithSlowdown(sprintf('W%d',v));
    end

    function saveSettings()
        ctrl_mode = jogControlMode;
        speed = get(sldSpeed, 'Value');
        torque = get(sldTorque, 'Value');
        max_speed = str2double(get(efMaxSpeed, 'String'));
        save(settingsFile, 'ctrl_mode', 'speed', 'torque', 'max_speed');
        appendLog('Settings saved');
    end

    function resetSettings()
        if exist(settingsFile, 'file'), delete(settingsFile); end
        jogControlMode = 0;
        set(sldSpeed, 'Value', 0); set(efSpeedVal, 'String', '0');
        set(sldTorque, 'Value', 0); set(efTorqueVal, 'String', '0.000');
        set(efMaxSpeed, 'String', '500');
        set(btnModeSpeed, 'BackgroundColor', [0.3 0.7 1.0], 'FontWeight', 'bold');
        set(btnModeTorque, 'BackgroundColor', [0.85 0.85 0.85], 'FontWeight', 'normal');
        appendLog('Settings reset to defaults');
    end


    %% === Helpers ===
    function appendLog(txt)
        if ~isvalid(ctrlFig), return; end
        old = get(taLog,'String');
        if ~iscell(old), old={old}; end
        if numel(old)>200, old=old(end-199:end); end
        new = [old; {sprintf('[%s] %s', timeStr(), txt)}];
        set(taLog,'String',new,'Value',numel(new));
    end

    function sendRawCmd()
        if ~bleConnected, appendLog('Not connected.'); return; end
        cmd=strtrim(get(efCmd,'String'));
        if isempty(cmd), return; end
        sendShort(cmd); appendLog(['TX> ' cmd]); set(efCmd,'String','');
    end

    function sendShort(cmd)
        if ~bleConnected, return; end
        try
            write(charRx, uint8([cmd 13 10]), 'WithoutResponse');
        catch e
            fprintf('TX err: %s\n', e.message);
            bleConnected=false;
            set(lblBleStatus,'String','BLE: Disconnected','ForegroundColor',[0.8 0 0]);
        end
    end

    function s = timeStr()
        s = datestr(now, 'HH:MM:SS.FFF');
    end

    function onClose(~,~)
        running=false; disconnectBLE();
        if isvalid(plotFig), delete(plotFig); end
        if isvalid(ctrlFig), delete(ctrlFig); end
    end
end
