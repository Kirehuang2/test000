function ble_config()
% BLE_CONFIG - Parameter configuration tool for motor driver
% Robust BLE communication: brute-force send + notification-based read
%
% Usage: ble_config()

    %% BLE Configuration
    DEVICE_NAME = "SPS_531";
    DSPS_SERVICE_UUID   = "0783B03E-8535-B5A0-7140-A304D2495CB7";
    SERVER_TX_UUID      = "0783B03E-8535-B5A0-7140-A304D2495CB8";
    SERVER_RX_UUID      = "0783B03E-8535-B5A0-7140-A304D2495CBA";
    FLOW_CTRL_UUID      = "0783B03E-8535-B5A0-7140-A304D2495CB9";

    %% Parameter reference table
    % {No, 種類, 定数名称, 定数内容, 単位, 下限, 上限, デフォルト, regIdx(0-based), 現在値}
    REF = {
     1, 'モーター制御', 'enable',             'モーター有効/無効',                    '-',    '0',     '1',      '0',     14, '--';
     2, 'モーター制御', 'control_mode',        '制御モード (0=速度, 1=トルク)',        '-',    '0',     '1',      '0',     15, '--';
     3, 'モーター制御', 'speed_ref_rpm',       '速度指令値',                          'RPM',  '0',     '1000',   '0',     16, '--';
     4, 'モーター制御', 'torque_ref_iq',       'トルク電流指令値',                    'PU',   '-0.3',  '0.3',    '0',     17, '--';
     5, 'モーター制御', 'torque_ref_iq_max',   '最大トルク電流リミット',              'PU',   '0.01',  '1.0',    '0.3',   18, '--';
     6, 'モーター制御', 'torque_max_speed',    'トルクモード最大速度',                'RPM',  '0',     '4400',   '500',   19, '--';
     7, '電流制御',     'Kp_iq',              'Iq比例ゲイン',                        '-',    '0.1',   '2.0',    '0.5',   26, '--';
     8, '電流制御',     'Ki_iq',              'Iq積分ゲイン (0=無効)',                '-',    '0.0',   '0.1',    '0.0',   27, '--';
     9, '電流制御',     'Kp_id',              'Id比例ゲイン',                        '-',    '0.1',   '2.0',    '0.3',   24, '--';
    10, '電流制御',     'Ki_id',              'Id積分ゲイン (0=無効)',                '-',    '0.0',   '0.1',    '0.0',   25, '--';
    11, '速度制御',     'Kp_speed',           '速度PI比例ゲイン',                    '-',    '0.1',   '5.0',    '1.0',   28, '--';
    12, '速度制御',     'Ki_speed',           '速度PI積分ゲイン',                    '-',    '0.0',   '1.0',    '0.1',   29, '--';
    13, '速度推定',     'speed_filter_alpha', '速度LPFフィルタ係数 (小=滑らか)',      '-',    '0.005', '1.0',    '0.02',  20, '--';
    14, '電圧補償',     'vdc_compensation_en','Vdcフィードフォワード補償',            '-',    '0',     '1',      '1',     21, '--';
    15, 'マキタ電池',   'makita_bat_en',      'マキタバッテリ保護 有効/無効',         '-',    '0',     '1',      '0',     30, '--';
    16, 'LED',          'led_mode',           'LEDモード (0=自動, 1=手動)',           '-',    '0',     '1',      '0',     22, '--';
    17, 'LED',          'led_state',          'LEDビットマスク b0=緑 b1=赤 b2=Turbo', '-',   '0',     '63',     '0',     23, '--';
    };
    nRef = size(REF, 1);

    READONLY = {
    18, 'モニタ', 'battery_voltage',  'バッテリ電圧',            'V',     2, '--';
    19, 'モニタ', 'temp_inverter',    'インバータ温度',          'degC',  3, '--';
    20, 'モニタ', 'temp_regen',       '回生回路温度',            'degC',  4, '--';
    21, 'モニタ', 'protection_state', '保護状態 (0=正常)',       '-',     5, '--';
    22, 'モニタ', 'as_voltage',       'AS端子電圧',              'V',    31, '--';
    23, 'モニタ', 'makita_bat_state', '電池状態 (1=許可,2=禁止)','--',   32, '--';
    24, 'モニタ', 'SpeedFb_Hall_PU',  '速度FB',                  'PU',   6, '--';
    25, 'モニタ', 'IdqRef1',          'Iq指令値',                'PU',    7, '--';
    };
    nReadonly = size(READONLY, 1);

    %% State
    running = true;
    b = []; charTx = []; charRx = []; charFlow = [];
    bleConnected = false;
    rxBuf = '';
    dataReceived = false;  % true when first streaming data arrives
    refTableHandle = [];

    %% GUI
    scrSz = get(0, 'ScreenSize');
    figW = 480; figH = 350;
    fig = figure('Name', 'BLE Config', 'NumberTitle', 'off', ...
        'MenuBar', 'none', 'ToolBar', 'none', ...
        'Position', [round((scrSz(3)-figW)/2), round((scrSz(4)-figH)/2), figW, figH], ...
        'CloseRequestFcn', @onClose);

    px=@(x)x/figW; py=@(y)y/figH; pw=@(w)w/figW; ph=@(h)h/figH;
    yy=figH-10; lx=10; rw=figW-20;

    % BLE
    yy=yy-28;
    lblStatus = uicontrol(fig,'Style','text','String','BLE: Not connected',...
        'Units','normalized','Position',[px(lx) py(yy) pw(rw*0.45) ph(22)],...
        'FontSize',10,'FontWeight','bold','ForegroundColor',[0.8 0 0],'HorizontalAlignment','left');
    uicontrol(fig,'Style','pushbutton','String','Connect',...
        'Units','normalized','Position',[px(rw*0.45+20) py(yy) pw(80) ph(24)],...
        'FontSize',10,'Callback',@(~,~) doConnect());
    uicontrol(fig,'Style','pushbutton','String','Disconnect',...
        'Units','normalized','Position',[px(rw*0.45+105) py(yy) pw(90) ph(24)],...
        'FontSize',9,'Callback',@(~,~) doDisconnect());

    % Ref Table + Read All
    yy=yy-32;
    uicontrol(fig,'Style','pushbutton','String','Ref Table',...
        'Units','normalized','Position',[px(lx) py(yy) pw(90) ph(28)],...
        'FontSize',10,'FontWeight','bold','BackgroundColor',[0.7 0.85 1.0],...
        'Callback',@(~,~) showRefTable());
    uicontrol(fig,'Style','pushbutton','String','Read All',...
        'Units','normalized','Position',[px(lx+100) py(yy) pw(80) ph(28)],...
        'FontSize',10,'Callback',@(~,~) readAllParams());

    % Write param
    yy=yy-32;
    uicontrol(fig,'Style','text','String','No:','Units','normalized',...
        'Position',[px(lx) py(yy) pw(25) ph(22)],'FontSize',10,'HorizontalAlignment','right');
    efNo = uicontrol(fig,'Style','edit','String','','Units','normalized',...
        'Position',[px(lx+28) py(yy) pw(40) ph(24)],'FontSize',11);
    uicontrol(fig,'Style','text','String','Value:','Units','normalized',...
        'Position',[px(lx+75) py(yy) pw(40) ph(22)],'FontSize',10,'HorizontalAlignment','right');
    efVal = uicontrol(fig,'Style','edit','String','','Units','normalized',...
        'Position',[px(lx+118) py(yy) pw(80) ph(24)],'FontSize',11);
    uicontrol(fig,'Style','pushbutton','String','Write',...
        'Units','normalized','Position',[px(lx+205) py(yy) pw(60) ph(26)],...
        'FontSize',10,'FontWeight','bold','BackgroundColor',[1 0.85 0.4],...
        'Callback',@(~,~) writeParam());
    lblResult = uicontrol(fig,'Style','text','String','',...
        'Units','normalized','Position',[px(lx+270) py(yy) pw(rw-270) ph(22)],...
        'FontSize',9,'HorizontalAlignment','left');

    % Raw command
    yy=yy-30;
    efCmd = uicontrol(fig,'Style','edit','String','','Units','normalized',...
        'Position',[px(lx) py(yy) pw(rw-80) ph(24)],'FontSize',10);
    uicontrol(fig,'Style','pushbutton','String','Send','Units','normalized',...
        'Position',[px(figW-90) py(yy) pw(70) ph(24)],'FontSize',9,...
        'Callback',@(~,~) sendRawCmd());

    % Log
    logH = max(yy-15, 40);
    taLog = uicontrol(fig,'Style','listbox','String',{'Ready.'},...
        'Units','normalized','Position',[px(lx) py(10/figH) pw(rw) ph(logH)],...
        'FontSize',8,'Max',2,'Enable','inactive');

    %% Main loop
    autoReadTic = tic;
    while running && isvalid(fig)
        pause(0.1);
        drawnow;
        % Auto refresh every 5 seconds
        if bleConnected && dataReceived && toc(autoReadTic) > 5.0
            autoReadTic = tic;
            sendCmd('D');  % single D (not blast — background refresh)
        end
    end
    doDisconnect();
    if isvalid(fig), delete(fig); end

    %% === BLE ===
    function doConnect()
        try
            set(lblStatus,'String','BLE: Scanning...','ForegroundColor',[0.85 0.55 0]); drawnow;
            if ~isempty(b), try delete(b); catch; end; b=[]; end
            b = ble(DEVICE_NAME);
            charTx = characteristic(b, DSPS_SERVICE_UUID, SERVER_TX_UUID);
            charRx = characteristic(b, DSPS_SERVICE_UUID, SERVER_RX_UUID);
            charFlow = characteristic(b, DSPS_SERVICE_UUID, FLOW_CTRL_UUID);
            write(charFlow, uint8(1), 'WithoutResponse');
            subscribe(charTx); charTx.DataAvailableFcn = @onNotify;
            bleConnected = true;
            set(lblStatus,'String','BLE: Connected','ForegroundColor',[0 0.6 0]);
            logMsg('Connected');

            % Start streaming: brute-force send until data arrives
            logMsg('Starting stream...');
            dataReceived = false;
            for attempt = 1:10
                blastCmd('R');  % 1 byte, most reliable
                pause(0.5);
                drawnow;
                if dataReceived
                    logMsg(sprintf('Stream OK (attempt %d)', attempt));
                    break;
                end
            end
            if ~dataReceived
                logMsg('Stream failed - try Read All manually');
            end

            % Auto read all params
            pause(0.5); drawnow;
            readAllParams();
        catch e
            bleConnected = false;
            set(lblStatus,'String','BLE: Failed','ForegroundColor',[0.8 0 0]);
            logMsg(sprintf('Failed: %s', e.message));
        end
    end

    function doDisconnect()
        try
            if bleConnected
                blastCmd('S');
                try unsubscribe(charTx); catch; end
            end
        catch; end
        bleConnected = false;
        if isvalid(fig)
            set(lblStatus,'String','BLE: Disconnected','ForegroundColor',[0.8 0 0]);
        end
    end

    % Send command 3 times rapidly (brute-force reliability)
    function blastCmd(cmd)
        if ~bleConnected, return; end
        for k = 1:3
            try
                write(charRx, uint8([cmd 13 10]), 'WithoutResponse');
            catch; end
            pause(0.05);
        end
    end

    function sendCmd(cmd)
        if ~bleConnected, return; end
        try
            write(charRx, uint8([cmd 13 10]), 'WithoutResponse');
        catch e
            logMsg(sprintf('TX err: %s', e.message));
            bleConnected = false;
        end
    end

    %% === Notify ===
    function onNotify(src, ~)
        try
            raw = read(src);
            if isempty(raw), return; end
            rxBuf = [rxBuf char(raw(:)')];
            while true
                nlPos = find(rxBuf == char(10), 1);
                if isempty(nlPos), break; end
                ln = strtrim(rxBuf(1:nlPos));
                rxBuf = rxBuf(nlPos+1:end);
                if ~isempty(ln), processResponse(ln); end
            end
        catch; end
    end

    function processResponse(line)
        if any(line < 32 & line ~= 9), return; end  % skip control chars

        if line(1) == 'V' && contains(line, '=')
            % Dump response: "V26=0.5000"
            tok = regexp(line, 'V(\d+)=([\S]+)', 'tokens', 'once');
            if ~isempty(tok)
                updateByIdx(str2double(tok{1}), tok{2});
            end
        elseif strcmp(line, 'VEND')
            logMsg('Dump complete');
            refreshRefTable();
        elseif line(1) == '#'
            % G/P response: "#26=0.5000"
            tok = regexp(line, '#(\d+)=([\S]+)', 'tokens', 'once');
            if ~isempty(tok)
                idx = str2double(tok{1});
                val = tok{2};
                updateByIdx(idx, val);
                % Show as Ref table No instead of regIdx
                refNo = idxToRefNo(idx);
                if refNo > 0
                    set(lblResult,'String',sprintf('No.%d = %s', refNo, val),'ForegroundColor',[0 0.6 0]);
                else
                    set(lblResult,'String',line,'ForegroundColor',[0 0.6 0]);
                end
            end
        elseif startsWith(line, 'OK ')
            logMsg(line);
            set(lblResult,'String',line,'ForegroundColor',[0 0.6 0]);
        elseif startsWith(line, 'ERR unknown')
            % BLE corruption noise - ignore
        elseif startsWith(line, 'ERR ')
            logMsg(line);
            set(lblResult,'String',line,'ForegroundColor',[0.8 0 0]);
        elseif line(1) == 'B'
            dataReceived = true;  % streaming is working
        else
            if all(line >= 32 & line <= 126)
                logMsg(line);
            end
        end
    end

    function updateByIdx(idx, val)
        for i=1:nRef
            if REF{i,9}==idx, REF{i,10}=val; end
        end
        for i=1:nReadonly
            if READONLY{i,6}==idx, READONLY{i,7}=val; end
        end
    end

    function no = idxToRefNo(idx)
        no = 0;
        for i=1:nRef
            if REF{i,9}==idx, no=REF{i,1}; return; end
        end
        for i=1:nReadonly
            if READONLY{i,6}==idx, no=READONLY{i,1}; return; end
        end
    end

    %% === Read / Write ===
    function readAllParams()
        if ~bleConnected, logMsg('Not connected'); return; end
        % D command x3 (brute-force) + wait
        blastCmd('D');
        pause(1.0); drawnow;
        blastCmd('D');
        logMsg('Dump requested');
    end

    function writeParam()
        if ~bleConnected, logMsg('Not connected'); return; end
        noStr = strtrim(get(efNo,'String'));
        valStr = strtrim(get(efVal,'String'));
        if isempty(noStr)||isempty(valStr), logMsg('Enter No and Value'); return; end
        no = str2double(noStr);
        if isnan(no)||no<1||no>nRef, logMsg(sprintf('No must be 1~%d',nRef)); return; end
        idx = REF{no, 9};
        cmd = sprintf('P%d=%s', idx, valStr);
        % Blast P command + verify with D
        blastCmd(cmd);
        set(lblResult,'String','Writing...','ForegroundColor',[0.5 0.5 0.5]);
        logMsg(sprintf('TX> %s', cmd));
        pause(0.5); drawnow;
        blastCmd('D');  % re-read all to verify
    end

    %% === Ref Table ===
    function showRefTable()
        if ~isempty(refTableHandle) && isvalid(refTableHandle)
            figure(refTableHandle); return;
        end
        refTableHandle = figure('Name','Parameter Reference','NumberTitle','off',...
            'MenuBar','none','ToolBar','none','Position',...
            [round((scrSz(3)-850)/2), round((scrSz(4)-550)/2), 850, 550]);
        refreshRefTable();
    end

    function refreshRefTable()
        if isempty(refTableHandle)||~isvalid(refTableHandle), return; end
        nTotal = nRef + 1 + nReadonly;
        td = cell(nTotal, 9);
        for r=1:nRef
            td{r,1}=REF{r,1}; td{r,2}=REF{r,2}; td{r,3}=REF{r,3};
            td{r,4}=REF{r,4}; td{r,5}=REF{r,5}; td{r,6}=REF{r,6};
            td{r,7}=REF{r,7}; td{r,8}=REF{r,8}; td{r,9}=REF{r,10};
        end
        sep=nRef+1;
        td{sep,1}=''; td{sep,2}='--- モニタ (読取専用) ---';
        for c=3:9, td{sep,c}=''; end
        for r=1:nReadonly
            row=sep+r;
            td{row,1}=READONLY{r,1}; td{row,2}=READONLY{r,2}; td{row,3}=READONLY{r,3};
            td{row,4}=READONLY{r,4}; td{row,5}=READONLY{r,5};
            td{row,6}='-'; td{row,7}='-'; td{row,8}='-'; td{row,9}=READONLY{r,7};
        end
        delete(get(refTableHandle,'Children'));
        uitable(refTableHandle,'Data',td,...
            'ColumnName',{'No','種類','定数名称','定数内容','単位','下限','上限','デフォルト','現在値'},...
            'ColumnWidth',{30,95,150,230,40,45,45,60,70},...
            'Units','normalized','Position',[0.01 0.01 0.98 0.98],...
            'FontSize',9,'RowName',[]);
    end

    %% === Helpers ===
    function sendRawCmd()
        if ~bleConnected, logMsg('Not connected'); return; end
        cmd=strtrim(get(efCmd,'String'));
        if isempty(cmd), return; end
        sendCmd(cmd); logMsg(sprintf('TX> %s',cmd)); set(efCmd,'String','');
    end

    function logMsg(txt)
        if ~isvalid(fig), return; end
        old=get(taLog,'String'); if ~iscell(old), old={old}; end
        if numel(old)>100, old=old(end-99:end); end
        new=[old;{sprintf('[%s] %s',datestr(now,'HH:MM:SS'),txt)}];
        set(taLog,'String',new,'Value',numel(new));
    end

    function onClose(~,~)
        running=false; doDisconnect();
        if ~isempty(refTableHandle)&&isvalid(refTableHandle), delete(refTableHandle); end
        delete(fig);
    end
end
