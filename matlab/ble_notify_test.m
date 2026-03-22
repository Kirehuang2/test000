%% Minimal BLE notification test
% Tests whether MATLAB receives ANY notifications from DA14531 SPS
% Run this BEFORE ble_monitor to isolate the issue.
%
% IMPORTANT: Close all other BLE connections (Python, SmartBond, nRF Connect)

clear; clc;

DEVICE_NAME = "SPS_531";
DSPS_SERVICE_UUID = "0783B03E-8535-B5A0-7140-A304D2495CB7";
SERVER_TX_UUID    = "0783B03E-8535-B5A0-7140-A304D2495CB8";
SERVER_RX_UUID    = "0783B03E-8535-B5A0-7140-A304D2495CBA";
FLOW_CTRL_UUID    = "0783B03E-8535-B5A0-7140-A304D2495CB9";

%% Step 1: Connect
fprintf('=== BLE Notification Test ===\n');
fprintf('Scanning for %s...\n', DEVICE_NAME);
b = ble(DEVICE_NAME);
fprintf('Connected: %s (%s)\n', b.Name, b.Address);

%% Step 2: List services and characteristics
fprintf('\nCharacteristics table: %d rows\n', height(b.Characteristics));
disp(b.Characteristics);

%% Step 3: Get characteristics
charTx   = characteristic(b, DSPS_SERVICE_UUID, SERVER_TX_UUID);
charRx   = characteristic(b, DSPS_SERVICE_UUID, SERVER_RX_UUID);
charFlow = characteristic(b, DSPS_SERVICE_UUID, FLOW_CTRL_UUID);

fprintf('\ncharTx properties: %s\n', strjoin(charTx.Attributes, ', '));
fprintf('charRx properties: %s\n', strjoin(charRx.Attributes, ', '));
fprintf('charFlow properties: %s\n', strjoin(charFlow.Attributes, ', '));

%% Step 4: Subscribe to notifications
notifyCount = 0;

fprintf('\nSubscribing to charTx (Server TX)...\n');
subscribe(charTx);
charTx.DataAvailableFcn = @onTxNotify;
fprintf('charTx subscribed. DataAvailableFcn set.\n');

fprintf('Subscribing to charFlow...\n');
subscribe(charFlow);
charFlow.DataAvailableFcn = @onFlowNotify;
fprintf('charFlow subscribed.\n');

%% Step 5: Send XON
fprintf('\nSending XON to Flow Control...\n');
try
    write(charFlow, uint8(1), 'WithoutResponse');
    fprintf('XON sent (WithoutResponse).\n');
catch
    write(charFlow, uint8(1));
    fprintf('XON sent (with response).\n');
end

%% Step 6: Wait and watch for notifications
fprintf('\n--- Waiting 5s for spontaneous notifications ---\n');
pause(5);
fprintf('Notifications received so far: %d\n', notifyCount);

%% Step 7: Send LOG START and wait
fprintf('\n--- Sending LOG START 1, waiting 15s ---\n');
data = uint8(['LOG START 1' 13 10]);
fprintf('TX bytes (%d): [%s]\n', numel(data), char(data(1:end-2)));
try
    write(charRx, data, 'WithoutResponse');
    fprintf('Sent via WithoutResponse.\n');
catch e
    fprintf('WithoutResponse failed: %s\n', e.message);
    try
        write(charRx, data);
        fprintf('Sent with response.\n');
    catch e2
        fprintf('Both failed: %s\n', e2.message);
    end
end

for sec = 1:15
    pause(1);
    fprintf('  %2ds: notifyCount = %d\n', sec, notifyCount);
    if notifyCount > 0
        break;
    end
end

%% Step 8: Try short R command
if notifyCount == 0
    fprintf('\n--- Sending R (short restart), waiting 15s ---\n');
    write(charRx, uint8(['R' 13 10]), 'WithoutResponse');
    for sec = 1:15
        pause(1);
        fprintf('  %2ds: notifyCount = %d\n', sec, notifyCount);
        if notifyCount > 0
            break;
        end
    end
end

%% Step 9: Try manual read
fprintf('\n--- Manual read(charTx) ---\n');
try
    val = read(charTx);
    fprintf('read(charTx) returned %d bytes: [%s]\n', numel(val), char(val));
catch e
    fprintf('read(charTx) error: %s\n', e.message);
end

%% Step 10: Send STOP
fprintf('\n--- Sending STOP ---\n');
write(charRx, uint8(['S' 13 10]), 'WithoutResponse');

%% Cleanup
fprintf('\nUnsubscribing...\n');
unsubscribe(charTx);
unsubscribe(charFlow);
fprintf('Done. Total notifications: %d\n', notifyCount);

%% Callbacks
function onTxNotify(src, evt)
    assignin('base', 'notifyCount', evalin('base', 'notifyCount') + 1);
    try
        val = read(src);
        fprintf('[NOTIFY-TX] %d bytes: %s\n', numel(val), char(val));
    catch e
        fprintf('[NOTIFY-TX] callback fired, read error: %s\n', e.message);
    end
end

function onFlowNotify(src, evt)
    try
        val = read(src);
        if ~isempty(val) && val(1) == 1
            fprintf('[NOTIFY-FLOW] XON\n');
        elseif ~isempty(val) && val(1) == 0
            fprintf('[NOTIFY-FLOW] XOFF\n');
        else
            fprintf('[NOTIFY-FLOW] %d bytes: [%s]\n', numel(val), mat2str(val));
        end
    catch e
        fprintf('[NOTIFY-FLOW] error: %s\n', e.message);
    end
end
