function arduino_pid_GUI_off_on
% ARDUINO_PID_FULL_GUI
% GUI completa per controllare Arduino (PID, SET, PGA) e fare acquisizione.
% - Richiede MATLAB R2019b+ (uso serialport).
%
% REVISIONE LAYOUT:
% - Aumentata la larghezza della figura principale a 1200 per accogliere i grafici a destra.
% - Il pannello di acquisizione (acqPanel) è stato ridotto in larghezza per lasciare spazio.
% - Il pannello dei grafici (plotPanel) è stato reso visibile di default e posizionato a destra.
% - Le dimensioni e le posizioni degli assi dei grafici sono state adattate al nuovo plotPanel.

close all; clc;

% ---------- Finestra principale ----------
% Aumentata la larghezza a 1200 per ospitare i grafici a destra
fig = uifigure('Name','Arduino PID Control & PGA','Position',[100 100 1200 700]);

% ---------- Pannello Connessione ----------
% Posizione invariata, ma larghezza ridotta per adattarsi alla nuova larghezza della figura
connPanel = uipanel(fig,'Title','Connection','Position',[10 610 880 80]);
uilabel(connPanel,'Text','Porta:', 'Position',[10 30 40 22]);
portField = uieditfield(connPanel,'text','Value','COM10','Position',[55 30 100 22]);
uilabel(connPanel,'Text','Baud:', 'Position',[170 30 40 22]);
baudField = uieditfield(connPanel,'numeric','Value',115200,'Position',[215 30 90 22]);

connectBtn = uibutton(connPanel,'Text','Connect','Position',[330 28 100 26],...
    'ButtonPushedFcn',@connectFcn);
disconnectBtn = uibutton(connPanel,'Text','Disconnect','Position',[440 28 100 26],...
    'Enable','off','ButtonPushedFcn',@disconnectFcn);

statusLabel = uilabel(connPanel,'Text','Stato: Disconnesso','Position',[560 30 300 22],'FontColor','red');

% ---------- Pannello PID / SET ----------
pidPanel = uipanel(fig,'Title','PID & SET','Position',[10 360 440 240]);

% Sliders / edit fields per Kp Ki Kd
uilabel(pidPanel,'Text','Kp','Position',[10 190 30 20]);
kpField = uieditfield(pidPanel,'numeric','Value',0.05,'Position',[45 190 70 22]);
uilabel(pidPanel,'Text','Ki','Position',[130 190 30 20]);
kiField = uieditfield(pidPanel,'numeric','Value',0.5,'Position',[165 190 70 22]);
uilabel(pidPanel,'Text','Kd','Position',[250 190 30 20]);
kdField = uieditfield(pidPanel,'numeric','Value',0.0,'Position',[285 190 70 22]);

uilabel(pidPanel,'Text','Offset (O)','Position',[10 150 80 20]);
offField = uieditfield(pidPanel,'numeric','Value',0.5,'Position',[95 150 80 22]);
uilabel(pidPanel,'Text','Amplitude (A)','Position',[190 150 90 20]);
ampField = uieditfield(pidPanel,'numeric','Value',0.05,'Position',[285 150 80 22]);

uilabel(pidPanel,'Text','sample_interval ms (T)','Position',[10 110 140 20]);
tField = uieditfield(pidPanel,'numeric','Value',50,'Position',[155 110 80 22]);

% Pulsanti SET
setPBtn = uibutton(pidPanel,'Text','SET P','Position',[10 70 80 28],'ButtonPushedFcn',@(s,e) setParam('P'));
setIBtn = uibutton(pidPanel,'Text','SET I','Position',[100 70 80 28],'ButtonPushedFcn',@(s,e) setParam('I'));
setDBtn = uibutton(pidPanel,'Text','SET D','Position',[190 70 80 28],'ButtonPushedFcn',@(s,e) setParam('D'));
setOBtn = uibutton(pidPanel,'Text','SET O','Position',[280 70 80 28],'ButtonPushedFcn',@(s,e) setParam('O'));
setABtn = uibutton(pidPanel,'Text','SET A','Position',[370 70 80 28],'ButtonPushedFcn',@(s,e) setParam('A'));
setTBtn = uibutton(pidPanel,'Text','SET T','Position',[10 30 80 28],'ButtonPushedFcn',@(s,e) setParam('T'));

pidOnBtn  = uibutton(pidPanel,'Text','PID ON','Position',[100 30 80 28],'ButtonPushedFcn',@(s,e) simpleCmd('PID ON'));
pidOffBtn = uibutton(pidPanel,'Text','PID OFF','Position',[190 30 80 28],'ButtonPushedFcn',@(s,e) simpleCmd('PID OFF'));

% ---------- Pannello PGA (ABCDEF) ----------
pgaPanel = uipanel(fig,'Title','Programmable Gain Amplifier (PGA)','Position',[460 360 430 240]);

uilabel(pgaPanel,'Text','CS','Position',[10 180 30 20]);
csDrop = uidropdown(pgaPanel,'Items',{'1','2','3','4'},'Position',[45 180 60 22]);

uilabel(pgaPanel,'Text','Gain','Position',[120 180 50 20]);
presetDrop = uidropdown(pgaPanel,'Items',{'1','2','3','4','5','6','7','8','9'},'Position',[175 180 60 22]);

uilabel(pgaPanel,'Text','Trim','Position',[10 140 50 20]);
chanItems = compose('%02d',1:16);
chanDrop = uidropdown(pgaPanel,'Items',chanItems,'Position',[70 140 80 22]);

uilabel(pgaPanel,'Text','Polarity','Position',[160 140 50 20]);
polDrop = uidropdown(pgaPanel,'Items',{'P','N'},'Position',[215 140 60 22]);

uilabel(pgaPanel,'Text','Enable','Position',[300 140 50 20]);
enDrop = uidropdown(pgaPanel,'Items',{'0','1'},'Position',[350 140 50 22]);

sendPGABtn = uibutton(pgaPanel,'Text','Invia ABCDEF','Position',[150 90 120 30],'ButtonPushedFcn',@sendPGAcommand);

% Mostra la stringa generata
pgaStrLabel = uilabel(pgaPanel,'Text','Comando: -','Position',[20 40 380 30],'FontSize',13);

% ---------- Pannello Acquisizione ----------
% Larghezza ridotta a 880 -> 880 (per allineamento con i pannelli sopra)
acqPanel = uipanel(fig,'Title','Acquisizione','Position',[10 10 880 340]);

uilabel(acqPanel,'Text','Numero di cicli (Times)','Position',[10 290 180 20]);
timesField = uieditfield(acqPanel,'numeric','Value',6,'Position',[10 265 200 22]);

% uilabel(acqPanel,'Text','ivec (es: 0:0.1:1)','Position',[230 290 180 20]);
% ivecField = uieditfield(acqPanel,'text','Value','0:0.1:1','Position',[230 265 200 22]);

uilabel(acqPanel,'Text','NUM_SAMPLES (per ciclo)','Position',[230 290 180 20]);
samplesField = uieditfield(acqPanel,'numeric','Value',200,'Position',[230 265 200 22]);

uilabel(acqPanel,'Text','Filename','Position',[460 290 100 20]);
fileField = uieditfield(acqPanel,'text','Value','Test_PID_ON_OFF','Position',[560 290 200 22]);

saveCheck = uicheckbox(acqPanel,'Text','Salva dati','Position',[460 260 100 22],'Value',true);
analyzeCheck = uicheckbox(acqPanel,'Text','Analizza dopo acquisizione','Position',[560 260 200 22],'Value',false);

startAcqBtn = uibutton(acqPanel,'Text','Start Acquisizione PID ON/OFF','Position',[10 225 200 32],'ButtonPushedFcn',@startAcquisition);
stopAcqBtn = uibutton(acqPanel,'Text','Stop','Position',[220 225 100 32],'Enable','off','ButtonPushedFcn',@stopAcquisition);
calibrateBtn = uibutton(acqPanel,'Text','CALIBRATE','Position',[340 225 120 32],'ButtonPushedFcn',@(s,e) simpleCmd('CALIBRATE'));
printBtn = uibutton(acqPanel,'Text','PRINT (one block)','Position',[480 225 120 32],'ButtonPushedFcn',@printOnce);
resetBtn = uibutton(acqPanel,'Text','RESET','Position',[620 225 120 32],'ButtonPushedFcn',@(s,e) simpleCmd('RESET'));
analyzeBtn = uibutton(acqPanel,'Text','Analizza ora','Position',[760 225 120 32],'Enable','off','ButtonPushedFcn',@runAnalysisNow);

% Area log testo
% Posizione invariata, ma l'altezza è stata aumentata per sfruttare lo spazio
logArea = uitextarea(acqPanel,'Position',[10 10 860 200],'Editable','off');


% --- Pannello laterale dei grafici (modificato) ---
% Posizionato a destra, visibile di default.
plotPanel = uipanel(fig, ...
    'Title','Visualizzazione dati (PRINT)', ...
    'FontSize',12, ...
    'BackgroundColor',[0.95 0.95 0.95], ...
    'Position',[900 10 290 680], ... % Posizione: x=900, y=10, Larghezza=290, Altezza=680
    'Visible','on');  % <-- reso visibile

% Assi 1: I1, I2, Q1, Q2
% Adattati al nuovo plotPanel (290x680)
ax_IQ = uiaxes(plotPanel,'Position',[20 460 250 200]);
ax_IQ.Title.String = 'I1, I2, Q1, Q2';
ax_IQ.XLabel.String = 'Sample';
ax_IQ.YLabel.String = 'Amplitude';

% Assi 2: Delta
ax_Delta = uiaxes(plotPanel,'Position',[20 240 250 200]);
ax_Delta.Title.String ='Delta';
ax_Delta.XLabel.String = 'Sample';
ax_Delta.YLabel.String = 'Delta';

% Assi 3: V
ax_V = uiaxes(plotPanel,'Position',[20 20 250 200]);
ax_V.Title.String='V offset';
ax_V.XLabel.String = 'Sample';
ax_V.YLabel.String = 'V';


% ---------- Variabili condivise ----------
app = struct();
app.ardu = [];
app.stopFlag = false;
app.latestData = [];

% Metti app in UserData per accesso da callback esterne
fig.UserData = app;

% -------------------- CALLBACKS --------------------

    function connectFcn(~,~)
        PORT = portField.Value;
        BAUD = baudField.Value;
        try
            app = fig.UserData;
            if ~isempty(app.ardu)
                appendLog("Già connesso.");
                return;
            end
            a = serialport(PORT,BAUD,Timeout=1);
            configureTerminator(a,"LF");
            flush(a);
            app.ardu = a;
            fig.UserData = app;
            statusLabel.Text = sprintf('Stato: Connesso (%s @ %d)',PORT,BAUD);
            statusLabel.FontColor = 'green';
            connectBtn.Enable = 'off';
            disconnectBtn.Enable = 'on';
            appendLog("Connessione stabilita.");
        catch ME
            appendLog(sprintf("Errore connessione: %s", ME.message));
            statusLabel.Text = 'Stato: Errore connessione';
            statusLabel.FontColor = 'red';
        end
    end

    function disconnectFcn(~,~)
        app = fig.UserData;
        try
            if isempty(app.ardu)
                appendLog("Nessuna connessione aperta.");
                return;
            end
            clear app.ardu;
            app.ardu = [];
            fig.UserData = app;
            connectBtn.Enable = 'on';
            disconnectBtn.Enable = 'off';
            statusLabel.Text = 'Stato: Disconnesso';
            statusLabel.FontColor = 'red';
            appendLog("Disconnesso.");
        catch ME
            appendLog("Errore durante disconnect.");
            appendLog(ME.message);
        end
    end

    function appendLog(txt)
        t = datestr(now,'HH:MM:SS');
        s = sprintf("[%s] %s\n",t,char(txt));
        logArea.Value = [s; logArea.Value]; %#ok<AGROW>
        drawnow;
    end

% Invia comandi semplici come "PID ON", "PRINT", "CALIBRATE", "RESET"
    function simpleCmd(cmd)
        app = fig.UserData;
        if isempty(app.ardu)
            appendLog("Errore: non connesso.");
            return;
        end
        try
            writeline(app.ardu, cmd);
            pause(0.05);
            % leggi risposta (se presente) in modo non-bloccante
            if app.ardu.NumBytesAvailable>0
                rsp = readline(app.ardu);
                appendLog(sprintf("-> %s",rsp));
            else
                appendLog(sprintf("Comando inviato: %s", cmd));
            end
        catch ME
            appendLog(sprintf("Errore invio comando: %s", ME.message));
        end
    end

% Funzione che costruisce e invia ABCDEF per il PGA
    function sendPGAcommand(~,~)
        cs = csDrop.Value;
        preset = presetDrop.Value;
        chan = chanDrop.Value;
        pol  = polDrop.Value;
        en   = enDrop.Value;

        % chan è stringa '01'...'16'
        cmd = sprintf('%s%s%s%s%s', cs, preset, chan, pol, en);
        pgaStrLabel.Text = ['Comando: ', cmd];
        appendLog(['Invio PGA: ', cmd]);

        app = fig.UserData;
        if isempty(app.ardu)
            appendLog("Errore: non connesso.");
            return;
        end
        try
            writeline(app.ardu, cmd);
            pause(0.05);
            if app.ardu.NumBytesAvailable>0
                r = readline(app.ardu);
                appendLog(['-> ', r]);
            end
        catch ME
            appendLog(['Errore invio PGA: ', ME.message]);
        end
    end

% Funzione per impostare i parametri PID/SET
    function setParam(param)
        app = fig.UserData;
        if isempty(app.ardu)
            appendLog("Errore: non connesso.");
            return;
        end

        switch param
            case 'P'
                val = kpField.Value;
            case 'I'
                val = kiField.Value;
            case 'D'
                val = kdField.Value;
            case 'O'
                val = offField.Value;
            case 'A'
                val = ampField.Value;
            case 'T'
                val = tField.Value;
            otherwise
                appendLog(['Parametro sconosciuto: ', param]);
                return;
        end

        cmd = sprintf('SET %s %.4f', param, val);
        try
            writeline(app.ardu, cmd);
            pause(0.05);
            if app.ardu.NumBytesAvailable>0
                r = readline(app.ardu);
                appendLog(['-> ', r]);
            else
                appendLog(['Comando inviato: ', cmd]);
            end
        catch ME
            appendLog(['Errore invio SET: ', ME.message]);
        end
    end

% Funzione per avviare l'acquisizione
    function startAcquisition(~,~)
        app = fig.UserData;
        if isempty(app.ardu)
            appendLog("Errore: non connesso.");
            return;
        end

        % Recupera i nuovi parametri
        times = timesField.Value;
        numSamples = samplesField.Value;
        filename = fileField.Value;
        doSave = saveCheck.Value;
        doAnalyze = analyzeCheck.Value;
        
        % Inizializzazione
        app.stopFlag = false;
        fig.UserData = app;
        startAcqBtn.Enable = 'off';
        stopAcqBtn.Enable = 'on';
        analyzeBtn.Enable = 'off';
        
        appendLog("Acquisizione PID ON/OFF avviata.");
        
        % Pre-allocazione dei Vettori
        total_samples = numSamples * times;
        I1 = zeros(total_samples, 1);
        I2 = zeros(total_samples, 1);
        Q1 = zeros(total_samples, 1);
        Q2 = zeros(total_samples, 1);
        Delta = zeros(total_samples, 1);
        V = zeros(total_samples, 1);
        
        % Invia i comandi di setup all'Arduino
        appendLog("Invio parametri di setup...");
        simpleCmd(sprintf('SET SAMPLES %d', numSamples)); % Imposta il numero di campioni per blocco
        simpleCmd('PID OFF'); % Inizia sempre con PID OFF come nel codice originale
        simpleCmd('SET O 0.5'); % Imposta l'Offset (come nel codice fornito)
        
        % Calcola il punto di switch (metà dei cicli)
        switch_point = round(times / 2);
        
        % Ciclo di Acquisizione
        for j = 1:times
            if app.stopFlag
                appendLog("Acquisizione interrotta dall'utente.");
                break;
            end
            
            % Calcola l'indice di partenza e fine per il blocco corrente
            start_index = (j - 1) * numSamples + 1;
            end_index = j * numSamples;
            
            % Logica PID ON/OFF
            if j > switch_point
                 simpleCmd('PID ON'); 
                 appendLog(sprintf('Ciclo %d/%d: PID ON', j, times));
            else
                 simpleCmd('PID OFF'); % Assicura che sia OFF per la prima metà
                 appendLog(sprintf('Ciclo %d/%d: PID OFF', j, times));
            end
            
            % Pausa per stabilizzazione (11 secondi come nel codice fornito)
            appendLog("Pausa di stabilizzazione (11 secondi)...");
            pause(11); 
            
            % Richiesta e lettura del blocco di dati
            appendLog(sprintf('Lettura blocco %d/%d (campioni da %d a %d)', j, times, start_index, end_index));
            
            % Invia il comando PRINT per chiedere i dati
            writeline(app.ardu, 'PRINT');
            pause(0.1); % Breve pausa dopo l'invio del comando
            
            % Legge i dati
            for i = 1:numSamples
                if app.stopFlag
                    break;
                end
                
                current_index = start_index + i - 1;
                
                try
                    % Legge una riga di dati da Arduino
                    line = readline(app.ardu); 
                    % Assumiamo che Arduino invii 6 valori: I1, Q1, I2, Q2, Delta, V
                    M = sscanf(line, '%f %f %f %f %f %f'); 
                    
                    if numel(M) == 6
                        I1(current_index) = M(1);
                        Q1(current_index) = M(2);
                        I2(current_index) = M(3);
                        Q2(current_index) = M(4);
                        Delta(current_index) = M(5);
                        V(current_index) = M(6);
                    else
                        % Gestione della riga non valida: la salta e non incrementa l'indice
                        warning('Riga non valida ricevuta da Arduino: "%s"', line);
                        % Decrementa l'indice per riprovare a scrivere nello stesso slot
                        % current_index = current_index - 1; % Rimosso per evitare loop infiniti o scrittura fuori range 
                    end
                catch ME
                    appendLog(sprintf('ERRORE: Durante la lettura dalla porta seriale: %s', ME.message));
                    app.stopFlag = true; % Interrompe l'acquisizione in caso di errore grave
                    break; 
                end
            end
            
            appendLog(sprintf('Blocco %d completato.', j));
        end
        
        % Fine Acquisizione
        simpleCmd('PID OFF'); % Spegne il PID alla fine
        
        if ~app.stopFlag
            appendLog("Acquisizione completata con successo.");
            
            % Prepara la struttura dati per il salvataggio e l'analisi
            D = struct('I1',I1, 'I2',I2, 'Q1',Q1, 'Q2',Q2, 'Delta',Delta, 'V',V, ...
                       'Times', times, 'NumSamples', numSamples);
            app.latestData = D;
            fig.UserData = app;
            
            if doSave
                saveData(D, filename);
            end
            if doAnalyze
                runAnalysisOn(D);
            end
        end

        startAcqBtn.Enable = 'on';
        stopAcqBtn.Enable = 'off';
        analyzeBtn.Enable = 'on';
    end

% Funzione per interrompere l'acquisizione
    function stopAcquisition(~,~)
        app = fig.UserData;
        app.stopFlag = true;
        fig.UserData = app;
        simpleCmd('STOP'); % Invia comando STOP ad Arduino
        appendLog("Richiesta di interruzione inviata.");
    end

% Funzione per salvare i dati
    function saveData(D, filename)
        try
            save(filename, '-struct', 'D');
            appendLog(['Dati salvati in: ', filename, '.mat']);
        catch ME
            appendLog(['Errore nel salvataggio: ', ME.message]);
        end
    end

% Funzione per il comando PRINT (visualizzazione di un blocco di dati)
    function printOnce(~,~)
        app = fig.UserData;
        if isempty(app.ardu)
            appendLog("Errore: non connesso.");
            return;
        end

        try
            % 1. Invia il comando PRINT
            writeline(app.ardu, 'PRINT');
            appendLog("Comando PRINT inviato. In attesa di dati...");
            
            num_points = 200;
            start_index = 1;
            for i = 1:num_points
                if app.stopFlag
                    break;
                end
                
                current_index = start_index + i - 1;
                
                try
                    % Legge una riga di dati da Arduino
                    line = readline(app.ardu); 
                    % Assumiamo che Arduino invii 6 valori: I1, Q1, I2, Q2, Delta, V
                    M = sscanf(line, '%f %f %f %f %f %f'); 
                    
                    if numel(M) == 6
                        I1(current_index) = M(1);
                        Q1(current_index) = M(2);
                        I2(current_index) = M(3);
                        Q2(current_index) = M(4);
                        Delta(current_index) = M(5);
                        V(current_index) = M(6);
                    else
                        % Gestione della riga non valida: la salta e non incrementa l'indice
                        warning('Riga non valida ricevuta da Arduino: "%s"', line);
                        % Decrementa l'indice per riprovare a scrivere nello stesso slot
                        % current_index = current_index - 1; % Rimosso per evitare loop infiniti o scrittura fuori range 
                    end
                catch ME
                    appendLog(sprintf('ERRORE: Durante la lettura dalla porta seriale: %s', ME.message));
                    app.stopFlag = true; % Interrompe l'acquisizione in caso di errore grave
                    break; 
                end
            end
            I1 = I1;
            I2 = I2;
            Q1 = Q1;
            Q2 = Q2;
            Delta = Delta;
            Vt =V;

            % 3. Aggiorna i grafici
            
            % Grafico 1: I1, I2, Q1, Q2
            cla(ax_IQ);
            hold(ax_IQ, 'on');
            plot(ax_IQ, I1, 'DisplayName', 'I1');
            plot(ax_IQ, I2, 'DisplayName', 'I2');
            plot(ax_IQ, Q1, 'DisplayName', 'Q1');
            plot(ax_IQ, Q2, 'DisplayName', 'Q2');
            hold(ax_IQ, 'off');
            legend(ax_IQ, 'show', 'Location', 'best');
            ax_IQ.XLabel.String='samples';
            ax_IQ.YLabel.String='Amplitude';

            % Grafico 2: Delta
            cla(ax_Delta);
            plot(ax_Delta, Delta);
            ax_Delta.XLabel.String='samples';
            ax_Delta.YLabel.String='Delta';

            % Grafico 3: V
            cla(ax_V);
            plot(ax_V, Vt);
            ax_V.XLabel.String='samples';
            ax_V.YLabel.String='V';
            
            appendLog("Dati PRINT ricevuti e grafici aggiornati.");

        catch ME
            appendLog(['Errore PRINT: ', ME.message]);
        end
    end


% Analizza ora i dati già acquisiti
    function runAnalysisNow(~,~)
        app = fig.UserData;
        if isempty(app.latestData)
            appendLog("Nessun dato disponibile per analisi.");
            return;
        end
        runAnalysisOn(app.latestData);
    end

    function runAnalysisOn(D)
        try
            appendLog("Avvio analisi con analyzer_simple_PID...");
            % chiamata alla tua funzione analyzer_simple_PID
            % NOTA: La funzione analyzer_simple_PID non è inclusa, ma la chiamata è mantenuta.
            analyzer_simple_PID(D.I1, D.I2, D.Q1, D.Q2, D.Delta, D.V);
            appendLog("Analisi completata (analyzer_simple_PID non eseguita, solo simulata).");
        catch ME
           % appeappendLog(['Errore in analyzer_simple_PID: ', ME.message]);]);
        end
    end

% ---------- Utility: RESET MCU (chiama sempliceCmd con RESET) ----------
% Nota: in Arduino usi NVIC_SystemReset(), qui inviamo il comando RESET.
% Il microcontroller eseguirà il reset solo se gestito lato MCU.

end
