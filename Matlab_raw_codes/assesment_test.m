close all, clearvars, clc;
%-----------
%Do you whant to save the values at the end of the code execution?
saveflag = 0;% if not 0
filename = "Test_PID_OFF_ON_t5ms_tau12ms";
% If you would like to analyze the data at the end of the acquisition move
% it to 1;
analyze =1;
%-----------
pause(1);
NUM_SAMPLES = 200;% (SAMPLE_RATE_HZ / FREQUENCY_HZ); % In questo caso, 100 campioni per blocco
sample_time = 0.051;
pausetime = (NUM_SAMPLES*sample_time)+2;
times = 2; % Numero di volte che vuoi leggere un blocco di dati

% --- Pre-allocazione dei Vettori ---
% La dimensione totale è il numero di campioni per blocco moltiplicato per il numero di blocchi
total_samples = NUM_SAMPLES * times;
I1 = zeros(total_samples, 1);
Q1 = zeros(total_samples, 1);
I2 = zeros(total_samples, 1);
Q2 = zeros(total_samples, 1);
Delta = zeros(total_samples, 1);
V = zeros(total_samples, 1);

% --- Configurazione della Porta Seriale ---
PORT = 'COM10'; 
BAUD = 115200; 

% Si consiglia di usare serialport invece della funzione 'serial' (obsoleta)
% Se usi una versione di MATLAB R2019b o successiva:
ardu = serialport(PORT, BAUD); 
flush(ardu); % Pulisce il buffer di comunicazione all'inizio

% Se usi una versione precedente di MATLAB, il tuo codice originale va bene:
% ardu = serial(PORT,'BaudRate',BAUD);
% fopen(ardu);
% pause(0.1); % Attesa per stabilizzare la comunicazione

disp('Comunicazione seriale avviata. Inizio lettura dati...');

fprintf(ardu,'%s',['PID OFF',' \n']);
pause(0.1);
waste = readline(ardu);
pause(0.1);
fprintf(ardu,'%s',['SET O 0.5',' \n']);
pause(0.1);
waste = readline(ardu);
pause(0.1);
% --- Ciclo di Lettura Dati ---
for j = 1:times
    % Calcola l'indice di partenza per il blocco corrente
    start_index = (j - 1) * NUM_SAMPLES + 1;
    
    % Calcola l'indice di fine per il blocco corrente
    end_index = j * NUM_SAMPLES;
    
         
    if j > times/2
         fprintf(ardu,'%s',['PID ON',' \n']); 
         pause(0.5);
         waste = readline(ardu);
         pause(0.5);
         waste = readline(ardu);
         pause(0.5);
    end
    pause(pausetime);

    % Stampa i valori
    fprintf('Lettura blocco %d/%d (campioni da %d a %d)\n', j, times, start_index, end_index);
    fprintf(ardu,'%s',['PRINT ',' \n']);    % chiede ad ardu di accumulare i dati
    pause(0.1);
    for i = 1:NUM_SAMPLES
        % Calcola l'indice corrente all'interno del vettore totale
        current_index = start_index + i - 1;
        
        % Legge una riga di dati da Arduino
        % Assicurati che Arduino invii 4 valori separati da spazio e seguiti da newline
        % Esempio: "1.23 4.56 7.89 0.12\n"
        try
            line = readline(ardu); % Legge una riga intera, più robusto di fscanf
            M = sscanf(line, '%f %f %f %f %f %f'); % Converte la stringa in 4 numeri float
            
            % Controlla se sono stati letti esattamente 4 valori
            if numel(M) == 6
                I1(current_index) = M(1);
                Q1(current_index) = M(2);
                I2(current_index) = M(3);
                Q2(current_index) = M(4);
                Delta(current_index) = M(5);
                V(current_index) = M(6);
            else
                warning('Riga non valida ricevuta da Arduino: "%s"', line);
            end
        catch ME
            warning('Errore durante la lettura dalla porta seriale: %s', ME.message);
            % Puoi decidere se interrompere o continuare
            break; % Interrompe il ciclo interno in caso di errore
        end
    end
    
    % Se vuoi una pausa tra la lettura di un blocco e il successivo
    fprintf('Blocco %d completato. Pausa di %f secondi...\n', j,pausetime);
    %pause(11);
end

disp('Lettura dati completata.');

% --- Chiusura Comunicazione ---
% Se usi serialport (MATLAB moderno):
clear ardu; 

%%
if saveflag == 1;
    save(filename,"I1","I2","Q1","Q2","Delta", "V","sample_time");
end 
%%
if analyze == 1;
    analyzer_simple_PID(I1,I2,Q1,Q2,Delta, V,sample_time);
end 