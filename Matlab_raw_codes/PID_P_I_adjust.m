close all, clearvars, clc;
%-----------
%Do you whant to save the values at the end of the code execution?
save = 1;% if not 0
filename = "Test_1_PID_p_vs_i";
% If you would like to analyze the data at the end of the acquisition move
% it to 1;
analyze =0;
%-----------
pause(1);
NUM_SAMPLES = 200;% (SAMPLE_RATE_HZ / FREQUENCY_HZ); % In questo caso, 100 campioni per blocco

pvec = [0:0.01:0.1];
ivec = [0:0.1:1];

% Calcolo il numero totale di iterazioni (blocchi di dati)
total_blocks = length(pvec) * length(ivec);
times = total_blocks; % Il numero totale di blocchi è dato dal prodotto delle lunghezze dei vettori

% --- Pre-allocazione dei Vettori ---
% La dimensione totale è il numero di campioni per blocco moltiplicato per il numero totale di blocchi
total_samples = NUM_SAMPLES * total_blocks;
I1 = zeros(total_samples, 1);
Q1 = zeros(total_samples, 1);
I2 = zeros(total_samples, 1);
Q2 = zeros(total_samples, 1);
Delta = zeros(total_samples, 1);
V = zeros(total_samples, 1);

% NUOVA MODIFICA: Vettori per memorizzare i parametri P e I per ogni singolo campione
P_param = zeros(total_samples, 1);
I_param = zeros(total_samples, 1);

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

% Inizializzo un contatore globale per il blocco di dati
global_block_index = 0;

% --- Ciclo di Lettura Dati ---
for j = 1:length(pvec)
    for k = 1:length(ivec) 
        
        % Incremento il contatore globale del blocco
        global_block_index = global_block_index + 1;
        
        % Calcola l'indice di partenza per il blocco corrente nel vettore totale
        start_index = (global_block_index - 1) * NUM_SAMPLES + 1;
        
        % Calcola l'indice di fine per il blocco corrente
        end_index = global_block_index * NUM_SAMPLES;
        
        % Valori correnti di P e I
        current_p = pvec(j);
        current_i = ivec(k);
        
        % Stampa un messaggio più informativo
        fprintf('Lettura blocco %d/%d (campioni da %d a %d) - P=%.4f, I=%.4f\n', ...
            global_block_index, total_blocks, start_index, end_index, current_p, current_i);
        
        % Imposta i nuovi valori di p e i sulla scheda
        fprintf(ardu,'%s',['set p ',num2str(current_p),' \n']); 
        pause(0.1);
        waste = readline(ardu); % Legge la risposta di conferma
        pause (0.1);
        
        fprintf(ardu,'%s',['set i ',num2str(current_i),' \n']); 
        pause(0.1);
        waste = readline(ardu); % Legge la risposta di conferma
        pause(0.1);
        
        % NUOVA MODIFICA: Memorizza i parametri P e I per l'intero blocco
        P_param(start_index:end_index) = current_p;
        I_param(start_index:end_index) = current_i;

        fprintf(ardu,'%s',['PID OFF',' \n']);
        pause(0.1);
        waste = readline(ardu);
        pause(0.1);
        fprintf(ardu,'%s',['SET O 0.5',' \n']);
        pause(0.1);
        waste = readline(ardu);
        pause(1);
        fprintf(ardu,'%s',['PID ON',' \n']);
        pause(0.1);
        waste = readline(ardu);
        pause(0.1);
        %
        pause(11);
        % Chiede ad ardu di accumulare i dati
        fprintf(ardu,'%s',['PRINT ',' \n']);    
        pause(0.1);
        
        for i = 1:NUM_SAMPLES
            % Calcola l'indice corrente all'interno del vettore totale
            current_index = start_index + i - 1;
            
            % Legge una riga di dati da Arduino
            try
                line = readline(ardu); % Legge una riga intera, più robusto di fscanf
                M = sscanf(line, '%f %f %f %f %f %f'); % Converte la stringa in 6 numeri float
                
                % Controlla se sono stati letti esattamente 6 valori
                if numel(M) == 6
                    I1(current_index) = M(1);
                    Q1(current_index) = M(2);
                    I2(current_index) = M(3);
                    Q2(current_index) = M(4);
                    Delta(current_index) = M(5);
                    V(current_index) = M(6);
                    
                    % I parametri P_param e I_param sono già stati impostati per l'intero blocco
                    % Non è necessario fare nulla qui, ma la logica è corretta.
                else
                    warning('Riga non valida ricevuta da Arduino: "%s"', line);
                end
            catch ME
                warning('Errore durante la lettura dalla porta seriale: %s', ME.message);
                break; % Interrompe il ciclo interno in caso di errore
            end
        end
        
        fprintf('Blocco %d completato. Pausa di 11 secondi...\n', global_block_index);
        
    end % Fine ciclo k
end % Fine ciclo j

disp('Lettura dati completata.');

% --- Chiusura Comunicazione ---
clear ardu;

%%
if save == 1;
    save(filename,"I1","I2","Q1","Q2","Delta", "V","pvec","ivec","P_param", "I_param");
end 
%%
if analyze == 1;
    analyzer_PID (I1,I2,Q1,Q2,Delta, V,pvec,ivec,P_param, I_param);
end 