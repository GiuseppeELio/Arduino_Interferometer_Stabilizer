%%
function analyzer_simple_PID (I1,I2,Q1,Q2,Delta, V)
indici_non_zeri = find(I1 ~= 0);
I1_NZ = I1(indici_non_zeri);
I2_NZ = I2(indici_non_zeri);
Q1_NZ = Q1(indici_non_zeri);
Q2_NZ = Q2(indici_non_zeri);
Delta_NZ = Delta(indici_non_zeri);
V_NZ = V(indici_non_zeri);
%%
figure()
plot(I1_NZ)
hold on
plot(I2_NZ)
plot(Q1_NZ)
plot(Q2_NZ)
legend('I1','I2','Q1','Q2')

%%
% ------------------------------------------------------------------
% 1. Calcolo delle Ampiezze (V_est)
% ------------------------------------------------------------------

% V1_est (Ampiezza del primo segnale)
V1_est = 2.0 * sqrt(I1_NZ.^2 + Q1_NZ.^2);

% V2_est (Ampiezza del secondo segnale)
V2_est = 2.0 * sqrt(I2_NZ.^2 + Q2_NZ.^2);


% ------------------------------------------------------------------
% 2. Calcolo delle Fasi (Phi_est)
% ------------------------------------------------------------------

% Fase del primo segnale (in radianti)
Phi1_rad = atan2(Q1_NZ, I1_NZ);

% Fase del secondo segnale (in radianti)
Phi2_rad = atan2(Q2_NZ, I2_NZ);

% Conversione opzionale in gradi
Phi1_deg = rad2deg(Phi1_rad);
Phi2_deg = rad2deg(Phi2_rad);
%%
% ------------------------------------------------------------------
% Risultati
% ------------------------------------------------------------------
figure()

% --- Primo Subplot (Ampiezze) ---
subplot(1, 2, 1) % Nota: ho corretto la sintassi a 1, 2, 1
plot(V1_est);
hold on
plot(V2_est);
% Usa la funzione 'legend'
legend('R1', 'R2');
title('Ampiezze Stimate'); % Aggiungi un titolo per chiarezza

% --- Secondo Subplot (Fasi) ---
subplot(1, 2, 2) % Nota: ho corretto la sintassi a 1, 2, 2
plot(Phi1_deg);
hold on
plot(Phi2_deg)
% Usa la funzione 'legend' e la sintassi TeX per i caratteri speciali
legend('\theta_1', '\theta_2');
title('Fasi Stimate (Gradi)'); % Aggiungi un titolo per chiarezza

%%
% plot(mod(Phi2_deg-Phi1_deg,360))
delta_0 = atan2(-(V1_est/180).*sign(I1_NZ),(V2_est/30).*sign(I2_NZ));
%delta_0 = atan2(-(I1_NZ/180)*avg_phi,(I2_NZ/30));
figure();
subplot(211)
plot(rad2deg(delta_0));
hold on
plot(rad2deg(Delta_NZ));
ylim([min(delta_0) max(delta_0)]*180/pi);
legend('\delta_0','\delta_{Ard.}');
xlabel('Collected samples');
ylabel('Angle (deg)');
subplot(212)
plot(V_NZ,'.-');
ylim([(min(V_NZ)-0.05) (max(V_NZ)+0.05)]);
legend('V_{off}');
xlabel('Collected samples');
ylabel('Voltage offset');

%%
figure();
len = round(length(Delta_NZ)/2);
tf_off = fft(Delta_NZ(1:len));
tf_on = fft(Delta_NZ((len+1):end));
freq_off = ([1:1:length(tf_off)])/(0.05*length(tf_off));
freq_on = ([1:1:length(tf_on)])/(0.05*length(tf_on));
plot(freq_off,abs(tf_off))
hold on
plot(freq_on,abs(tf_on))
xlabel('Frequencies (Hz)')
title('FFT');
legend('PID OFF','PID ON');
end