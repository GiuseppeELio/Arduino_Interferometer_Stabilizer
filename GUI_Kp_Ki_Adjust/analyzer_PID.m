function analyzer_PID (I1,I2,Q1,Q2,Delta, V,pvec,ivec,P_param, I_param)
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
plot(radtodeg(delta_0));
hold on
plot(radtodeg(Delta_NZ));
ylim([min(delta_0) max(delta_0)]*180/pi);
legend('\delta_0','\delta_{Ard.}');
subplot(212)
plot(V_NZ,'.-');
ylim([min(V_NZ) max(V_NZ)]);
legend('V_{off}');

%%
% Esempio di come usare i dati:

% pvec = [0:0.01:0.1];
% ivec = [0:0.1:1];
for j = 1:length(pvec)
    for k = 1:length(ivec)
        target_p = pvec(j);
        target_i = ivec(k);
        indices = (P_param == target_p) & (I_param == target_i);
        Delta_filtered = Delta(indices);
        Delta_mean(j,k) = mean(Delta_filtered);
        Delta_std(j,k) = std(Delta_filtered);
        V_filtered = V(indices);
        V_mean(j,k) = mean(V_filtered);
        V_std(j,k) = std(V_filtered);

    end
end
Delta_mean_deg = rad2deg(Delta_mean);
Delta_std_deg = rad2deg(Delta_std);

%% ---- Parametri di soglia ----
Delta_target = 90;        % gradi
Delta_tol    = 0.05;      % ±5%

V_target = 0.5;
V_tol    = 0.05;          % ±10%

% Soglie Delta
Delta_min = Delta_target * (1 - Delta_tol);
Delta_max = Delta_target * (1 + Delta_tol);

% Soglie V
V_min = V_target * (1 - V_tol);
V_max = V_target * (1 + V_tol);

% ---- Crea griglie ----
[Pgrid, Igrid] = meshgrid(pvec, ivec);

% ---- Maschera per Delta ----
mask_Delta = (Delta_mean_deg >= Delta_min) & (Delta_mean_deg <= Delta_max);

P_hits_Delta = Pgrid(mask_Delta);
I_hits_Delta = Igrid(mask_Delta);

% ---- Maschera per V ----
mask_V = (V_mean >= V_min) & (V_mean <= V_max);

P_hits_V = Pgrid(mask_V);
I_hits_V = Igrid(mask_V);

%% ---- Plot figure ----
figure();
% --- 1) Delta MEAN Map ---
subplot(2,2,1);
imagesc(pvec, ivec, Delta_mean_deg);
set(gca,'YDir','normal');
colormap(gca, hsv);
caxis([min(Delta_mean_deg(:))-10 max(Delta_mean_deg(:))+10]);
colorbar('northoutside');

title('\Delta Mean Map');
xlabel('Kp parameter');
ylabel('Ki parameter');
set(gca, 'FontSize', 16);
hold on;

% Cerchi per la maschera Delta
plot(P_hits_Delta, I_hits_Delta, 'o', ...
    'MarkerSize', 10, 'LineWidth', 1.8, ...
    'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'w');

% --- 1) Delta STD Map ---
subplot(2,2,2);
imagesc(pvec, ivec, Delta_std_deg);
set(gca,'YDir','normal');
colormap(gca, "parula");
caxis([min(Delta_std_deg(:))-1 max(Delta_std_deg(:))+1]);
colorbar('northoutside');

title('\Delta Std Map');
xlabel('Kp parameter');
ylabel('Ki parameter');
set(gca, 'FontSize', 16);
hold on;

% --- 2) V Map ---
subplot(2,2,3);
imagesc(pvec, ivec, V_mean);
set(gca,'YDir','normal');
colormap(gca, jet);
caxis([min(V_mean(:)) max(V_mean(:))]);
colorbar('northoutside');

title('V Mean Map');
xlabel('Kp parameter');
ylabel('Ki parameter');
set(gca, 'FontSize', 16);
hold on;

% Cerchi per la maschera V
plot(P_hits_V, I_hits_V, 'o', ...
    'MarkerSize', 10, 'LineWidth', 1.8, ...
    'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'w');

% --- 4) V std Map ---
subplot(2,2,4);
imagesc(pvec, ivec, V_std);
set(gca,'YDir','normal');
colormap(gca, jet);
caxis([min(V_std(:)) max(V_std(:))]);
colorbar('northoutside');

title('V Std Map');
xlabel('Kp parameter');
ylabel('Ki parameter');
set(gca, 'FontSize', 16);
hold on;
end