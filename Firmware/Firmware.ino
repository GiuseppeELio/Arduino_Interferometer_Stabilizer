// ===DEBUG SETTING===
// Comment the definition to disable debug commands
//#define DEBUG_PID
//#define DEBUG_LOCK_IN
//#define _START_UP_MESSAGE_
// ====================================================

const float FREQUENCY_HZ = 1000.0;
const float SAMPLE_RATE_HZ = 100000.0;
const int NUM_SAMPLES = (int)(SAMPLE_RATE_HZ / FREQUENCY_HZ);
const int PERIOD = (int)(1000000 / FREQUENCY_HZ);
const int SAMPLE_INTERVAL_US = (int)(1000000.0 / SAMPLE_RATE_HZ);
const int DAC_RESOLUTION = 4096;
const int DAC_MAX_VALUE = DAC_RESOLUTION - 1;

float sine_wave_table[NUM_SAMPLES];
float sine_wave_table_2[NUM_SAMPLES];

float vin_store[NUM_SAMPLES];
float sin_store[NUM_SAMPLES];
float cos_store[NUM_SAMPLES];

int current_sample_index = 0;
const int NUM_COLL = 200;
unsigned long previous_time_ms = 0;
volatile long sample_interval_ms = 50; // 50ms per campionare 100 volte in 5s

float I1_store[NUM_COLL];
float Q1_store[NUM_COLL];
float I2_store[NUM_COLL];
float Q2_store[NUM_COLL];
float VO_store[NUM_COLL];
float DELTA_store[NUM_COLL];

int write_value_DAC0;
int write_value_DAC1;

int sample_index = 0;
static unsigned long next_sample_time = 0;

int pinsdi = 35;  // SDI - shared]
int pinclk = 34;  // CLK - shared

// Chip select pins for the 4 PICs
int pincs[4] = { 30, 32, 33, 31 };  // CS0 to CS3

// Voffset controllato (inizializzato al valore di prima)
float voltage_offset = 0.5;

float amplitude_scale =  0.05; // non più di 0.5
// Coefficients for fundamental (a1) and second harmonic (a2)
float a1_coeff = 0.0;
float a2_coeff = 0.0;

// === Lock-in Detection ===
const int analogInput = A1;  // Ingresso da analizzare
float alpha = 6E-4;    // coefficiente filtro IIR
float beta = 6E-4;
float av_vin = 0, vin = 0;
// Variabili filtro doppio per fondamentale
volatile float I1_f1 = 0, I1_f2 = 0;
volatile float Q1_f1 = 0, Q1_f2 = 0;

// Variabili filtro doppio per seconda armonica
volatile float I2_f1 = 0, I2_f2 = 0;
volatile float Q2_f1 = 0, Q2_f2 = 0;

// Risultati lock-in
volatile float V1_est = 0.0;  // ampiezza stimata fondamentale
volatile float V2_est = 0.0;  // ampiezza stimata seconda armonica
volatile float Phi1_rad, Phi2_rad = 0;
volatile float delta_0 = 0;
const float pi = 3.14159;
const float delta_target = pi / 2;

//--------------------Variabili per calibrazione
// Variabili di calibrazione (sostituiscono 180 e 30)
volatile float V1_max_cal = 180.0; // Valore iniziale di default valgono per amplitude di 0.05
volatile float V2_max_cal = 30.0;  // Valore iniziale di default
// Variabili di Stato e Configurazione per la Calibrazione
const long CALIBRATION_INTERVAL_MS = sample_interval_ms; 
unsigned long calibration_previous_time_ms = 0; // Nuovo timer per la calibrazione
int calibration_state = 0; // 0: Non iniziata, 1: In corso, 2: Completata
int calibration_sweep_index = 0; 
// Definisce su quante delle ultime calibrazioni fare la media.
// Puoi cambiare questo valore per rendere la media più stabile (valore più alto)
// o più reattiva (valore più basso).
const int CALIBRATION_HISTORY_SIZE = 2;
// Array per memorizzare i risultati delle ultime N calibrazioni
float v1_history[CALIBRATION_HISTORY_SIZE] = {0.0};
float v2_history[CALIBRATION_HISTORY_SIZE] = {0.0};
// Contatore per tenere traccia di quante calibrazioni sono state eseguite in totale.
// Ci aiuta a gestire correttamente la media quando abbiamo meno di N risultati.
int calibration_count = 0;

/*------------- PID ------------------*/
// Variabile globale per abilitare/disabilitare il controllo PID
bool pid_enabled = true; // Inizialmente abilitato (o come preferisci)
volatile float pidError = 0.0, pidIntegral = 0.0, pidDerivative = 0.0;
volatile float prevError = 0.0;

// Guadagni PID (da tarare)
volatile float Kp = 0.07;
volatile float Ki = 0.5;
volatile float Kd = 0.0;

const float DT = sample_interval_ms / 1000.0;            // Tempo di campionamento in secondi

// Voff_pid --> voltage offset controllato dal PID
float Voff_pid = 0.0; // deve tornarci un numero tra 0 e 1 oppure tra -1 e 1?

// --- LED Control Definitions ---
const int LED_A1_RED_PIN = 23; // D22
const int LED_A1_GREEN_PIN = 22; // D23

const int LED_SCALE_ON_PIN = 28; // D28 (amplitude_scale != 0)
const int LED_SCALE_OFF_PIN = 29; // D29 (amplitude_scale == 0)

// --- Setup ---
void setup() {
  // Inizializza la comunicazione seriale per il debug (opzionale)
  Serial.begin(115200);
  
  pinMode(pinsdi, OUTPUT);  // SDI
  pinMode(pinclk, OUTPUT);  // CLK
  for (int i = 0; i < 4; i++) {
    pinMode(pincs[i], OUTPUT);
    digitalWrite(pincs[i], HIGH);  // CS idle
  }


  // 1. Pre-calcola la lookup table dell'onda sinusoidale
  for (int i = 0; i < NUM_SAMPLES; i++) {
    sine_wave_table[i] = sin(2.0 * PI * i / NUM_SAMPLES);
    sine_wave_table_2[i] = sin(4.0 * PI * i / NUM_SAMPLES);
  }

  #ifdef _START_UP_MESSAGE_
  while (!Serial); // Attende la connessione seriale (solo per schede native USB)
  Serial.print("Stabilizer interferometer V2.0");
  Serial.print('\t');
  Serial.println("Authors: Giuseppe E. Lio and Simone Zanotto");
  Serial.println("General Information and start up parameters");
  Serial.println(" ");
  Serial.println("Generatore di Onda Sinusoidale 1 kHz su DAC0 (A12)");
  Serial.print("Frequenza: ");
  Serial.println(FREQUENCY_HZ);

  Serial.print("Amplitude: "); Serial.println(amplitude_scale,4);
  Serial.print("Voltage Offset: "); Serial.println(voltage_offset,4);
  Serial.print("Kp: "); Serial.println(Kp,4);
  Serial.print("Ki: "); Serial.println(Ki,4);
  #endif
  
  initDigitalPots();
  analogWriteResolution(12);
  analogReadResolution(12);
  pid_enabled = false; 
  init_leds(); // Inizializza i pin dei LED

  calibration_state = 1; // Inizia la calibrazione non appena il setup è completo
  Serial.println("Starting calibration mode - PID deactivated");
}


// Funzione di aggiornamento del DAC (da chiamare nell'ISR del Timer)
void update_dac(int sample_index) {
  write_value_DAC0 = (int)((sine_wave_table[sample_index] * amplitude_scale + voltage_offset) * DAC_MAX_VALUE);
  // Scrive il valore corrente del campione sul DAC0 (pin A12)
  write_value_DAC1 = (int)(((a1_coeff * sine_wave_table[sample_index] + a2_coeff * sine_wave_table_2[sample_index]) + 0.5) * DAC_MAX_VALUE);
  analogWrite(A12, write_value_DAC0);
  analogWrite(A13, write_value_DAC1);

  float rawInput = analogRead(analogInput);
  av_vin += beta * (rawInput - av_vin); //(rawInput - 2048) / 2048.0;
  vin = rawInput - av_vin;
  //vin_store[sample_index] = vin;

  float ref1_cos = (float)(sine_wave_table[(sample_index + NUM_SAMPLES / 4) % NUM_SAMPLES]);
  float ref1_sin = (float)(sine_wave_table[sample_index]);
  float ref2_cos = (float)(sine_wave_table_2[(sample_index + NUM_SAMPLES / 8) % NUM_SAMPLES]);
  float ref2_sin = (float)(sine_wave_table_2[sample_index]);

  // Prodotti segnale × riferimenti (Miscelazione)
  float xi1 = vin * ref1_cos;
  //cos_store[sample_index] = xi1;
  float xq1 = vin * ref1_sin;
  //sin_store[sample_index] = xq1;
  float xi2 = vin * ref2_cos;
  float xq2 = vin * ref2_sin;

  // Filtro esponenziale doppio (Passa-Basso)
  // I filtri devono essere aggiornati ad ogni campione
  I1_f1 += alpha * (xi1 - I1_f1);
  I1_f2 += alpha * (I1_f1 - I1_f2);
  Q1_f1 += alpha * (xq1 - Q1_f1);
  Q1_f2 += alpha * (Q1_f1 - Q1_f2);

  I2_f1 += alpha * (xi2 - I2_f1);
  I2_f2 += alpha * (I2_f1 - I2_f2);
  Q2_f1 += alpha * (xq2 - Q2_f1);
  Q2_f2 += alpha * (Q2_f1 - Q2_f2);

}

void lockin() {
  // Ampiezze stimate
  // Il fattore 2.0 è corretto per compensare la miscelazione con un riferimento di ampiezza 1

  V1_est = 2.0 * sqrt(I1_f2 * I1_f2 + Q1_f2 * Q1_f2);
  V2_est = 2.0 * sqrt(I2_f2 * I2_f2 + Q2_f2 * Q2_f2);
  //Phi1_rad = atan2(Q1_f2, I1_f2); //Fase del primo segnale (in radianti)
  //Phi2_rad = atan2(Q2_f2, I2_f2); //Fase del secondo segnale (in radianti)
  delta_0 = atan2(-(V1_est / V1_max_cal)*sign(I1_f2), (V2_est / V2_max_cal)*sign(I2_f2));

  I1_store[current_sample_index] = I1_f2;
  Q1_store[current_sample_index] = Q1_f2;
  I2_store[current_sample_index] = I2_f2;
  Q2_store[current_sample_index] = Q2_f2;
  VO_store[current_sample_index] = voltage_offset;
  DELTA_store[current_sample_index] = delta_0;

  current_sample_index++;
  if (current_sample_index >= NUM_COLL) {
    current_sample_index = 0;
  }
  #ifdef DEBUG_LOCK_IN
  Serial.print("V1_est=");
  Serial.print(V1_est, 4);
  Serial.print("\t");
  Serial.print("V2_est=");
  Serial.print("\t");
  Serial.print(V2_est, 4);
  Serial.print("delta_0: ");
  Serial.println(delta_0);
  #endif

}

float sign(float x){
  if (x > 0.0) return 1;
  if (x < 0.0) return -1;
  return 0;
  }

// --- Loop Principale ---
void loop() {
  handleSerialCommands();
  unsigned long current_time = micros();
  update_dac((micros() % PERIOD) / SAMPLE_INTERVAL_US); // Aggiorna il DAC
  /**/
  unsigned long current_time_ms = millis();

  if (current_time_ms - previous_time_ms >= sample_interval_ms) {
    previous_time_ms = current_time_ms;
    lockin(); // Chiama la funzione senza indice
    if (pid_enabled) {
    PID();
    }
  }
  /**/
  unsigned long current_time_ms_2 = millis();
  if (current_time_ms_2 % 1000 == 0) {
    update_leds(); // Aggiorna lo stato dei LED
    //print_lock_in_old();
  }

  // 3. MACCHINA A STATI PER LA CALIBRAZIONE NON BLOCCANTE (Timer di 50ms)
  if (calibration_state == 1) {
    unsigned long current_cal_time_ms = millis();

    // Controlla se è trascorso l'intervallo di 50ms
    if (current_cal_time_ms - calibration_previous_time_ms >= CALIBRATION_INTERVAL_MS) {
      calibration_previous_time_ms = current_cal_time_ms; // Salva il tempo corrente

      // 3a. Imposta il voltage_offset per il passo corrente
      voltage_offset = (float)calibration_sweep_index / NUM_COLL; 
      
      Serial.print("Calibrazione passo "); Serial.print(calibration_sweep_index + 1); 
      Serial.print("/"); Serial.print(NUM_COLL);
      Serial.print(" - Offset: "); Serial.println(voltage_offset, 4);

      // 3b. L'acquisizione del dato avviene nel prossimo ciclo di lockin()
      // Il dato verrà memorizzato in *_store[calibration_sweep_index]
      
      // 3c. Avanza all'indice successivo
      calibration_sweep_index++;

      // 3d. Controlla se lo sweep è finito
      if (calibration_sweep_index >= NUM_COLL) {
        run_calibration_calculation();
        // La funzione run_calibration_calculation() imposta calibration_state = 2 e pid_enabled = true
      }
    }
  }
  // Se calibration_state == 2, la logica di calibrazione viene ignorata.
}

void PID() {
  volatile float VOFF_MIN = amplitude_scale;
  volatile float VOFF_MAX = 1.0 - amplitude_scale;
  // 1. Definizione dell'Errore
  float error = delta_0 - delta_target;
  // 2. Termine Proporzionale
  float proportional = Kp * error;
  // 3. Termine Derivativo
  // usando l'errore attuale e l'errore precedente.
  float derivative = Kd * (error - prevError) / DT;
  // 4. Aggiornamento dell'Errore Precedente
  prevError = error;
  // L'output è il valore assoluto che l'integrale sta cercando di raggiungere.
  float pidOutput = proportional + (Ki * pidIntegral) + derivative;
  // 6. Calcolo del NUOVO voltage_offset (Output del PID)
  // L'output del PID è la correzione totale da applicare.
  voltage_offset = 0.5 + pidOutput;

  pidIntegral += error * DT;

  // 8. Clamping Finale e Aggiornamento del pidIntegral
  if (voltage_offset > VOFF_MAX) pidIntegral = 0.0;
  if (voltage_offset < VOFF_MIN) pidIntegral = 0.0;

  #ifdef DEBUG_PID
  Serial.print("Delta_0: ");
  Serial.print(delta_0, 4);
  Serial.print(" | Errore: ");
  Serial.print(error, 4);
  Serial.print(" | pidIntegral: ");
  Serial.print(pidIntegral, 4);
  Serial.print(" | Voff: ");
  Serial.println(voltage_offset, 4);
  #endif
}

// Funzione che esegue il calcolo della calibrazione sui dati raccolti
void run_calibration_calculation() {
    float max_V1 = 0.0;
    float max_V2 = 0.0;

    for (int i = 0; i < NUM_COLL; i++) {
      float temp_V1 = 2.0 * sqrt(I1_store[i] * I1_store[i] + Q1_store[i] * Q1_store[i]);
      float temp_V2 = 2.0 * sqrt(I2_store[i] * I2_store[i] + Q2_store[i] * Q2_store[i]);

      if (temp_V1 > max_V1) max_V1 = temp_V1;
      if (temp_V2 > max_V2) max_V2 = temp_V2;
    }

    // 2.1. Aggiungi i nuovi valori (max_V1, max_V2) alla cronologia.
    // Usiamo l'operatore modulo (%) per creare un "buffer circolare": una volta riempito
    // l'array, si ricomincia a scrivere dalla prima posizione.
    int current_index = calibration_count % CALIBRATION_HISTORY_SIZE;
    v1_history[current_index] = max_V1;
    v2_history[current_index] = max_V2;
    calibration_count++; // Incrementa il contatore totale delle calibrazioni

    // 2.2. Calcola la media semplice degli elementi nella cronologia.
    float v1_sum = 0.0;
    float v2_sum = 0.0;
    
    // Determina su quanti elementi fare la media. Se non abbiamo ancora fatto
    // N calibrazioni, mediamo solo su quelle disponibili.
    int num_items_to_average = (calibration_count > CALIBRATION_HISTORY_SIZE) ? CALIBRATION_HISTORY_SIZE : calibration_count;

    for (int i = 0; i < num_items_to_average; i++) {
        v1_sum += v1_history[i];
        v2_sum += v2_history[i];
    }
    // 2.3. Calcola il valore finale della media e aggiorna le variabili globali.
    V1_max_cal = v1_sum / num_items_to_average;
    V2_max_cal = v2_sum / num_items_to_average;

    Serial.print("Nuova Calibrazione V1 (media su "); Serial.print(num_items_to_average); Serial.print(" campioni): "); Serial.println(V1_max_cal, 4);
    Serial.print("Nuova Calibrazione V2 (media su "); Serial.print(num_items_to_average); Serial.print(" campioni): "); Serial.println(V2_max_cal, 4);
    // Imposta lo stato su Completata
    calibration_state = 2; 
    // Resetta l'offset a un valore di default per il funzionamento normale
    voltage_offset = 0.5; 
    // *** RIATTIVA IL PID ***
    pid_enabled = true; 
    Serial.println("Calibration completed. PID started.");
}

// 📌 Mappa dei preset B e canali C in 4 bit (bool[4])
const bool presetMap[9][4] = {
  { 0, 0, 0, 0 },  // 1
  { 1, 0, 0, 0 },  // 2
  { 0, 1, 0, 0 },  // 3
  { 1, 1, 0, 0 },  // 4
  { 0, 0, 1, 0 },  // 5
  { 1, 0, 1, 0 },  // 6
  { 0, 1, 1, 0 },  // 7
  { 1, 1, 1, 0 },  // 8
  { 0, 0, 0, 1 }   // 9
};

const bool channelMap[16][4] = {
  { 0, 0, 0, 0 },  // 01
  { 1, 0, 0, 0 },  // 02
  { 0, 1, 0, 0 },  // 03
  { 1, 1, 0, 0 },  // 04
  { 0, 0, 1, 0 },  // 05
  { 1, 0, 1, 0 },  // 06
  { 0, 1, 1, 0 },  // 07
  { 1, 1, 1, 0 },  // 08
  { 0, 0, 0, 1 },  // 09
  { 1, 0, 0, 1 },  // 10
  { 0, 1, 0, 1 },  // 11
  { 1, 1, 0, 1 },  // 12
  { 0, 0, 1, 1 },  // 13
  { 1, 0, 1, 1 },  // 14
  { 0, 1, 1, 1 },  // 15
  { 1, 1, 1, 1 }   // 16
};

void initDigitalPots() {
  for (int csIndex = 0; csIndex < 4; csIndex++) {
    int csPin = pincs[csIndex];

    // Inizializza GAIN con preset 1 e enableBit=0
    buildDigitalDataFromInput(csPin, 0, 0, 'P', false, true);

    // Inizializza TRIM con channel=1, polarity=P e enableBit=0
    buildDigitalDataFromInput(csPin, 0, 0, 'P', false, false);

    // 🔹 Stampa di log su seriale
    Serial.print("Init Pot ");
    Serial.print(csIndex + 1);
    Serial.println(" → Gain=1 Trim=1 P0");
  }
}

void handleSerialCommands() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    input.toUpperCase(); // NUOVA RIGA: converte tutto in maiuscolo
    //Serial.print("Received input: ");
    //Serial.println(input);

    // === Comando di Ricalibrazione ===
    if (input == "CALIBRATE") {
      Serial.println("Calibration command received");
      
      // 1. Disabilita il PID per evitare interferenze durante la calibrazione
      pid_enabled = false;
      Serial.println("PID temporary deactivated.");

      // 2. Resetta l'indice dello sweep
      calibration_sweep_index = 0;
      
      // 3. Imposta lo stato a "in corso" per far partire la macchina a stati nel loop()
      calibration_state = 1; 
      
      Serial.println("Starting new calibration");
      return; // Esci per non processare altri comandi
    }
    // === Enabling PID ===
    // === To Activate PID ===
    if (input == "PID ON") {
      if (!pid_enabled) { // Controlla se è già attivo per evitare messaggi ridondanti
        pid_enabled = true;
        pidIntegral = 0.0;
        voltage_offset = 0.5;
        Serial.println("PID activated.");
      } else {
        Serial.println("PID already activated.");
      }
      return; // Esci per non processare altri comandi
    }
    // === To Deactivate PID ===
    if (input == "PID OFF") {
      if (pid_enabled) { // Controlla se è già disattivo
        pid_enabled = false;
        pidIntegral = 0.0;
        voltage_offset = 0.5;
        Serial.println("PID deactivated.");
      } else {
        Serial.println("PID already deactivated.");
      }
      return; // Esci per non processare altri comandi
    }
    // === To print on the Serial I1,I2,Q1,Q2, Delta and V_offset ===
    if (input == "PRINT") {
      print_lock_in();
      return;
    }
     // === Parameters ===
    if (input.startsWith("SET ")) { // Deve esserci uno spazio dopo SET
      if (input.length() < 7) { // Controllo di lunghezza minima per evitare crash in caso di SET incompleto
        Serial.println("Comando SET incompleto. Formato: SET [A|S|O|1|2] [valore]");
        return;
      }
      char param = input.charAt(4);  // A, S, O, 1, 2
      String valueStr = input.substring(6);
      float val = valueStr.toFloat();

      switch (param) {

        case 'A':  // amplitude_scale
          if (val >= 0.0 && val <= 0.5) {
            amplitude_scale = val;
            Serial.print("Amplitude scale aggiornata: ");
            Serial.println(amplitude_scale, 3);
          } else {
            Serial.println("Amplitude_scale >=0  <=0.5");
          }
          break;

        case '1':  // a1_coeff (fondamentale)
          if (val >= 0.0 && val <= 1.0) {
            a1_coeff = val / 2;
            Serial.print("a1 coefficient (1KHz): ");
            Serial.println(a1_coeff, 2);
          } else {
            Serial.println("a1 >=0.0  <=1.0");
          }
          break;

        case '2':  // a2_coeff (seconda armonica)
          if (val >= 0.0 && val <= 1.0) {
            a2_coeff = val / 2;
            Serial.print("a2 coefficient (2KHz): ");
            Serial.println(a2_coeff, 2);
          } else {
            Serial.println("a2 >=0.0 <=1.0");
          }
          break;

        case 'O':  // voltage_offset
          if (val >= 0 && val <= 1) {
            voltage_offset = (float)val;
            Serial.print("Voltage offset: ");
            Serial.println(voltage_offset);
          } else {
            Serial.println("Voltage_offset >=0  <=1");
          }
          break;

        case 'I':  // voltage_offset
          if (val >= 0 && val <= 1) {
            Ki = (float)val;
            Serial.print("Ki: ");
            Serial.println(Ki);
          } else {
            Serial.println("Ki  >=0 and <=1");
          }
          break;

        case 'P':  // voltage_offset
          if (val >= 0 && val <= 1) {
            Kp = (float)val;
            Serial.print("Kp: ");
            Serial.println(Kp);
          } else {
            Serial.println("Kp >=0 <=1");
          }
          break;

        case 'D':  // voltage_offset
          if (val >= 0 && val <= 1) {
            Kd = (float)val;
            Serial.print("Kd: ");
            Serial.println(Kd);
          } else {
            Serial.println("Kd >=0 <=1");
          }
          break;

          
        case 'T':  // voltage_offset
          if (val >= 50 ) {
            sample_interval_ms = (int)val;
            Serial.print("sample_interval_ms: ");
            Serial.println(sample_interval_ms);
          } else {
            Serial.println("sample_interval_ms (int) >50");
          }
          break;
        default:
          Serial.println("Parametro sconosciuto. Usa S,O,I,P,D1,2,T.");
      }
      return;  // esci, non interpretare come comando pot
    }

    // === Comando RESET ===
    if (input.equalsIgnoreCase("RESET")) {
      Serial.println("System restart...");
      delay(10);
      NVIC_SystemReset();  // STM32/ARM reset
    }

    // === Se non è un comando SET, interpretalo come comando digitale ===
     if (input.length() != 6) {
       Serial.println("Invalid input. Format: ABCDEF (e.g., 1205P1 or SET A 0.05)");
       return;
      }

    // A: CS index (1–4)
    int csIndex = input.charAt(0) - '1';
    if (csIndex < 0 || csIndex > 3) {
      Serial.println("Invalid CS value. Use 1 to 4.");
      return;
    }

    // B: Preset (1–9)
    int presetIndex = input.charAt(1) - '1';
    if (presetIndex < 0 || presetIndex > 8) {
      Serial.println("Invalid preset. Use 1 to 9.");
      return;
    }

    // C + D: Channel (01–16)
    String channelStr = input.substring(2, 4);
    int channelIndex = channelStr.toInt() - 1;
    if (channelIndex < 0 || channelIndex > 15) {
      Serial.println("Invalid channel. Use 01 to 16.");
      return;
    }

    // E: Polarity
    char polarity = input.charAt(4);
    if (polarity != 'P' && polarity != 'N') {
      Serial.println("Invalid polarity. Use P or N.");
      return;
    }

    // F: Enable bit
    char valueChar = input.charAt(5);
    if (valueChar != '0' && valueChar != '1') {
      Serial.println("Invalid bit value. Use 0 or 1.");
      return;
    }
    bool enableBit = (valueChar == '1');

    int csPin = pincs[csIndex];

    // Prima: GAIN (preset + enableBit)
    buildDigitalDataFromInput(csPin, presetIndex, 0, 'P', enableBit, true);

    // Poi: TRIM (channel + polarity + enableBit)
    buildDigitalDataFromInput(csPin, 0, channelIndex, polarity, enableBit, false);
  }
}

void buildDigitalDataFromInput(
  int csPin,
  int presetIndex,
  int channelIndex,
  char polarity,
  bool enableBit,
  bool isGain) {
  bool digitaldata[8];

  // Bit 0: Mode
  digitaldata[0] = isGain ? 1 : 0;

  // Bit 1-4: dati da tabella
  for (int i = 0; i < 4; i++) {
    digitaldata[i + 1] = isGain
                         ? presetMap[presetIndex][i]
                         : channelMap[channelIndex][i];
  }

  // Bit 5
  digitaldata[5] = isGain ? 0 : (polarity == 'N' ? 1 : 0);

  // Bit 6
  digitaldata[6] = enableBit;

  // Bit 7
  digitaldata[7] = 0;

  Serial.print(isGain ? "GAIN  -> " : "TRIM  -> ");
  for (int i = 0; i < 8; i++) {
    Serial.print(digitaldata[i]);
  }
  Serial.println();

  digitalPotWrite(csPin, digitaldata);
}

// 📤 Shift 8 bits to the selected PIC
void digitalPotWrite(int pincs, bool digitaldata[8]) {
  digitalWrite(pincs, LOW);
  delayMicroseconds(50);

  for (int i = 0; i < 8; i++) {
    digitalWrite(pinsdi, digitaldata[i]);
    delayMicroseconds(1); // Un piccolo delay per stabilizzare il pin SDI
    digitalWrite(pinclk, LOW);
    delayMicroseconds(1); // Un piccolo delay per garantire la larghezza dell'impulso CLK
    digitalWrite(pinclk, HIGH);
    delayMicroseconds(1); // Un piccolo delay per garantire il tempo di mantenimento
  }

  delayMicroseconds(50);
  digitalWrite(pincs, HIGH);
  // delayMicroseconds(100); // Rimosso: 100us è un tempo enorme, non necessario per PIC
}

void print_vin () {
  for (int i = 0; i < NUM_SAMPLES; i++) {
    Serial.print(vin_store[i]);
    Serial.print('\t');
    Serial.print(sin_store[i]);
    Serial.print('\t');
    Serial.println(cos_store[i]);
  }
  Serial.println();
}

void print_lock_in () {
  // Stampa i dati a partire dal campione più vecchio (quello successivo all'ultimo scritto)
  // fino all'ultimo campione scritto.

  // Usiamo l'indice globale corretto
  int start_index = current_sample_index;

  for (int i = 0; i < NUM_COLL; i++) {
    // L'indice di stampa scorre da 0 a NUM_COLL-1
    // L'indice dell'array usa il modulo per iniziare dal campione più vecchio
    int print_index = (start_index + i) % NUM_COLL;

    Serial.print(I1_store[print_index]);
    Serial.print('\t');
    Serial.print(Q1_store[print_index]);
    Serial.print('\t');
    Serial.print(I2_store[print_index]);
    Serial.print('\t');
    Serial.print(Q2_store[print_index]);
    Serial.print('\t');
    Serial.print(DELTA_store[print_index], 4);
    Serial.print('\t');
    Serial.println(VO_store[print_index], 4);
  }
  Serial.println();
}

// --- LED Control Functions ---

// Function to initialize LED pins
void init_leds() {
  pinMode(LED_A1_RED_PIN, OUTPUT);
  pinMode(LED_A1_GREEN_PIN, OUTPUT);
  pinMode(LED_SCALE_ON_PIN, OUTPUT);
  pinMode(LED_SCALE_OFF_PIN, OUTPUT);

  // Initial state: turn off all LEDs
  digitalWrite(LED_A1_RED_PIN, LOW);
  digitalWrite(LED_A1_GREEN_PIN, LOW);
  digitalWrite(LED_SCALE_ON_PIN, LOW);
  digitalWrite(LED_SCALE_OFF_PIN, LOW);
}

// Function to control LED pair D22/D23 based on A1 reading
void control_a1_leds() {

  float rawInput = analogRead(analogInput); // analogInput is A1

  // Assuming 12-bit resolution, 512 is 1/8 of the full range (4096)
  if (rawInput > 512) {
    // Signal is high: Turn on Green (D23), Turn off Red (D22)
    digitalWrite(LED_A1_GREEN_PIN, HIGH);
    digitalWrite(LED_A1_RED_PIN, LOW);
  } else {
    // Signal is low: Turn on Red (D22), Turn off Green (D23)
    digitalWrite(LED_A1_GREEN_PIN, LOW);
    digitalWrite(LED_A1_RED_PIN, HIGH);
  }
}

// Function to control LED pair D28/D29 based on amplitude_scale
void control_scale_leds() {
  // amplitude_scale is a volatile float
  if (amplitude_scale <= 0.01 ) {
    // amplitude_scale is active: Turn on D28, Turn off D29
    digitalWrite(LED_SCALE_ON_PIN, LOW);
    digitalWrite(LED_SCALE_OFF_PIN, HIGH);
  } else {
    // amplitude_scale is 0: Turn on D29, Turn off D28
    digitalWrite(LED_SCALE_ON_PIN, HIGH);
    digitalWrite(LED_SCALE_OFF_PIN, LOW);
  }
}

// Function to be called in the main loop to update all LEDs
void update_leds() {
  control_a1_leds();
  control_scale_leds();
}
