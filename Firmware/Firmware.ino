// ===DEBUG SETTING===
// Comment the definition to disable debug commands
//#define DEBUG_PID
//#define DEBUG_LOCK_IN
//#define DEBUG_LOCK_IN_TAU // change the number of NUM_COLL and the sample_interval_ms = 5
//#define _START_UP_MESSAGE_
// ====================================================

const float FREQUENCY_HZ = 1000.0;  //Sinusoidal frequency
const float SAMPLE_RATE_HZ = 100000.0;
const int NUM_SAMPLES = (int)(SAMPLE_RATE_HZ / FREQUENCY_HZ);
const int PERIOD = (int)(1000000 / FREQUENCY_HZ);
const int SAMPLE_INTERVAL_US = (int)(1000000.0 / SAMPLE_RATE_HZ);
const int DAC_RESOLUTION = 4096;
const int DAC_MAX_VALUE = DAC_RESOLUTION - 1;

float sine_wave_table[NUM_SAMPLES];
float sine_wave_table_2[NUM_SAMPLES];
float ttl_table[NUM_SAMPLES];

float vin_store[NUM_SAMPLES];
float sin_store[NUM_SAMPLES];
float cos_store[NUM_SAMPLES];

int current_sample_index = 0;
unsigned long previous_time_ms = 0;
volatile long sample_interval_ms = 51;  // 50ms per campionare 100 volte in 5s

#ifdef DEBUG_LOCK_IN_TAU  // If activated it enable new paramters for state machine and overwrite previous sample time parameter
unsigned long previous_time_ms_3 = 0;
bool togle = 1;
volatile int NUM_COLL = 1000;
volatile long sample_interval_ms = 5;  //For the Debug Lock In Tau we advice to use a fast sample interval
#endif

bool monitor_flag = false;  // boolean to enable the V+ and V- DC monitor
const int monitor_buffer_size = 500;  // Numero di campioni per blocco
float vp_buffer[monitor_buffer_size];
float vm_buffer[monitor_buffer_size];
int z_buffer[monitor_buffer_size];
int monitor_idx = 0;
// Temporary variables
unsigned long last_monitor_millis = 0;
const unsigned long monitor_interval = 0.5; // campionamento ogni 10 ms (1 KHz)


const int MAX_COLL = 4000;
volatile int NUM_COLL = 200;
float X1_store[MAX_COLL];
float Y1_store[MAX_COLL];
float X2_store[MAX_COLL];
float Y2_store[MAX_COLL];
float VO_store[MAX_COLL];
float DELTA_store[MAX_COLL];

int write_value_DAC0;
int write_value_DAC1;

int sample_index = 0;
static unsigned long next_sample_time = 0;

int pinsdi = 35;  // SDI - shared]
int pinclk = 34;  // CLK - shared

// Chip select pins for the 4 PGAs Programmable Gain Amplifiers MAX9939
int pincs[4] = { 30, 32, 33, 31 };  // CS0 to CS3
// Real Gain V/V reference values for MAX9939
const float gainTable[4] = {0.25, 1, 10, 120
};
int presetSelected[4];  // 4 chip select
// Voffset controllato (inizializzato al valore di prima)
float voltage_offset = 0.5;

float amplitude_scale = 0.08;  // non più di 0.5
// Coefficients for fundamental (a1) and second harmonic (a2)
float a1_coeff = 0.5;  // set to be at 0.5 and produce a sinusoidal wave for reference
float a2_coeff = 0.0;

// === Lock-in Detection ===
const int analogInput = A1;       // Analogic Input V+ AC
const float analogInput_vp = A0;  // Analogic Input V+ DC
const float analogInput_vm = A2;  // Analogic Input V- DC
float alpha = 6E-4;               // IIR filter coefficient
float beta = 6E-4;
float av_vin = 0, vin = 0;
// variables for the IIR filter X1 and Y1 first harmonics
volatile float X1_f1 = 0, X1_f2 = 0;
volatile float Y1_f1 = 0, Y1_f2 = 0;

// variables for the IIR filter X1 and Y1 second harmonics
volatile float X2_f1 = 0, Y2_f2 = 0;
volatile float Y2_f1 = 0, X2_f2 = 0;

// Results lock-in
volatile float R1_est = 0.0;  // ampiezza stimata fondamentale
volatile float R2_est = 0.0;  // ampiezza stimata seconda armonica
//volatile float Theta_1_rad, Theta_2_rad = 0; //Not used directly on Arduino, if decleared see the function that recall them and use where needed
volatile float delta_0 = 0;
const float pi = 3.14159;
const float delta_target = pi / 2;

//--------------------Variable for calibration
volatile float R1_max_cal = 180.0;  // Initial gues amplitude for R1
volatile float R2_max_cal = 30.0;   // Initial gues amplitude for R2
// Variables for Calibration configuration
const long CALIBRATION_INTERVAL_MS = sample_interval_ms;
unsigned long calibration_previous_time_ms = 0;  // Nuovo timer per la calibrazione
int calibration_state = 0;                       // 0: Non iniziata, 1: In corso, 2: Completata
int calibration_sweep_index = 0;
const int CALIBRATION_HISTORY_SIZE = 2;
// Array to record results of last N calibrations
float v1_history[CALIBRATION_HISTORY_SIZE] = { 0.0 };
float v2_history[CALIBRATION_HISTORY_SIZE] = { 0.0 };
// Calibrations counter that have been done.
int calibration_count = 0;

/*------------- PID ------------------*/
// Boolean to enable PID
bool pid_enabled = true;
volatile float pidError = 0.0, pidIntegral = 0.0, pidDerivative = 0.0;
volatile float prevError = 0.0;

// PID parameters
volatile float Kp = 0.07;
volatile float Ki = 0.5;
volatile float Kd = 0.0;

const float DT = sample_interval_ms / 1000.0;  // Sampling time for the PID integral and derivate

// Voff_pid --> voltage offset controlled by PID
float Voff_pid = 0.0;
// TTL control for Freq Ref.
bool ttl_mode = false;
// --- LED Control Definitions ---
// ---- Signal from DTV+
const int LED_A1_RED_PIN = 23;    // D22
const int LED_A1_GREEN_PIN = 22;  // D23

// ---- Signal from DTV-
const int LED_A2_GREEN_PIN = 24;  // D24
const int LED_A2_RED_PIN = 25;    // D25

// --- Freqe Ref Output
const int LED_FreqRef_GREEN_PIN = 26;  // D26
const int LED_FreqRef_RED_PIN = 27;    // D27

// ---- DAC 0 Output
const int LED_SCALE_ON_PIN = 28;   // D28
const int LED_SCALE_OFF_PIN = 29;  // D29

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

  //==== TTL table
  for (int i = 0; i < NUM_SAMPLES; i++) {
    if (i < NUM_SAMPLES / 2)
      ttl_table[i] = 1.0;  // HIGH
    else
      ttl_table[i] = 0.0;  // LOW
  }

#ifdef _START_UP_MESSAGE_
  while (!Serial)
    ;  // Attende la connessione seriale (solo per schede native USB)
  Serial.print("Stabilizer interferometer");
  Serial.print('\t');
  Serial.println("Authors: Giuseppe E. Lio and Simone Zanotto");
  Serial.println("General Information and start up parameters");
  Serial.println(" ");
  Serial.println("Sinwave generator 1 kHz on DAC0 (A12)");
  Serial.print("Frequency (Hz): ");
  Serial.println(FREQUENCY_HZ);

  Serial.print("Amplitude: ");
  Serial.println(amplitude_scale, 4);
  Serial.print("Voltage Offset: ");
  Serial.println(voltage_offset, 4);
  Serial.print("Kp: ");
  Serial.println(Kp, 4);
  Serial.print("Ki: ");
  Serial.println(Ki, 4);
#endif

  initDigitalPots();
  analogWriteResolution(12);
  analogReadResolution(12);
  pid_enabled = false;
  init_leds();  // Inizialize LED pins

  calibration_state = 1;  // Initialize Calibration immediately after Setup
  Serial.println("Starting calibration mode - PID deactivated");
}


// DAC update function (to be called in the Timer ISR)
void update_dac(int sample_index) {
  // DAC0 sinwave 1 KHz for Piezo
  write_value_DAC0 = (int)((sine_wave_table[sample_index] * amplitude_scale + voltage_offset) * DAC_MAX_VALUE);
  analogWrite(A12, write_value_DAC0);
  // DAC1 Sinwave or TTL for Freq Ref
  if (ttl_mode) {
    // TTL MODE ACTIVE
    a1_coeff = 0.0;
    a2_coeff = 0.0;
    float ttl_value = ttl_table[sample_index];  // either 0.0 or 1.0 duty cycle 50%.
    write_value_DAC1 = (int)(ttl_value * DAC_MAX_VALUE);
    analogWrite(A13, write_value_DAC1);
  } else {
    // NORMAL SINE MODE at 1 KHz, 2 KHZ or mixed.
    write_value_DAC1 = (int)(((a1_coeff * sine_wave_table[sample_index] + a2_coeff * sine_wave_table_2[sample_index]) + 0.5) * DAC_MAX_VALUE);
    analogWrite(A13, write_value_DAC1);
  }

  float rawInput = analogRead(analogInput);  // Reading the analog values from A1 the AC input signal
  av_vin += beta * (rawInput - av_vin);      //(rawInput - 2048) / 2048.0;
  vin = rawInput - av_vin;
  //vin_store[sample_index] = vin; // for debugging
  float ref1_cos = (float)(sine_wave_table[(sample_index + NUM_SAMPLES / 4) % NUM_SAMPLES]);
  float ref1_sin = (float)(sine_wave_table[sample_index]);
  float ref2_cos = (float)(sine_wave_table_2[(sample_index + NUM_SAMPLES / 8) % NUM_SAMPLES]);
  float ref2_sin = (float)(sine_wave_table_2[sample_index]);

  // Product Signal × Reference
  float xi1 = vin * ref1_cos;
  //cos_store[sample_index] = xi1; // for debugging
  float xq1 = vin * ref1_sin;
  //sin_store[sample_index] = xq1; // for debugging
  float xi2 = vin * ref2_cos;
  float xq2 = vin * ref2_sin;

  // Digital low pass filter
  // Filter will be updated at each sampling
  X1_f1 += alpha * (xi1 - X1_f1);
  X1_f2 += alpha * (X1_f1 - X1_f2);
  Y1_f1 += alpha * (xq1 - Y1_f1);
  Y1_f2 += alpha * (Y1_f1 - Y1_f2);

  X2_f1 += alpha * (xi2 - X2_f1);
  X2_f2 += alpha * (X2_f1 - X2_f2);
  Y2_f1 += alpha * (xq2 - Y2_f1);
  Y2_f2 += alpha * (Y2_f1 - Y2_f2);
}

void lockin() {
  // Ampiezze stimate
  // Il fattore 2.0 è corretto per compensare la miscelazione con un riferimento di ampiezza 1

  R1_est = 2.0 * sqrt(X1_f2 * X1_f2 + Y1_f2 * Y1_f2);
  R2_est = 2.0 * sqrt(X2_f2 * X2_f2 + Y2_f2 * Y2_f2);
  //Theta_1_rad = atan2(Y1_f2, X1_f2); //Phase of the first signal (radians)
  //Theta_2_rad = atan2(Y2_f2, X2_f2); //Phase of the second signal (radians)
  delta_0 = atan2(-(R1_est / R1_max_cal) * sign(X1_f2), (R2_est / R2_max_cal) * sign(X2_f2));

  X1_store[current_sample_index] = X1_f2;
  Y1_store[current_sample_index] = Y1_f2;
  X2_store[current_sample_index] = X2_f2;
  Y2_store[current_sample_index] = Y2_f2;
  VO_store[current_sample_index] = voltage_offset;
  DELTA_store[current_sample_index] = delta_0;

  current_sample_index++;
  if (current_sample_index >= NUM_COLL) {
    current_sample_index = 0;
  }
#ifdef DEBUG_LOCK_IN
  Serial.print("R1 =");
  Serial.print(R1_est, 4);
  Serial.print("\t");
  Serial.print("R2 =");
  Serial.print("\t");
  Serial.print(R2_est, 4);
  Serial.print("delta_0: ");
  Serial.println(delta_0);
#endif
}

float sign(float x) {
  if (x > 0.0) return 1;
  if (x < 0.0) return -1;
  return 0;
}

// --- Loop Principale ---
void loop() {
  handleSerialCommands();
  unsigned long current_time = micros();
  update_dac((micros() % PERIOD) / SAMPLE_INTERVAL_US);  // Aggiorna il DAC
  /**/
  unsigned long current_time_ms = millis();

  if (current_time_ms - previous_time_ms >= sample_interval_ms) {
    previous_time_ms = current_time_ms;
    lockin();  // Chiama la funzione senza indice
    if (pid_enabled) {
      PID();
    }
  }
#ifdef DEBUG_LOCK_IN_TAU
  pid_enabled = false;
  unsigned long current_time_ms_3 = millis();
  if (current_time_ms_3 - previous_time_ms_3 >= 1000) {
    previous_time_ms_3 = current_time_ms_3;
    if (togle) {
      voltage_offset = 0.5;
      togle = 0;
    } else {
      voltage_offset = 0.6;
      togle = 1;
    }
  }
#endif

  /* Monitor to measure V+ and V-*/
  unsigned long current_time_monitor = millis();
   if (monitor_flag && (current_time_monitor - last_monitor_millis >= monitor_interval)) {
        last_monitor_millis = current_time_monitor;

        float gain_p = gainTable[presetSelected[0]];
        float gain_m = gainTable[presetSelected[3]];

        float v_adc_p = analogRead(analogInput_vp) * (3.3 / 4095.0);
        float monitor_vp = (1.64 - v_adc_p) / gain_p;

        float v_adc_m = analogRead(analogInput_vm) * (3.3 / 4095.0);
        float monitor_vm = (1.64 - v_adc_m) / gain_m;

        int raw_osc = analogRead(analogInput);

        // collecting
        vp_buffer[monitor_idx] = monitor_vp;
        vm_buffer[monitor_idx] = monitor_vm;
        z_buffer[monitor_idx] = raw_osc;
        monitor_idx++;

         if (monitor_idx >= monitor_buffer_size) {
            for (int i = 0; i < monitor_buffer_size; i++) {
                Serial.print("M,");
                Serial.print(vp_buffer[i], 4);
                Serial.print(",");
                Serial.print(vm_buffer[i], 4);
                Serial.print(",");
                Serial.println(z_buffer[i]);
            }
            monitor_idx = 0; // reset buffer
        }
    }
  /*unsigned long current_time_monitor = millis();
  if (monitor_flag == true && current_time_monitor % 500 == 0) {
    float gain_p = gainTable[presetSelected[0]];
    float gain_m = gainTable[presetSelected[3]];

    float v_adc_p = analogRead(analogInput_vp) * (3.3 / 4095.0);
    float monitor_vp = (1.64 - v_adc_p) / gain_p;

    float v_adc_m = analogRead(analogInput_vm) * (3.3 / 4095.0);
    float monitor_vm = (1.64 - v_adc_m) / gain_m;

    int raw_osc = analogRead(analogInput);


    Serial.print("M,");
    Serial.print(monitor_vp, 4);
    Serial.print(",");
    Serial.print(monitor_vm, 4);
    Serial.print(",");
    Serial.println(raw_osc, 1);
  }*/
  /*---- LED ---- */
  unsigned long current_time_ms_2 = millis();
  if (current_time_ms_2 % 1000 == 0) {
    update_leds();  // Update LED status
  }

  // 3. State machine for calibration
  if (calibration_state == 1) {
    unsigned long current_cal_time_ms = millis();

    // Check the time counter
    if (current_cal_time_ms - calibration_previous_time_ms >= CALIBRATION_INTERVAL_MS) {
      calibration_previous_time_ms = current_cal_time_ms;  // Salva il tempo corrente

      // Sets the voltage_offset to the next value for calibrating it
      voltage_offset = (float)calibration_sweep_index / NUM_COLL;

      Serial.print("Calibrazione passo ");
      Serial.print(calibration_sweep_index + 1);
      Serial.print("/");
      Serial.print(NUM_COLL);
      Serial.print(" - Offset: ");
      Serial.println(voltage_offset, 4);

      calibration_sweep_index++;

      // Controlla se lo sweep è finito
      if (calibration_sweep_index >= NUM_COLL) {
        run_calibration_calculation();
        // run_calibration_calculation() sets calibration_state = 2 and pid_enabled = true
      }
    }
  }
  // If calibration_state == 2, Calibration is ignored.
}

void PID() {
  volatile float VOFF_MIN = amplitude_scale;
  volatile float VOFF_MAX = 1.0 - amplitude_scale;
  // 1. Error definition
  float error = delta_0 - delta_target;
  // 2. Proporational
  float proportional = Kp * error;
  // 3. Derivate
  float derivative = Kd * (error - prevError) / DT;
  // 4. Update fo the previous error
  prevError = error;
  float pidOutput = proportional + (Ki * pidIntegral) + derivative;
  // 6. New voltage_offset (PID Output)
  //PID output is the new correction to be applied.
  voltage_offset = 0.5 + pidOutput;

  pidIntegral += error * DT;

  // 8. Clamping and update od pidIntegral
  if (voltage_offset > VOFF_MAX) pidIntegral = 0.0;
  if (voltage_offset < VOFF_MIN) pidIntegral = 0.0;

#ifdef DEBUG_PID
  Serial.print("Delta_0: ");
  Serial.print(delta_0, 4);
  Serial.print(" | Error: ");
  Serial.print(error, 4);
  Serial.print(" | pidIntegral: ");
  Serial.print(pidIntegral, 4);
  Serial.print(" | V_off: ");
  Serial.println(voltage_offset, 4);
#endif
}

// Calculation about R1 and R2 max after calibration, some temperaty variable have been used V1 and V2 linked to R1 and R2, respectively.
void run_calibration_calculation() {
  float max_V1 = 0.0;
  float max_V2 = 0.0;

  for (int i = 0; i < NUM_COLL; i++) {
    float temp_V1 = 2.0 * sqrt(X1_store[i] * X1_store[i] + Y1_store[i] * Y1_store[i]);
    float temp_V2 = 2.0 * sqrt(X2_store[i] * X2_store[i] + Y2_store[i] * Y2_store[i]);

    if (temp_V1 > max_V1) max_V1 = temp_V1;
    if (temp_V2 > max_V2) max_V2 = temp_V2;
  }

  // 2.1. Add the new values (max_V1, max_V2) to history.
  int current_index = calibration_count % CALIBRATION_HISTORY_SIZE;
  v1_history[current_index] = max_V1;
  v2_history[current_index] = max_V2;
  calibration_count++;  // Increases the calibration counter

  // 2.2. Calculate the average of the collected values
  float v1_sum = 0.0;
  float v2_sum = 0.0;

  int num_items_to_average = (calibration_count > CALIBRATION_HISTORY_SIZE) ? CALIBRATION_HISTORY_SIZE : calibration_count;

  for (int i = 0; i < num_items_to_average; i++) {
    v1_sum += v1_history[i];
    v2_sum += v2_history[i];
  }
  // 2.3. Calculate the new max values of R1 and R2
  R1_max_cal = v1_sum / num_items_to_average;
  R2_max_cal = v2_sum / num_items_to_average;

  Serial.print("New Calibration R1_Max (mediated on ");
  Serial.print(num_items_to_average);
  Serial.print(" samples): ");
  Serial.println(R1_max_cal, 4);
  Serial.print("New Calibration R2_Max (mediated on ");
  Serial.print(num_items_to_average);
  Serial.print(" samples): ");
  Serial.println(R2_max_cal, 4);
  // Calibration completed and new state setled
  calibration_state = 2;
  // Voltage offset reset
  voltage_offset = 0.5;
  // *** Enable PID ***
  pid_enabled = true;
  Serial.println("Calibration completed. PID started.");
}

// GAIN table B and TRIM C in 4 bit (bool[4]) // for reference see the datasheet of MAX9939
const bool presetMap[4][4] = {
  { 1, 0, 0, 1 },   // 1 --> 0.25
  { 0, 0, 0, 0 },  // 2 --> 1
  { 1, 0, 0, 0 },  // 3 --> 10
  { 1, 1, 1, 0 }  // 4 --> 120
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

void initDigitalPots() {  // This function is fundamental for resetting PGAs at every restart, otherwise they retain their previous setting
  for (int csIndex = 0; csIndex < 4; csIndex++) {
    int csPin = pincs[csIndex];

    // Initialize GAIN with gain 0.25 and enableBit=0
    buildDigitalDataFromInput(csPin, 0, 0, 'P', false, true);

    // Initialize TRIM with value 1, polarity=P and enableBit=0
    buildDigitalDataFromInput(csPin, 0, 0, 'P', false, false);

    // Print the log on Serial
    Serial.print("Init Pot ");
    Serial.print(csIndex + 1);
    Serial.println(" → Gain=1 Trim=1 P0");
  }
}

void handleSerialCommands() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    input.toUpperCase();  // All commands are converted in all caps
    //Serial.print("Received input: ");
    //Serial.println(input);

    // ===  Calibration command ===
    if (input == "CALIBRATE") {
      Serial.println("Calibration command received");

      // 1. Disable PID to avoid interferences during calibration
      pid_enabled = false;
      Serial.println("PID temporary deactivated.");

      // 2. Reset sweep index
      calibration_sweep_index = 0;

      // 3. Set the satus as "Calibration on" to star the state machine
      calibration_state = 1;

      Serial.println("Starting new calibration");
      return;  // Exit to not process other commands
    }
    // Enabling TTL Freq REF
    if (input == "TTL ON") {
      ttl_mode = true;
      Serial.println("TTL ON");
      return;  // Exit to not process other commands
    }
    // === Disabling TTL Freq REF
    else if (input == "TTL OFF") {
      ttl_mode = false;
      Serial.println("TTL OFF");
      return;  // Exit to not process other commands
    }
    // === Enabling Monitor V+ and V-
    if (input == "MONITOR ON") {
      monitor_flag = true;
      return;  // Exit to not process other commands
    }
    // === Disabling Monitor V+ and V-
    else if (input == "MONITOR OFF") {
      monitor_flag = false;
      return;  // Exit to not process other commands
    }
    // === Enabling PID ===
    // === To Activate PID ===
    if (input == "PID ON") {
      if (!pid_enabled) {  // Check if it is already activated
        pid_enabled = true;
        pidIntegral = 0.0;
        voltage_offset = 0.5;
        Serial.println("PID activated.");
      } else {
        Serial.println("PID already activated.");
      }
      return;  // Exit to not process other commands
    }
    // === To Deactivate PID ===
    if (input == "PID OFF") {
      if (pid_enabled) {  // Check if it is already deactivated
        pid_enabled = false;
        pidIntegral = 0.0;
        voltage_offset = 0.5;
        Serial.println("PID deactivated.");
      } else {
        Serial.println("PID already deactivated.");
      }
      return;  // Exit to not process other commands
    }
    // === To print on the Serial X1,Y1,X2,Y2, Delta and V_offset === //Pay attention to the order
    if (input == "PRINT") {
      print_lock_in();
      return;
    }
    // === Parameters ===
    if (input.startsWith("SET ")) {  // There must be a space after SET
      if (input.length() < 7) {      // Minimum length check to prevent crashes in case of incomplete SET
        Serial.println("Incomplete SET command. Format: SET [A|S|O|1|2] [value]");
        return;
      }
      char param = input.charAt(4);  // A, S, O, 1, 2
      String valueStr = input.substring(6);
      float val = valueStr.toFloat();

      switch (param) {

        case 'A':  // amplitude_scale
          if (val >= 0.0 && val <= 0.5) {
            amplitude_scale = val;
            Serial.print("Amplitude scale updated: ");
            Serial.println(amplitude_scale, 3);
          } else {
            Serial.println("Amplitude_scale >=0  <=0.5");
          }
          break;

        case '1':  // a1_coeff (first harmonics)
          if (val >= 0.0 && val <= 1.0) {
            a1_coeff = val;
            Serial.print("a1 coefficient (1KHz): ");
            Serial.println(a1_coeff, 2);
          } else {
            Serial.println("a1 >=0.0  <=1.0");
          }
          break;

        case '2':  // a2_coeff (second harmonics)
          if (val >= 0.0 && val <= 1.0) {
            a2_coeff = val;
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

        case 'I':  // Ki
          if (val >= 0 && val <= 1) {
            Ki = (float)val;
            Serial.print("Ki: ");
            Serial.println(Ki);
          } else {
            Serial.println("Ki  >=0 and <=1");
          }
          break;

        case 'P':  // Kp
          if (val >= 0 && val <= 1) {
            Kp = (float)val;
            Serial.print("Kp: ");
            Serial.println(Kp);
          } else {
            Serial.println("Kp >=0 <=1");
          }
          break;

        case 'D':  // Kd
          if (val >= 0 && val <= 1) {
            Kd = (float)val;
            Serial.print("Kd: ");
            Serial.println(Kd);
          } else {
            Serial.println("Kd >=0 <=1");
          }
          break;

        case 'T':  // sampling interval in ms
          if (val >= 1) {
            sample_interval_ms = (int)val;
            Serial.print("sample_interval_ms: ");
            Serial.println(sample_interval_ms);
          } else {
            Serial.println("sample_interval_ms (int) >50");
          }
          break;

        case 'N':
          if (val >= 10 && val <= 4000) {
            NUM_COLL = (int)val;
            Serial.print("number of collected samples: ");
            Serial.println(NUM_COLL);
          } else {
            Serial.println("number of collected samples (int) >10");
          }
          break;

        default:
          Serial.println("Unknown parameter. Use S, O, I, P, D1, 2, T, N.");
      }
      return;  // Exit to not process other commands
    }

    // === Comando RESET ===
    if (input.equalsIgnoreCase("RESET")) {
      Serial.println("System restart...");
      delay(10);
      NVIC_SystemReset();  // STM32/ARM reset
    }

    // === If it is not a SET command, interpret it as a digital command for PGAs ===
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
    if (presetIndex < 0 || presetIndex > 3) {
      Serial.println("Invalid preset.");
      return;
    }
    presetSelected[csIndex] = presetIndex;

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

  // Bit 1-4: from table
  for (int i = 0; i < 4; i++) {
    digitaldata[i + 1] = isGain
                           ? presetMap[presetIndex][i]     //GAIN
                           : channelMap[channelIndex][i];  //TRIM
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
    delayMicroseconds(1);  // A small delay to stabilize the SDI pin
    digitalWrite(pinclk, LOW);
    delayMicroseconds(1);  // A small delay to ensure the CLK pulse width
    digitalWrite(pinclk, HIGH);
    delayMicroseconds(1);  // A small delay to ensure maintenance time
  }

  delayMicroseconds(50);
  digitalWrite(pincs, HIGH);
}

void print_vin() {
  for (int i = 0; i < NUM_SAMPLES; i++) {
    Serial.print(vin_store[i]);
    Serial.print('\t');
    Serial.print(sin_store[i]);
    Serial.print('\t');
    Serial.println(cos_store[i]);
  }
  Serial.println();
}

void print_lock_in() {
  // Print the data starting from the oldest sample (the one after the last one written)
  // up to the last sample written.

  // Use the correct global index
  int start_index = current_sample_index;

  for (int i = 0; i < NUM_COLL; i++) {
    // The print index ranges from 0 to NUM_COLL-1
    // The array index uses the modulus to start from the oldest sample
    int print_index = (start_index + i) % NUM_COLL;

    Serial.print(X1_store[print_index]);
    Serial.print('\t');
    Serial.print(Y1_store[print_index]);
    Serial.print('\t');
    Serial.print(X2_store[print_index]);
    Serial.print('\t');
    Serial.print(Y2_store[print_index]);
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
  pinMode(LED_A2_RED_PIN, OUTPUT);
  pinMode(LED_A2_GREEN_PIN, OUTPUT);
  pinMode(LED_FreqRef_GREEN_PIN, OUTPUT);
  pinMode(LED_FreqRef_RED_PIN, OUTPUT);
  pinMode(LED_SCALE_ON_PIN, OUTPUT);
  pinMode(LED_SCALE_OFF_PIN, OUTPUT);

  // Initial state: turn off all LEDs
  digitalWrite(LED_A1_RED_PIN, LOW);
  digitalWrite(LED_A1_GREEN_PIN, LOW);
  digitalWrite(LED_A2_RED_PIN, LOW);
  digitalWrite(LED_A2_GREEN_PIN, LOW);
  digitalWrite(LED_FreqRef_GREEN_PIN, LOW);
  digitalWrite(LED_FreqRef_RED_PIN, LOW);
  digitalWrite(LED_SCALE_ON_PIN, LOW);
  digitalWrite(LED_SCALE_OFF_PIN, LOW);
}

// Function to control LED pair D22/D23 based on A0 and A2 DTV (+ and -) DC reading
void control_a0_a2_leds() {

  //float v_adc_p = analogRead(analogInput_vp) * (3.3 / 4095.0);
  //float v_adc_m = analogRead(analogInput_vm) * (3.3 / 4095.0);
  float gain_p = gainTable[presetSelected[0]];
  float gain_m = gainTable[presetSelected[3]];

  float v_adc_p = analogRead(analogInput_vp) * (3.3 / 4095.0);
  float monitor_vp = (1.64 - v_adc_p) / gain_p;

  float v_adc_m = analogRead(analogInput_vm) * (3.3 / 4095.0);
  float monitor_vm = (1.64 - v_adc_m) / gain_m;

  // First monitor V+
  if (monitor_vp > 0.01 && monitor_vp < 1.6) {
    // Signal is high: Turn on Green (D23), Turn off Red (D22)
    digitalWrite(LED_A1_GREEN_PIN, HIGH);
    digitalWrite(LED_A1_RED_PIN, LOW);
  } else {
    // Signal is low: Turn on Red (D22), Turn off Green (D23)
    digitalWrite(LED_A1_GREEN_PIN, LOW);
    digitalWrite(LED_A1_RED_PIN, HIGH);
  }
  // Second monitor V-
  if (monitor_vm > 0.01 && v_adc_m < 1.6) {
    // Signal is high: Turn on Green (D25), Turn off Red (D24)
    digitalWrite(LED_A2_GREEN_PIN, HIGH);
    digitalWrite(LED_A2_RED_PIN, LOW);
  } else {
    // Signal is low: Turn on Red (D25), Turn off Green (D24)
    digitalWrite(LED_A2_GREEN_PIN, LOW);
    digitalWrite(LED_A2_RED_PIN, HIGH);
  }
}

void control_freq_ref_led() {  //It uses leds A26 and A27 to state if the FREQ REF is enabled or not.
  if (a1_coeff > 0.0 || ttl_mode) {
    digitalWrite(LED_FreqRef_GREEN_PIN, HIGH);
    digitalWrite(LED_FreqRef_RED_PIN, LOW);
  } else {
    digitalWrite(LED_FreqRef_GREEN_PIN, LOW);
    digitalWrite(LED_FreqRef_RED_PIN, HIGH);
  }
}
// Function to control LED pair D28/D29 based on amplitude_scale
void control_scale_leds() {
  // amplitude_scale is a volatile float
  if (amplitude_scale <= 0.0 || !pid_enabled) {
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
  control_a0_a2_leds();
  control_freq_ref_led();
  control_scale_leds();
}
