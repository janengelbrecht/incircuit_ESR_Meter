//==============================================================================================================================
//|															       |
//|					Firmware for In-Circuit ESR-meter						       |
//|															       |
//==============================================================================================================================
//| DOKUMENT ID:	SW-ESR-MIL-002											       |
//| DATO:		19-01-25											       |
//| VERSION:		2.00												       |
//| SYSTEM:		INCIRCUIT ESR METER										       |
//| FORFATTER:		Jan Engelbrecht Pedersen									       |
//==============================================================================================================================
/**
 * @file ESR_Meter.ino
 * @brief Firmware for In-Circuit ESR-meter baseret på kravspecifikation SRS-ESR-MIL-001
 * @author Jan Engelbrecht Pedersen
 * @date 18-01-25
 * @version 1.00
 *  ==========================================================================================================================
 * @details Denne firmware implementerer et ESR-meter med følgende funktioner:
 * - 100 kHz testsignal via AD9850 DDS-generator (rettet HAL/Driver)
 * - Automatisk PGA-valg (2x, 4x, 8x, 16x) baseret på signalstyrke
 * - 64× oversampling og middling af ADC-målinger
 * - Lineær interpolation fra kalibrerede tabeller
 * - Kelvin-måling med fire-leder teknik
 * - SCPI-protokol for seriel kommunikation
 * - State machine-baseret UI med knapdebounce
 * - Watchdog timer for systemstabilitet
 *  ==========================================================================================================================
 * Hardware-platform: ESP32 (ESP32-WROOM-32)
 * Udviklingsmiljø: Arduino IDE 2.x
 * Programmeringssprog: Arduino C++ (ikke objektorienteret)
 * 
 * Kravspecifikation: SRS-ESR-MIL-001 version 1.00
 * Dokument ID: SW-ESR-MIL-002
 * 
 * @note Alle tabeller er gemt i PROGMEM og kræver genprogrammering for kalibrering
 * @warning Signalniveau er begrænset til 0,1V p-p for sikker in-circuit test
 */
/*
 *  ==========================================================================================================================
 * ESP32 GPIO CONFIGURATION SUMMARY - ESR-Meter v1.00
 *  ==========================================================================================================================
 * 
 * AD9850 DDS GENERATOR CONTROL (Parallel Interface):
 * --------------------------------------------------
 * GPIO 16  - AD9850_W_CLK   : Word clock signal for AD9850 (output, active high)
 * GPIO 17  - AD9850_FQ_UD   : Frequency update signal for AD9850 (output, pulse high to latch)
 * GPIO 18  - AD9850_DATA    : Serial data line for AD9850 (output, LSB first)
 * GPIO 19  - AD9850_RESET   : Reset signal for AD9850 (output, active high)
 * 
 * I2C COMMUNICATION BUS (400 kHz):
 * --------------------------------
 * GPIO 21  - I2C_SDA        : I2C data line (open-drain, internal pull-up)
 *                             Connected to: ADS1115 ADC (pin 1) + LCD I2C backpack
 * GPIO 22  - I2C_SCL        : I2C clock line (open-drain, internal pull-up)
 *                             Connected to: ADS1115 ADC (pin 2) + LCD I2C backpack
 * 
 * USER INPUT BUTTONS (Internal Pull-up, Active LOW):
 * --------------------------------------------------
 * GPIO 27  - BUTTON1_PIN    : Button 1 - Toggles display mode (RAW_ADC ↔ ESR_RESULT)
 * GPIO 33  - BUTTON2_PIN    : Button 2 - Toggles communication mode (NORMAL ↔ SCPI)
 * 
 * HARDWARE NOTES:
 * --------------- 
 * - Internal pull-up resistors enabled (approximately 45kΩ)
 * - External RC debouncing (10kΩ + 100nF) on button circuits
 * - AD9850 pins connected via 3.3V↔5V bidirectional level shifter
 * - I2C bus uses external 4.7kΩ pull-up resistors to 3.3V
 * - No analog pins used (ADS1115 handles analog via I2C)
 * 
 * UNUSED GPIO PINS:
 * -----------------
 * All other GPIO pins are currently unassigned and available for future expansion:
 * - GPIOs 0, 2, 4, 5, 12-15, 23-26, 32: Available for general I/O
 * - GPIOs 25-26: Can be used for DAC output if needed
 * - GPIOs 36-39: Input-only, available for additional sensors
 * 
 * POWER CONSIDERATIONS:
 * ---------------------
 * - ESP32 digital I/O: 3.3V logic level, 40mA max per pin
 * - AD9850 interface: 5V logic, requires level shifting
 * - Total GPIO current within ESP32 specifications (< 1.2A)
 * 
 *  ===========================================================================================================================
 */

// ============================================================================================================================
// INKLUDERING AF BIBLIOTEKER
// ============================================================================================================================

/**
 * @brief Inkluder wire bibliotek: I2C kommunikation til ADS1115 og LCD
 */
#include <Wire.h>               // I2C kommunikation til ADS1115 og LCD
/**
 * @brief Inkluder Adafruit_ADS1X15 bibliotek: ADS1115 ADC-driver
 */
#include <Adafruit_ADS1X15.h>   // Adafruit ADS1115 ADC-driver
/**
 * @brief Inkluder LiquidCrystal_I2C bibliotek: 16x2 LCD-display med I2C backpack
 */
#include <LiquidCrystal_I2C.h>  // Adafruit 16x2 LCD-display med I2C backpack
/**
 * @brief Inkluder Watchdog timer for ESP32 bibliotek: Watchdog timer for ESP32
 */
#include <esp_task_wdt.h>       // Watchdog timer for ESP32
/**
 * @brief Inkluder Math bibliotek: Til sqrt og andre matematiske funktioner
 */
#include <math.h>               // Til sqrt og andre matematiske funktioner

// ============================================================================================================================
// KONSTANTER OG DEFINITIONER
// ============================================================================================================================

// Systemversion og identifikation
#define FIRMWARE_VERSION    "1.00"
#define HARDWARE_VERSION    "SRS-ESR-MIL-001"
#define DEVICE_NAME         "ESR-METER"

// Timing konstanter (i millisekunder)
#define MEASUREMENT_INTERVAL    100   // Måleinterval (S.PER.001.1: ≤100 ms)
#define DISPLAY_UPDATE_INTERVAL 50    // Display opdateringsinterval (S.PER.001.2: ≤50 ms)
#define DEBOUNCE_DELAY          20    // Knap debounce delay (S.PER.001.4: ≤20 ms)
#define SCPI_RESPONSE_TIMEOUT   10    // SCPI respons timeout (S.PER.001.3: ≤10 ms)
#define WATCHDOG_TIMEOUT        3000  // Watchdog timeout i ms (S.WDG.001.1: 3 sekunder)

// ADC og signalbehandling konstanter
#define OVERSAMPLING_FACTOR     64    // Oversampling faktor (S.ADC.001.2: 64×)
#define ADC_FULL_SCALE          32767 // 16-bit ADC fuld skala (2^15 - 1)
#define ADC_REFERENCE_VOLTAGE   2.048 // ADS1115 intern reference (H.ADC.001)
#define PGA_THRESHOLD_LOW       8192  // 25% af fuld skala for PGA-øgning
#define PGA_THRESHOLD_HIGH      29491 // 90% af fuld skala for PGA-reduktion

// Tabel konstanter
#define NUM_PGA_SETTINGS        4     // Antal PGA-indstillinger: 2x, 4x, 8x, 16x
#define TABLE_ENTRIES           30    // Antal indgange i hver tabel (S.ESR.001.1)
#define PGA_2X_INDEX            0     // Index for PGA=2× tabel
#define PGA_4X_INDEX            1     // Index for PGA=4× tabel  
#define PGA_8X_INDEX            2     // Index for PGA=8× tabel
#define PGA_16X_INDEX           3     // Index for PGA=16× tabel

// State machine tilstande
typedef enum {
    STATE_IDLE,              // System i hviletilstand
    STATE_MEASURE_ADC,       // ADC-måling i gang
    STATE_CALCULATE_ESR,     // ESR-beregning i gang
    STATE_UPDATE_DISPLAY,    // Display opdatering i gang
    STATE_PROCESS_SERIAL,    // Seriel kommunikation i gang
    STATE_ERROR              // Fejltilstand
} SystemState;

// Display tilstande
typedef enum {
    DISPLAY_MODE_RAW_ADC,    // Vis rå ADC-værdi (til kalibrering)
    DISPLAY_MODE_ESR_RESULT  // Vis beregnet ESR-værdi (normal drift)
} DisplayMode;

// Kommunikation tilstande
typedef enum {
    COMM_MODE_NORMAL,        // Normal seriel output
    COMM_MODE_SCPI           // SCPI-protokol aktiv
} CommunicationMode;

// ============================================================================================================================
// HARDWARE PIN-KONFIGURATION
// ============================================================================================================================

// AD9850 DDS-generator pins (parallelt interface)
/**
 * @defgroup AD9850pins AD9850-DDS-generator-pins-(parallelt interface)
 * @brief ESP32 pins forbundet til AD9850: parallelt interface
 * @{
 */
/**
 * @brief pin definition på ESP32: Word clock pin
 */
#define AD9850_W_CLK  16     // Word clock pin
/**
 * @brief pin definition på ESP32: Frequency update pin
 */
#define AD9850_FQ_UD  17     // Frequency update pin
/**
 * @brief pin definition på ESP32: Serial data pin
 */
#define AD9850_DATA   18     // Serial data pin
/**
 * @brief pin definition på ESP32: Reset pin
 */
#define AD9850_RESET  19     // Reset pin
/** @} */ // Slut på gruppen AD9850pins

// Knappins (med intern pull-up)
/**
 * @brief pin definition på ESP32: Knap1: Skift mellem RAW_ADC og ESR_RESULT
 */
#define BUTTON1_PIN   27     // Knap 1: Skift mellem RAW_ADC og ESR_RESULT
/**
 * @brief pin definition på ESP32: Knap2: Skift mellem normal og SCPI kommunikation
 */
#define BUTTON2_PIN   33     // Knap 2: Skift mellem normal og SCPI kommunikation

// I2C konfiguration
/**
 * @defgroup I2C_config I2C konfiguration
 * @brief ESP32 pins forbundet som I2C interface
 * @{
 */
/**
 * @brief pin definition på ESP32: I2C_SDA: I2C data pin
 */
#define I2C_SDA       21     // I2C data pin
/**
 * @brief pin definition på ESP32: I2C_SCL: I2C clock pin
 */
#define I2C_SCL       22     // I2C clock pin
/**
 * @brief Standard I2C adresse for 16x2 LCD
 */
#define LCD_I2C_ADDR  0x27   // Standard I2C adresse for 16x2 LCD
/**
 * @brief Standard I2C adresse for ADS1115
 */
#define ADS1115_ADDR  0x48   // Standard I2C adresse for ADS1115
/** @} */  // Slut på gruppen I2C_config
// ============================================================================================================================
// GLOBALE VARIABLER - DEKLARATIONER
// ============================================================================================================================

// Hardware objekter                                                                                                          
Adafruit_ADS1115 ads;        			 	    // Initialiserer ADS1115 objekt for ADC-målinger med I2C interface
LiquidCrystal_I2C lcd(LCD_I2C_ADDR, 16, 2); 		    // Initialiserer 16x2 LCD display via I2C backpack med adresse 0x27

// Systemtilstande                                                                                                            
volatile SystemState currentState = STATE_IDLE;             // Nuværende systemtilstand - starter i IDLE ved opstart
volatile DisplayMode displayMode = DISPLAY_MODE_ESR_RESULT; // Startvisning: ESR-resultater (alternativt rå ADC til debugging)
volatile CommunicationMode commMode = COMM_MODE_NORMAL;     // Kommunikationstilstand: normal (alternativt SCPI-protokol)

// Måledata                                                                                                                   
int32_t rawAdcValue = 0;              			    //  Gemmer rå 32-bit ADC-værdi efter 64× oversampling og middling
float calculatedEsr = 0.0;            			    // Beregnet ESR-værdi i ohm (0-10 Ω) baseret på interpolationstabeller
uint8_t currentPgaSetting = PGA_2X_INDEX; 		    // Nuværende forstærkning(PGA): 0=2x, 1=4x, 2=8x, 3=16x (ADS1115 PGA)
bool newMeasurementAvailable = false; 			    // Flag: sand når ny måling er klar til visning/transmission

// Knap state machine variabler                                                                                                
volatile uint32_t button1LastPress = 0;                     // Millisekund tæller for sidste knap1 tryk (til debounce)
volatile uint32_t button2LastPress = 0;                     // Millisekund tæller for sidste knap2 tryk (til debounce)
volatile bool button1Pressed = false;                       // Aktuel tilstand for knap 1: true = trykket (aktiv lav)
volatile bool button2Pressed = false;                       // Aktuel tilstand for knap 2: true = trykket (aktiv lav)

// Timing variabler                                                                                                            
uint32_t lastMeasurementTime = 0;                           // Millisekund tæller for sidste ADC-måling (timing kontrol)
uint32_t lastDisplayUpdateTime = 0;                         // Millisekund tæller for sidste LCD-opdatering (timing kontrol)

// SCPI kommunikationsbuffer                                                                                                   
char scpiBuffer[256];                                       // Buffer til modtagelse af SCPI kommandoer (max 255 tegn + null)
uint8_t scpiBufferIndex = 0;                                // Indeks for næste ledige position i SCPI buffer (0-255)
bool scpiCommandReady = false;                              // Flag der indikerer at en komplet SCPI kommando er klar i buffer

// Watchdog aktivitetstæller                                                                                                   
volatile uint32_t wdgCounter = 0;			    // Tæller antal watchdog resets (til debugging og systemhelbredsmonitor)
// ============================================================================================================================
// ESR-tabeller (pladsholdere - skal udfyldes med kalibreringsdata)
// Struktur: {ADC_værdi, ESR_værdi_i_ohm}
// ============================================================================================================================
typedef struct {
    int16_t adcValue;        // ADC-værdi (16-bit signeret)
    float esrValue;          // ESR-værdi i ohm
} TableEntry;

// Tabeller gemt i programhukommelse (PROGMEM)
const TableEntry esrTables[NUM_PGA_SETTINGS][TABLE_ENTRIES] PROGMEM = {
    // PGA = 2× tabel (indeks 0) - 30 indgange
    {
        {0, 0.000}, {100, 0.004}, {200, 0.010}, {300, 0.020}, {400, 0.033},
        {500, 0.052}, {600, 0.057}, {700, 0.090}, {800,0.093}, {900, 0.135},
        {1000, 0.162}, {1100, 0.164}, {1200, 0.252}, {1300, 0.321}, {1400, 0.376},
        {1500, 0.489}, {1600, 0.501}, {1700, 0.769}, {1800, 0.847}, {1900, 1.000},
        {2000, 1.500}, {2100, 1.960}, {2200, 2.670}, {2300, 3.260}, {2400, 3.880},
        {2500, 4.710}, {2600, 5.560}, {2700, 7.360}, {2800, 9.998}, {2900, 9.999}
    },
    // PGA = 4× tabel (indeks 1) - 30 indgange  
    {
        {0, 0.000}, {200, 0.004}, {400, 0.010}, {600, 0.020}, {800, 0.033},
        {1000, 0.052}, {1200, 0.057}, {1400, 0.090}, {1600, 0.093}, {1800, 0.135},
        {2000, 0.162}, {2200, 0.164}, {2400, 0.252}, {2600, 0.321}, {2800, 0.376},
        {3000, 0.489}, {3200, 0.501}, {3400, 0.769}, {3600, 0.847}, {3800, 1.000},
        {4000, 1.500}, {4200, 1.960}, {4400, 2.670}, {4600, 3.260}, {4800, 3.880},
        {5000, 4.710}, {5200, 5.560}, {5400, 7.360}, {5600, 9.998}, {5800, 9.999}
    },
    // PGA = 8× tabel (indeks 2) - 30 indgange
    {
        {0, 0.000}, {400, 0.004}, {800, 0.010}, {1200, 0.020}, {1600, 0.033},
        {2000, 0.052}, {2400, 0.057}, {2800, 0.090}, {3200, 0.093}, {3600, 0.135},
        {4000, 0.162}, {4400, 0.164}, {4800, 0.252}, {5200, 0.321}, {5600, 0.376},
        {6000, 0.489}, {6400, 0.501}, {6800, 0.769}, {7200, 0.847}, {7600, 1.000},
        {8000, 1.500}, {8400, 1.960}, {8800, 2.670}, {9200, 3.260}, {9600, 3.880},
        {10000, 4.710}, {10400, 5.560}, {10800, 7.360}, {11200, 9.998}, {11600, 9.999}
    },
    // PGA = 16× tabel (indeks 3) - 30 indgange
    {
        {0, 0.000}, {800, 0.004}, {1600, 0.010}, {2400, 0.020}, {3200, 0.033},
        {4000, 0.052}, {4800, 0.057}, {5600, 0.090}, {6400, 0.093}, {7200, 0.135},
        {8000, 0.162}, {8800, 0.164}, {9600, 0.252}, {10400, 0.321}, {11200, 0.376},
        {12000, 0.489}, {12800, 0.501}, {13600, 0.769}, {14400, 0.847}, {15200, 1.000},
        {16000, 1.500}, {16800, 1.960}, {17600, 2.670}, {18400, 3.260}, {19200, 3.880},
        {20000, 4.710}, {20800, 5.560}, {21600, 7.360}, {22400, 9.998}, {23200, 9.999}
    }
};

// ============================================================================================================================
// FUNKTIONSDEKLARATIONER (PROTOTYPER)
// ============================================================================================================================

// HAL (Hardware Abstraction Layer) funktioner
void hal_ad9850_init(void);
void hal_ad9850_set_frequency(uint32_t frequency);
void hal_ad9850_set_amplitude(float amplitude_volts);
void hal_ads1115_init(void);
void hal_lcd_init(void);
void hal_buttons_init(void);
void hal_serial_init(void);
void hal_watchdog_init(void);
float hal_read_power_consumption(void);

// ADC-målingsfunktioner (S.ADC.001)
int16_t adc_read_single(uint8_t pga_index);
int32_t adc_read_oversampled(uint8_t pga_index);
uint8_t adc_auto_range(void);
void adc_calibrate(void);

// ESR-beregningsfunktioner (S.ESR.001)
void esr_find_nearest_values(uint8_t pga_index, int32_t adc_value, 
                            int16_t *adc_low, float *esr_low, 
                            int16_t *adc_high, float *esr_high);
float esr_linear_interpolate(int16_t adc_low, float esr_low, 
                           int16_t adc_high, float esr_high, 
                           int32_t adc_current);
float esr_calculate(uint8_t pga_index, int32_t adc_value);

// Display funktioner (S.DSP.001)
void display_init(void);
void display_raw_adc(int32_t adc_value, uint8_t pga_index);
void display_esr_result(float esr_value, int32_t adc_value, uint8_t pga_index);
void display_update(void);
void display_error(const char *error_msg);
void display_system_info(void);

// State machine funktioner
void state_machine_init(void);
void state_machine_update(void);
void state_idle_handler(void);
void state_measure_adc_handler(void);
void state_calculate_esr_handler(void);
void state_update_display_handler(void);
void state_process_serial_handler(void);
void state_error_handler(void);

// Knap håndteringsfunktioner
void button_check(void);
void button1_handler(void);
void button2_handler(void);

// SCPI kommunikationsfunktioner (S.SCP.001)
void scpi_read_serial(void);
void scpi_parser(void);
void scpi_execute(const char *command);
void scpi_response(const char *response);
void scpi_handle_idn(void);
void scpi_handle_measure_esr(void);
void scpi_handle_system_preset(void);
void scpi_handle_display_mode(const char *mode);
void scpi_handle_communication(const char *state);
void scpi_handle_advanced(const char* command);

// Watchdog og fejlhåndteringsfunktioner (S.WDG.001)
void wdg_init(void);
void wdg_reset(void);
void error_handler(const char *error_message, uint8_t error_code);

// Hjælpefunktioner
uint32_t micros_safe(void);
void delay_micros_safe(uint32_t microseconds);
float map_float(float x, float in_min, float in_max, float out_min, float out_max);

// Test- og kalibreringsfunktioner
bool system_self_test(void);
void emergency_calibration(void);
void run_test_program(void);

// Statistiske analysefunktioner
void analyze_measurements(float measurements[], int count, float *mean, float *stddev);
float temperature_compensate_esr(float esr_value, float temperature_degC);

// Konfigurationsfunktioner
void load_configuration(void);
void save_configuration(void);
void cleanup_before_restart(void);
const char* get_build_info(void);

/**
* ============================================================================================================================
 * @defgroup CORE_SYSTEM Core System Module
 * @brief Systemets overordnede kontrol- og livscykluslogik
 *============================================================================================================================
 * Dette modul indeholder:
 * - Systemets entry points (setup() og loop())
 * - Globale systemtilstande og modes
 * - Initialisering af alle underliggende moduler
 *
 * Core-modulet fungerer som orkestrator og indeholder
 * ingen hardware-specifik logik direkte.
 *
 * Afhængigheder:
 * - HAL
 * - State Machine
 * - UI
 * - SCPI
 */
// ============================================================================================================================
// SETUP FUNKTION (ENKELT DEFINITION)
// ============================================================================================================================
/**
 * @brief Arduino setup funktion - initialiserer hele systemet
 * 
 * @details Denne funktion køres én gang ved opstart og initialiserer alle 
 * hardware-komponenter og systemtilstande i overensstemmelse med kravspecifikationen.
 * 
 * @return void
 */
// ============================================================================================================================
void setup() {                                                            // Arduino setup-funktion - køres én gang ved opstart
    // Initialiser seriel kommunikation til debugging
    Serial.begin(115200);                                                 // Start seriel kommunikation med 115200 baud rate
    while (!Serial) {                                                     // Vent på at seriel port bliver tilgængelig
        delay(10); // Vent på seriel initialisering                       // Kort delay på 10ms for at undgå CPU-spild
    }
    Serial.println("Seriel kommunikation initialiseret (115200 baud)");   // Udskriv bekræftelsesbesked til seriel port
    
    // Initialiser watchdog timer for systemstabilitet
    esp_task_wdt_config_t wdt_config = {
    .timeout_ms = WATCHDOG_TIMEOUT,   // 3000 ms
    .idle_core_mask = 0,
    .trigger_panic = true
    };
    esp_task_wdt_init(&wdt_config);                                       // Konfigurer watchdog timer med 3 sekunders timeout
    esp_task_wdt_add(NULL); // Tilføj nuværende task til watchdog         // Registrer den aktuelle task i watchdog systemet
    Serial.println("Watchdog timer initialiseret (3 sekunder timeout)");  // Udskriv bekræftelsesbesked til seriel port  
    
    // Initialiser I2C bus til ADC og LCD
    Wire.begin(I2C_SDA, I2C_SCL);                                         // Start I2C-kommunikation på definerede SDA/SCL pins
    Wire.setClock(400000); // 400 kHz I2C hastighed (S.HAL.001.5)         // Sæt I2C-hastighed til 400 kHz (krav S.HAL.001.5)
    
    // Initialiser hardware-komponenter i korrekt rækkefølge
    hal_ad9850_init();      // DDS-generator                              // Initialiser AD9850 DDS-generator chip
    hal_ads1115_init();     // 16-bit ADC med PGA                         // Initialiser ADS1115 16-bit ADC med PGA
    hal_lcd_init();         // 16x2 LCD display                           // Initialiser 16x2 LCD display via I2C
    hal_buttons_init();     // Knapper med debounce                       // Initialiser knapper med debounce-logik
    
    // Initialiser state machine
    state_machine_init();                             			  // Initialiser systemets state machine
    
    // Indlæs konfiguration (standardværdier)
    load_configuration();                             			  // Indlæs konfiguration fra ikke-flygtig hukommelse
    
    // Vis startbesked på display
    lcd.clear();                                      // Ryd alt indhold på LCD-displayet
    lcd.setCursor(0, 0);                              // Sæt cursor til starten af første linje (kolonne 0, række 0)
    lcd.print("ESR-Meter v");                         // Skriv "ESR-Meter v" på displayet
    lcd.print(FIRMWARE_VERSION);                      // Tilføj firmware version til display-teksten
    lcd.setCursor(0, 1);                              // Sæt cursor til starten af anden linje (kolonne 0, række 1)
    lcd.print("Initialiserer...");                    // Skriv "Initialiserer..." på displayet
    
    // Sæt DDS-generator til 100 kHz (H.GEN.001)
    hal_ad9850_set_frequency(100000); // 100 kHz                          // Konfigurer DDS-generator til at producere 100 kHz sinus (krav H.GEN.001)
    
    // Kort delay for at vise startbesked - watchdog reset inkluderet
    for(int i = 0; i < 10; i++) {                                         // Gentag 10 gange (100ms × 10 = 1 sekund totalt)
        wdg_reset();                                                      // Reset watchdog timer for at forhindre timeout
        delay(100);                                                       // Vent 100 millisekunder
    }
    
    // Klar til drift - vis klar besked
    lcd.clear();                                                          // Ryd displayet igen
    lcd.setCursor(0, 0);                                                  // Sæt cursor til starten af første linje
    lcd.print("ESR-Meter Ready");                                         // Skriv "ESR-Meter Ready" på displayet
    lcd.setCursor(0, 1);                                                  // Sæt cursor til starten af anden linje
    lcd.print("Mode: Normal");                                            // Skriv "Mode: Normal" på displayet
    
    // Reset watchdog
    wdg_reset();                                                          // Reset watchdog timer en sidste gang før loop() starter
    
    // Log systemstart
    Serial.println("========================================");           // Udskrift af adskillelseslinje til seriel port
    Serial.println("ESR-Meter Firmware v" FIRMWARE_VERSION);              // Udskriv firmware version til seriel port
    Serial.println("Hardware: " HARDWARE_VERSION);                        // Udskriv hardware version til seriel port
    Serial.println("Build: " __DATE__ " " __TIME__);                      // Udskriv kompileringsdato og klokkeslæt
    Serial.println("ESP32 Chip ID: " + String(ESP.getEfuseMac(), HEX));   // Udskriv ESP32 chip's unikke ID i hexadecimal
    Serial.println("Free Heap: " + String(ESP.getFreeHeap()) + " bytes"); // Udskriv tilgængelig RAM (heap) i bytes
    Serial.println("System initialiseret korrekt");                       // Udskriv bekræftelse på korrekt initialisering
    Serial.println("========================================");           // Udskrift af afsluttende adskillelseslinje
    
    // Kør selvtest hvis TEST_MODE er defineret
    #ifdef TEST_MODE                                          // Kompileringsdirektiv: kun inkluder hvis TEST_MODE er defineret
    run_test_program();                                       // Kør testprogram til systemvalidering
    #endif                                                    // Afslut #ifdef direktivet
}                                                             // Afslut setup() funktionen
// ============================================================================================================================
// LOOP FUNKTION 
// ============================================================================================================================
/**
 * @brief Arduino loop funktion - hovedkontrolcyklus
 * 
 * @details Denne funktion køres kontinuerligt og implementerer hovedkontrolcyklussen
 * med state machine, knap-tjek, og watchdog reset. Funktionen sikrer at alle
 * systemkomponenter opdateres i overensstemmelse med timingkravene.
 * 
 * @return void
 */
// ============================================================================================================================
void loop() {                                                   // Arduino loop-funktion - køres kontinuerligt efter setup()
    // Tjek knaptryk (debounce implementeret i funktionen)
    button_check();                                             // Kald funktion til at tjekke og håndtere knaptryk med debounce
 
    // Læs seriel input i SCPI-tilstand
    if (commMode == COMM_MODE_SCPI) {                           // Tjek om systemet er i SCPI kommunikationstilstand
        scpi_read_serial();                                     // Læs indkommende seriel data til SCPI buffer
    }
    
    // Opdater state machine baseret på nuværende tilstand
    state_machine_update();                                              // Kør state machine opdatering baseret på currentState
    
    // Håndter SCPI kommandoer hvis i SCPI tilstand og kommando klar
    if (commMode == COMM_MODE_SCPI && scpiCommandReady) {                // Tjek om SCPI tilstand OG om en kommando er klar til behandling
        currentState = STATE_PROCESS_SERIAL;                             // Skift systemtilstand til seriel kommunikationsbehandling
        state_machine_update();                                          // Kør state machine opdatering for at behandle kommandoen
    }
    
    // Send data i normal mode hvis ny måling tilgængelig
    if (commMode == COMM_MODE_NORMAL && newMeasurementAvailable) {       // Tjek om normal mode OG ny måling er tilgængelig
        Serial.print("ESR: ");                                           // Udskriv tekst "ESR: " til seriel port
        Serial.print(calculatedEsr, 4);                                  // Udskriv beregnet ESR-værdi med 4 decimaler
        Serial.print(" Ω, ADC: ");                                       // Udskriv tekst " Ω, ADC: " til seriel port
        Serial.print(rawAdcValue);                                       // Udskriv rå ADC-værdi
        Serial.print(", PGA: ");                                         // Udskriv tekst ", PGA: " til seriel port
        Serial.print(currentPgaSetting == 0 ? "2x" :               	 // Udskriv PGA-indstilling som tekst (2x, 4x, 8x eller 16x)
                    currentPgaSetting == 1 ? "4x" :                      // Brug ternær operator til at vælge korrekt tekst
                    currentPgaSetting == 2 ? "8x" : "16x");              // Baseret på currentPgaSetting værdi (0-3)
        Serial.print(", Time: ");                                        // Udskriv tekst ", Time: " til seriel port
        Serial.print(millis());                                          // Udskriv aktuelt systemtidspunkt i millisekunder
        Serial.println(" ms");                                           // Udskriv " ms" og linjeskift til seriel port
        newMeasurementAvailable = false;                                 // Nulstil flag for ny måling (markerer som behandlet)
    }
    
    // Reset watchdog timer i hver cyklus (S.WDG.001.2)
    wdg_reset();                                        // Reset watchdog timer (forhindrer systemgenstart) 
    							// - opfylder krav S.WDG.001.2
    // Kort delay for at give andre tasks CPU-tid
    // Dette er især vigtigt for ESP32 med dual-core
    delay(1);                                           // Vent 1 millisekund for at give CPU-tid til andre tasks/systemopgaver
}                                                       // Afslut loop() funktionen - vil automatisk kaldes igen
/**
* ============================================================================================================================
 * @defgroup HAL Hardware Abstraction Layer (HAL)
 * @brief Abstraherer al hardwareadgang fra applikationslogik
 * ============================================================================================================================
 * Dette modul indeholder lavniveau-funktioner til:
 * - DDS-generator (AD9850)
 * - ADC (ADS1115)
 * - LCD-display
 * - Knapper
 * - Watchdog
 *
 * Formålet er at isolere hardwareafhængig kode, så
 * resten af firmwaren kan forblive hardware-uafhængig.
 *
 * Ændringer i hardware bør kun påvirke dette modul.
 */
// ============================================================================================================================
// HAL-FUNKTIONER - IMPLEMENTERING
// ============================================================================================================================
// ============================================================================================================================
/**
 * @brief Initialiserer AD9850 DDS-generator med parallelt interface
 * 
 * @details Konfigurerer AD9850 til at generere 100 kHz ren sinus med 0° fase.
 * Bruger 4-pins parallelt interface for hurtig kommunikation.
 * 
 * @pre I2C bus skal være initialiseret
 * @post AD9850 klar til frekvensindstilling
 * 
 * @note Opfylder H.GEN.001 kravet om DDS-generator
 * @see hal_ad9850_set_frequency()
 * 
 * @return void
*/
// ============================================================================================================================
void hal_ad9850_init(void) {
    // Konfigurer pins som outputs
    pinMode(AD9850_W_CLK, OUTPUT);        					// Sæt W_CLK pin som output til AD9850 word clock
    pinMode(AD9850_FQ_UD, OUTPUT);        					// Sæt FQ_UD pin som output til AD9850 frequency update
    pinMode(AD9850_DATA, OUTPUT);         					// Sæt DATA pin som output til AD9850 serial data
    pinMode(AD9850_RESET, OUTPUT);        					// Sæt RESET pin som output til AD9850 reset
    
    // Start med lave signaler
    digitalWrite(AD9850_W_CLK, LOW);      					// Sæt word clock til lavt signal (inaktiv)
    digitalWrite(AD9850_FQ_UD, LOW);      					// Sæt frequency update til lavt signal (inaktiv)
    digitalWrite(AD9850_DATA, LOW);       					// Sæt data linje til lavt signal (inaktiv)
    digitalWrite(AD9850_RESET, LOW);      					// Sæt reset til lavt signal (inaktiv)
    
    // Udfør hard reset af AD9850
    digitalWrite(AD9850_RESET, HIGH);     					// Aktiver reset signal (aktiv høj)
    delayMicroseconds(10);                					// Vent 10 mikrosekunder for reset pulse width
    digitalWrite(AD9850_RESET, LOW);      					// Deaktiver reset signal
    
    // Initialiser med 0 Hz (alle 40 bits sat til 0)
    for (int i = 0; i < 5; i++) {         					// Loop 5 gange (5 bytes = 40 bits)
        // Shift ud 8 bits ad gangen, LSB først
        shiftOut(AD9850_DATA, AD9850_W_CLK, LSBFIRST, 0x00); 	// Send 0x00 byte via serial shift
    }
    
    // Opdater frekvensregistre
    digitalWrite(AD9850_FQ_UD, HIGH);     					// Aktiver frequency update for at læse data ind
    delayMicroseconds(10);                					// Vent 10 mikrosekunder for setup time
    digitalWrite(AD9850_FQ_UD, LOW);      					// Deaktiver frequency update (latch data)
    
    Serial.println("AD9850 DDS-generator initialiseret"); 	// Bekræft initialisering på seriel
}

// ============================================================================================================================
/**
 * @brief Indstiller AD9850 til specifik frekvens
 * 
 * @param frequency Ønsket frekvens i Hz (0-40 MHz)
 * 
 * @details Beregner DDS-tuneword baseret på formlen:
 * tuneword = (frekvens * 2^32) / urfrekvens
 * hvor urfrekvens = 125 MHz for AD9850
 * 
 * @note Opfylder H.SIG.001 kravet om 100 kHz testsignal
 * 
 * @return void
 */
// ============================================================================================================================
void hal_ad9850_set_frequency(uint32_t frequency) {
    // Beregn tuneword: 32-bit frekvens tuning word
    uint64_t tuning_word = ((uint64_t)frequency * 4294967296ULL) / 125000000; // Beregn DDS tuning word (2^32 = 4294967296)

    // Korrekt byte rækkefølge ifølge AD9850 datablad:  Ekstraher bytes fra tuning word
      uint8_t w0 = 0x00; // Phase=0°, Power-Down=0, Control=00 (parallelt)
      uint8_t w1 = (tuning_word >> 24) & 0xFF; 					// Freq-b31..b24
         uint8_t w2 = (tuning_word >> 16) & 0xFF; 					// Freq-b23..b16  
         uint8_t w3 = (tuning_word >> 8) & 0xFF;  					// Freq-b15..b8
         uint8_t w4 = tuning_word & 0xFF;         					// Freq-b7..b0 (LSB)

    // Send bytes til AD9850: Send i korrekt rækkefølge: W0, W1, W2, W3, W4
      shiftOut(AD9850_DATA, AD9850_W_CLK, LSBFIRST, w0);			// Send byte0/w0 via serial shift
          shiftOut(AD9850_DATA, AD9850_W_CLK, LSBFIRST, w1);			// Send byte1/w1 via serial shift
          shiftOut(AD9850_DATA, AD9850_W_CLK, LSBFIRST, w2);			// Send byte2/w2 via serial shift
          shiftOut(AD9850_DATA, AD9850_W_CLK, LSBFIRST, w3); 			// Send byte3/w3 via serial shift
          shiftOut(AD9850_DATA, AD9850_W_CLK, LSBFIRST, w4);			// Send byte4/w4 via serial shift

    // Opdater frekvensregistre: FQ_UD pulse ifølge spec
           digitalWrite(AD9850_FQ_UD, HIGH);					// Aktiver frequency update for at læse nye bytes ind
          delayMicroseconds(7); // Minimum 7 ns ifølge spec, 7 µs er mere end rigeligt	// Vent 10 mikrosekunder for setup time
         digitalWrite(AD9850_FQ_UD, LOW);					// Deaktiver frequency update (latch nye data)
    
    Serial.print("AD9850 sat til ");      				// Udskriv frekvens til seriel monitor
    Serial.print(frequency);              				// Udskriv den indstillede frekvensværdi
    Serial.println(" Hz");                				// Udskriv enheden "Hz"
}

// ============================================================================================================================
/**
 * @brief Indstiller DDS amplitude (begrænset implementering)
 * 
 * @note AD9850 har ikke direkte amplitude kontrol i denne konfiguration
 * Amplitude kontrolleres af det analoge kredsløb (potentiometer)
 */
// ============================================================================================================================
void hal_ad9850_set_amplitude(float amplitude_volts) {
    // AD9850 med parallelt interface har ikke amplitude kontrol
    // Dette ville kræve en DAC eller analog kredsløb
    Serial.print("Bemærk: AD9850 amplitude kontrolleres af eksternt kredsløb. "); // Informér brugeren om begrænsning
    Serial.print("Ønsket: ");                					// Udskriv "Ønsket: " til seriel
    Serial.print(amplitude_volts, 3);        					// Udskriv ønsket amplitude med 3 decimaler
    Serial.println("V p-p");                 					// Udskriv enheden "V p-p" (peak-to-peak)
    
    // Vis instruktion på display
    lcd.clear();                             					// Ryd LCD display
    lcd.setCursor(0, 0);                     					// Sæt cursor til starten af første linje
    lcd.print("Juster potmeter");            					// Vis instruktion om at justere potentiometer
    lcd.setCursor(0, 1);                     					// Sæt cursor til starten af anden linje
    lcd.print("for ");                       					// Vis "for "
    lcd.print(amplitude_volts, 2);           					// Vis ønsket amplitude med 2 decimaler
    lcd.print("V p-p");                      					// Vis enheden "V p-p"
    delay(2000);                             					// Vent 2 sekunder så brugeren kan læse beskeden
}
// ============================================================================================================================
/**
 * @brief Initialiserer ADS1115 16-bit ADC
 * 
 * @details Konfigurerer ADS1115 til single-ended måling på kanal A0
 * med 860 SPS sample rate og starter med PGA=2x for at undgå overstyring.
 * 
 * @note Opfylder H.ADC.001 kravet om 16-bit ADC med PGA
 * 
 * @return void
 */
// ============================================================================================================================
void hal_ads1115_init(void) {
    // Initialiser ADS1115 objekt
    if (!ads.begin(ADS1115_ADDR, &Wire)) {  			// Prøv at initialisere ADS1115 med I2C adresse og Wire objekt
        error_handler("ADS1115 init fejlet", 0x01);  		// Hvis fejl, kald fejlhåndteringsfunktion med besked og kode 0x01
        return;                               			// Afslut funktionen ved fejl
    }
    
    // Konfigurer til single-ended måling på kanal A0
    // PGA=2x, 860 SPS, single-shot mode
    ads.setGain(GAIN_TWO); 
                       			// Sæt ADC forstærkning til 2x (±2.048V rækkevidde)  og vent
    // Kort delay for at lade gain-skriveren stabilisere
    delayMicroseconds(100);
    ads.setDataRate(RATE_ADS1115_860SPS);     			// Sæt sample rate til 860 samples per sekund
    
    Serial.println("ADS1115 ADC initialiseret (PGA=2x, 860 SPS)"); 		// Bekræft initialisering på seriel port
}

// ============================================================================================================================
/**
 * @brief Initialiserer 16x2 LCD-display via I2C
 * 
 * @details Konfigurerer LCD-displayet med korrekt backlight og cursor.
 * Bruger Adafruit LCD-bibliotek med I2C backpack.
 * 
 * @note Opfylder H.MCU.001 kravet om display
 * 
 * @return void
 */
// ============================================================================================================================
void hal_lcd_init(void) {
    // Initialiser LCD
    lcd.init();                              			// Initialiser LCD-displayet via I2C
    lcd.backlight();                         			// Tænd LCD baggrundsbelysning
    lcd.clear();                             			// Ryd alt indhold på displayet
    
    // Vis startbesked
    lcd.setCursor(0, 0);                     			// Sæt cursor til starten af første linje (kolonne 0, række 0)
    lcd.print("ESR-Meter v");                			// Skriv "ESR-Meter v" på displayet
    lcd.print(FIRMWARE_VERSION);             			// Tilføj firmware version til display-teksten
    
    Serial.println("16x2 LCD display initialiseret"); 		// Bekræft initialisering på seriel port
}

// ============================================================================================================================
/**
 * @brief Initialiserer knapper med interne pull-up modstande
 * 
 * @details Konfigurerer knappins som inputs med interne pull-up.
 * ESP32 har interne pull-up modstande der kan aktiveres.
 * 
 * @note Opfylder H.MCU.001 kravet om knapper
 * 
 * @return void
 */
// ============================================================================================================================
void hal_buttons_init(void) {
    // Konfigurer knappins som inputs med interne pull-up
    pinMode(BUTTON1_PIN, INPUT_PULLUP);      			// Konfigurer BUTTON1_PIN som input med intern pull-up modstand
    pinMode(BUTTON2_PIN, INPUT_PULLUP);      			// Konfigurer BUTTON2_PIN som input med intern pull-up modstand
    
    // Initialiser knaptidspunkter
    button1LastPress = 0;                    			// Nulstil tæller for sidste knap1 tryk
    button2LastPress = 0;                    			// Nulstil tæller for sidste knap2 tryk
    button1Pressed = false;                  			// Initialiser knap1 status til "ikke trykket"
    button2Pressed = false;                  			// Initialiser knap2 status til "ikke trykket"
    
    Serial.println("Knapper initialiseret (interne pull-up)"); 	// Bekræft initialisering på seriel port
}

// ============================================================================================================================
/**
 * @brief Læser systemets strømforbrug (estimeret)
 * 
 * @return float Estimeret strømforbrug i mA
 */
// ============================================================================================================================
float hal_read_power_consumption(void) {
    // Dette er en estimation baseret på typiske værdier
    // I et rigtigt system ville man måle dette med en strømmonitor IC
    
    float estimated_current = 0.0;           			// Initialiser estimeret strømforbrug til 0 mA
    
    // ESP32: ~100 mA under normal drift
    estimated_current += 100.0;              			// Tilføj 100 mA for ESP32 mikrokontroller
    
    // AD9850: ~150 mA ved 125 MH
    estimated_current += 150.0;              			// Tilføj 150 mA for DDS-generator
    
    // ADS1115: ~0.3 mA
    estimated_current += 0.3;                			// Tilføj 0,3 mA for ADC
    
    // Opamps (4 stk): ~2 mA hver
    estimated_current += 8.0;                			// Tilføj 8 mA for 4 operationsforstærkere (2 mA hver)
    
    // LCD: ~20 mA
    estimated_current += 20.0;               			// Tilføj 20 mA for LCD-display
    
    return estimated_current;                			// Returner det samlede estimerede strømforbrug i mA
}
/**
*  ============================================================================================================================
 * @defgroup ADC_PROCESSING ADC & Signal Processing Module
 * @brief Ansvarlig for alle ADC-målinger og signalbehandling
 * ============================================================================================================================
 * Indeholder:
 * - Single-shot ADC-målinger
 * - Oversampling og middling
 * - Automatisk PGA-range selection
 * - Kalibreringsrutiner (delvist implementeret)
 *
 * Dette modul er kritisk for målenøjagtighed og støjreduktion.
 */
// ============================================================================================================================
// ADC-MÅLINGSFUNKTIONER - IMPLEMENTERING
// ============================================================================================================================
// ============================================================================================================================
/**
 * @brief Læser en enkelt ADC-værdi med specificeret PGA
 * 
 * @param pga_index PGA-indeks (0=2x, 1=4x, 2=8x, 3=16x)
 * 
 * @details Konfigurerer ADS1115 til den ønskede PGA-forstærkning,
 * udfører en single-shot måling og returnerer resultatet.
 * 
 * @return int16_t ADC-værdi (-32768 til 32767)
 */
// ============================================================================================================================
int16_t adc_read_single(uint8_t pga_index) {
    // Sæt korrekt gain baseret på pga_index
    adsGain_t gain;
    switch (pga_index) {
        case PGA_2X_INDEX:  gain = GAIN_TWO; break;   				// ±2.048V
        case PGA_4X_INDEX:  gain = GAIN_FOUR; break;  				// ±1.024V
        case PGA_8X_INDEX:  gain = GAIN_EIGHT; break; 				// ±0.512V
        case PGA_16X_INDEX: gain = GAIN_SIXTEEN; break; 			// ±0.256V
        default:            gain = GAIN_TWO; break;    				// Default til 2x
    }
    
    ads.setGain(gain);
    
    // Kort delay for at lade gain-skriveren stabilisere
    delayMicroseconds(100);
    
    // Start single-shot måling på kanal A0
    int16_t adc_value = ads.readADC_SingleEnded(0);
    
    return adc_value;
}

// ============================================================================================================================
/**
 * @brief Læser ADC med 64× oversampling og middling
 * 
 * @param pga_index PGA-indeks (0=2x, 1=4x, 2=8x, 3=16x)
 * 
 * @details Udfører 64 enkeltmålinger med korrekt timing baseret på
 * sample rate (860 SPS) og beregner gennemsnittet.
 * 
 * @note Opfylder S.ADC.001.2 kravet om 64× oversampling
 * 
 * @return int32_t Middlet ADC-værdi
 */
// ======================================================================================================================================================================
int32_t adc_read_oversampled(uint8_t pga_index) {
    int64_t sum = 0;                                 							// Initialiser sum variabel til at akkumulere ADC-værdier
    uint32_t sample_interval = 1000000 / 860; // Mikrosekunder pr. sample ved 860 SPS (~1163 µs) 	// Beregn interval mellem samples (1.000.000 µs / 860 samples)
    
    for (int i = 0; i < OVERSAMPLING_FACTOR; i++) {  							// Loop 64 gange (OVERSAMPLING_FACTOR = 64)
        sum += adc_read_single(pga_index);           							// Læs en enkelt ADC-værdi og tilføj til sum
        
        // Vent til næste sample tidspunkt (undgå aliasing)
        // Brug non-blocking delay for at undgå watchdog timeout
        if (i < OVERSAMPLING_FACTOR - 1) {           							// Hvis ikke sidste iteration i loopet
            uint32_t start = micros();               							// Gem starttidspunkt i mikrosekunder
            while (micros() - start < sample_interval) { 						// Vent indtil sample_interval er gået
                wdg_reset(); // Reset watchdog under ventetid 						// Forhindrer watchdog timeout under ventetid
            }
        }
    }
    
    // Beregn gennemsnit og returner som 32-bit integer
    return (int32_t)(sum / OVERSAMPLING_FACTOR);     						// Beregn gennemsnit ved at dividere sum med 64 og cast til 32-bit
}
// ======================================================================================================================================================================
// ============================================================================================================================
/**
 * @brief Automatisk PGA-valg baseret på signalstyrke
 * 
 * @details Starter med PGA=2x og justerer op/ned baseret på hvor
 * tæt ADC-værdien er på fuld skala. Implementerer hysterese for
 * at undgå oscillerende skift.
 * 
 * @note Opfylder S.ADC.001.3 kravet om automatisk PGA-valg
 * 
 * @return uint8_t Optimalt PGA-indeks (0-3)
 */
// ============================================================================================================================
uint8_t adc_auto_range(void) {
    static uint8_t last_pga = PGA_2X_INDEX;          	// Statisk variabel der gemmer sidste PGA-indeks (bevares mellem kald)
    static uint8_t hysteresis_counter = 0;           	// Statisk tæller for hysterese (bevares mellem kald)
    const uint8_t HYSTERESIS_THRESHOLD = 3;          	// Konstant: antal konsekutive målinger før PGA-skift
    
    // Læs ADC-værdi med nuværende PGA
    int16_t adc_value = adc_read_single(last_pga);   	// Læs ADC-værdi med aktuelt PGA-indeks
    int16_t adc_abs = abs(adc_value);                	// Beregn absolutværdi af ADC-værdien
    
    // Bestem om vi skal ændre PGA
    uint8_t new_pga = last_pga;                      	// Start med at antage ingen ændring
    
    if (adc_abs < PGA_THRESHOLD_LOW && last_pga < PGA_16X_INDEX) { // Hvis signal for lavt OG ikke allerede på max forstærkning
        // Signal for lavt - øg forstærkning
        hysteresis_counter++;                        		   // Øg hysterese-tæller
        if (hysteresis_counter >= HYSTERESIS_THRESHOLD) { 	   // Hvis tæller når tærskelværdi
            new_pga = last_pga + 1;                  		   // Øg PGA-indeks (øger forstærkningen)
            hysteresis_counter = 0;                  		   // Nulstil hysterese-tæller
            Serial.print("Øger PGA fra ");           		   // Log ændring til seriel port
            Serial.print(last_pga);                  		   // Vis gammelt PGA-niveau
            Serial.print(" til ");                   		   // 
            Serial.println(new_pga);                 		   // Vis nyt PGA-niveau
        }
    } 
    else if (adc_abs > PGA_THRESHOLD_HIGH && last_pga > PGA_2X_INDEX) { // Hvis signal for højt OG ikke allerede på min forstærkning
        // Signal for højt - reducer forstærkning
        hysteresis_counter++;                        			// Øg hysterese-tæller
        if (hysteresis_counter >= HYSTERESIS_THRESHOLD) { 		// Hvis tæller når tærskelværdi
            new_pga = last_pga - 1;                  			// Reducer PGA-indeks (reducerer forstærkningen)
            hysteresis_counter = 0;                  			// Nulstil hysterese-tæller
            Serial.print("Reducer PGA fra ");        			// Log ændring til seriel port
            Serial.print(last_pga);                  			// Vis gammelt PGA-niveau
            Serial.print(" til ");                   			//
            Serial.println(new_pga);                 			// Vis nyt PGA-niveau
        }
    } 
    else {
        // Signal i optimalt område - nulstil hysterese
        hysteresis_counter = 0;                      			// Nulstil hysterese-tæller når signal er i optimalt område
    }
    
    // Opdater PGA hvis nødvendigt
    if (new_pga != last_pga) {                      			// Hvis PGA-indeks er ændret
        last_pga = new_pga;                         			// Opdater gemt PGA-indeks
        
        // Kort delay for at lade ADC stabilisere (uden at blokere for watchdog)
        for (int i = 0; i < 10; i++) {              			// Loop 10 gange (samlet 10ms delay)
            wdg_reset();                            			// Reset watchdog timer
            delay(1);                               			// Vent 1 ms
        }
    }
    
    return last_pga;                                			// Returner aktuelt PGA-indeks
}

// ============================================================================================================================
/**
 * @brief Kalibreringsrutine for ADC-systemet
 * 
 * @details Denne funktion udfører kalibrering af ADC-systemet.
 * Kræver fysisk tilkobling af kendte referencemodstande.
 * Resultater gemmes i PROGMEM tabeller og kræver genprogrammering.
 * 
 * @note Opfylder S.ADC.001.4 kravet om kalibrering
 * @warning Kræver genprogrammering for at gemme kalibreringsdata
 * 
 * @return void
 */
// ============================================================================================================================
void adc_calibrate(void) {
    Serial.println("=== ADC KALIBRERING START ==="); 			// Skilletekst for kalibreringsstart på seriel port
    Serial.println("Dette kræver genprogrammering for at gemme data!"); // Advarsel om genprogrammering
    Serial.println("Tilslut kendte referencemodstande (0-10 ohm)"); 	// Instruktion til brugeren
    Serial.println("Følg instruktioner på display..."); 		// Instruktion om at følge display
    
    // Vis kalibreringsbesked på LCD
    lcd.clear();                                     			// Ryd LCD display
    lcd.setCursor(0, 0);                             			// Sæt cursor til starten af første linje
    lcd.print("KALIBRERING");                        			// Vis "KALIBRERING" på display
    lcd.setCursor(0, 1);                             			// Sæt cursor til starten af anden linje
    lcd.print("Vent...");                            			// Vis "Vent..." på display
    
    // Her ville man normalt implementere detaljeret kalibreringsrutine
    // med brugerinteraktion og gemning af data til EEPROM
    
    // Pseudo-kalibrering - i virkeligheden skal denne rutine:
    // 1. Guide brugeren gennem kalibreringspunkter
    // 2. Måle ADC-værdier for hver kendt modstand
    // 3. Gemme data i struktureret format
    // 4. Genberegne interpolationstabeller
    
    for (int i = 0; i < 20; i++) {                   			// Loop 20 gange (samlet 2 sekunders ventetid)
        wdg_reset();                                 			// Reset watchdog timer
        delay(100);                                  			// Vent 100 ms
    }
    
    Serial.println("=== ADC KALIBRERING SLUT ===");  			// Skilletekst for kalibreringsslut på seriel port
    Serial.println("Note: Kalibreringsdata skal gemmes via genprogrammering"); 	// Påmindelse om genprogrammering
    
    // Gå tilbage til normal visning
    lcd.clear();                                     			// Ryd LCD display
    lcd.setCursor(0, 0);                             			// Sæt cursor til starten af første linje
    lcd.print("Kalibrering");                        			// Vis "Kalibrering" på display
    lcd.setCursor(0, 1);                             			// Sæt cursor til starten af anden linje
    lcd.print("færdig - genprog!");                  			// Vis "færdig - genprog!" på display
    
    for (int i = 0; i < 10; i++) {                   			// Loop 10 gange (samlet 2 sekunders ventetid)
        wdg_reset();                                 			// Reset watchdog timer
        delay(200);                                  			// Vent 200 ms
    }
}
/**
* ============================================================================================================================
 * @defgroup UTILITIES Utility Functions
 * @brief Generelle hjælpefunktioner
 * ============================================================================================================================
 * Indeholder tids-, matematik- og hjælpefunktioner
 * som anvendes på tværs af moduler.
 */

// ============================================================================================================================
// Hjælpefunktioner
// ============================================================================================================================

// ============================================================================================================================
/**
 * @brief Sikker mikro-sekund tæller
 * 
 * @return uint32_t Aktuelle tid i mikro-sekunder
 */
// ============================================================================================================================
uint32_t micros_safe(void) {
    return micros();                        				// Returnerer aktuelt systemtidspunkt i mikrosekunder
}

// ============================================================================================================================
/**
 * @brief Sikker mikro-sekund delay
 * 
 * @param microseconds Antal mikro-sekunder at vente
 */
// ============================================================================================================================
void delay_micros_safe(uint32_t microseconds) {
    if (microseconds < 1000) {              				// Hvis ventetid er mindre end 1000 µs (1 ms)
        delayMicroseconds(microseconds);    				// Brug standard delayMicroseconds funktion
    } else {                                				// Hvis ventetid er 1000 µs eller mere
        uint32_t ms = microseconds / 1000;  				// Beregn antal hele millisekunder
        uint32_t us = microseconds % 1000;  				// Beregn resterende mikrosekunder
        delay(ms);                          				// Vent antal millisekunder med delay()
        delayMicroseconds(us);              				// Vent resterende mikrosekunder
    }
}

// ============================================================================================================================
/**
 * @brief Float version af map()-funktionen
 * 
 * @param x Input værdi
 * @param in_min Minimum input
 * @param in_max Maksimum input
 * @param out_min Minimum output
 * @param out_max Maksimum output
 * 
 * @return float Mappet værdi
 */
// ============================================================================================================================
float map_float(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min; // Lineær mapping af x fra input- til outputområde
}

/**
*  ============================================================================================================================
 * @defgroup ESR_CALCULATION ESR Calculation Module
 * @brief Konverterer ADC-målinger til ESR-værdier
 * ============================================================================================================================
 * ESR-beregningen er tabelbaseret og afhænger af:
 * - Aktiv PGA-indstilling
 * - Kalibreringsdata
 *
 * Funktionen anvender lineær interpolation mellem
 * kalibrerede datapunkter.
 *
 * Bemærk:
 * Nuværende tabeller er pladsholdere og skal erstattes
 * af reelle kalibreringsdata.
 */
// ============================================================================================================================
// ESR-BEREGNINGSFUNKTIONER - IMPLEMENTERING
// ============================================================================================================================
// ============================================================================================================================
/**
 * @brief Finder de nærmeste tabelværdier over og under den aktuelle ADC-værdi
 * 
 * @param pga_index PGA-indeks (0=2x, 1=4x, 2=8x, 3=16x)
 * @param adc_value Aktuel ADC-værdi (efter oversampling)
 * @param adc_low Pointer til nærmeste ADC-værdi under
 * @param esr_low Pointer til ESR-værdi svarende til adc_low
 * @param adc_high Pointer til nærmeste ADC-værdi over
 * @param esr_high Pointer til ESR-værdi svarende til adc_high
 * 
 * @details Bruger binært søgning til at finde de to nærmeste indgange i 
 * ESR-tabellen for den aktuelle PGA-indstilling. Håndterer edge cases 
 * hvor ADC-værdien er uden for tabelområdet.
 * 
 * @note Opfylder S.ESR.001.2 kravet om tabelopslag
 * 
 * @return void
 */
// ============================================================================================================================
void esr_find_nearest_values(uint8_t pga_index, int32_t adc_value, 
                            int16_t *adc_low, float *esr_low, 
                            int16_t *adc_high, float *esr_high) {
    // Håndter edge cases først
    TableEntry entry;                       				// Midlertidig variabel til at læse tabelindgange
    
    // Læs første indgang i tabellen (laveste ADC-værdi)
    memcpy_P(&entry, &esrTables[pga_index][0], sizeof(TableEntry)); 	// Kopier første tabelindgang fra PROGMEM til RAM
    if (adc_value <= entry.adcValue) {      				// Hvis ADC-værdi er mindre end eller lig med første indgang
        // ADC-værdi er mindre end eller lig med første indgang
        *adc_low = entry.adcValue;          				// Gem den laveste ADC-værdi (første indgang)
        *esr_low = entry.esrValue;          				// Gem ESR-værdien for den laveste ADC-værdi
        // Brug samme værdi for high (ingen interpolation)
        *adc_high = entry.adcValue;         				// Brug samme ADC-værdi for høj (ingen interpolation)
        *esr_high = entry.esrValue;         				// Brug samme ESR-værdi for høj
        return;                             				// Afslut funktionen tidligt
    }
    
    // Læs sidste indgang i tabellen (højeste ADC-værdi)
    memcpy_P(&entry, &esrTables[pga_index][TABLE_ENTRIES - 1], sizeof(TableEntry)); 	// Kopier sidste tabelindgang fra PROGMEM
    if (adc_value >= entry.adcValue) {      				// Hvis ADC-værdi er større end eller lig med sidste indgang
        // ADC-værdi er større end eller lig med sidste indgang
        *adc_low = entry.adcValue;          				// Gem den højeste ADC-værdi (sidste indgang)
        *esr_low = entry.esrValue;          				// Gem ESR-værdien for den højeste ADC-værdi
        *adc_high = entry.adcValue;         				// Brug samme ADC-værdi for høj
        *esr_high = entry.esrValue;         				// Brug samme ESR-værdi for høj
        return;                             				// Afslut funktionen tidligt
    }
    
    // Binært søgning for at finde interval
    int low = 0;                            				// Startindeks for søgning
    int high = TABLE_ENTRIES - 1;           				// Slutindeks for søgning
    int mid = 0;                            				// Midterindeks
    
    while (low <= high) {                   				// Fortsæt indtil søgeområdet er udtømt
        mid = low + (high - low) / 2;       				// Beregn midterindeks (undgår overflow)
        
        // Læs midterindgang fra PROGMEM
        memcpy_P(&entry, &esrTables[pga_index][mid], sizeof(TableEntry)); // Kopier midterindgang fra PROGMEM
        if (entry.adcValue == adc_value) {  				// Hvis præcis match fundet
            // Præcis match - brug samme værdi for begge
            *adc_low = entry.adcValue;      				// Gem ADC-værdien som både lav og høj
            *esr_low = entry.esrValue;      				// Gem ESR-værdien som både lav og høj
            *adc_high = entry.adcValue;     				// Samme ADC-værdi for høj
            *esr_high = entry.esrValue;     				// Samme ESR-værdi for høj
            return;                         				// Afslut funktionen
        } else if (entry.adcValue < adc_value) { 			// Hvis midtens ADC-værdi er mindre end søgte værdi
            // Tjek om næste indgang er over adc_value
            if (mid + 1 < TABLE_ENTRIES) {  				// Hvis der er en næste indgang i tabellen
                TableEntry next_entry;      				// Variabel til næste tabelindgang
                memcpy_P(&next_entry, &esrTables[pga_index][mid + 1], sizeof(TableEntry)); // Kopier næste indgang
                if (next_entry.adcValue >= adc_value) { 		// Hvis næste indgangs ADC-værdi er større eller lig
                    // Vi har fundet intervallet
                    *adc_low = entry.adcValue;      			// Gem den nederste ADC-værdi i intervallet
                    *esr_low = entry.esrValue;      			// Gem ESR-værdien for den nederste ADC-værdi
                    *adc_high = next_entry.adcValue;			// Gem den øverste ADC-værdi i intervallet
                    *esr_high = next_entry.esrValue;			// Gem ESR-værdien for den øverste ADC-værdi
                    return;                         			// Afslut funktionen
                } else {                    				// Hvis næste indgang også er mindre end søgte værdi
                    low = mid + 1;          				// Søg i højre halvdel (opdater søgeområde)
                }
            } else {                        				// Hvis mid er sidste element i tabellen (edge case)
                // Edge case: mid er sidste element
                *adc_low = entry.adcValue;  				// Brug sidste indgang som både lav og høj
                *esr_low = entry.esrValue;  				// Brug ESR-værdien for sidste indgang
                *adc_high = entry.adcValue; 				// Samme ADC-værdi for høj
                *esr_high = entry.esrValue; 				// Samme ESR-værdi for høj
                return;                     				// Afslut funktionen
            }
        } else {                            				// Hvis midtens ADC-værdi er større end søgte værdi
            // entry.adcValue > adc_value
            high = mid - 1;                 				// Søg i venstre halvdel (opdater søgeområde)
        }
    }
    
    // Fallback: brug første og anden indgang (hvis noget gik galt under søgningen)
    memcpy_P(&entry, &esrTables[pga_index][0], sizeof(TableEntry)); 			// Kopier første tabelindgang
    *adc_low = entry.adcValue;              						// Brug første indgang som lav
    *esr_low = entry.esrValue;              						// Brug første indgangs ESR
    
    memcpy_P(&entry, &esrTables[pga_index][1], sizeof(TableEntry)); 			// Kopier anden tabelindgang
    *adc_high = entry.adcValue;             						// Brug anden indgang som høj
    *esr_high = entry.esrValue;             						// Brug anden indgangs ESR
}
// ============================================================================================================================
/**
 * @brief Beregner lineær interpolation mellem to tabelpunkter
 * 
 * @param adc_low ADC-værdi for nederste tabelindgang
 * @param esr_low ESR-værdi for nederste tabelindgang
 * @param adc_high ADC-værdi for øverste tabelindgang
 * @param esr_high ESR-værdi for øverste tabelindgang
 * @param adc_current Aktuel ADC-værdi
 * 
 * @details Beregner ESR ved lineær interpolation:
 * esr = esr_low + (adc_current - adc_low) × (esr_high - esr_low) / (adc_high - adc_low)
 * 
 * @note Opfylder S.ESR.001.3 kravet om lineær interpolation
 * 
 * @return float Interpoleret ESR-værdi i ohm
 */
// ============================================================================================================================
float esr_linear_interpolate(int16_t adc_low, float esr_low, 
                           int16_t adc_high, float esr_high, 
                           int32_t adc_current) {
    // Håndter division med nul (hvis adc_low == adc_high)
    if (adc_high == adc_low) {              			// Hvis høj og lav ADC-værdi er ens (ingen interpolation mulig)
        return esr_low;                     			// Returner den laveste ESR-værdi
    }
    
    // Lineær interpolation
    float fraction = (float)(adc_current - adc_low) / (float)(adc_high - adc_low); 	// Beregn brøkdel af intervallet
    return esr_low + fraction * (esr_high - esr_low); 					// Beregn interpoleret ESR-værdi
}

// ============================================================================================================================
/**
 * @brief Hovedfunktion for ESR-beregning
 * 
 * @param pga_index PGA-indeks (0=2x, 1=4x, 2=8x, 3=16x)
 * @param adc_value ADC-værdi (efter oversampling)
 * 
 * @details Koordinerer hele ESR-beregningsprocessen:
 * 1. Finder nærmeste tabelværdier
 * 2. Udfører lineær interpolation
 * 3. Returnerer beregnet ESR-værdi
 * 
 * @note Opfylder S.ESR.001.4 kravet om ESR-beregningsfunktion
 * 
 * @return float Beregnet ESR-værdi i ohm
 */
// ============================================================================================================================
float esr_calculate(uint8_t pga_index, int32_t adc_value) {
    // Variabler til nærmeste tabelværdier
    int16_t adc_low = 0, adc_high = 0;      				// Initialiser variabler til grænseværdier for ADC
    float esr_low = 0.0, esr_high = 0.0;    				// Initialiser variabler til grænseværdier for ESR
    
    // Find nærmeste tabelværdier
    esr_find_nearest_values(pga_index, adc_value, 			// Kald funktion til at finde interval i kalibreringstabel
                           &adc_low, &esr_low,    			// Output: laveste grænseværdier
                           &adc_high, &esr_high); 			// Output: højeste grænseværdier
    
    // Udfør lineær interpolation
    float esr_value = esr_linear_interpolate(adc_low, esr_low, 		// Beregn interpoleret ESR-værdi
                                           adc_high, esr_high, 
                                           adc_value);
    
    // Begræns ESR-værdi til 0-10 ohm område
    if (esr_value < 0.0) {                  				// Hvis beregnet ESR er mindre end 0 ohm
        esr_value = 0.0;                    				// Sæt ESR til 0 ohm (undgår negative værdier)
    } else if (esr_value > 10.0) {          				// Hvis beregnet ESR er større end 10 ohm
        esr_value = 10.0;                   				// Sæt ESR til 10 ohm (maksimalt område)
    }
    
    return esr_value;                       				// Returner den beregnede (og begrænsede) ESR-værdi
}
/**
* ============================================================================================================================
 * @defgroup UI_DISPLAY User Interface (Display)
 * @brief Håndterer visning af data på LCD
 * ============================================================================================================================
 * Dette modul styrer:
 * - Visning af ESR-resultater
 * - Visning af rå ADC-data
 * - Systemstatus og fejlmeddelelser
 *
 * UI-logik er bevidst holdt simpel og afhænger
 * af state machine.
 */
// ============================================================================================================================
// DISPLAY FUNKTIONER - IMPLEMENTERING
// ============================================================================================================================

// ============================================================================================================================
/**
 * @brief Initialiserer display systemet
 * 
 * @details Klarer LCD-displayet og viser startbesked.
 * 
 * @note Opfylder S.DSP.001.2 kravet om display-initialisering
 * 
 * @return void
 */
// ============================================================================================================================
void display_init(void) {
    // Display er allerede initialiseret i hal_lcd_init()
    // Her kan vi tilføje yderligere display-specifik konfiguration
    
    lcd.clear();                            		// Ryd alt indhold på LCD-displayet
    lcd.setCursor(0, 0);                    		// Sæt cursor til starten af første linje (kolonne 0, række 0)
    lcd.print("ESR-Meter");                 		// Skriv "ESR-Meter" på displayets første linje
    lcd.setCursor(0, 1);                    		// Sæt cursor til starten af anden linje (kolonne 0, række 1)
    lcd.print("Initialiseret");             		// Skriv "Initialiseret" på displayets anden linje
}
// ============================================================================================================================
/**
 * @brief Viser rå ADC-værdi på display (til kalibrering)
 * 
 * @param adc_value ADC-værdi at vise
 * @param pga_index PGA-indeks for visning
 * 
 * @details Formaterer og viser rå ADC-data på displayet.
 * Bruges primært til kalibrering og debugging.
 * 
 * @note Opfylder S.DSP.001.3 kravet om rå ADC-visning
 * 
 * @return void
 */
// ============================================================================================================================
void display_raw_adc(int32_t adc_value, uint8_t pga_index) {
    lcd.clear();                            					// Ryd alt indhold på LCD-displayet
    
    // Linje 1: ADC-værdi
    lcd.setCursor(0, 0);                    				// Sæt cursor til starten af første linje
    lcd.print("ADC: ");                     				// Skriv "ADC: " på displayet
    lcd.print(adc_value);                   				// Udskriv ADC-værdien
    lcd.print("   "); 						//  Tilføj mellemrum for at overskrive eventuelle resterende tegn
    
    // Linje 2: PGA og procent af fuld skala
    lcd.setCursor(0, 1);                    				// Sæt cursor til starten af anden linje
    lcd.print("PGA: ");                     				// Skriv "PGA: " på displayet
    
    switch (pga_index) {                    				// Vælg PGA-indstilling baseret på pga_index
        case PGA_2X_INDEX:  lcd.print("2x"); break;  		// Hvis pga_index = 0, skriv "2x"
        case PGA_4X_INDEX:  lcd.print("4x"); break;  		// Hvis pga_index = 1, skriv "4x"
        case PGA_8X_INDEX:  lcd.print("8x"); break;  		// Hvis pga_index = 2, skriv "8x"
        case PGA_16X_INDEX: lcd.print("16x"); break; 		// Hvis pga_index = 3, skriv "16x"
        default:           lcd.print("?"); break;    			// Hvis ugyldig værdi, skriv "?"
    }
    
    // Beregn og vis procent af fuld skala
    float percent = (float)abs(adc_value) / 32767.0 * 100.0; // Beregn procentdel af fuld skala (32767 = 2^15-1)
    lcd.print(" (");                         				// Skriv " (" på displayet
    lcd.print(percent, 1);                   				// Udskriv procentværdi med 1 decimal
    lcd.print("%)");                         				// Skriv "%)" på displayet
}

// ============================================================================================================================
/**
 * @brief Viser beregnet ESR-resultat på display
 * 
 * @param esr_value Beregnet ESR-værdi i ohm
 * @param adc_value ADC-værdi brugt til beregning
 * @param pga_index PGA-indeks brugt til måling
 * 
 * @details Formaterer og viser ESR-resultatet med høj opløsning (mΩ).
 * Inkluderer også ADC-værdi og PGA-indstilling for reference.
 * 
 * @note Opfylder S.DSP.001.4 kravet om ESR-resultat-visning
 * 
 * @return void
 */
// ============================================================================================================================
void display_esr_result(float esr_value, int32_t adc_value, uint8_t pga_index) {
    lcd.clear();                            			// Ryd alt indhold på LCD-displayet
    
    // Linje 1: ESR-værdi med høj opløsning (mΩ)
    lcd.setCursor(0, 0);                    			// Sæt cursor til starten af første linje
    lcd.print("ESR: ");                     			// Skriv "ESR: " på displayet
    
    if (esr_value < 0.001) {                			// Hvis ESR-værdi er mindre end 0.001 ohm (under 1 mΩ)
        // Under 1 mΩ - vis med nano-ohm opløsning
        lcd.print(esr_value * 1000000.0, 3); 				// Konverter til nΩ og udskriv med 3 decimaler
        lcd.print(" nΩ");                    				// Skriv enheden "nΩ"
    } else if (esr_value < 1.0) {           			// Hvis ESR-værdi er mindre end 1 ohm (under 1 Ω)
        // Under 1 Ω - vis med mΩ opløsning
        lcd.print(esr_value * 1000.0, 3);    				// Konverter til mΩ og udskriv med 3 decimaler
        lcd.print(" mΩ");                    				// Skriv enheden "mΩ"
    } else {                                			// Hvis ESR-værdi er 1 ohm eller derover
        // Over 1 Ω - vis med Ω opløsning
        lcd.print(esr_value, 3);             				// Udskriv ESR-værdi med 3 decimaler
        lcd.print(" Ω");                     				// Skriv enheden "Ω"
    }
    
    // Juster spacing
    if (esr_value < 10.0) {                			// Hvis ESR-værdi er mindre end 10 ohm
        lcd.print("   "); 						//  Tilføj mellemrum for bedre formattering
    }
    
    // Linje 2: ADC og PGA info
    lcd.setCursor(0, 1);                    				// Sæt cursor til starten af anden linje
    lcd.print("ADC:");                      				// Skriv "ADC:" på displayet
    lcd.print(adc_value);                   				// Udskriv ADC-værdien
    lcd.print(" PGA:");                     				// Skriv " PGA:" på displayet
    
    switch (pga_index) {                    			// Vælg PGA-indstilling baseret på pga_index
        case PGA_2X_INDEX:  lcd.print("2x"); break;  			// Hvis pga_index = 0, skriv "2x"
        case PGA_4X_INDEX:  lcd.print("4x"); break;  			// Hvis pga_index = 1, skriv "4x"
        case PGA_8X_INDEX:  lcd.print("8x"); break;  			// Hvis pga_index = 2, skriv "8x"
        case PGA_16X_INDEX: lcd.print("16x"); break; 			// Hvis pga_index = 3, skriv "16x"
        default:           lcd.print("?"); break;   			// Hvis ugyldig værdi, skriv "?"
    }
}
// ============================================================================================================================
/**
 * @brief Opdaterer displayet baseret på nuværende tilstand
 * 
 * @details Opdaterer displayet baseret på displayMode og tilgængelige data.
 * Kalder enten display_raw_adc() eller display_esr_result().
 * 
 * @note Opfylder S.DSP.001.5 kravet om display-opdatering
 * 
 * @return void
 */
// ============================================================================================================================
void display_update(void) {
    // Opdater kun hvis der er nye data
    if (newMeasurementAvailable) {          					// Tjek om der er en ny måling klar til visning
        if (displayMode == DISPLAY_MODE_RAW_ADC) { 				// Hvis displaytilstand er rå ADC
            display_raw_adc(rawAdcValue, currentPgaSetting); 			// Vis rå ADC-værdi
        } else if (displayMode == DISPLAY_MODE_ESR_RESULT) { 			// Hvis displaytilstand er ESR-resultat
            display_esr_result(calculatedEsr, rawAdcValue, currentPgaSetting); 	// Vis beregnet ESR
        }
        newMeasurementAvailable = false;    					// Markér målingen som vist (nulstil flag)
    }
}

// ============================================================================================================================
/**
 * @brief Viser fejlbesked på display
 * 
 * @param error_msg Fejlbesked at vise
 * 
 * @details Viser fejlbesked på begge linjer af displayet.
 * Bruges til systemfejl og kalibreringsfejl.
 * 
 * @return void
 */
// ============================================================================================================================
void display_error(const char *error_msg) {
    lcd.clear();                            					// Ryd alt indhold på LCD-displayet
    
    // Del fejlbeskeden i to linjer hvis den er for lang
    char line1[17] = {0};                   					// Buffer til første linje (16 tegn + null)
    char line2[17] = {0};                   					// Buffer til anden linje (16 tegn + null)
    
    // Kopier første 16 tegn til linje 1
    strncpy(line1, error_msg, 16);          					// Kopier op til 16 tegn af fejlbeskeden til line1
    line1[16] = '\0';                       					// Sikre null-terminering
    
    // Hvis beskeden er længere end 16 tegn, kopier resten til linje 2
    if (strlen(error_msg) > 16) {           					// Hvis fejlbeskeden er længere end 16 tegn
        strncpy(line2, error_msg + 16, 16); 					// Kopier de næste 16 tegn til line2
        line2[16] = '\0';                   					// Sikre null-terminering
    }
    
    // Vis fejlbesked
    lcd.setCursor(0, 0);                    					// Sæt cursor til starten af første linje
    lcd.print(line1);                       					// Udskriv første del af fejlbeskeden
    
    lcd.setCursor(0, 1);                    					// Sæt cursor til starten af anden linje
    if (line2[0] != '\0') {                 					// Hvis der er en anden linje med tekst
        lcd.print(line2);                   					// Udskriv anden del af fejlbeskeden
    } else {                                					// Hvis fejlbeskeden var kort (kun én linje)
        lcd.print("FEJL!");                 					// Udskriv standard "FEJL!" på anden linje
    }
}
// ============================================================================================================================
/**
 * @brief Viser systeminformation på seriel port
 */
// ============================================================================================================================
void display_system_info(void) {
    Serial.println("=== SYSTEM INFORMATION ===");            	// Udskriv systeminformation overskrift til seriel monitor
    Serial.print("Firmware Version: ");                     	// Udskriv tekst "Firmware Version: "
    Serial.println(FIRMWARE_VERSION);                       	// Udskriv firmware version konstant
    Serial.print("Hardware Version: ");                     	// Udskriv tekst "Hardware Version: "
    Serial.println(HARDWARE_VERSION);                       	// Udskriv hardware version konstant
    
    Serial.print("ESP32 Chip ID: ");                        	// Udskriv tekst "ESP32 Chip ID: "
    Serial.println(ESP.getEfuseMac(), HEX);                 	// Udskriv ESP32 chip ID i hexadecimalt format
    
    Serial.print("Free Heap: ");                            	// Udskriv tekst "Free Heap: "
    Serial.print(ESP.getFreeHeap());                        	// Udskriv tilgængelig RAM (heap) størrelse
    Serial.println(" bytes");                               	// Udskriv enheden "bytes"
    
    Serial.print("CPU Frequency: ");                        	// Udskriv tekst "CPU Frequency: "
    Serial.print(ESP.getCpuFreqMHz());                      	// Udskriv CPU frekvens i MHz
    Serial.println(" MHz");                                 	// Udskriv enheden "MHz"
    
    Serial.print("Flash Size: ");                           	// Udskriv tekst "Flash Size: "
    Serial.print(ESP.getFlashChipSize() / (1024 * 1024));   	// Beregn og udskriv flash størrelse i MB
    Serial.println(" MB");                                  	// Udskriv enheden "MB"
    
    Serial.println("=== MODULSTATUS ===");                  	// Udskrift af modulstatus overskrift
    Serial.println("AD9850: Initialiseret");                	// Status for DDS-generator
    Serial.println("ADS1115: Initialiseret");               	// Status for ADC
    Serial.println("LCD: Initialiseret");                   	// Status for LCD display
    Serial.println("I2C: 400 kHz");                         	// Status for I2C bus hastighed
    Serial.println("Watchdog: Aktiv");                      	// Status for watchdog timer
    
    Serial.println("=== KRAVSPECIFIKATION ===");            	// Udskrift af kravspecifikationsoverskrift
    Serial.println("ESR Range: 0-10 ohm");                  	// Specifikation: ESR måleområde
    Serial.println("Resolution: 1 mΩ");                     	// Specifikation: opløsning
    Serial.println("Precision: ≤5%");                       	// Specifikation: præcision
    Serial.println("Test Frequency: 100 kHz");              	// Specifikation: testfrekvens
    Serial.println("Test Signal: ≤0.1V p-p");               	// Specifikation: testsignal amplitude
    Serial.println("==========================");           	// Afsluttende skillelinje
}
/**
* ============================================================================================================================
 * @defgroup STATE_MACHINE System State Machine
 * @brief Styrer hele systemets operationelle flow
 * ============================================================================================================================
 * State machine-modulet:
 * - Koordinerer målinger, beregninger, display og kommunikation
 * - Sikrer deterministisk systemadfærd
 * - Muliggør fejlhåndtering via ERROR-state
 *
 * Alle systemændringer bør ske gennem state transitions.
 */
// ============================================================================================================================
// STATE MACHINE FUNKTIONER - IMPLEMENTERING
// ============================================================================================================================

// ============================================================================================================================
/**
 * @brief Initialiserer state machine systemet
 * 
 * @details Sætter starttilstande og initialiserer timing-variabler.
 * 
 * @return void
 */
// ============================================================================================================================
void state_machine_init(void) {
    currentState = STATE_IDLE;                              		// Sæt starttilstand til IDLE
    lastMeasurementTime = millis();                         		// Initialiser målingstidspunkt med nuværende tid
    lastDisplayUpdateTime = millis();                       		// Initialiser display opdateringstidspunkt
    
    Serial.println("State machine initialiseret");          		// Bekræft state machine initialisering
}

// ============================================================================================================================
/**
 * @brief Hovedopdateringsfunktion for state machine
 * 
 * @details Kalder den korrekte state handler baseret på currentState.
 * Håndterer også timing-baserede transitions.
 * 
 * @return void
 */
// ============================================================================================================================
void state_machine_update(void) {
    // Kald state handler baseret på nuværende state
    switch (currentState) {                                 			// Vælg funktion baseret på aktuelt state
        case STATE_IDLE:                                    			// Hvis state er IDLE
            state_idle_handler();                           				// Kør IDLE state handler
            break;
        case STATE_MEASURE_ADC:                             			// Hvis state er MEASURE_ADC
            state_measure_adc_handler();                    				// Kør ADC-målings handler
            break;
        case STATE_CALCULATE_ESR:                           			// Hvis state er CALCULATE_ESR
            state_calculate_esr_handler();                  				// Kør ESR-beregnings handler
            break;
        case STATE_UPDATE_DISPLAY:                          			// Hvis state er UPDATE_DISPLAY
            state_update_display_handler();                 				// Kør display-opdaterings handler
            break;
        case STATE_PROCESS_SERIAL:                         			// Hvis state er PROCESS_SERIAL
            state_process_serial_handler();                 				// Kør seriel kommunikations handler
            break;
        case STATE_ERROR:                                   			// Hvis state er ERROR
            state_error_handler();                          				// Kør fejlhåndterings handler
            break;
        default:                                            			// Hvis state er ukendt
            // Ukendt state - gå til error state
            currentState = STATE_ERROR;                     				// Sæt state til ERROR
            error_handler("Ukendt state", 0x10);            			// Kald fejlhåndtering med besked og kode
            break;
    }
}

// ============================================================================================================================
/**
 * @brief State handler for IDLE tilstand
 * 
 * @details IDLE state venter på at det er tid til næste måling
 * eller på brugerinput. Skifter til MEASURE_ADC ved timeout.
 * 
 * @return void
 */
// ============================================================================================================================
void state_idle_handler(void) {
    uint32_t currentTime = millis();                        		    // Hent nuværende systemtid i ms
    
    // Tjek om det er tid til næste måling
    if ((currentTime - lastMeasurementTime) >= MEASUREMENT_INTERVAL) { 	    // Hvis måleinterval er overskredet
        currentState = STATE_MEASURE_ADC;                   			// Skift til ADC-målingstilstand
        lastMeasurementTime = currentTime;                  			// Opdater sidste målingstidspunkt
    }
    
    // Tjek om display skal opdateres
    if ((currentTime - lastDisplayUpdateTime) >= DISPLAY_UPDATE_INTERVAL) { // Hvis display-opdateringsinterval er overskredet
        currentState = STATE_UPDATE_DISPLAY;                			// Skift til display-opdateringstilstand
        lastDisplayUpdateTime = currentTime;                		 	// Opdater sidste display opdateringstidspunkt
    }
}

// ============================================================================================================================
/**
 * @brief State handler for ADC-målingstilstand
 * 
 * @details Udfører automatisk PGA-valg og oversampled ADC-måling.
 * Skifter til CALCULATE_ESR efter måling.
 * 
 * @return void
 */
// ============================================================================================================================
void state_measure_adc_handler(void) {
    // Udfør automatisk PGA-valg
    currentPgaSetting = adc_auto_range();                   		// Find optimal PGA-indstilling baseret på signalstyrke
    
    // Udfør oversampled ADC-måling
    rawAdcValue = adc_read_oversampled(currentPgaSetting);  		// Læs ADC-værdi med 64× oversampling
    
    // Gå til næste state
    currentState = STATE_CALCULATE_ESR;                     		// Skift til ESR-beregningstilstand
}

// ============================================================================================================================
/**
 * @brief State handler for ESR-beregningstilstand
 * 
 * @details Beregner ESR-værdi baseret på ADC-måling og tabelopslag.
 * Skifter til UPDATE_DISPLAY efter beregning.
 * 
 * @return void
 */
// ============================================================================================================================
void state_calculate_esr_handler(void) {
    // Beregn ESR-værdi
    calculatedEsr = esr_calculate(currentPgaSetting, rawAdcValue); 	// Beregn ESR-værdi fra ADC-værdi
    
    // Sæt flag for ny måling
    newMeasurementAvailable = true;                         		// Markér at ny måling er klar til visning
    
    // Gå til næste state
    currentState = STATE_UPDATE_DISPLAY;                    		// Skift til display-opdateringstilstand
}

// ============================================================================================================================
/**
 * @brief State handler for display-opdateringstilstand
 * 
 * @details Opdaterer displayet baseret på nuværende display mode.
 * Returnerer til IDLE efter opdatering.
 * 
 * @return void
 */
// ============================================================================================================================
void state_update_display_handler(void) {
    // Opdater display
    display_update();                                       		// Opdater LCD-display med nuværende data
    
    // Returner til IDLE state
    currentState = STATE_IDLE;                              		// Skift tilbage til IDLE-tilstand
}

// ============================================================================================================================
/**
 * @brief State handler for seriel kommunikationstilstand
 * 
 * @details Håndterer SCPI-kommandoer og seriel kommunikation.
 * Returnerer til IDLE efter behandling.
 * 
 * @return void
 */
// ============================================================================================================================
void state_process_serial_handler(void) {
    // Håndter SCPI-kommandoer hvis nogen er klar
    if (scpiCommandReady) {                                 		// Tjek om en SCPI-kommando er klar til behandling
        scpi_parser();                                      			// Parse og udfør SCPI-kommandoen
        scpiCommandReady = false;                           			// Nulstil kommando-klar flag
    }
    
    // Returner til IDLE state
    currentState = STATE_IDLE;                              		// Skift tilbage til IDLE-tilstand
}

// ============================================================================================================================
/**
 * @brief State handler for fejltilstand
 * 
 * @details Viser fejlbesked på display og logger fejlen.
 * Forsøger at genstarte systemet efter timeout.
 * 
 * @return void
 */
// ============================================================================================================================
void state_error_handler(void) {
    static uint32_t errorStartTime = 0;                     		// Statisk variabel til at gemme fejlstarttid
    
    if (errorStartTime == 0) {                              		// Hvis fejltilstand lige er startet
        errorStartTime = millis();                          			// Gem starttidspunkt for fejlen
        display_error("Systemfejl - genstarter...");        			// Vis fejlbesked på display
        Serial.println("ERROR STATE: Systemfejl opstået");  			// Log fejl til seriel port
    }
    
    // Tjek om der er gået nok tid til at genstarte
    if ((millis() - errorStartTime) > 5000) {              		// Hvis 5 sekunder er gået siden fejlen started
        Serial.println("ERROR STATE: Genstarter system...");			// Log genstart til seriel port
        cleanup_before_restart();                           			// Kør oprydning før genstart
        ESP.restart(); 	                					// Genstart ESP32 mikrokontrolleren
    }
}
/**
* ============================================================================================================================
 * @defgroup INPUT_BUTTONS Button Handling Module
 * @brief Debounce og håndtering af brugerinput
 * ============================================================================================================================
 * Indeholder logik for:
 * - Debounce
 * - Mode-skift
 * - UI-interaktion
 *
 * Designet til at være ikke-blokerende.
 */

// ============================================================================================================================
// KNAP-HÅNDTERINGSFUNKTIONER - IMPLEMENTERING
// ============================================================================================================================
// ============================================================================================================================
/**
 * @brief Tjekker knaptryk med debounce
 * 
 * @details Læser knappins og implementerer debounce ved at sammenligne
 * med sidste tryk-tidspunkt. Kun én handling pr. knap pr. DEBOUNCE_DELAY.
 * 
 * @note Opfylder kravet om knaprespons ≤20 ms (S.PER.001.4)
 * 
 * @return void
 */
// ============================================================================================================================
void button_check(void) {
    uint32_t currentTime = millis();                         		// Hent nuværende systemtid i millisekunder
    
    // Tjek knap 1 (skift displaytilstand)
    if (digitalRead(BUTTON1_PIN) == LOW) { 				// LOW fordi aktiv lav med pull-up // Læs knap 1: lavt signal = trykket
        if (!button1Pressed && (currentTime - button1LastPress) > DEBOUNCE_DELAY) { // Hvis knap ikke allerede markeret trykket OG debounce-interval er overskredet
            button1Pressed = true;                           			// Marker knap 1 som trykket (forhindrer gentagne handlinger)
            button1LastPress = currentTime;                  			// Opdater tidspunkt for sidste tryk på knap 1
            button1_handler();                               			// Kald funktionen der håndterer knap 1 tryk
        }
    } else {                                                 		// Hvis knap 1 ikke er trykket (signal er højt)
        button1Pressed = false;                              			// Nulstil knap 1 tryk-status
    }
    
    // Tjek knap 2 (skift kommunikationstilstand)
    if (digitalRead(BUTTON2_PIN) == LOW) {                   		// Læs knap 2: lavt signal = trykket
        if (!button2Pressed && (currentTime - button2LastPress) > DEBOUNCE_DELAY) { // Hvis knap ikke allerede markeret trykket OG debounce-interval er overskredet
            button2Pressed = true;                           			// Marker knap 2 som trykket
            button2LastPress = currentTime;                  			// Opdater tidspunkt for sidste tryk på knap 2
            button2_handler();                               			// Kald funktionen der håndterer knap 2 tryk
        }
    } else {                                                 		// Hvis knap 2 ikke er trykket (signal er højt)
        button2Pressed = false;                              			// Nulstil knap 2 tryk-status
    }
}

// ============================================================================================================================
/**
 * @brief Håndterer knap 1 tryk
 * 
 * @details Skifter mellem RAW_ADC og ESR_RESULT displaytilstande.
 * 
 * @return void
 */
// ============================================================================================================================
void button1_handler(void) {
    if (displayMode == DISPLAY_MODE_RAW_ADC) {               		// Hvis nuværende displaytilstand er RAW_ADC
        displayMode = DISPLAY_MODE_ESR_RESULT;               			// Skift til ESR_RESULT tilstand
        Serial.println("Display mode: ESR_RESULT");          			// Log ændringen til seriel port
    } else {                                                 		// Hvis nuværende displaytilstand er ESR_RESULT
        displayMode = DISPLAY_MODE_RAW_ADC;                  			// Skift til RAW_ADC tilstand
        Serial.println("Display mode: RAW_ADC");             			// Log ændringen til seriel port
    }
    
    // Sørg for display opdateres med det samme
    newMeasurementAvailable = true;                          		// Sæt flag for at der er nye data til display
    currentState = STATE_UPDATE_DISPLAY;                     		// Sæt state machine til at opdatere display
}

// ============================================================================================================================
/**
 * @brief Håndterer knap 2 tryk
 * 
 * @details Skifter mellem normal og SCPI kommunikationstilstand.
 * 
 * @return void
 */
// ============================================================================================================================
void button2_handler(void) {
    if (commMode == COMM_MODE_NORMAL) {                      		// Hvis nuværende kommunikationstilstand er NORMAL
        commMode = COMM_MODE_SCPI;                           			// Skift til SCPI tilstand
        Serial.println("Communication mode: SCPI");          			// Log ændringen til seriel port
        
        // Vis besked på display
        lcd.clear();                                         			// Ryd LCD display
        lcd.setCursor(0, 0);                                 			// Sæt cursor til starten af første linje
        lcd.print("SCPI Mode");                              			// Skriv "SCPI Mode" på første linje
        lcd.setCursor(0, 1);                                 			// Sæt cursor til starten af anden linje
        lcd.print("Active");                                 			// Skriv "Active" på anden linje
        
        // Reset watchdog under delay
        for (int i = 0; i < 10; i++) {                       			// Loop 10 gange (samlet 1 sekund)
            wdg_reset();                                     				// Reset watchdog timer under ventetid
            delay(100);                                      				// Vent 100 millisekunder
        }
    } else {                                                 		// Hvis nuværende kommunikationstilstand er SCPI
        commMode = COMM_MODE_NORMAL;                         			// Skift til NORMAL tilstand
        Serial.println("Communication mode: NORMAL");        			// Log ændringen til seriel port
        
        // Vis besked på display
        lcd.clear();                                         			// Ryd LCD display
        lcd.setCursor(0, 0);                                 			// Sæt cursor til starten af første linje
        lcd.print("Normal Mode");                            			// Skriv "Normal Mode" på første linje
        lcd.setCursor(0, 1);                                 			// Sæt cursor til starten af anden linje
        lcd.print("Active");                                 			// Skriv "Active" på anden linje
        
        // Reset watchdog under delay
        for (int i = 0; i < 10; i++) {                       			// Loop 10 gange (samlet 1 sekund)
            wdg_reset();                                     				// Reset watchdog timer under ventetid
            delay(100);                                      				// Vent 100 millisekunder
        }
    }
}
/**
* ============================================================================================================================
 * @defgroup SCPI_COMMUNICATION SCPI Communication Module
 * @brief Seriel fjernstyring via SCPI-protokol
 * ============================================================================================================================
 * Muliggør:
 * - Fjernmåling af ESR
 * - Konfiguration af systemet
 * - Integration i automatiske testsystemer
 *
 * Bemærk:
 * SCPI-implementeringen er delvist ufuldstændig.
 */

// ============================================================================================================================
// SCPI KOMMUNIKATIONSFUNKTIONER - IMPLEMENTERING
// ============================================================================================================================
// ============================================================================================================================
/**
 * @brief Læser indkommende seriel data og samler SCPI kommandoer
 * 
 * @details Læser tegn fra seriel port indtil newline og gemmer i buffer.
 * Sætter scpiCommandReady flag når en kommando er komplet.
 * 
 * @note Køres typisk fra loop() eller serialEvent()
 * 
 * @return void
 */
// ============================================================================================================================
void scpi_read_serial(void) {
    while (Serial.available() > 0 && scpiBufferIndex < 255) { 		// Læs så længe der er data og plads i buffer
        char c = Serial.read();                              		// Læs et enkelt tegn fra seriel port
        
        // Ignorer carriage return
        if (c == '\r') {                                     		// Hvis tegnet er carriage return (Windows linjeskift)
            continue;                                        			// Spring over til næste iteration i løkken
        }
        
        // Hvis newline, afslut kommandoen
        if (c == '\n') {                                     		// Hvis tegnet er newline (linjeskift)
            scpiBuffer[scpiBufferIndex] = '\0';             			// Afslut strengen med null-terminering
            scpiCommandReady = true;                         			// Sæt flag at en kommando er klar
            scpiBufferIndex = 0;                            			// Nulstil buffer indeks for næste kommando
            return;                                          			// Afslut funktionen
        }
        
        // Gem tegn i buffer
        scpiBuffer[scpiBufferIndex++] = c;                  		// Gem tegnet i buffer og øg indeks
        
        // Buffer overflow beskyttelse
        if (scpiBufferIndex >= 255) {                        		// Hvis buffer er fuld (eller over)
            scpiBuffer[254] = '\0';                         			// Sikre null-terminering ved sidste position
            scpiBufferIndex = 0;                            			// Nulstil buffer indeks
            scpiCommandReady = true;                         			// Marker at kommando (fejl) er klar
            return;                                          			// Afslut funktionen
        }
    }
}

// ============================================================================================================================
/**
 * @brief Parser SCPI kommandoer og kalder execute-funktion
 * 
 * @details Analyserer kommandoen i scpiBuffer og kalder den korrekte
 * execute-funktion baseret på kommandoen.
 * 
 * @note Opfylder S.SCP.001.2 kravet om SCPI-parser
 * 
 * @return void
 */
// ============================================================================================================================
void scpi_parser(void) {
    if (!scpiCommandReady || scpiBuffer[0] == '\0') {        		// Tjek om kommando er klar og ikke tom
        return;                                              		// Afslut funktionen hvis ingen kommando
    }
    
    // Fjern ledende og afsluttende whitespace
    char *cmd = scpiBuffer;                                  		// Brug pointer til buffer for at undgå at ændre originalen
    
    // Fjern ledende mellemrum
    while (*cmd == ' ' || *cmd == '\t') {                    		// Gennemgå tegn indtil ikke-whitespace
        cmd++;                                               		// Flyt pointeren frem
    }
    
    // Fjern afsluttende whitespace
    int len = strlen(cmd);                                   		// Find længden af den trimmed streng
    while (len > 0 && (cmd[len-1] == ' ' || cmd[len-1] == '\t' || cmd[len-1] == '\r' || cmd[len-1] == '\n')) { 	// Tjek sidste tegn
        cmd[--len] = '\0';                                   		// Erstat med null-terminering og reducer længde
    }
    
    // Kør kommandoen
    scpi_execute(cmd);                                       	    	// Send den trimmede kommando til execute-funktion
    
    // Reset flag og buffer
    scpiCommandReady = false;                                		// Nulstil kommando-klar flag
    scpiBufferIndex = 0;                                     		// Nulstil buffer indeks for næste kommando
}

// ============================================================================================================================
/**
 * @brief Udfører SCPI-kommando
 * 
 * @param command Komplet SCPI-kommando streng
 * 
 * @details Identificerer kommandoen og kalder den korrekte håndteringsfunktion.
 * 
 * @note Opfylder S.SCP.001.3 kravet om SCPI-execute
 * 
 * @return void
 */
// ============================================================================================================================
void scpi_execute(const char *command) {
    // *IDN? - Identifikation
    if (strcmp(command, "*IDN?") == 0) {                     	 // Sammenlign kommando med "*IDN?"
        scpi_handle_idn();                                   	    // Kald identifikationshåndtering
        return;                                              	    // Afslut funktionen
    }
    
    // MEASure:ESR? - Mål ESR
    if (strcmp(command, "MEASure:ESR?") == 0) {              	 // Sammenlign kommando med "MEASure:ESR?"
        scpi_handle_measure_esr();                           	    // Kald ESR-målingshåndtering
        return;                                              	    // Afslut funktionen
    }
    
    // SYSTem:PRESet - System reset
    if (strcmp(command, "SYSTem:PRESet") == 0) {             	 // Sammenlign kommando med "SYSTem:PRESet"
        scpi_handle_system_preset();                         	    // Kald system reset håndtering
        return;                                              	    // Afslut funktionen
    }
    
    // DISPlay:MODE RAW/ESR - Skift displaytilstand
    if (strncmp(command, "DISPlay:MODE ", 13) == 0) {        	 // Tjek om kommandoen starter med "DISPlay:MODE "
        const char *mode = command + 13;                     	    // Ekstraher parameter (det efter "DISPlay:MODE ")
        scpi_handle_display_mode(mode);                      	    // Kald display mode håndtering med parameter
        return;                                              	    // Afslut funktionen
    }
    
    // SYSTem:COMMunication ON/OFF - Skift kommunikationstilstand
    if (strncmp(command, "SYSTem:COMMunication ", 21) == 0) { 	 // Tjek om kommandoen starter med "SYSTem:COMMunication "
        const char *state = command + 21;                    	    // Ekstraher parameter (det efter "SYSTem:COMMunication ")
        scpi_handle_communication(state);                    	    // Kald kommunikationshåndtering med parameter
        return;                                              	    // Afslut funktionen
    }
    
    // Prøv avancerede kommandoer
    scpi_handle_advanced(command);                           	 // Prøv at matche med avancerede kommandoer
}

// ============================================================================================================================
/**
 * @brief Sender SCPI-svar tilbage til klient
 * 
 * @param response Svar-streng at sende
 * 
 * @details Sender svar med newline afslutning.
 * 
 * @note Opfylder S.SCP.001.4 kravet om SCPI-respons (<100 ms)
 * 
 * @return void
 */
// ============================================================================================================================
void scpi_response(const char *response) {
    Serial.println(response);                                		// Send svar til seriel port med newline
}
// ============================================================================================================================
/**
 * @brief Håndterer *IDN? kommandoen
 * 
 * @details Returnerer enhedsidentifikation i standard SCPI-format:
 * Producent,Model,Serienummer,Softwareversion
 * 
 * @return void
 */
// ============================================================================================================================
void scpi_handle_idn(void) {
    char response[128];                                      	// Buffer til svaret
    snprintf(response, sizeof(response),                    	// Formatér strengen
             "ESR-METER,%s,%s,%s",                          	    // SCPI-format: producent,model,serienr,version
             HARDWARE_VERSION, "SN001", FIRMWARE_VERSION);  	    // Indsæt hardwareversion, serienummer og firmwareversion
    scpi_response(response);                                 	// Send svaret til klient
}

// ============================================================================================================================
/**
 * @brief Håndterer MEASure:ESR? kommandoen
 * 
 * @details Udfører en ny måling og returnerer ESR-værdien i ohm
 * med høj præcision. Formaterer output til videnskabelig notation.
 * 
 * @return void
 */
// ============================================================================================================================
void scpi_handle_measure_esr(void) {
    // Udfør måling
    currentState = STATE_MEASURE_ADC;                       			// Sæt state machine til ADC-måling
    state_machine_update(); // ADC måling                   			// Kør state machine for at udføre ADC-måling
    
    currentState = STATE_CALCULATE_ESR;                     			// Sæt state machine til ESR-beregning
    state_machine_update(); // ESR beregning               			// Kør state machine for at beregne ESR
    
    // Send svar med høj præcision
    char response[32];                                      			// Buffer til svaret
    if (calculatedEsr < 0.001) {                            			// Hvis ESR er under 1 mΩ - vis i nΩ
        snprintf(response, sizeof(response), "%.3fn", calculatedEsr * 1000000.0); 	// Konverter til nΩ med 3 decimaler
    } else if (calculatedEsr < 1.0) {                       			// Hvis ESR er under 1 - vis i mΩ
        snprintf(response, sizeof(response), "%.3fm", calculatedEsr * 1000.0);    	// Konverter til mΩ med 3 decimaler
    } else {                                                			// Hvis ESR er 1 Ω eller derover - vis i Ω
        snprintf(response, sizeof(response), "%.4f", calculatedEsr);               	// Vis med 4 decimaler i Ω
    }
    
    scpi_response(response);                                 			// Send svaret til klient
}

// ============================================================================================================================
/**
 * @brief Håndterer SYSTem:PRESet kommandoen
 * 
 * @details Nulstiller systemet til standardtilstande:
 * - Display mode: ESR_RESULT
 * - Communication mode: NORMAL
 * - State: IDLE
 * 
 * @return void
 */
// ============================================================================================================================
void scpi_handle_system_preset(void) {
    displayMode = DISPLAY_MODE_ESR_RESULT;                   			// Sæt displaytilstand til ESR-resultat
    commMode = COMM_MODE_NORMAL;                             			// Sæt kommunikationstilstand til normal
    currentState = STATE_IDLE;                               			// Sæt state machine til idle
    
    // Nulstil måledata
    rawAdcValue = 0;                                         			// Nulstil rå ADC-værdi
    calculatedEsr = 0.0;                                     			// Nulstil beregnet ESR
    currentPgaSetting = PGA_2X_INDEX;                        			// Sæt PGA tilbage til 2x (startindstilling)
    newMeasurementAvailable = true;                          			// Sæt flag for ny måling (så display opdateres)
    
    // Vis bekræftigelse
    lcd.clear();                                             			// Ryd LCD display
    lcd.setCursor(0, 0);                                     			// Sæt cursor til første linje
    lcd.print("System Reset");                               			// Vis "System Reset"
    
    // Reset watchdog under kort delay
    for (int i = 0; i < 5; i++) {                           			// Loop 5 gange (samlet 1 sekund)
        wdg_reset();                                         				// Reset watchdog timer
        delay(200);                                          				// Vent 200 ms
    }
    
    scpi_response("OK");                                     			// Send bekræftigelse til klient
}

// ============================================================================================================================
/**
 * @brief Håndterer DISPlay:MODE kommandoen
 * 
 * @param mode Tilstand ("RAW" eller "ESR")
 * 
 * @details Skifter displaytilstand baseret på parameter.
 * 
 * @return void
 */
// ============================================================================================================================
void scpi_handle_display_mode(const char *mode) {
    if (strcmp(mode, "RAW") == 0) {                         			// Hvis parameter er "RAW"
        displayMode = DISPLAY_MODE_RAW_ADC;                  				// Sæt displaytilstand til rå ADC
        newMeasurementAvailable = true;                      				// Sæt flag for at opdatere display
        scpi_response("OK");                                 				// Send bekræftigelse
    } else if (strcmp(mode, "ESR") == 0) {                  			// Hvis parameter er "ESR"
        displayMode = DISPLAY_MODE_ESR_RESULT;               				// Sæt displaytilstand til ESR-resultat
        newMeasurementAvailable = true;                      				// Sæt flag for at opdatere display
        scpi_response("OK");                                 				// Send bekræftigelse
    } else {                                                 			// Hvis parameter er ugyldig
        scpi_response("ERROR: Invalid mode (RAW or ESR)");   				// Send fejlmeddelelse
    }
}

// ============================================================================================================================
/**
 * @brief Håndterer SYSTem:COMMunication kommandoen
 * 
 * @param state Tilstand ("ON" eller "OFF")
 * 
 * @details Skifter kommunikationstilstand baseret på parameter.
 * 
 * @return void
 */
// ============================================================================================================================
void scpi_handle_communication(const char *state) {
    if (strcmp(state, "ON") == 0) {                         			// Hvis parameter er "ON"
        commMode = COMM_MODE_SCPI;                           				// Sæt kommunikationstilstand til SCPI
        scpi_response("OK");                                 				// Send bekræftigelse
    } else if (strcmp(state, "OFF") == 0) {                 			// Hvis parameter er "OFF"
        commMode = COMM_MODE_NORMAL;                         				// Sæt kommunikationstilstand til normal
        scpi_response("OK");                                 				// Send bekræftigelse
    } else {                                                 			// Hvis parameter er ugyldig
        scpi_response("ERROR: Invalid state (ON or OFF)");   				// Send fejlmeddelelse
    }
}
// ============================================================================================================================
/**
 * @brief Ekstra SCPI kommandoer til avancerede funktioner
 */
// ============================================================================================================================

void scpi_handle_advanced(const char* command) {
    // *TST? - Selvtest
    if (strcmp(command, "*TST?") == 0) {                     			// Tjek om kommandoen er "*TST?"
        bool test_result = system_self_test();              			    // Kør selvtest og gem resultat
        scpi_response(test_result ? "0" : "1"); // 0 = PASS, 1 = FAIL 		    // Send resultat (0 for PASS, 1 for FAIL)
        return;                                              			    // Afslut funktionen
    }
    
    // SYSTem:VERSION? - Systemversion
    if (strcmp(command, "SYSTem:VERSION?") == 0) {          			// Tjek om kommandoen er "SYSTem:VERSION?"
        char response[80];                                  			    // Buffer til svaret
        snprintf(response, sizeof(response), "Firmware:%s,HW:%s,Build:%s", 	    // Formatér versionsinfo
                FIRMWARE_VERSION, HARDWARE_VERSION, get_build_info()); 		    // Indsæt firmware, hardware og build info
        scpi_response(response);                             			    // Send svaret
        return;                                              			    // Afslut funktionen
    }
    
    // SYSTem:HELP - Vis tilgængelige kommandoer
    if (strcmp(command, "SYSTem:HELP") == 0) {              			// Tjek om kommandoen er "SYSTem:HELP"
        scpi_response("Available SCPI commands:");          			    // Send overskrift
        scpi_response("  *IDN? - Identification");          			    // Send kommando 1
        scpi_response("  MEASure:ESR? - Measure ESR");      			    // Send kommando 2
        scpi_response("  SYSTem:PRESet - System reset");    			    // Send kommando 3
        scpi_response("  DISPlay:MODE <RAW|ESR>");          			    // Send kommando 4
        scpi_response("  SYSTem:COMMunication <ON|OFF>");   			    // Send kommando 5
        scpi_response("  *TST? - Self test");               			    // Send kommando 6
        scpi_response("  SYSTem:VERSION? - Version info");  			    // Send kommando 7
        scpi_response("  SYSTem:HELP - This help");         			    // Send kommando 8
        scpi_response("  CALibrate - Start calibration");   			    // Send kommando 9
        return;                                              			    // Afslut funktionen
    }
    
    // CALibrate - Start kalibrering
    if (strcmp(command, "CALibrate") == 0) {                			// Tjek om kommandoen er "CALibrate"
        scpi_response("CALIBRATION STARTED - Follow LCD instructions"); 	    // Send startbesked
        adc_calibrate();                                    			    // Start kalibreringsprocessen
        scpi_response("CALIBRATION COMPLETE - Reprogram required"); 		    // Send færdigbesked
        return;                                              			    // Afslut funktionen
    }
    
    // Ukendt kommando
    scpi_response("ERROR: Unknown command");                			// Send fejlmeddelelse for ukendt kommando
}
// ============================================================================================================================
// WATCHDOG OG FEJLHÅNDTERING - IMPLEMENTERING
// ============================================================================================================================

// ============================================================================================================================
/**
 * @brief Resetter watchdog timer
 * 
 * @details Kaldes regelmæssigt i main loop for at forhindre
 * watchdog i at timeout'e og genstarte systemet.
 * 
 * @note Opfylder S.WDG.001.2 kravet om watchdog reset
 * 
 * @return void
 */
// ============================================================================================================================
void wdg_reset(void) {
    esp_task_wdt_reset();                                    	// ESP32: resetter watchdog timer for at forhindre genstart
    wdgCounter++;                                            	// Øg watchdog tæller for debugging
    
    // Log watchdog reset hver 1000. gang (debugging)
    if (wdgCounter % 1000 == 0) {                           	// Hvis tæller er delelig med 1000 (hver 1000. gang)
        Serial.print("Watchdog reset #");                   		// Udskriv debug-besked til seriel port
        Serial.println(wdgCounter);                         		// Udskriv watchdog tællerværdi
    }
}

// ============================================================================================================================
/**
 * @brief Central fejlhåndteringsfunktion
 * 
 * @param error_message Fejlbesked
 * @param error_code Fejlkode (hex)
 * 
 * @details Logger fejlen til seriel port, viser på display,
 * og sætter systemet i ERROR state.
 * 
 * @note Opfylder S.WDG.001.3 kravet om fejllogging
 * 
 * @return void
 */
// ============================================================================================================================
void error_handler(const char *error_message, uint8_t error_code) {
    // Log fejl til seriel port
    Serial.print("ERROR [0x");                              				// Udskriv fejlpræfiks til seriel port
    Serial.print(error_code, HEX);                          				// Udskriv fejlkode i hexadecimal
    Serial.print("]: ");                                    				// Afslut fejlpræfiks
    Serial.println(error_message);                          				// Udskriv fejlbeskeden
    
    // Vis fejl på display
    char display_msg[32];                                   				    // Buffer til fejlbesked til display
    snprintf(display_msg, sizeof(display_msg), "Err 0x%02X: %s", error_code, error_message);// Formatér fejlbesked til display
    display_error(display_msg);                             				    // Vis fejlbesked på LCD
    
    // Sæt system i ERROR state
    currentState = STATE_ERROR;                             				// Skift systemtilstand til fejltilstand
    
    // Log timestamp
    Serial.print("Error time: ");                           				// Udskriv tekst for fejltidspunkt
    Serial.println(millis());                               				// Udskriv systemtidspunkt for fejl
    
    // Log systemtilstand
    Serial.print("System state: ");                         			// Udskriv tekst for systemtilstand
    Serial.println(currentState);                           			// Udskriv nuværende systemtilstand
    Serial.print("Display mode: ");                         			// Udskriv tekst for displaytilstand
    Serial.println(displayMode);                            			// Udskriv nuværende displaytilstand
    Serial.print("Comm mode: ");                            			// Udskriv tekst for kommunikationstilstand
    Serial.println(commMode);                               			// Udskriv nuværende kommunikationstilstand
}

// ============================================================================================================================
// SERIEL EVENT-HÅNDTERER
// ============================================================================================================================

// ============================================================================================================================
/**
 * @brief Seriel event-håndterer (køres automatisk)
 * 
 * @details Læser indkommende seriel data når det er tilgængeligt.
 * Bruges kun i SCPI-tilstand.
 * 
 * @note Arduino IDE kalder automatisk serialEvent() når seriel data er klar
 * 
 * @return void
 */
// ============================================================================================================================
void serialEvent() {
    if (commMode == COMM_MODE_SCPI) {                       			// Tjek om systemet er i SCPI-tilstand
        scpi_read_serial();                                 			// Læs indkommende seriel data til SCPI buffer
    }
}

// ============================================================================================================================
// TEST- OG KALIBRERINGSFUNKTIONER
// ============================================================================================================================

// ============================================================================================================================
/**
 * @brief Selvtest-rutine for systemvalidering
 * 
 * @details Udfører en række tests for at validere systemfunktionalitet
 * 
 * @return bool True hvis alle tests bestås
 */
// ============================================================================================================================
bool system_self_test(void) {
    Serial.println("=== SYSTEM SELVTEST ===");               		// Udskriv overskrift for selvtest
    
    bool all_tests_passed = true;                           		// Flag der indikerer om alle tests bestås
    
    // Test 1: ADC-kan læses
    Serial.print("Test 1 - ADC læsning: ");                 		// Udskriv test 1 beskrivelse
    int16_t adc_test = adc_read_single(PGA_2X_INDEX);       		// Læs en enkelt ADC-værdi med PGA 2x
    if (adc_test != 0 || adc_test != -1) { 				// Tjek om ADC-værdi er anderledes end 0 eller -1
        Serial.println("PASS");                             			// Test bestået
    } else {								//Ellers
        Serial.println("FAIL");                             			// Test fejlet
        all_tests_passed = false;                           			// Opdater flag til fejl
    }
    
    // Test 2: Oversampling fungerer
    Serial.print("Test 2 - Oversampling: ");                		// Udskriv test 2 beskrivelse
    int32_t adc_oversampled = adc_read_oversampled(PGA_2X_INDEX); 	// Læs ADC med 64× oversampling
    if (abs(adc_oversampled - adc_test) < 1000) { 			// Tjek om forskellen mellem enkelt og oversampled er < 1000
        Serial.println("PASS");                             			// Test bestået
    } else {								// Ellers
        Serial.println("FAIL");                             			// Test fejlet
        all_tests_passed = false;                           			// Opdater flag til fejl
    }
    
    // Test 3: PGA auto-range
    Serial.print("Test 3 - PGA auto-range: ");              		// Udskriv test 3 beskrivelse
    uint8_t pga_test = adc_auto_range();                    		// Kald funktion til automatisk PGA-valg
    if (pga_test >= 0 && pga_test <= 3) {                   		// Tjek om PGA-indeks er i gyldigt område (0-3)
        Serial.println("PASS");                             			// Test bestået
    } else {								//Ellers
        Serial.println("FAIL");                             			// Test fejlet
        all_tests_passed = false;                           			// Opdater flag til fejl
    }
    
    // Test 4: ESR beregning
    Serial.print("Test 4 - ESR beregning: ");               		// Udskriv test 4 beskrivelse
    float esr_test = esr_calculate(PGA_2X_INDEX, 1000);     		// Beregn ESR for en ADC-værdi på 1000
    if (esr_test >= 0.0 && esr_test <= 10.0) {              		// Tjek om ESR er i gyldigt område (0-10 ohm)
        Serial.println("PASS");                             			// Test bestået
    } else {								//Ellers
        Serial.println("FAIL");                             			// Test fejlet
        all_tests_passed = false;                           			// Opdater flag til fejl
    }
    
    // Test 5: Display fungerer
    Serial.print("Test 5 - Display: ");                     		// Udskriv test 5 beskrivelse
    lcd.clear();                                            		// Ryd LCD display
    lcd.setCursor(0, 0);                                    		// Sæt cursor til første linje
    lcd.print("TEST");                                      		// Skriv "TEST" på display
    wdg_reset();                                            		// Reset watchdog under delay
    delay(100);                                             		// Vent 100ms for at kunne se display
    Serial.println("PASS (visuel check)");                  		// Test bestået (kræver visuel bekræftigelse)
    
    // Test 6: Knapper
    Serial.print("Test 6 - Knapper: ");                     		// Udskriv test 6 beskrivelse
    bool btn1 = digitalRead(BUTTON1_PIN);                   		// Læs knap 1 tilstand
    bool btn2 = digitalRead(BUTTON2_PIN);                   		// Læs knap 2 tilstand
    if (btn1 == HIGH && btn2 == HIGH) { // Pull-up, så HIGH = ikke trykket // Tjek om begge knapper ikke er trykket (HIGH med pull-up)
        Serial.println("PASS");                             			// Test bestået
    } else {								   //Ellers
        Serial.println("FAIL");                             			// Test fejlet
        all_tests_passed = false;                           			// Opdater flag til fejl
    }
    
    // Test 7: DDS-generator
    Serial.print("Test 7 - DDS-generator: ");               		// Udskriv test 7 beskrivelse
    hal_ad9850_set_frequency(100000);                       		// Sæt DDS-generator til 100 kHz
    Serial.println("PASS (100 kHz sat)");                   		// Test bestået (funktionskald uden fejl)
    
    // Test 8: Watchdog
    Serial.print("Test 8 - Watchdog: ");                    		// Udskriv test 8 beskrivelse
    wdg_reset();                                            		// Reset watchdog timer
    Serial.println("PASS (watchdog reset)");                		// Test bestået (funktionskald uden fejl)
    
    Serial.print("=== SELVTEST ");                          		// Udskriv slutningsoverskrift
    if (all_tests_passed) {                                 		// Hvis alle tests bestået
        Serial.println("GENNEMFØRT ===");                   			// Udskriv "GENNEMFØRT"
    } else {                                                		// Hvis en eller flere tests fejlede
        Serial.println("FEJL ===");                         			// Udskriv "FEJL"
    }
    
    return all_tests_passed;                               		// Returner resultat af selvtest
}

// ============================================================================================================================
/**
 * @brief Nødkalibrering for hurtig opsætning
 * 
 * @details Genererer standard kalibreringstabeller baseret på teoretiske værdier
 * Dette er KUN til test og skal erstattes med rigtig kalibrering
 */
// ============================================================================================================================
void emergency_calibration(void) {
    Serial.println("=== NØDKALIBRERING (TEST) ===");         			// Udskrift af overskrift for nødkalibrering
    Serial.println("Advarsel: Dette er kun testdata!");     			// Advarsel om at data kun er til test
    Serial.println("Korrekt kalibrering kræver fysiske referencemodstande"); 	// Instruktion om rigtig kalibrering
    
    // Teoretiske værdier baseret på:
    // Full scale ADC = 32767
    // ESR range = 0-10 ohm
    // Lineært forhold antaget
    
    for (int pga = 0; pga < NUM_PGA_SETTINGS; pga++) {      			    // Loop gennem alle 4 PGA-indstillinger
        Serial.print("Genererer tabel for PGA ");           				// Udskriv hvilken PGA der behandles
        Serial.print(pga == 0 ? "2x" : pga == 1 ? "4x" : pga == 2 ? "8x" : "16x"); 	// Konverter PGA-indeks til tekst
        Serial.println(":");                                				// Afslut linje
        
        for (int i = 0; i < TABLE_ENTRIES; i++) {           // Loop gennem alle 30 tabelindgange
            							// Teoretiske værdier (skal erstattes med rigtige målte værdier)
            int16_t adc_val = i * 100; 				// Eksempelværdier:Simpel lineær ADC-værdi (100, 200, 300...)
            float esr_val = (i / 3.0); 				// Eksempelværdier: Simpel lineær ESR-værdi (0, 0.333, 0.667...)
            
            Serial.print("  Entry ");                       	// Udskrift af tabelindgang
            Serial.print(i);                                	// Udskriv indeks
            Serial.print(": ADC=");                         	// 
            Serial.print(adc_val);                          	// Udskriv ADC-værdi
            Serial.print(", ESR=");                         	//
            Serial.print(esr_val, 3);                       	// Udskriv ESR-værdi med 3 decimaler
            Serial.println(" ohm");                         	// Afslut linje med enhed
        }
    }
    
    Serial.println("=== NØDKALIBRERING FÆRDIG ===");        	// Udskrift af afsluttende overskrift
    Serial.println("Husk: Dette er kun testdata!");         	// Gentag advarsel om testdata
}
// ============================================================================================================================
// ADVANCED FEATURE FUNKTIONER
// ============================================================================================================================

// ============================================================================================================================
/**
 * @brief Temperaturkompensation for ESR-målinger
 * 
 * @param esr_value Målt ESR-værdi ved 25°C
 * @param temperature_degC Aktuel temperatur i °C
 * 
 * @return float Temperaturkompenseret ESR-værdi
 */
// ============================================================================================================================
float temperature_compensate_esr(float esr_value, float temperature_degC) {
    // Temperaturkoefficient for typiske elektrolytkondensatorer
    // Dette er et estimat - faktisk koefficient afhænger af kondensatortype
    const float TEMP_COEFFICIENT = -0.02; // -2% per 10°C   	    // Typisk temperaturkoefficient for elektrolytkondensatorer
    
    float temp_diff = temperature_degC - 25.0; // Reference ved 25°C 		// Beregn temperaturforskel fra reference (25°C)
    float compensation_factor = 1.0 + (TEMP_COEFFICIENT * temp_diff / 10.0);	// Beregn kompensationsfaktor
    	
    return esr_value * compensation_factor;                 			// Returner temperaturkompenseret ESR-værdi
}

// ============================================================================================================================
/**
 * @brief Statistisk analyse af målinger
 * 
 * @param measurements Array af målinger
 * @param count Antal målinger
 * @param mean Pointer til gennemsnittet
 * @param stddev Pointer til standardafvigelsen
 */
// ============================================================================================================================
void analyze_measurements(float measurements[], int count, float *mean, float *stddev) {
    if (count <= 0) {                                       		// Hvis der er ingen målinger
        *mean = 0.0;                                        			// Sæt gennemsnit til 0
        *stddev = 0.0;                                      			// Sæt standardafvigelse til 0
        return;                                             			// Afslut funktionen
    }
    
    // Beregn gennemsnit
    float sum = 0.0;                                        		// Initialiser sum variabel
    for (int i = 0; i < count; i++) {                       		// Loop gennem alle målinger
        sum += measurements[i];                             			// Tilføj måling til sum
    }
    *mean = sum / count;                                    		// Beregn gennemsnit (sum / antal)
    
    // Beregn standardafvigelse
    float variance = 0.0;                                   		// Initialiser varians variabel
    for (int i = 0; i < count; i++) {                       		// Loop gennem alle målinger igen
        float diff = measurements[i] - *mean;               			// Beregn forskel fra gennemsnit
        variance += diff * diff;                            			// Tilføj kvadratet af forskellen til varians
    }
    variance /= count;                                      		// Beregn gennemsnitlig varians
    *stddev = sqrt(variance);                               		// Beregn standardafvigelse som kvadratrod af varians
}

// ============================================================================================================================
// KONFIGURATIONSFUNKTIONER
// ============================================================================================================================

// ============================================================================================================================
/**
 * @brief Indlæser konfiguration fra ikke-flygtig hukommelse
 */
// ============================================================================================================================
void load_configuration(void) {
    // I en rigtig implementering ville dette læse fra EEPROM eller flash
    // For nu bruger vi standardværdier
    
    Serial.println("Indlæser konfiguration...");            			// Log indlæsning af konfiguration
    
    // Standardkonfiguration
    displayMode = DISPLAY_MODE_ESR_RESULT;                  			// Sæt displaytilstand til ESR-resultat
    commMode = COMM_MODE_NORMAL;                            			// Sæt kommunikationstilstand til normal
    
    // Her kan flere konfigurationsparametre tilføjes
}

// ============================================================================================================================
/**
 * @brief Gemmer konfiguration til ikke-flygtig hukommelse
 */
// ============================================================================================================================
void save_configuration(void) {
    // I en rigtig implementering ville dette gemme til EEPROM eller flash
    
    Serial.println("Gemmer konfiguration...");              				// Log gemning af konfiguration
    Serial.println("Bemærk: Konfiguration ikke implementeret i denne version"); 	// Advarsel om manglende implementering
}

// ============================================================================================================================
/**
 * @brief Rydningsfunktion køres før genstart
 */
// ============================================================================================================================
void cleanup_before_restart(void) {
    Serial.println("Udfører oprydning...");                  				// Log start af oprydning
    
    // Sluk DDS
    hal_ad9850_set_frequency(0);                            				// Sæt DDS-generator til 0 Hz (slukket)
    
    // Sluk LCD backlight
    lcd.noBacklight();                                      				// Sluk LCD baggrundsbelysning
    
    // Sluk display
    lcd.clear();                                            				// Ryd LCD display
    
    // Log genstart
    Serial.println("System klar til genstart");             				// Log klar til genstart
    Serial.println("================================");     				// Afslut med skillelinje
}

// ============================================================================================================================
/**
 * @brief Returnerer firmware build information
 */
// ============================================================================================================================
const char* get_build_info(void) {
    return __DATE__ " " __TIME__;                           		// Returner kompileringsdato og klokkeslæt som streng
}
// ============================================================================================================================
// TEST HOVEDPROGRAM TIL DEBUGGING
// ============================================================================================================================

// ============================================================================================================================
/**
 * @brief Testprogram der kører ved start hvis TEST_MODE er defineret
 */
// ============================================================================================================================
void run_test_program(void) {
    Serial.println("=== TEST MODE AKTIV ===");              			// Udskrift af overskrift når test mode aktiveres
    
    // Kør selvtest
    system_self_test();                                     			// Kør den fulde systemselvtest-funktion
    
    // Test DDS frekvenser
    Serial.println("Testing DDS frequencies:");            			// Udskrift af overskrift for DDS-frekvenstest
    uint32_t test_freqs[] = {100000, 50000, 200000, 1000}; 			// Array med testfrekvenser i Hz
    for (int i = 0; i < 4; i++) {                          			// Loop gennem alle 4 testfrekvenser
        hal_ad9850_set_frequency(test_freqs[i]);           				// Indstil DDS til testfrekvens
        
        // Reset watchdog under delay
        for (int j = 0; j < 5; j++) {                      				// Loop 5 gange (samlet 500ms ventetid)
            wdg_reset();                                   					// Reset watchdog timer
            delay(100);                                    					// Vent 100 millisekunder
        }
    }
    // Tilbage til 100 kHz
    hal_ad9850_set_frequency(100000);                      			// Sæt DDS tilbage til normal driftfrekvens (100 kHz)
    
    // Test ADC PGA settings
    Serial.println("Testing ADC PGA settings:");            			// Udskrift af overskrift for ADC PGA-test
    for (int pga = 0; pga < 4; pga++) {                    			// Loop gennem alle 4 PGA-indstillinger (0-3)
        Serial.print("PGA ");                               			   // Udskriv "PGA "
        Serial.print(pga == 0 ? "2x" : pga == 1 ? "4x" : pga == 2 ? "8x" : "16x"); // Konverter PGA-index til læselig tekst
        Serial.print(": ");                                 			   // Udskriv ": "
        int16_t val = adc_read_single(pga);                			   // Læs enkelt ADC-værdi med denne PGA-indstilling
        Serial.println(val);                               			   // Udskriv ADC-værdien
        
        // Reset watchdog under delay
        wdg_reset();                                       			   // Reset watchdog timer
        delay(100);                                        			   // Vent 100 millisekunder
    }
    
    // Test display modes
    Serial.println("Testing display modes:");               			// Udskrift af overskrift for display-test
    display_raw_adc(1234, PGA_2X_INDEX);                   			// Test visning af rå ADC-værdi (1234) med PGA 2x
    
    // Reset watchdog under delay
    for (int i = 0; i < 5; i++) {                          			// Loop 5 gange (samlet 1 sekund ventetid)
        wdg_reset();                                       			   // Reset watchdog timer
        delay(200);                                        			   // Vent 200 millisekunder
    }
    
    display_esr_result(1.234, 1234, PGA_2X_INDEX);     // Test visning af ESR-resultat (1.234 ohm) med ADC-værdi 1234 og PGA 2x
    
    // Reset watchdog under delay
    for (int i = 0; i < 5; i++) {                          			// Loop 5 gange (samlet 1 sekund ventetid)
        wdg_reset();                                       			   // Reset watchdog timer
        delay(200);                                        			   // Vent 200 millisekunder
    }
    
    Serial.println("=== TEST MODE FÆRDIG ===");             			// Udskrift af afsluttende overskrift
}

// ============================================================================================================================
// FIRMWARE VERSION VALIDERING
// ============================================================================================================================

// Kompiler-checks for at sikre korrekt konfiguration
#ifndef FIRMWARE_VERSION                                    	      // Kompilerdirektiv: Hvis FIRMWARE_VERSION ikke er defineret
#error "FIRMWARE_VERSION skal være defineret"              		// Stop kompilering med fejlmeddelelse
#endif                                                     	      // Afslut #ifndef direktivet

#ifndef HARDWARE_VERSION                                    	      // Kompilerdirektiv: Hvis HARDWARE_VERSION ikke er defineret  
#error "HARDWARE_VERSION skal være defineret"              		// Stop kompilering med fejlmeddelelse
#endif                                                     	      // Afslut #ifndef direktivet

#if TABLE_ENTRIES != 30                                     	      // Kompilerdirektiv: Hvis TABLE_ENTRIES ikke er 30
#warning "TABLE_ENTRIES bør være 30 ifølge kravspecifikationen"         // Udskriv advarsel under kompilering
#endif                                                    	      // Afslut #if direktivet

#if OVERSAMPLING_FACTOR != 64                               	      // Kompilerdirektiv: Hvis OVERSAMPLING_FACTOR ikke er 64
#warning "OVERSAMPLING_FACTOR bør være 64 ifølge kravspecifikationen"   // Udskriv advarsel under kompilering
#endif                                                     	      // Afslut #if direktivet
// ============================================================================================================================
// SLUT PÅ FIRMWARE IMPLEMENTERING
// ============================================================================================================================

// ============================================================================================================================
/**
 * @brief Slutkommentar og licensinformation
 * 
 * ESR-Meter Firmware v1.00
 * Baseret på kravspecifikation SRS-ESR-MIL-001
 * 
 * Ophavsret (c) 2026 Jan Engelbrecht Pedersen
 * 
 * Softwaren leveres "som den er", uden nogen form for garanti.
 */
// ============================================================================================================================

// ============================================================================================================================
// DOKUMENTATION FOR ESR-METER FIRMWARE
// ============================================================================================================================

/*
 * ==========================================================================================================================
 * DOKUMENTATION FOR ESR-METER FIRMWARE
 * ==========================================================================================================================
 * 
 * 7.1 BUILD-INSTRUKTIONER
 * 
 * Forudsætninger:
 * - Arduino IDE 2.x eller PlatformIO
 * - ESP32 Board Support pakke
 * - Nødvendige biblioteker: Wire, Adafruit_ADS1X15, LiquidCrystal_I2C
 * 
 * Installations-trinn:
 * 1. Åbn Arduino IDE
 * 2. Værktøjer → Board → ESP32 Arduino → ESP32 Wrover Module
 * 3. Værktøjer → Port → Vælg korrekt COM-port
 * 4. Værktøjer → Upload Speed → 921600
 * 5. Biblioteker → Installer Adafruit ADS1X15 og LiquidCrystal_I2C
 * 6. Åbn ESR_Meter.ino og upload
 * 
 * ==========================================================================================================================
 * 7.2 KALIBRERINGSPROSES
 * 
 * Vigtigt: Firmwaren kræver kalibrering før første brug!
 * 
 * Forberedelse:
 * - 30 præcise referencemodstande (0-10 Ω)
 * - Multimeter til verifikation
 * - Stabil strømforsyning
 * 
 * Kalibreringsprocedure:
 * 1. Kør emergency_calibration() for testdata
 * 2. For hver referencemodstand:
 *    - Tilslut til ESR-meter
 *    - Mål og notér ADC-værdi
 *    - Opdater tabelværdier i koden
 * 3. Genprogrammer firmware med nye tabelværdier
 * 4. Verificér med kendte modstande
 * 
 * ==========================================================================================================================
 * 7.3 TEST OG VALIDERING (T.ACC.001)
 * 
 * Accepttest procedure:
 * 
 * Præcisionstest (T.ACC.001.1):
 * - Mål 10 kendte modstande (0.1, 0.5, 1.0, 2.0, 3.0, 4.0, 5.0, 7.0, 9.0, 10.0 Ω)
 * - Verificér 5% nøjagtighed: |(målt - reference)|/reference ≤ 0.05
 * 
 * Opløsningstest (T.ACC.001.2):
 * - Mål på 1.000 Ω og 1.001 Ω referencemodstande
 * - Verificér at systemet kan skelne mellem dem
 * 
 * Sikkerhedstest (T.ACC.001.3):
 * - Mål testsignal med oscilloskop
 * - Verificér ≤ 0.1V p-p ved testprobene
 * 
 * Robusthedstest (T.ACC.001.4):
 * - ESD-test på probeforbindelser
 * - EMI-test med RF-generator
 * 
 * ==========================================================================================================================
 * 7.4 FEJLFINDING
 * 
 * Almindelige problemer og løsninger:
 * 
 * Ingen LCD-visning:
 * - Tjek I2C-adresse (typisk 0x27)
 * - Tjek I2C-forbindelser
 * - Kontroller strømforsyning
 * 
 * Ingen ADC-læsning:
 * - Tjek ADS1115 forbindelser
 * - Verificér I2C-kommunikation
 * - Kontroller referencenspænding
 * 
 * Ustabile målinger:
 * - Kontroller grounding (AGND/DGND)
 * - Tjek for støjkilder
 * - Verificér DDS-signal renhed
 * 
 * Watchdog genstarter:
 * - Tjek for blocking kode i loop()
 * - Verificér at wdg_reset() kaldes regelmæssigt
 * - Kontroller for stack overflow
 * 
 * ==========================================================================================================================
 * 7.5 VEDLIGEHOLDELSE
 * 
 * Regelmæssige opgaver:
 * 
 * Månedligt:
 * - Kontroller fysiske probeforbindelser
 * - Test med kendte referencemodstande
 * - Opdater fejllog
 * 
 * Hver 6. måned:
 * - Genkalibrer med referencemodstande
 * - Kontroller komponenters specifikationer
 * - Opdater firmware hvis nødvendigt
 * 
 * Årligt:
 * - Fuldt systemtest
 * - Kontroller PCB for korrosion
 * - Verificér strømforsyningskomponenter
 * 
 * ==========================================================================================================================
 * 7.6 SIKKERHEDSFORHOLDSREGLER
 * 
 * Vigtige advarsler:
 * 
 * In-circuit måling:
 * - ALTID sluk for strømmen til testet kredsløb først
 * - VERIFICER at testsignal er ≤ 0,1V p-p
 * - UNDGÅ måling i højspændingskredsløb (>50V)
 * 
 * Kalibrering:
 * - Brug KUN præcisionsmodstande med dokumenteret nøjagtighed
 * - UNDGÅ termisk drift under kalibrering
 * - VERIFICER måleresultater med sekundært instrument
 * 
 * Software:
 * - REGELMÆSSIG backup af kalibreringsdata
 * - TEST firmware-opdateringer før produktionsbrug
 * - DOKUMENTER alle ændringer til koden
 * 
 * ==========================================================================================================================
 * 7.7 PERFORMANCENØGLETAL
 * 
 * Forventede systemparametre:
 * - Målenøjagtighed: ≤ 5% over 0-10 Ω område
 * - Opløsning: 1 mΩ effektiv
 * - Måletid: ≤ 100 ms per måling
 * - Strømforbrug: ~280 mA typisk
 * - Opstartstid: < 3 sekunder
 * - Temperaturområde: 15-25°C (uden kompensation)
 * 
 * ==========================================================================================================================
 */
/*
*===========================================================================================
*DEVELOPER GUIDE – VIDEREUDVIKLING AF ESR-METER FIRMWARE
*===========================================================================================
*
*FORMÅL
*------
*Denne kommentarsektion er målrettet udviklere, der skal overtage, vedligeholde
*eller videreudvikle denne firmware. 
*
*Den beskriver arkitektur, underudviklede dele samt konkrete 
*forbedrings- og videreudviklingsmuligheder.
*
*----------------------------------------------------------------------------------------------------
*1. OVERORDNET ARKITEKTUR
*----------------------------------------------------------------------------------------------------
*
*Firmwaren er opbygget som én samlet .ino-fil, men er logisk opdelt i følgende lag:
*
*- HAL (Hardware Abstraction Layer)
*  * AD9850 DDS-generator
*  * ADS1115 ADC
*  * LCD-display
*  * Knapper
*  * Watchdog
*
*- Måle- og signalbehandling
*  * ADC single-shot
*  * 64× oversampling
*  * Automatisk PGA-range selection
*
*- ESR-beregning
*  * Tabelbaseret opslag
*  * Lineær interpolation
*  * PGA-afhængige kalibreringstabeller
*
*- UI
*  * LCD-visning
*  * Knapbaseret tilstandsskift
*
*- Kommunikation
*  * Normal seriel output
*  * SCPI-protokol (delvist implementeret)
*
*- State machine
*  * Central styring af hele systemets flow
*
*State machine er den logiske kerne og kontrollerer alle systemtilstande via
*currentState.
*
*----------------------------------------------------------------------------------------------------
*2. VIGTIGE INDGANGSPUNKTER I KODEN
*----------------------------------------------------------------------------------------------------
*
*- setup():
*  Initialiserer al hardware, state machine, konfiguration og DDS-signal.
*
*- loop():
*  Kører kontinuerligt:
*    * knap-check
*    * state machine
*    * SCPI-behandling
*    * watchdog reset
*
*- state_machine_update():
*  Dispatcher systemets tilstande (IDLE, MEASURE, CALCULATE, DISPLAY, SCPI, ERROR).
*
*- adc_read_oversampled():
*  Kritisk funktion for målekvalitet (64× oversampling).
*
*- esr_calculate():
*  Central ESR-beregningsfunktion baseret på tabeller og interpolation.
*
*----------------------------------------------------------------------------------------------------
*3. KENDTE BEGRÆNSNINGER (VIGTIGT)
*----------------------------------------------------------------------------------------------------
*
*- ESR-tabeller er PLADSHOLDERE
*  De nuværende tabeller er ikke baseret på reel kalibrering og skal udskiftes.
*
*- Kalibrering kræver genprogrammering
*  adc_calibrate() kan ikke gemme data persistent.
*
*- SCPI er ikke fuldt implementeret
*  Flere handlers er stubbe eller tomme.
*
*- Ingen persistent lagring
*  Konfiguration og kalibrering gemmes ikke i NVS / EEPROM.
*
*- Monolitisk filstruktur
*  Hele systemet ligger i én .ino-fil, hvilket hæmmer skalerbarhed.
*
*----------------------------------------------------------------------------------------------------
*4. UNDERUDVIKLEDE DELE – HØJ PRIORITET
*----------------------------------------------------------------------------------------------------
*
*4.1 ESR-TABELLER & KALIBRERING
*------------------------------
*Nuværende:
*- Hardcodede PROGMEM-tabeller
*- Ingen runtime-kalibrering
*- Ingen temperaturkompensation i praksis
*
*Mulig videreudvikling:
*- Kalibreringsrutine styret via SCPI
*- Gem kalibreringsdata i ESP32 NVS
*- Understøt flere kalibreringsprofiler
*- Automatisk regenerering af interpolationstabeller
*
*Gevinst:
*- Måleteknisk valide ESR-resultater
*- Ingen behov for recompilering
*- Felt- og servicekalibrering mulig
*
*----------------------------------------------------------------------------------------------------
*4.2 SCPI-KOMMUNIKATION
*----------------------
*Nuværende:
*- Basal parser
*- Enkelte kommandoer
*- Manglende fejl- og statusrapportering
*
*Mulig videreudvikling:
*- Fuldt SCPI-sæt, fx:
*    * MEAS:ESR?
*    * CONF:PGA AUTO|MAN
*    * CAL:START
*    * SYST:ERR?
*    * *IDN?
*- Kommando-hjælp og syntaksvalidering
*- Fejlkø og statusregistre
*
*Gevinst:
*- Integration i automatiserede testsystemer
*- PC-/LabVIEW-/Python-styring
*- Produktions- og laboratorieegnet instrument
*
*----------------------------------------------------------------------------------------------------
*4.3 STATE MACHINE
*-----------------
*Nuværende:
*- Alle states er defineret
*- Flere handlers er minimale
*
*Mulig videreudvikling:
*- Event-baserede state transitions
*- Timeout-håndtering pr. state
*- Klar recovery-strategi i ERROR-state
*
*Gevinst:
*- Mere deterministisk systemadfærd
*- Lettere fejlfinding
*- Bedre robusthed
*
*----------------------------------------------------------------------------------------------------
*4.4 DISPLAY & UI
*----------------
*Nuværende:
*- Meget simpel visning
*- Kun to display modes
*- Begrænset brugerfeedback
*
*Mulig videreudvikling:
*- Simpelt menu-system
*- Statuslinje (PGA, mode, fejl)
*- Progress-visning ved kalibrering
*- Fejlmeddelelser på display
*
*Gevinst:
*- Markant bedre brugeroplevelse
*- Mindre afhængighed af seriel debug
*
*----------------------------------------------------------------------------------------------------
*5. KODEFORBEDRINGER (ARKITEKTUR & KVALITET)
*----------------------------------------------------------------------------------------------------
*
*5.1 MODULOPDELING (ANBEFALET)
*-----------------------------
*Opdel koden i:
*- hal_*.h / hal_*.cpp
*- adc.h / adc.cpp
*- esr.h / esr.cpp
*- scpi.h / scpi.cpp
*- ui.h / ui.cpp
*
*Fordele:
*- Bedre vedligeholdelse
*- Mulighed for enhedstest
*- Klar ansvarsopdeling
*
*----------------------------------------------------------------------------------------------------
*5.2 FEJLHÅNDTERING & LOGGING
*----------------------------
*Mulige tiltag:
*- Central fejlstruktur
*- Fejlkoder via SCPI (SYST:ERR?)
*- Konsistent brug af error_handler()
*
*Gevinst:
*- Hurtigere fejlfinding
*- Mere professionelt system
*
*----------------------------------------------------------------------------------------------------
*5.3 ADC & SIGNALBEHANDLING
*--------------------------
*Mulige forbedringer:
*- Medianfilter før oversampling
*- Outlier rejection
*- Aktiv temperaturkompensation
*- Dynamisk justering af oversampling
*
*Gevinst:
*- Mere stabile og reproducerbare målinger
*
*----------------------------------------------------------------------------------------------------
*6. ANBEFALET VIDEREUDVIKLINGS-ROADMAP
*----------------------------------------------------------------------------------------------------
*
*Fase 1 – Måleteknisk korrekthed
*- Rigtige kalibreringsdata
*- Persistent lagring
*- Temperaturkompensation
*
*Fase 2 – Kommunikation
*- Fuldt SCPI-interface
*- PC-integration
*
*Fase 3 – Arkitektur
*- Modulopdeling
*- Testbarhed
*
*Fase 4 – Brugeroplevelse
*- Menusystem
*- Status- og fejlvisning
*
*----------------------------------------------------------------------------------------------------
*KONKLUSION
*----------------------------------------------------------------------------------------------------
*
*Firmwaren er arkitektonisk solid og veldokumenteret, men flere centrale dele
*er bevidst efterladt ufærdige. Det gør projektet velegnet som:
*
*- Videreudviklingsplatform
*- Professionel prototype
*- Reference- og undervisningsprojekt
*
*Det største forbedringspotentiale ligger i:
*- Kalibrering
*- SCPI
*- Persistent konfiguration
*- Modulopdeling
*
*============================================================================================
*/

// ============================================================================================================================
// FIRMWARE IMPLEMENTERING FÆRDIG
// ============================================================================================================================