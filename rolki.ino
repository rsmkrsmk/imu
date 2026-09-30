/*
  RolkiIMU_Async_Logger_Export_v3.ino — M5StickS3 / BMI270
  Logger terenowy v3: zadanie IMU o wysokim priorytecie + kolejka RAM + writerTask.

  Arduino IDE:
  - Board: M5Stack -> M5StickS3
  - Flash Size: 8MB (64Mb)
  - Partition Scheme: 8M with spiffs (3MB APP/1.5MB SPIFFS)
  - Erase All Flash Before Sketch Upload: Disabled
  - M5Stack Board Manager 3.3.8

  Uwaga: M5Unified nie udostepnia stabilnego publicznego FIFO BMI270.
  Wersja v3 stosuje bezpieczna alternatywe: zadanie IMU 200 Hz, kolejke RAM
  i osobne zadanie zapisu. QDROP informuje o ewentualnej utracie rekordu.
*/

#include <M5Unified.h>
#include <FS.h>
#include <LittleFS.h>
#include <WiFi.h>
#include <WebServer.h>
#include <esp_partition.h>
#include <mbedtls/base64.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <math.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

// ---------------------------------------------------------------------------
// Konfiguracja: osie, zapis, zadania i algorytmy
// ---------------------------------------------------------------------------
const uint16_t UI_RED = TFT_RED;
const uint16_t UI_DARK_RED = 0x5800;
const uint16_t UI_WHITE = TFT_WHITE;

// Montaz referencyjny: X+ do przodu, Y+ do nieba, Z+ w lewo.
// Kanaly logiczne F/L/U sa mapowane przed wszystkimi algorytmami i zapisem.
const int FORWARD_AXIS = 0;    // czujnik X+ -> przod / kierunek jazdy
const int FORWARD_SIGN = +1;
const int LATERAL_AXIS = 2;    // czujnik Z+ -> lewa strona
const int LATERAL_SIGN = +1;
const int VERTICAL_AXIS = 1;   // czujnik Y+ -> niebo / gora
const int VERTICAL_SIGN = +1;
const int ROLL_GYRO_AXIS = 0; // obrot wokol osi przod-tyl
const int ROLL_GYRO_SIGN = +1;

const char* LITTLEFS_PARTITION_LABEL = "spiffs";
const char* LITTLEFS_MOUNT_PATH = "/littlefs";
// Sesje NORMAL korzystaja z PNT/RIMU04. RAW LAB ma osobny, rozszerzony
// format RRAW02 (.rwl).
const char* SESSION_FILE_EXTENSION = ".pnt";
const char* RAW_LAB_FILE_EXTENSION = ".rwl";
const char* LEGACY_SESSION_FILE_EXTENSION = ".rim";
// Archiwum RLE po zapisie sesji: .pz dla PNT/RIM, .rz dla RAW LAB.
const char* COMPRESSED_EXTENSION = ".pz";
const char* COMPRESSED_RAW_EXTENSION = ".rz";
// Pakowanie strumieniowe "w locie" (blokowy LZSS): osobne rozszerzenia plikow,
// aby odroznic je od archiwow po fakcie. .pzs dla NORMAL, .rzs dla RAW LAB.
const char* STREAM_COMPRESSED_EXTENSION = ".pzs";
const char* STREAM_COMPRESSED_RAW_EXTENSION = ".rzs";
const uint32_t RAW_LAB_MIN_FREE_BYTES_TO_START = 192UL * 1024UL;
const uint8_t LITTLEFS_MAX_OPEN_FILES = 10;
const uint32_t SERIAL_BAUD = 115200;

// Lokalny panel telefonu: urządzenie jest wlasnym punktem dostepowym,
// bez polaczenia z Internetem. Zmien haslo przed uzyciem poza treningiem.
const char* WIFI_AP_SSID = "Rolki-Silesia-IMU";
const char* WIFI_AP_PASSWORD = "SilesiaIMU2026";  // WPA2, minimum 8 znakow
const IPAddress WIFI_AP_IP(192, 168, 4, 1);
const IPAddress WIFI_AP_GATEWAY(192, 168, 4, 1);
const IPAddress WIFI_AP_SUBNET(255, 255, 255, 0);
const uint8_t WIFI_AP_MAX_CLIENTS = 1;

const uint8_t SPEAKER_VOLUME = 96;          // 0..255, domyslna glosnosc treningowa
const uint8_t SPEAKER_VOLUME_MAX = 255;
const uint32_t SPLASH_DURATION_MS = 5000;
const uint32_t SAVED_TONE_DURATION_MS = 650;
const float COUNTDOWN_TONE_HZ = 1320.0f;
const uint32_t COUNTDOWN_BEEP_MS = 70;
const uint32_t COUNTDOWN_PERIOD_MS = 1000;
const uint32_t START_BANNER_MS = 350;
// Kalibracja sesji trwa podczas pierwszych trzech sekund odliczania: 10, 9, 8.
const uint32_t CALIBRATION_MS = 3000;
const uint8_t CALIBRATION_COUNTDOWN_STEPS = 3;
const uint16_t CALIBRATION_MIN_STABLE_SAMPLES = 80;
const float CALIBRATION_STABLE_ACC_TOLERANCE_G = 0.12f;
const float CALIBRATION_STABLE_GYRO_DPS = 15.0f;
const uint32_t CALIBRATION_RETRY_NOTICE_MS = 1200;
const uint32_t CALIBRATION_FAST_BEEP_MS = 36;
const uint32_t DISPLAY_PERIOD_MS = 250;
const uint32_t MIN_FREE_BYTES_TO_START = 128UL * 1024UL;
// Historia do wykresow telefonu: jedna probka na sekunde, ostatnie 120 s.
// Nie jest zapisywana do pliku i nie zmienia formatu PNT/RIMU04.
const uint16_t LIVE_HISTORY_POINTS = 120;
const uint32_t LIVE_HISTORY_PERIOD_US = 1000000UL;
const uint16_t WEB_FILE_LIST_LIMIT = 80;
const uint32_t MOUNT_SNAPSHOT_PERIOD_MS = 100;
// Odczyt PMIC jest celowo rzadki i nie jest wykonywany podczas rejestracji,
// aby nie dodawac operacji I2C konkurujacej z zadaniem IMU 200 Hz.
const uint32_t BATTERY_TELEMETRY_PERIOD_MS = 30000UL;
const uint32_t BATTERY_ESTIMATE_MIN_WINDOW_MS = 60000UL;
const size_t EXPORT_RAW_CHUNK_BYTES = 192;

// 512 * 23 B = 11 776 B, czyli okolo 2,56 s zapasu przy 200 Hz.
const uint16_t SAMPLE_QUEUE_LENGTH = 512;
const uint16_t WRITER_BLOCK_RECORDS = 64;
const uint32_t IMU_PERIOD_MS = 5;                 // 200 Hz
// Alarm diagnostyczny, nie błąd po pojedynczym braku data-ready BMI270.
const uint32_t IMU_STALL_THRESHOLD_US = 500000UL;
const uint32_t IMU_STACK_SAMPLE_PERIOD_US = 1000000UL;
const UBaseType_t IMU_TASK_PRIORITY = 3;
const UBaseType_t WRITER_TASK_PRIORITY = 1;
// Writer ma zapas na LittleFS i finalizacje pliku. Duze bloki zapisu sa globalne,
// aby nie zajmowaly stosu tasku w chwili startu sesji.
const uint32_t WRITER_TASK_STACK_BYTES = 8192;
const BaseType_t IMU_TASK_CORE = 1;
const BaseType_t WRITER_TASK_CORE = 0;

const float MIN_DT_S = 0.002f;
const float GAP_DT_S = 0.008f;
const float BAD_DT_S = 0.050f;
const float HP_CUTOFF_HZ = 1.0f;        // filtr gornoprzepustowy przyspieszen (poprawka 2.5: 20->1 Hz, aby nie tlumic pasma odepchniecia ~2-5 Hz)
const float SURGE_LP_CUTOFF_HZ = 8.0f;  // wygładzanie uderzeniowości (surge)
const float LIVE_INTENSITY_CUTOFF_HZ = 1.5f; // wygładzanie intensywności wyświetlanej

// ---------------------------------------------------------------------------
// ODPCHNIĘCIE (STROKE) v4 — żesplan metrук techniki jazdy na rolkach.
// Liczymy siłę odepchnięcia (STROKE_STRENGTH 0-100), kadencję (CADENCE) i moc proxy.
// ---------------------------------------------------------------------------
// Wejście do kandydata: uderzeniowość (surge) lub przyspieszenie hp w przód.
// Progi dobrane na danych (ses00038/40, HP=1.0 Hz) tak, aby kadencja per-lyzwa
// wynosila ~70/min zamiast ~130 (eliminacja podwojnego liczenia impulsow).
const float STROKE_ENTER_SURGE_GPS = 20.0f;   // prog uderzeniowosci na wejsciu (g/s)
const float STROKE_ENTER_HP_G = 0.80f;        // prog przysp. hp na wejsciu (g)
// Potwierdzenie w oknie ~60-100 ms: minimum uderzeniowosc albo szczyt hp.
const float STROKE_CONFIRM_SURGE_GPS = 40.0f;
const float STROKE_CONFIRM_HP_G = 1.30f;
const uint32_t STROKE_REFRACTORY_US = 450000; // min. odstep miedzy odepchnieciami (rytm)
const uint32_t STROKE_WINDOW_US = 120000;     // max. czas okna kandydata (potwierdzenie do ~120 ms)
const uint8_t STROKE_CADENCE_WINDOW = 12;     // liczba odepchnięć do średniej ruchomej kadencji

// --- Kadencja z RYTMU (autokorelacja) — dokladniejsza niz zliczanie odepchniec ---
// Obwiednia surge probkowana CADENCE_ENV_HZ, bufor kolowy CADENCE_BUF (~6 s),
// autokorelacja skanuje lag od MIN do MAX (okres cyklu odepchniecia jednej nogi).
// Wynik x2 = kadencja KROKU (obie nogi), porownywalna z norma 55-80/min.
const uint8_t CADENCE_ENV_HZ = 20;            // czestotliwosc obwiedni (probka co 50 ms)
const uint16_t CADENCE_ENV_PERIOD_US = 50000; // 1/20 s
const uint8_t CADENCE_BUF = 120;              // 6 s historii obwiedni
const uint8_t CADENCE_LAG_MIN = 8;            // 0.40 s (okres nogi min -> szybka jazda)
const uint8_t CADENCE_LAG_MAX = 44;           // 2.20 s (okres nogi max -> wolna jazda)
const uint32_t CADENCE_CALC_PERIOD_US = 1000000; // przeliczaj raz na sekunde
const float CADENCE_SUBHARM_RATIO = 0.80f;    // korekta: gdy 2*lag koreluje >80% peak
const uint8_t CADENCE_MEDIAN_WINDOW = 5;      // mediana z N ostatnich okien (stabilizacja)

// Siła odepchnięcia: suma ważona szczytu przysp., całki impulsu i uderzeniowości.
const float STROKE_W_PEAK = 0.40f;            // waga szczytu przyspieszenia w przód
const float STROKE_W_IMPULSE = 0.30f;         // waga całki impulsu (pomiar "pędu")
const float STROKE_W_SURGE = 0.30f;           // waga maksymalnej uderzeniowości
// Referencje normalizacji dobrane na danych (HP=1.0 Hz): typowe odepchniecie ~45-60,
// mocne ~85-100. Poprzednie wartosci (0.55/0.05/45) zanizaly i zapychaly skale.
const float STROKE_REF_PEAK_G = 5.0f;         // referencja szczytu przysp. hp (g)
const float STROKE_REF_IMPULSE_GS = 0.30f;    // referencja calki impulsu (g*s)
const float STROKE_REF_SURGE_GPS = 130.0f;    // referencja uderzeniowosci (g/s)

// INTENSITY v4: względny wskaźnik wysiłku (0-100), z niższym klipingiem niż v3 —
// rzadkie szczyty nie "zapychają" skali (referencje niższe niż w v3).
const float INTENSITY_ACC_REF_G = 0.90f;      // referencja przyspieszenia (g)
const float INTENSITY_SURGE_REF_GPS = 90.0f;  // referencja uderzeniowości (g/s)
const float INTENSITY_ACC_WEIGHT = 0.55f;     // waga przysp. w intensywności
const float INTENSITY_SURGE_WEIGHT = 0.45f;   // waga uderzeniowości w intensywności

// Strefy intensywności liczone wg SIŁY ODPCHNIĘCIA (strefy B):
// Z0: 0-15, Z1: 15-30, Z2: 30-50, Z3: 50-70, Z4: 70-100.
const float ZONE0_MAX = 15.0f;
const float ZONE1_MAX = 30.0f;
const float ZONE2_MAX = 50.0f;
const float ZONE3_MAX = 70.0f;
const float ZONE4_MAX = 100.0f;
// Odepchniecie uznajemy za SILNE, gdy sily chwilowa >= progu.
const float STROKE_STRONG_MIN = 15.0f;

const float ACTIVE_INTENSITY_THRESHOLD = 10.0f;
const float ACTIVE_GYRO_THRESHOLD_DPS = 60.0f;

// Wzgledny przechyl: ograniczony, aby nie udawac kata absolutnego.
const float REL_LEAN_LIMIT_DEG = 75.0f;
// Poprawka 2.7: mocniejsza korekta grawitacyjna (bazowa 0.004->0.010) ogranicza
// akumulacje dryfu zyroskopu podczas dlugiej jazdy; MAX bez zmian (bezruch).
const float LEAN_BASE_CORRECTION = 0.010f;
const float LEAN_MAX_CORRECTION = 0.030f;
// Poprawka 2.7: powolny zanik integratora przechylu przy bezruchu (na sekunde),
// zeruje dryf skumulowany w postojach bez udawania kata absolutnego.
const float LEAN_IDLE_DECAY_PER_S = 0.5f;

// Strona rolki: czujnik zamontowany na prawej rolce (R=2) do czasu ustawienia.
const uint8_t DEVICE_SIDE = 2;               // 0=nieokreslona, 1=lewa, 2=prawa
const char* DEVICE_SIDE_NAME = "R";          // nazwa strony do naglowka/dashboardu

// ---------------------------------------------------------------------------
// Format NORMAL: magic "RIMU04", formatVersion=4, rekord 32 B (SampleRecord).
// RAW LAB:       magic "RRAW02", formatVersion=4, rekord 48 B (RawLabRecord).
// Obie generacje v4 dziela ten sam naglowek 64 B; formatVersion=4 dla obu, a
// format rozroznia magic ("RIMU04"/"RRAW02"). Rozmiar rekordu jest zapisany
// jawnie w naglowku (pole recordBytes) dla obu formatow (poprawka 2.1).
// Stare pliki RIMU03/RRAW01 (.pnt/.rwl) obsluguje wylacznie zewnetrzny parser.
// ---------------------------------------------------------------------------
struct __attribute__((packed)) SessionHeader {
  char magic[8];
  uint16_t formatVersion;
  uint16_t headerBytes;
  uint32_t sessionId;
  uint32_t startMillis;
  uint16_t targetRateHz;
  int8_t forwardAxis, forwardSign;
  int8_t lateralAxis, lateralSign;
  int8_t verticalAxis, verticalSign;
  int8_t rollGyroAxis, rollGyroSign;
  float gyroBiasX, gyroBiasY, gyroBiasZ;
  uint32_t recordBytes;  // rozmiar pojedynczego rekordu w bajtach (poprawka 2.1: dawne "reserved", teraz wypelniane dla obu formatow)
  // --- meta v4 ---
  uint8_t side;          // 0=nieokreslona, 1=lewa, 2=prawa
  uint8_t zoneVersion;   // 2 = strefy wg sily odepchniecia (strefy B)
  uint8_t reservedByte;
  uint8_t reservedByte2;
  float refPeakG;        // referencja szczytu przysp. dla sily (STROKE_REF_PEAK_G)
  float refImpulseGs;    // referencja calki impulsu (STROKE_REF_IMPULSE_GS)
  float refSurgeGps;     // referencja uderzeniowosci (STROKE_REF_SURGE_GPS)
  uint8_t pad0, pad1;    // wyrownanie do 64 B
};

// PNT v4 (RIMU04) - rekord 32 B.
struct __attribute__((packed)) SampleRecord {
  uint32_t tUs;
  int16_t af_mg, al_mg, au_mg;
  int16_t gf_dps10, gl_dps10, gu_dps10;
  uint16_t accNorm_mg;          // przeciążenie (g*1000)
  int16_t lean_cdeg;            // przechyl wzgledny (cdeg, +/-75 st)
  uint16_t intensity_x10;       // INTENSITY chwilowa 0..100 (x10)
  uint16_t strokeStrength_x10;  // SILA odepchniecia 0..100 (x10)
  uint16_t cadenceMs;           // aktualny rytm kadencji (ms)
  int16_t surge_dps10;          // uderzeniowosc (g/s *10)
  uint8_t strokePhaseMs;        // czas TRWANIA aktywnej fazy odepchniecia (ms) — poprawka 2.8
  uint8_t flags;
  uint8_t side;                 // strona rolki (0/1/2)
  uint8_t reserved;
};

// RRAW02 zachowuje osie fizyczne zwracane przez M5Unified/BMI270 bez mapowania
// F/L/U i bez odejmowania biasu. Dodatkowo zawiera dokladnie ten sam slad
// logiczny (SILA, INTENSITY, CADENCE, SURGE, LEAN), ktory robi firmware v4.
struct __attribute__((packed)) RawLabRecord {
  uint32_t tUs;
  int16_t rawAx_mg, rawAy_mg, rawAz_mg;
  int16_t rawGx_dps10, rawGy_dps10, rawGz_dps10;
  int16_t af_mg, al_mg, au_mg;
  int16_t gf_dps10, gl_dps10, gu_dps10;
  uint16_t accNorm_mg;
  int16_t lean_cdeg;
  uint16_t intensity_x10;
  uint16_t strokeStrength_x10;
  uint16_t cadenceMs;
  int16_t surge_dps10;
  int16_t intensitySmooth_x10;
  uint8_t strokePhaseMs;
  uint8_t flags;
  uint8_t side;
  uint8_t pad0;                 // wyrownanie do 48 B
  uint16_t reserved;            // wyrownanie do 48 B
};

static_assert(sizeof(SessionHeader) == 64, "SessionHeader size mismatch");
static_assert(sizeof(SampleRecord) == 32, "SampleRecord size mismatch");
static_assert(sizeof(RawLabRecord) == 48, "RawLabRecord size mismatch");

// Flagi rekordu v4.
const uint8_t FLAG_IDLE = 1 << 0;          // bezzruch / cisza (dawne QUASI_STATIC)
const uint8_t FLAG_STROKE = 1 << 1;        // zatwierdzone odepchniecie (szczyt)
const uint8_t FLAG_STROKE_STRONG = 1 << 2; // odepchniecie SILNE (sila >= 15)
const uint8_t FLAG_SATURATION = 1 << 3;    // nasycenie czujnika
const uint8_t FLAG_TIME_GAP = 1 << 4;      // przerwa w probkowaniu

// ---------------------------------------------------------------------------
// Stany i filtry
// ---------------------------------------------------------------------------
enum SessionMode : uint8_t {
  SESSION_MODE_NORMAL,
  SESSION_MODE_RAW_LAB
};

enum DeviceState : uint8_t {
  STATE_BOOT,
  STATE_SPLASH,
  STATE_MAIN_MENU,
  STATE_VOLUME_SELECT,
  STATE_START_COUNTDOWN,
  STATE_FILE_MANAGER,
  STATE_DELETE_ONE_SELECT,
  STATE_DELETE_ONE_CONFIRM,
  STATE_DELETE_ALL_CONFIRM,
  STATE_RECORDING,
  STATE_STOPPING,
  STATE_SAVED,
  STATE_EXPORT_SELECT,
  STATE_EXPORTING,
  STATE_MOUNT_GUIDE,
  STATE_DIAGNOSTICS,
  STATE_STORAGE_INIT_REQUIRED,
  STATE_ERROR
};

struct OnePoleLowPass {
  float y = 0.0f;
  bool initialized = false;
  void reset(float initial = 0.0f) { y = initial; initialized = false; }
  float update(float x, float dt, float cutoffHz) {
    if (!initialized) { y = x; initialized = true; return y; }
    const float rc = 1.0f / (2.0f * PI * cutoffHz);
    const float alpha = dt / (rc + dt);
    y += alpha * (x - y);
    return y;
  }
};

struct OnePoleHighPass {
  float y = 0.0f;
  float prevX = 0.0f;
  bool initialized = false;
  void reset() { y = 0.0f; prevX = 0.0f; initialized = false; }
  float update(float x, float dt, float cutoffHz) {
    if (!initialized) { prevX = x; initialized = true; return 0.0f; }
    const float rc = 1.0f / (2.0f * PI * cutoffHz);
    const float alpha = rc / (rc + dt);
    y = alpha * (y + x - prevX);
    prevX = x;
    return y;
  }
};

// Punkt historii live (1 Hz) — v4: metryki techniki.
struct LiveHistoryPoint {
  uint16_t elapsedSeconds;
  uint8_t intensity;        // INTENSITY 0..100 (chwilowa)
  uint8_t strokeStrength;   // SILA odepchniecia 0..100
  uint16_t cadenceMs;       // aktualny rytm kadencji (ms)
  uint16_t motionDps;       // ruch obrotowy (spin, dps)
  int16_t leanCdeg;         // przechyl wzgledny (cdeg)
  uint8_t event;            // bylo odepchniecie w tej sekundzie?
  uint8_t strokePhaseMs;    // czas impulsu odepchniecia (ms)
  uint8_t side;             // strona rolki (0/1/2)
  uint8_t reserved;
};

struct MountSnapshot {
  float forwardG;
  float lateralG;
  float verticalG;
  bool valid;
};

// Akumulatory bieżącej kalibracji sesji. Dane trafiają wyłącznie do RAM;
// wynik jest zapisywany w nagłówku jako bias żyroskopu nowej sesji.
struct CountdownCalibration {
  double gyroSumX;
  double gyroSumY;
  double gyroSumZ;
  double accelSumX;
  double accelSumY;
  double accelSumZ;
  uint32_t stableSamples;
  uint32_t rejectedSamples;
  uint8_t retryCount;
  bool complete;
  bool lastAttemptMoved;
};

// Migawka PMIC wykorzystywana tylko przez LCD/HTTP. Nie trafia do pliku sesji.
struct PowerSnapshot {
  int16_t levelPercent;
  int16_t voltageMv;
  bool valid;
  bool charging;
  bool chargeKnown;
  bool estimateValid;
  uint16_t estimateMinutes;
  float dischargePctPerHour;
  uint32_t lastUpdatedMs;
};

// Statystyki sesji v4 — wszystkie metryki techniki jazdy.
struct RuntimeStats {
  // Jakość zapisu / niskopoziomowa diagnostyka.
  uint32_t samplesAcquired;
  uint32_t recordsQueued;
  uint32_t recordsWritten;
  uint32_t gap8Count;
  uint32_t badGapCount;
  uint32_t saturationCount;
  uint32_t qDropCount;
  uint32_t writerErrorCount;
  uint32_t imuStallCount;
  uint32_t imuStackMinFreeBytes;
  uint32_t writerStackMinFreeBytes;
  uint32_t firstSampleUs;
  uint32_t lastSampleUs;
  uint32_t maxQueueDepth;

  // Odepchniecia (STROKE).
  uint32_t strokeCount;          // zatwierdzone odepchniecia (wszystkie)
  uint32_t strokeStrongCount;    // z tego SILNE (sila >= 15)
  uint32_t strokeSum100;         // suma sil (do sredniej)
  uint32_t strokeMax100;         // max sila
  uint32_t strokePhaseSumMs;     // suma czasow impulsow
  uint32_t strokePhaseMaxMs;     // max czas impulsu
  uint32_t cadenceMsAccum;       // suma rytmow (ms) do sredniej
  float strokeStrengthNow;       // sila ostatniego odepchniecia (0..100)

  // Kadencja i moc.
  uint32_t cadenceMsNow;         // aktualny rytm (sr. ruchoma, ms)
  float cadencePerMin;           // kadencja /min (srednia ruchoma, ze zliczania odepchniec)
  float strideCadencePerMin;     // KADENCJA KROKU (obie nogi) z rytmu/autokorelacji
  float legCadencePerMin;        // kadencja tej nogi z rytmu (stride/2)
  float powerProxyNow;           // moc proxy = kadencja * sila / wspolczynnik
  float powerProxyMax;

  // Intensywnosc.
  float intensityNow;            // INTENSITY chwilowa (0..100)
  float intensityAverage;
  float intensityMax;

  // Inne metryki.
  float surgeNow;                // uderzeniowosc (g/s)
  float surgeMax;
  float spinNow;                 // ruch obrotowy (dps)
  float gloadNow;                // przeciążenie (g)
  float relLeanDeg;              // przechyl wzgledny

  // Czas aktywny / bezzruchowy.
  uint32_t activeSamples;
  uint32_t idleSamples;          // dawna quasi-static

  // Strefy wg siły odepchniecia (strefy B): 0..4.
  uint32_t zone0Count, zone1Count, zone2Count, zone3Count, zone4Count;

  // Histogramy 0..100 do wyliczenia P90/P95 (usw wa human).
  uint32_t intensityHist[101];
  uint32_t strokeHist[101];
};

DeviceState state = STATE_BOOT;
DeviceState lastDrawnState = STATE_ERROR;

File logFile;
// Bufory writerTask celowo sa globalne. Dwa lokalne bloki (64 x 40 B i 64 x 23 B)
// przekraczaly bezpieczny margines stosu i powodowaly reset tuz po START.
RawLabRecord writerRawBlock[WRITER_BLOCK_RECORDS];
SampleRecord writerNormalBlock[WRITER_BLOCK_RECORDS];
// Bufory pakowania strumieniowego (globalne — nie na stosie writera, jak bloki wyzej).
// src: max 64 rekordy * 48 B = 3072 B; dst: zapas na naglowek + ewentualny narzut.
uint8_t streamCompDst[WRITER_BLOCK_RECORDS * sizeof(RawLabRecord) + 64];

// Bufor obwiedni surge do autokorelacji kadencji (globalny, uzywany tylko przez imuTask).
float cadenceEnv[CADENCE_BUF] = {0};
uint8_t cadenceEnvIdx = 0;
uint8_t cadenceEnvFilled = 0;
QueueHandle_t sampleQueue = nullptr;  // Zawsze przenosi RawLabRecord; NORMAL zapisuje jego czesc PNT.
TaskHandle_t imuTaskHandle = nullptr;
TaskHandle_t writerTaskHandle = nullptr;
portMUX_TYPE statsMux = portMUX_INITIALIZER_UNLOCKED;
RuntimeStats stats = {};

// Bufor historii jest zapisywany tylko przez imuTask i kopiowany krótko przez
// handler HTTP. Dzięki temu HTTP nigdy nie czyta zmiennych IMU w trakcie zapisu.
portMUX_TYPE historyMux = portMUX_INITIALIZER_UNLOCKED;
LiveHistoryPoint liveHistory[LIVE_HISTORY_POINTS] = {};
LiveHistoryPoint webHistoryCopy[LIVE_HISTORY_POINTS] = {};
uint16_t liveHistoryHead = 0;
uint16_t liveHistoryCount = 0;
uint32_t liveHistoryLastUs = 0;
bool liveHistoryPendingEvent = false;
uint8_t liveHistoryPendingPhaseMs = 0;
// 16 KiB wystarcza: 120 punktow x ok. 10 pol + pole truncated.
char webHistoryJson[16384] = {};
char webFilesJson[4096] = {};
MountSnapshot mountSnapshot = {};
CountdownCalibration countdownCalibration = {};
uint32_t calibrationRetryUntilMs = 0;
uint32_t lastMountSnapshotMs = 0;
PowerSnapshot powerSnapshot = { -1, 0, false, false, false, false, 0, 0.0f, 0 };
int16_t batteryEstimateStartPercent = -1;
uint32_t batteryEstimateStartMs = 0;
char phonePendingFilename[32] = {};

// Wi-Fi i HTTP sa obslugiwane w glownej petli. Zadanie IMU nigdy nie
// wywoluje serwera ani Wi-Fi, dzieki czemu odczyt 200 Hz pozostaje niezalezny.
WebServer webServer(80);
bool wifiPanelReady = false;
// Migawka pamięci dla dashboardu. W czasie nagrywania nie otwieraj katalogu
// LittleFS z handlera HTTP, bo writerTask ma wyłączność na ścieżkę zapisu.
uint16_t phoneKnownSessionFiles = 0;
uint32_t phoneKnownFreeKBytes = 0;
enum PhoneCommand : uint8_t {
  PHONE_COMMAND_NONE,
  PHONE_COMMAND_START_DEFAULT,
  PHONE_COMMAND_START_MAX,
  PHONE_COMMAND_START_RAW_DEFAULT,
  PHONE_COMMAND_START_RAW_MAX,
  PHONE_COMMAND_STOP,
  PHONE_COMMAND_CANCEL,
  PHONE_COMMAND_DELETE_ONE,
  PHONE_COMMAND_DELETE_ALL,
  PHONE_COMMAND_PACK_ONE,
  PHONE_COMMAND_PACK_ALL
};
volatile PhoneCommand pendingPhoneCommand = PHONE_COMMAND_NONE;

volatile bool captureActive = false;
volatile bool imuTaskDone = true;
volatile bool writerStopRequested = false;
volatile bool writerTaskDone = true;
volatile bool writerFailed = false;
// Po przekroczeniu czasu zatrzymania IMU glowna petla zatrzyma writerTask
// dopiero po faktycznym zakonczeniu nadawania rekordow przez imuTask.
volatile bool stopWriterWhenImuDone = false;
// Ustawiane tylko przy nieudanym starcie sesji. writerTask usuwa wtedy
// niekompletny plik dopiero po jego bezpiecznym zamknieciu.
volatile bool discardCurrentLogOnClose = false;

bool storageReady = false;
bool speakerReady = false;
uint8_t countdownStep = 0;
uint8_t volumeSelection = 0;  // 0 = DEFAULT, 1 = MAX
uint32_t nextCountdownMs = 0;
uint32_t startBannerUntilMs = 0;
uint32_t volumeResetAtMs = 0;
uint32_t fsPartitionBytes = 0;
char currentFilename[32] = "-";
char selectedFilename[32] = "";
char errorText[34] = "";
uint16_t selectedFileIndex = 1;
uint16_t deleteFileIndex = 1;
uint8_t menuIndex = 0;
uint8_t fileMenuIndex = 0;
SessionMode selectedSessionMode = SESSION_MODE_NORMAL;
SessionMode activeSessionMode = SESSION_MODE_NORMAL;
// Pakowanie strumieniowe "w locie": wybrane/aktywne dla biezacej sesji.
bool selectedCompression = false;
bool activeCompression = false;
char deleteFilename[32] = "";
uint32_t lastExportProgressPercent = 101;
uint32_t lastDisplayMs = 0;

uint32_t sessionId = 0;
uint32_t sessionStartUs = 0;
float gyroBiasX = 0.0f, gyroBiasY = 0.0f, gyroBiasZ = 0.0f;
float calibrationLeanAbsDeg = 0.0f;

char commandBuffer[64] = "";
size_t commandLength = 0;

const char* MENU_ITEMS[] = {
  "RECORD NORMAL",
  "RAW LAB",
  "EXPORT",
  "FILES",
  "MOUNT",
  "DIAGNOSTICS"
};
const uint8_t MENU_ITEM_COUNT = sizeof(MENU_ITEMS) / sizeof(MENU_ITEMS[0]);

// ---------------------------------------------------------------------------
// Pomocnicze funkcje
// ---------------------------------------------------------------------------
float pickAxis(float x, float y, float z, int axis, int sign) {
  const float value = (axis == 0) ? x : ((axis == 1) ? y : z);
  return sign * value;
}

float vectorNorm(float x, float y, float z) {
  return sqrtf(x * x + y * y + z * z);
}

float clampFloat(float value, float lo, float hi) {
  return value < lo ? lo : (value > hi ? hi : value);
}

int16_t clampI16(float value) {
  if (value > 32767.0f) return 32767;
  if (value < -32768.0f) return -32768;
  return (int16_t)lroundf(value);
}

uint16_t clampU16(float value) {
  if (value > 65535.0f) return 65535;
  if (value < 0.0f) return 0;
  return (uint16_t)lroundf(value);
}

// Strefy intensywnosci v4 — strefy B: wg SILY ODPCHNIECIA (nie wg DYNa).
uint8_t strokeStrengthZone(float s) {
  if (s < ZONE0_MAX) return 0;
  if (s < ZONE1_MAX) return 1;
  if (s < ZONE2_MAX) return 2;
  if (s < ZONE3_MAX) return 3;
  return 4;
}

const char* strokeStrengthZoneName(uint8_t zone) {
  switch (zone) {
    case 0: return "BARDZO LEKKIE";
    case 1: return "LEKKIE";
    case 2: return "SREDNIE";
    case 3: return "MOCNE";
    default: return "BARDZO MOCNE";
  }
}

// Liczy percentyl (0..100) z histogramu 0..100 -> zwraca wartosc ok. p. percentyla.
uint8_t histogramPercentile(const uint32_t* hist, uint8_t p) {
  uint64_t total = 0;
  for (int i = 0; i <= 100; ++i) total += hist[i];
  if (!total) return 0;
  const uint64_t target = (total * p) / 100ULL;
  uint64_t acc = 0;
  for (int i = 0; i <= 100; ++i) {
    acc += hist[i];
    if (acc >= target) return (uint8_t)i;
  }
  return 100;
}

float wrapDeg180(float value) {
  while (value > 180.0f) value -= 360.0f;
  while (value <= -180.0f) value += 360.0f;
  return value;
}

RuntimeStats snapshotStats() {
  RuntimeStats copy;
  portENTER_CRITICAL(&statsMux);
  copy = stats;
  portEXIT_CRITICAL(&statsMux);
  return copy;
}

uint32_t freeFlashBytes() {
  if (!storageReady) return 0;
  const size_t total = LittleFS.totalBytes();
  const size_t used = LittleFS.usedBytes();
  return total > used ? (uint32_t)(total - used) : 0;
}

const char* sessionModeName(SessionMode mode) {
  return mode == SESSION_MODE_RAW_LAB ? "RAW LAB" : "NORMAL";
}

const char* sessionExtensionForMode(SessionMode mode) {
  return mode == SESSION_MODE_RAW_LAB ? RAW_LAB_FILE_EXTENSION : SESSION_FILE_EXTENSION;
}

uint16_t estimatedRecordingMinutes(uint32_t freeBytes, SessionMode mode = SESSION_MODE_NORMAL) {
  // Konserwatywna estymacja: NORMAL 23 B/rekord, RAW LAB 40 B/rekord przy 200 Hz.
  const uint32_t recordBytes = mode == SESSION_MODE_RAW_LAB ? sizeof(RawLabRecord) : sizeof(SampleRecord);
  const uint32_t bytesPerMinute = recordBytes * 200UL * 60UL;
  return bytesPerMinute ? (uint16_t)(freeBytes / bytesPerMinute) : 0;
}

void updateBatteryTelemetry(bool force = false) {
  // PMIC moze korzystac z tej samej magistrali co IMU. Zostawiamy imuTask pelna
  // wylacznosc w trakcie sesji; dashboard otrzymuje ostatnia bezpieczna migawke.
  if (!force && (state == STATE_RECORDING || state == STATE_STOPPING || state == STATE_START_COUNTDOWN)) return;
  const uint32_t now = millis();
  if (!force && powerSnapshot.lastUpdatedMs && now - powerSnapshot.lastUpdatedMs < BATTERY_TELEMETRY_PERIOD_MS) return;

  const int32_t level = M5.Power.getBatteryLevel();
  const int16_t voltage = M5.Power.getBatteryVoltage();
  const auto chargeState = M5.Power.isCharging();
  const bool chargeKnown = chargeState != m5::Power_Class::charge_unknown;
  const bool charging = chargeState == m5::Power_Class::is_charging;

  powerSnapshot.lastUpdatedMs = now;
  powerSnapshot.levelPercent = (level >= 0 && level <= 100) ? (int16_t)level : -1;
  powerSnapshot.voltageMv = voltage > 0 ? voltage : 0;
  powerSnapshot.valid = powerSnapshot.levelPercent >= 0;
  powerSnapshot.charging = charging;
  powerSnapshot.chargeKnown = chargeKnown;
  powerSnapshot.estimateValid = false;
  powerSnapshot.estimateMinutes = 0;
  powerSnapshot.dischargePctPerHour = 0.0f;

  // Czas pracy jest uczony z rzeczywistego spadku procentu baterii. Nie zgadujemy
  // pojemnosci akumulatora ani poboru pradu, a ladowanie resetuje model.
  if (!powerSnapshot.valid || charging) {
    batteryEstimateStartPercent = -1;
    batteryEstimateStartMs = 0;
    return;
  }
  if (batteryEstimateStartPercent < 0 || powerSnapshot.levelPercent > batteryEstimateStartPercent) {
    batteryEstimateStartPercent = powerSnapshot.levelPercent;
    batteryEstimateStartMs = now;
    return;
  }
  const int16_t droppedPercent = batteryEstimateStartPercent - powerSnapshot.levelPercent;
  const uint32_t elapsedMs = now - batteryEstimateStartMs;
  if (droppedPercent < 1 || elapsedMs < BATTERY_ESTIMATE_MIN_WINDOW_MS) return;

  const float pctPerHour = droppedPercent * 3600000.0f / elapsedMs;
  if (pctPerHour <= 0.0f) return;
  powerSnapshot.dischargePctPerHour = pctPerHour;
  powerSnapshot.estimateMinutes = (uint16_t)clampFloat(
      powerSnapshot.levelPercent * 60.0f / pctPerHour, 0.0f, 65535.0f);
  powerSnapshot.estimateValid = true;
}

void resetLiveHistory() {
  portENTER_CRITICAL(&historyMux);
  liveHistoryHead = 0;
  liveHistoryCount = 0;
  liveHistoryLastUs = 0;
  liveHistoryPendingEvent = false;
  liveHistoryPendingPhaseMs = 0;
  portEXIT_CRITICAL(&historyMux);
}

void appendLiveHistory(uint32_t elapsedUs, float intensity, float strokeStrength, float cadenceMs,
                       float motionDps, float leanDeg, bool strokePeak, float strokePhaseMs) {
  // Decymacja do 1 Hz. Odepchniecie wykryte pomiedzy punktami zostaje zapamietane.
  LiveHistoryPoint point = {};
  portENTER_CRITICAL(&historyMux);
  if (strokePeak) {
    liveHistoryPendingEvent = true;
    liveHistoryPendingPhaseMs = (uint8_t)clampU16(clampFloat(strokePhaseMs, 0.0f, 255.0f));
  }
  if (liveHistoryLastUs && elapsedUs - liveHistoryLastUs < LIVE_HISTORY_PERIOD_US) {
    portEXIT_CRITICAL(&historyMux);
    return;
  }
  point.elapsedSeconds = (uint16_t)(elapsedUs / 1000000UL);
  point.intensity = (uint8_t)clampU16(intensity);
  point.strokeStrength = (uint8_t)clampU16(strokeStrength);
  point.cadenceMs = clampU16(cadenceMs);
  point.motionDps = clampU16(motionDps);
  point.leanCdeg = clampI16(leanDeg * 100.0f);
  point.event = liveHistoryPendingEvent ? 1 : 0;
  point.strokePhaseMs = liveHistoryPendingPhaseMs;
  point.side = DEVICE_SIDE;
  liveHistoryPendingEvent = false;
  liveHistoryPendingPhaseMs = 0;
  liveHistory[liveHistoryHead] = point;
  liveHistoryHead = (liveHistoryHead + 1U) % LIVE_HISTORY_POINTS;
  if (liveHistoryCount < LIVE_HISTORY_POINTS) ++liveHistoryCount;
  liveHistoryLastUs = elapsedUs;
  portEXIT_CRITICAL(&historyMux);
}

const esp_partition_t* findLittleFsPartition() {
  return esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
                                  ESP_PARTITION_SUBTYPE_DATA_SPIFFS,
                                  LITTLEFS_PARTITION_LABEL);
}

bool mountLittleFS(bool formatOnFail) {
  LittleFS.end();
  return LittleFS.begin(formatOnFail, LITTLEFS_MOUNT_PATH,
                        LITTLEFS_MAX_OPEN_FILES, LITTLEFS_PARTITION_LABEL);
}

bool initStorage() {
  const esp_partition_t* partition = findLittleFsPartition();
  if (!partition) { fsPartitionBytes = 0; return false; }
  fsPartitionBytes = partition->size;
  return mountLittleFS(false);
}

bool formatAndMountLittleFS() {
  return findLittleFsPartition() != nullptr && mountLittleFS(true);
}

void setError(const char* message) {
  strncpy(errorText, message, sizeof(errorText) - 1);
  errorText[sizeof(errorText) - 1] = '\0';
  state = STATE_ERROR;
  Serial.printf("# ERROR: %s\n", errorText);
}

bool hasSessionExtension(const char* path) {
  static const char* const extensions[] = {
    SESSION_FILE_EXTENSION, RAW_LAB_FILE_EXTENSION, LEGACY_SESSION_FILE_EXTENSION,
    COMPRESSED_EXTENSION, COMPRESSED_RAW_EXTENSION,
    STREAM_COMPRESSED_EXTENSION, STREAM_COMPRESSED_RAW_EXTENSION
  };
  const size_t n = strlen(path);
  for (size_t i = 0; i < sizeof(extensions) / sizeof(extensions[0]); ++i) {
    const size_t len = strlen(extensions[i]);
    if (n >= len && strcmp(path + n - len, extensions[i]) == 0) return true;
  }
  return false;
}

static bool hasExt(const char* path, const char* ext) {
  const size_t n = strlen(path), len = strlen(ext);
  return n >= len && strcmp(path + n - len, ext) == 0;
}

bool isPackedFilename(const char* path) {
  // Za "spakowane" uznajemy archiwa po fakcie (.pz/.rz) oraz pliki skompresowane
  // strumieniowo w locie (.pzs/.rzs) — tych nie pakujemy ponownie.
  return hasExt(path, COMPRESSED_EXTENSION) || hasExt(path, COMPRESSED_RAW_EXTENSION) ||
         hasExt(path, STREAM_COMPRESSED_EXTENSION) || hasExt(path, STREAM_COMPRESSED_RAW_EXTENSION);
}

void copyLittleFsPath(char* destination, size_t size, const char* source) {
  if (size == 0) return;
  if (!source || !source[0]) { destination[0] = '\0'; return; }
  if (source[0] == '/') {
    strncpy(destination, source, size - 1);
    destination[size - 1] = '\0';
  } else {
    snprintf(destination, size, "/%s", source);
  }
}

uint16_t countSessionFiles() {
  File root = LittleFS.open("/");
  if (!root || !root.isDirectory()) return 0;
  uint16_t count = 0;
  for (File f = root.openNextFile(); f; f = root.openNextFile()) {
    if (!f.isDirectory() && hasSessionExtension(f.name())) count++;
  }
  return count;
}

void listSessionFilesToSerial() {
  Serial.println("LIST_BEGIN");
  File root = LittleFS.open("/");
  uint16_t count = 0;
  if (root && root.isDirectory()) {
    for (File f = root.openNextFile(); f; f = root.openNextFile()) {
      if (!f.isDirectory() && hasSessionExtension(f.name())) {
        count++;
        Serial.printf("FILE %u %s %lu\n", count, f.name(), (unsigned long)f.size());
      }
    }
  }
  Serial.printf("LIST_END count=%u\n", count);
}

bool selectSessionFile(uint16_t wanted, bool announce = true) {
  File root = LittleFS.open("/");
  if (!root || !root.isDirectory()) return false;
  uint16_t count = 0;
  for (File f = root.openNextFile(); f; f = root.openNextFile()) {
    if (!f.isDirectory() && hasSessionExtension(f.name()) && ++count == wanted) {
      copyLittleFsPath(selectedFilename, sizeof(selectedFilename), f.name());
      selectedFileIndex = wanted;
      if (announce) Serial.printf("SELECTED %u %s %lu\n", count, selectedFilename, (unsigned long)f.size());
      return true;
    }
  }
  selectedFilename[0] = '\0';
  return false;
}

uint32_t fileSizeByName(const char* path) {
  if (!path || !path[0]) return 0;
  File f = LittleFS.open(path, FILE_READ);
  if (!f) return 0;
  const uint32_t size = f.size();
  f.close();
  return size;
}

uint32_t selectedFileSize() {
  return fileSizeByName(selectedFilename);
}

bool selectDeleteFile(uint16_t wanted) {
  File root = LittleFS.open("/");
  if (!root || !root.isDirectory()) return false;
  uint16_t count = 0;
  for (File f = root.openNextFile(); f; f = root.openNextFile()) {
    if (!f.isDirectory() && hasSessionExtension(f.name()) && ++count == wanted) {
      copyLittleFsPath(deleteFilename, sizeof(deleteFilename), f.name());
      deleteFileIndex = wanted;
      return true;
    }
  }
  deleteFilename[0] = '\0';
  return false;
}

bool deleteOneSessionFile() {
  if (!deleteFilename[0] || !hasSessionExtension(deleteFilename)) return false;
  const bool removed = LittleFS.remove(deleteFilename);
  if (removed) {
    Serial.printf("# DELETE_ONE,%s\n", deleteFilename);
    selectedFilename[0] = '\0';
    deleteFilename[0] = '\0';
  }
  return removed;
}

bool deleteAllSessionFiles() {
  // Usuwaj po jednym pliku i otwieraj katalog od nowa. Dziala niezaleznie od
  // liczby sesji oraz nie usuwa /session_counter.txt ani plikow systemowych.
  uint16_t removed = 0;
  bool success = true;
  while (true) {
    char path[32] = "";
    File root = LittleFS.open("/");
    if (!root || !root.isDirectory()) { success = false; break; }
    for (File f = root.openNextFile(); f; f = root.openNextFile()) {
      if (!f.isDirectory() && hasSessionExtension(f.name())) {
        copyLittleFsPath(path, sizeof(path), f.name());
        f.close();
        break;
      }
    }
    root.close();
    if (!path[0]) break;
    if (!LittleFS.remove(path)) {
      Serial.printf("# DELETE_ALL_FAIL,%s\n", path);
      success = false;
      break;
    }
    removed++;
  }
  selectedFilename[0] = '\0';
  deleteFilename[0] = '\0';
  selectedFileIndex = 1;
  deleteFileIndex = 1;
  Serial.printf("# DELETE_ALL,count=%u,ok=%u\n", removed, success ? 1 : 0);
  return success;
}

// Lekki LZSS (bez bibliotek). Format archiwum sesji:
//   [6B "RZIP01"][4B rozmiar oryginalny LE][strumien tokenow]
// Strumien: bajt znacznikow 8 decyzji (MSB jako pierwsza), pozniej tokeny:
//   bit=0 -> literal (1 B);
//   bit=1 -> dopasowanie (2 B): off = b0 | ((b1>>5)&7)<<8, dl = (b1&0x1F)+3.
//            Dekompresor kopiuje 'dl' bajtow 'off' bajtow wstecz.
static const uint32_t LZSS_WINDOW = 2047;
static const uint32_t LZSS_BLOCK = 2048;

static inline uint8_t lzssHistAt(const uint8_t* hist, uint32_t writeIdx, uint32_t dist) {
  return hist[(writeIdx + LZSS_WINDOW - dist) % (LZSS_WINDOW + 1)];
}

// ---------------------------------------------------------------------------
// Pakowanie strumieniowe "w locie": enkoder LZSS pojedynczego bloku w RAM.
// Slownik zerowany na starcie bloku (dopasowania szukane tylko w tym bloku,
// max LZSS_WINDOW wstecz), dzieki czemu awaria psuje tylko ostatni blok.
// Ten sam format tokenow co packFileLzss (off 11-bit, dl 3..34, grupa 8 decyzji).
// Zwraca liczbe bajtow zapisanych do 'dst' albo 0, gdy nie miesci sie w dstCap
// (wowczas wolajacy zapisze blok jako surowy). 'src'/'dst' to bufory w RAM.
static uint32_t lzssEncodeBlock(const uint8_t* src, uint32_t srcLen,
                                uint8_t* dst, uint32_t dstCap) {
  uint32_t di = 0, pos = 0;
  uint8_t groupByte = 0; uint32_t groupCount = 0;
  uint8_t pend[16]; uint32_t pendLen = 0;
  auto emit = [&](uint8_t byte) -> bool { if (di >= dstCap) return false; dst[di++] = byte; return true; };
  auto flushGroup = [&]() -> bool {
    if (groupCount == 0) return true;
    if (!emit(groupByte)) return false;
    for (uint32_t i = 0; i < pendLen; ++i) if (!emit(pend[i])) return false;
    groupByte = 0; groupCount = 0; pendLen = 0;
    return true;
  };
  while (pos < srcLen) {
    const uint32_t maxDist = pos < LZSS_WINDOW ? pos : LZSS_WINDOW;
    const uint32_t avail = srcLen - pos;
    const uint32_t limit = avail < 34 ? avail : 34;
    uint32_t bestOff = 0, bestLen = 0;
    if (limit >= 3) {
      for (uint32_t off = 1; off <= maxDist; ++off) {
        if (src[pos - off] != src[pos]) continue;
        uint32_t len = 1;
        while (len < limit && (off - len) >= 1 && src[pos - off + len] == src[pos + len]) ++len;
        if (len > bestLen) { bestLen = len; bestOff = off; if (len == limit) break; }
      }
    }
    if (bestLen >= 3) {
      groupByte |= (uint8_t)(1u << (7 - groupCount));
      pend[pendLen++] = (uint8_t)(bestOff & 0xFF);
      pend[pendLen++] = (uint8_t)((((bestOff >> 8) & 0x07) << 5) | (bestLen - 3));
      pos += bestLen;
    } else {
      pend[pendLen++] = src[pos];
      pos += 1;
    }
    if (++groupCount == 8) { if (!flushGroup()) return 0; }
  }
  if (!flushGroup()) return 0;
  return di;
}

bool packFileLzss(const char* inPath, const char* outPath) {
  File in = LittleFS.open(inPath, FILE_READ);
  if (!in) return false;
  File out = LittleFS.open(outPath, FILE_WRITE);
  if (!out) { in.close(); return false; }
  const uint32_t originalSize = in.size();
  out.write((const uint8_t*)"RZIP01", 6);
  uint8_t sizeLe[4] = { (uint8_t)(originalSize & 0xFF), (uint8_t)((originalSize >> 8) & 0xFF),
                        (uint8_t)((originalSize >> 16) & 0xFF), (uint8_t)((originalSize >> 24) & 0xFF) };
  out.write(sizeLe, 4);

  uint8_t hist[LZSS_WINDOW + 1] = {};
  uint32_t histWrite = 0;
  uint8_t block[LZSS_BLOCK] = {};
  uint32_t blockLen = 0, pos = 0;
  bool eofReached = false;
  uint8_t groupByte = 0;
  uint32_t groupCount = 0;
  uint8_t pend[16] = {};
  uint32_t pendLen = 0;
  bool ok = true;

  auto flushGroup = [&]() -> bool {
    if (groupCount == 0) return true;
    if (out.write(&groupByte, 1) != 1) return false;
    if (pendLen && out.write(pend, pendLen) != pendLen) return false;
    groupByte = 0; groupCount = 0; pendLen = 0;
    return true;
  };

  while (true) {
    if (pos >= blockLen) {
      if (eofReached) break;
      const size_t got = in.read(block, sizeof(block));
      if (got == 0) { eofReached = true; break; }
      blockLen = (uint32_t)got; pos = 0;
    }
    const uint32_t avail = blockLen - pos;
    const uint32_t cur = pos;
    const uint32_t maxDist = histWrite < LZSS_WINDOW ? histWrite : LZSS_WINDOW;
    uint32_t bestOff = 0, bestLen = 0;
    if (avail >= 3) {
      for (uint32_t off = 1; off <= maxDist; ++off) {
        if (lzssHistAt(hist, histWrite, off) != block[cur]) continue;
        uint32_t len = 1;
        const uint32_t limit = avail < 34 ? avail : 34;
        while (len < limit && (off - len) >= 1 &&
               lzssHistAt(hist, histWrite, off - len) == block[cur + len]) ++len;
        if (len > bestLen) {
          bestLen = len; bestOff = off;
          if (len == limit) break;
        }
      }
    }
    if (bestLen >= 3) {
      groupByte |= (uint8_t)(1u << (7 - groupCount));
      const uint8_t b1 = (uint8_t)((((bestOff >> 8) & 0x07) << 5) | (bestLen - 3));
      pend[pendLen++] = (uint8_t)(bestOff & 0xFF);
      pend[pendLen++] = b1;
      for (uint32_t i = 0; i < bestLen; ++i) {
        hist[histWrite % (LZSS_WINDOW + 1)] = block[cur + i];
        ++histWrite;
      }
      pos += bestLen;
    } else {
      const uint8_t b = block[cur];
      pend[pendLen++] = b;
      hist[histWrite % (LZSS_WINDOW + 1)] = b;
      ++histWrite;
      pos += 1;
    }
    if (++groupCount == 8 && !flushGroup()) { ok = false; break; }
  }
  if (ok) ok = flushGroup();
  in.close();
  out.close();
  return ok;
}

// Weryfikacja na urzadzeniu: dekompresja archiwum i porownanie z oryginalem.
// Zapobiega zapisowi uszkodzonego archiwum (nawet gdyby enkoder mial blad).
bool verifyLzssFile(const char* inPath, const char* packedPath) {
  File src = LittleFS.open(inPath, FILE_READ);
  if (!src) return false;
  File p = LittleFS.open(packedPath, FILE_READ);
  if (!p) { src.close(); return false; }
  uint8_t magic[6] = {};
  if (p.read(magic, 6) != 6 || memcmp(magic, "RZIP01", 6) != 0) { src.close(); p.close(); return false; }
  uint8_t sizeLe[4] = {};
  if (p.read(sizeLe, 4) != 4) { src.close(); p.close(); return false; }
  const uint32_t orig = (uint32_t)sizeLe[0] | ((uint32_t)sizeLe[1] << 8) |
                        ((uint32_t)sizeLe[2] << 16) | ((uint32_t)sizeLe[3] << 24);
  uint8_t hist[LZSS_WINDOW + 1] = {};
  uint32_t histWrite = 0;
  uint32_t outCount = 0;
  bool ok = true;
  while (ok && outCount < orig) {
    const int g = p.read();
    if (g < 0) { ok = false; break; }
    for (int k = 0; k < 8 && outCount < orig; ++k) {
      if (((g >> (7 - k)) & 1) == 1) {
        const int b0 = p.read(); const int b1 = p.read();
        if (b0 < 0 || b1 < 0) { ok = false; break; }
        const uint32_t off = (uint32_t)b0 | ((((uint32_t)b1 >> 5) & 0x07) << 8);
        const uint32_t len = ((uint32_t)b1 & 0x1F) + 3;
        for (uint32_t i = 0; i < len && ok; ++i) {
          const uint8_t b = hist[(histWrite + LZSS_WINDOW - off) % (LZSS_WINDOW + 1)];
          hist[histWrite % (LZSS_WINDOW + 1)] = b;
          ++histWrite; ++outCount;
          const int s = src.read();
          if (s < 0 || (uint8_t)s != b) ok = false;
        }
      } else {
        const int pb = p.read();
        if (pb < 0) { ok = false; break; }
        const uint8_t b = (uint8_t)pb;
        hist[histWrite % (LZSS_WINDOW + 1)] = b;
        ++histWrite; ++outCount;
        const int s = src.read();
        if (s < 0 || (uint8_t)s != b) ok = false;
      }
    }
  }
  if (ok && src.read() >= 0) ok = false;
  src.close();
  p.close();
  return ok;
}

// Pakuje jedna sesje do archiwum RZIP01 z weryfikacja. Gdy archiwum juz
// istnieje, usuwamy oryginal. Po sukcesie oryginal zawsze zostaje usuniety.
bool packOneFile(const char* source, const char* target) {
  if (LittleFS.exists(target)) { LittleFS.remove(source); return true; }
  char tmpPath[40] = "";
  snprintf(tmpPath, sizeof(tmpPath), "%s.t", target);
  if (!packFileLzss(source, tmpPath)) { LittleFS.remove(tmpPath); return false; }
  if (!verifyLzssFile(source, tmpPath)) { LittleFS.remove(tmpPath); return false; }
  if (!LittleFS.remove(source)) { LittleFS.remove(tmpPath); return false; }
  if (!LittleFS.rename(tmpPath, target)) { LittleFS.remove(tmpPath); return false; }
  return true;
}

const char* packedNameForPath(const char* sessionPath, char* destination, size_t size) {
  copyLittleFsPath(destination, size, sessionPath);
  const size_t n = strlen(destination);
  if (n < 4) return nullptr;
  const char* extension = destination + n - 4;
  if (strcmp(extension, SESSION_FILE_EXTENSION) == 0 ||
      strcmp(extension, LEGACY_SESSION_FILE_EXTENSION) == 0) {
    snprintf(destination + n - 4, size - (n - 4), "%s", COMPRESSED_EXTENSION);
    return destination;
  }
  if (strcmp(extension, RAW_LAB_FILE_EXTENSION) == 0) {
    snprintf(destination + n - 4, size - (n - 4), "%s", COMPRESSED_RAW_EXTENSION);
    return destination;
  }
  return nullptr;
}

// Pakuje wszystkie sesje (PNT/RWL/RIM) ktore nie maja jeszcze archiwum.
// Po udanym pakowaniu oryginal zostaje usunity (zwalnia miejsce).
bool packAllSessionFiles(uint16_t& packedCount, uint32_t& freedBytes) {
  packedCount = 0; freedBytes = 0;
  bool success = true;
  while (true) {
    char source[32] = "";
    File root = LittleFS.open("/");
    if (!root || !root.isDirectory()) { success = false; break; }
    for (File f = root.openNextFile(); f; f = root.openNextFile()) {
      if (f.isDirectory() || !hasSessionExtension(f.name())) continue;
      if (isPackedFilename(f.name())) continue;
      copyLittleFsPath(source, sizeof(source), f.name());
      f.close();
      break;
    }
    root.close();
    if (!source[0]) break;
    char packed[32] = "";
    const char* target = packedNameForPath(source, packed, sizeof(packed));
    if (!target) { success = false; break; }
    const uint32_t sourceSize = fileSizeByName(source);
    if (!packOneFile(source, target)) { success = false; break; }
    ++packedCount;
    freedBytes += sourceSize;
  }
  return success;
}

bool writeSessionCounter(uint32_t value) {
  File counter = LittleFS.open("/session_counter.txt", FILE_WRITE);
  if (!counter) return false;
  const size_t written = counter.printf("%lu\n", (unsigned long)value);
  counter.flush();
  counter.close();
  return written > 0;
}

uint32_t nextSessionId() {
  uint32_t value = 0;
  File counter = LittleFS.open("/session_counter.txt", FILE_READ);
  if (counter) { value = (uint32_t)counter.parseInt(); counter.close(); }

  // Nie nadpisuj sesji nawet wtedy, gdy licznik zostal uszkodzony lub skasowany.
  // Limit jest znacznie wiekszy niz liczba plikow, ktora zmiesci aktualny LittleFS.
  for (uint16_t attempt = 0; attempt < 10000; ++attempt) {
    if (value == 0xFFFFFFFFUL) return 0;
    ++value;
    char filename[32];
    snprintf(filename, sizeof(filename), "/ses%05lu%s", (unsigned long)value, SESSION_FILE_EXTENSION);
    if (LittleFS.exists(filename)) continue;
    // Nie przypisuj numeru istniejacej sesji RAW LAB ani starszej sesji RIM.
    snprintf(filename, sizeof(filename), "/ses%05lu%s", (unsigned long)value, RAW_LAB_FILE_EXTENSION);
    if (LittleFS.exists(filename)) continue;
    snprintf(filename, sizeof(filename), "/ses%05lu%s", (unsigned long)value, LEGACY_SESSION_FILE_EXTENSION);
    if (LittleFS.exists(filename)) continue;
    return writeSessionCounter(value) ? value : 0;
  }
  return 0;
}

void discardCurrentLogFileNow() {
  if (logFile) logFile.close();
  if (currentFilename[0] == '/') {
    LittleFS.remove(currentFilename);
  }
  strncpy(currentFilename, "-", sizeof(currentFilename) - 1);
  currentFilename[sizeof(currentFilename) - 1] = '\0';
}

void cleanupSessionResources() {
  // Kolejka nalezy do obu zadan. Wolno ja usunac dopiero, gdy oba zakonczyly prace.
  if (sampleQueue && imuTaskDone && writerTaskDone) {
    vQueueDelete(sampleQueue);
    sampleQueue = nullptr;
    imuTaskHandle = nullptr;
    writerTaskHandle = nullptr;
  }
}

// ---------------------------------------------------------------------------
// UI Rolki Silesia
// ---------------------------------------------------------------------------
void drawSkaterLogo(int16_t x, int16_t y, uint16_t color) {
  M5.Lcd.fillCircle(x + 8, y + 3, 2, color);
  M5.Lcd.drawLine(x + 8, y + 6, x + 11, y + 11, color);
  M5.Lcd.drawLine(x + 10, y + 8, x + 4, y + 9, color);
  M5.Lcd.drawLine(x + 10, y + 8, x + 16, y + 7, color);
  M5.Lcd.drawLine(x + 11, y + 11, x + 6, y + 15, color);
  M5.Lcd.drawLine(x + 11, y + 11, x + 17, y + 15, color);
  M5.Lcd.drawLine(x + 2, y + 16, x + 9, y + 16, color);
  M5.Lcd.drawLine(x + 13, y + 16, x + 20, y + 16, color);
  M5.Lcd.drawCircle(x + 4, y + 18, 1, color);
  M5.Lcd.drawCircle(x + 8, y + 18, 1, color);
  M5.Lcd.drawCircle(x + 15, y + 18, 1, color);
  M5.Lcd.drawCircle(x + 19, y + 18, 1, color);
}

void drawHeroSkaterLogo(int16_t x, int16_t y, uint16_t color) {
  M5.Lcd.fillCircle(x + 18, y + 4, 4, color);
  M5.Lcd.drawLine(x + 18, y + 9, x + 23, y + 20, color);
  M5.Lcd.drawLine(x + 21, y + 13, x + 7, y + 16, color);
  M5.Lcd.drawLine(x + 22, y + 13, x + 36, y + 10, color);
  M5.Lcd.drawLine(x + 23, y + 20, x + 12, y + 31, color);
  M5.Lcd.drawLine(x + 23, y + 20, x + 35, y + 31, color);
  M5.Lcd.drawLine(x + 6, y + 33, x + 18, y + 33, color);
  M5.Lcd.drawLine(x + 28, y + 33, x + 41, y + 33, color);
  for (int i = 0; i < 3; ++i) {
    M5.Lcd.drawCircle(x + 9 + i * 4, y + 36, 2, color);
    M5.Lcd.drawCircle(x + 31 + i * 4, y + 36, 2, color);
  }
  M5.Lcd.drawLine(x, y + 23, x + 9, y + 23, UI_WHITE);
  M5.Lcd.drawLine(x + 2, y + 27, x + 10, y + 27, UI_WHITE);
}

void drawHeader(const char* title) {
  M5.Lcd.fillScreen(TFT_BLACK);
  drawSkaterLogo(4, 1, UI_RED);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setTextColor(UI_WHITE, TFT_BLACK);
  M5.Lcd.setCursor(29, 2); M5.Lcd.print("ROLKI");
  M5.Lcd.setTextColor(UI_RED, TFT_BLACK);
  M5.Lcd.setCursor(29, 11); M5.Lcd.print("SILESIA");
  M5.Lcd.setCursor(142, 7); M5.Lcd.print(title);
  M5.Lcd.drawFastHLine(0, 24, M5.Lcd.width(), UI_RED);
  M5.Lcd.setTextColor(UI_WHITE, TFT_BLACK);
}

void printTime(uint32_t seconds) {
  M5.Lcd.printf("%02lu:%02lu", (unsigned long)(seconds / 60U), (unsigned long)(seconds % 60U));
}

void playTone(float frequencyHz, uint32_t durationMs, bool stopCurrent = true) {
  if (speakerReady) M5.Speaker.tone(frequencyHz, durationMs, 0, stopCurrent);
}

void playStartSignal() {
  // Dwa szybkie tony o rosnacej wysokosci po faktycznym uruchomieniu logowania.
  playTone(1760.0f, 70, true);
  playTone(2093.0f, 70, false);
}

void playSavedSignal() {
  // Ten sygnal brzmi dopiero po oproznieniu kolejki i zamknieciu pliku LittleFS.
  playTone(880.0f, SAVED_TONE_DURATION_MS, true);
}

void restoreDefaultSpeakerVolume() {
  if (speakerReady) M5.Speaker.setVolume(SPEAKER_VOLUME);
  volumeSelection = 0;
  volumeResetAtMs = 0;
}

void applySessionSpeakerVolume() {
  if (!speakerReady) return;
  M5.Speaker.setVolume(volumeSelection ? SPEAKER_VOLUME_MAX : SPEAKER_VOLUME);
}

void updateSpeakerVolumeReset() {
  if (volumeResetAtMs && (int32_t)(millis() - volumeResetAtMs) >= 0) {
    restoreDefaultSpeakerVolume();
  }
}

void drawSplashSkaterLogo(int16_t x, int16_t y, uint8_t frame) {
  // Lekka, lokalnie rysowana animacja: zmiana kroku, pozycji glowy i smug.
  const int16_t bob = (frame % 6U) < 3U ? 0 : 1;
  const int16_t stride = (frame & 1U) ? 3 : -3;
  const int16_t bodyY = y + bob;
  M5.Lcd.fillCircle(x + 30 + stride / 3, bodyY + 8, 7, UI_WHITE);
  M5.Lcd.drawLine(x + 30, bodyY + 17, x + 39, bodyY + 38, UI_RED);
  M5.Lcd.drawLine(x + 37, bodyY + 25, x + 12 - stride, bodyY + 31, UI_RED);
  M5.Lcd.drawLine(x + 38, bodyY + 25, x + 62 + stride, bodyY + 18, UI_RED);
  M5.Lcd.drawLine(x + 39, bodyY + 38, x + 19 - stride, bodyY + 55, UI_RED);
  M5.Lcd.drawLine(x + 39, bodyY + 38, x + 57 + stride, bodyY + 54, UI_RED);
  M5.Lcd.drawFastHLine(x + 8 - stride, bodyY + 58, 22, UI_WHITE);
  M5.Lcd.drawFastHLine(x + 48 + stride, bodyY + 58, 22, UI_WHITE);
  for (int i = 0; i < 4; ++i) {
    M5.Lcd.fillCircle(x + 12 - stride + i * 5, bodyY + 63, 3, UI_RED);
    M5.Lcd.fillCircle(x + 52 + stride + i * 5, bodyY + 63, 3, UI_RED);
  }
  const int16_t trail = (frame % 4U) * 3;
  M5.Lcd.drawFastHLine(x - 5 - trail, bodyY + 38, 17 + trail, UI_DARK_RED);
  M5.Lcd.drawFastHLine(x - 10 - trail, bodyY + 45, 22 + trail, UI_DARK_RED);
}

void drawSplashScreen(uint8_t frame = 0) {
  M5.Lcd.fillScreen(TFT_BLACK);
  M5.Lcd.fillRoundRect(6, 8, 228, 119, 8, UI_DARK_RED);
  M5.Lcd.drawRoundRect(6, 8, 228, 119, 8, UI_RED);
  drawSplashSkaterLogo(15, 31, frame);
  M5.Lcd.setTextColor(UI_WHITE, UI_DARK_RED);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(105, 37); M5.Lcd.print("SILESIA");
  M5.Lcd.setTextColor(UI_RED, UI_DARK_RED);
  M5.Lcd.setCursor(105, 58); M5.Lcd.print("SKATING");
  M5.Lcd.drawFastHLine(104, 82, 112, UI_RED);
  M5.Lcd.setTextColor(UI_WHITE, UI_DARK_RED);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(105, 91); M5.Lcd.print("IMU LOG");
  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(83, 115); M5.Lcd.print("ROLKI SILESIA");
  M5.Lcd.setTextColor(UI_WHITE, TFT_BLACK);
}

void playSplashAnimation() {
  const uint32_t startedMs = millis();
  uint8_t frame = 0;
  while ((uint32_t)(millis() - startedMs) < SPLASH_DURATION_MS) {
    drawSplashScreen(frame++);
    M5.update();
    delay(90);
  }
}

void drawVolumeSelectScreen() {
  drawHeader(selectedSessionMode == SESSION_MODE_RAW_LAB ? "RAW LAB AUDIO" : "NORMAL AUDIO");
  M5.Lcd.setTextColor(selectedSessionMode == SESSION_MODE_RAW_LAB ? UI_RED : TFT_LIGHTGREY, TFT_BLACK);
  M5.Lcd.setCursor(10, 31); M5.Lcd.print(selectedSessionMode == SESSION_MODE_RAW_LAB ? "TEST MODE - RAW + TRACE" : "CHOOSE SPEAKER VOLUME");
  const char* labels[] = { "DEFAULT", "MAX" };
  const char* values[] = { "96 / 255", "255 / 255" };
  for (uint8_t i = 0; i < 2; ++i) {
    const int16_t y = 48 + i * 28;
    const bool selected = volumeSelection == i;
    const uint16_t fill = selected ? UI_RED : UI_DARK_RED;
    M5.Lcd.fillRoundRect(10, y, 220, 23, 5, fill);
    M5.Lcd.drawRoundRect(10, y, 220, 23, 5, selected ? UI_WHITE : UI_DARK_RED);
    M5.Lcd.setTextColor(UI_WHITE, fill);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setCursor(18, y + 3); M5.Lcd.print(labels[i]);
    M5.Lcd.setTextSize(1);
    M5.Lcd.setCursor(150, y + 8); M5.Lcd.print(values[i]);
  }
  M5.Lcd.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  M5.Lcd.setCursor(10, 119); M5.Lcd.print("A: VOLUME  B: START  HOLD: MENU");
  M5.Lcd.setTextColor(UI_WHITE, TFT_BLACK);
}

void drawCountdownScreen(uint8_t step) {
  const bool retryNotice = calibrationRetryUntilMs && (int32_t)(millis() - calibrationRetryUntilMs) < 0;
  const bool calibrating = step < CALIBRATION_COUNTDOWN_STEPS && !countdownCalibration.complete;
  drawHeader(retryNotice ? "CAL RETRY" : (calibrating ? "CALIBRATING" : "GET READY"));
  M5.Lcd.setTextColor(UI_WHITE, TFT_BLACK);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(10, 32); M5.Lcd.print("A/B: CANCEL");
  if (selectedSessionMode == SESSION_MODE_RAW_LAB) {
    M5.Lcd.setTextColor(UI_RED, TFT_BLACK);
    M5.Lcd.setCursor(165, 32); M5.Lcd.print("RAW LAB");
    M5.Lcd.setTextColor(UI_WHITE, TFT_BLACK);
  }
  if (retryNotice) {
    M5.Lcd.setTextColor(UI_RED, TFT_BLACK);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setCursor(23, 57); M5.Lcd.print("MOVE DETECTED");
    M5.Lcd.setCursor(43, 80); M5.Lcd.print("TRY AGAIN");
    M5.Lcd.setTextSize(1);
    M5.Lcd.setTextColor(UI_WHITE, TFT_BLACK);
    M5.Lcd.setCursor(29, 113); M5.Lcd.print("KEEP SKATE STILL");
    return;
  }
  const uint8_t number = 10 - step;  // 10, 9, 8 (cal) potem 7 ... 1.
  M5.Lcd.setTextColor(UI_RED, TFT_BLACK);
  M5.Lcd.setTextSize(5);
  M5.Lcd.setCursor(number == 10 ? 54 : 74, 54); M5.Lcd.printf("%u", number);
  M5.Lcd.setTextSize(1);
  if (calibrating) {
    M5.Lcd.setTextColor(UI_RED, TFT_BLACK);
    M5.Lcd.setCursor(39, 103); M5.Lcd.print("CAL: DO NOT MOVE");
    M5.Lcd.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    M5.Lcd.setCursor(26, 118); M5.Lcd.printf("OK %lu  MOVE %lu", (unsigned long)countdownCalibration.stableSamples,
                                              (unsigned long)countdownCalibration.rejectedSamples);
  } else {
    M5.Lcd.setTextColor(UI_WHITE, TFT_BLACK);
    M5.Lcd.setCursor(42, 118); M5.Lcd.print("COUNTDOWN TO START");
  }
  M5.Lcd.setTextColor(UI_WHITE, TFT_BLACK);
}

void drawStartBanner() {
  drawHeader(activeSessionMode == SESSION_MODE_RAW_LAB ? "RAW LAB" : "START");
  M5.Lcd.setTextColor(UI_RED, TFT_BLACK);
  M5.Lcd.setTextSize(5);
  M5.Lcd.setCursor(38, 58); M5.Lcd.print("START");
  M5.Lcd.setTextSize(1);
  M5.Lcd.setTextColor(UI_WHITE, TFT_BLACK);
  M5.Lcd.setCursor(activeSessionMode == SESSION_MODE_RAW_LAB ? 35 : 61, 118);
  M5.Lcd.print(activeSessionMode == SESSION_MODE_RAW_LAB ? "RAW + TRACE ACTIVE" : "NORMAL RECORDING");
}

void drawLiveFrame() {
  drawHeader(activeSessionMode == SESSION_MODE_RAW_LAB ? "RAW LIVE" : "LIVE");
  M5.Lcd.fillRoundRect(4, 29, 232, 40, 5, UI_DARK_RED);
  M5.Lcd.setTextColor(TFT_LIGHTGREY, UI_DARK_RED);
  M5.Lcd.setCursor(12, 35); M5.Lcd.print("SILA ODPCH");
  M5.Lcd.setTextColor(UI_WHITE, TFT_BLACK);
  M5.Lcd.setCursor(10, 76); M5.Lcd.print("KADENCJA");
  M5.Lcd.setCursor(116, 76); M5.Lcd.print("MOC");
  M5.Lcd.setCursor(10, 101); M5.Lcd.print("LOG");
  M5.Lcd.setTextColor(UI_RED, TFT_BLACK);
  M5.Lcd.setCursor(178, 119); M5.Lcd.print("A: STOP");
  M5.Lcd.setTextColor(UI_WHITE, TFT_BLACK);
}

void updateLiveValues() {
  const RuntimeStats s = snapshotStats();
  const uint32_t elapsed = sessionStartUs ? (micros() - sessionStartUs) / 1000000UL : 0;
  const float rate = (s.recordsQueued > 1 && s.lastSampleUs > s.firstSampleUs)
      ? (s.recordsQueued - 1) * 1000000.0f / (s.lastSampleUs - s.firstSampleUs) : 0.0f;
  const float strokesPerMin = elapsed > 2 ? s.strokeCount * 60.0f / elapsed : 0.0f;

  M5.Lcd.fillRect(12, 47, 214, 18, UI_DARK_RED);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(UI_WHITE, UI_DARK_RED);
  M5.Lcd.setCursor(12, 47); M5.Lcd.printf("S %3.0f", s.strokeStrengthNow);
  M5.Lcd.setCursor(130, 47); printTime(elapsed);
  M5.Lcd.setTextSize(1);

  M5.Lcd.fillRect(10, 86, 220, 11, TFT_BLACK);
  M5.Lcd.fillRect(10, 111, 220, 9, TFT_BLACK);
  M5.Lcd.setTextColor(UI_WHITE, TFT_BLACK);
  M5.Lcd.setCursor(10, 86); M5.Lcd.printf("%5.1f/min", strokesPerMin);
  M5.Lcd.setCursor(116, 86); M5.Lcd.printf("%4.1f", s.powerProxyNow);
  M5.Lcd.setTextColor((s.qDropCount == 0 && s.badGapCount == 0) ? UI_WHITE : UI_RED, TFT_BLACK);
  M5.Lcd.setCursor(10, 111); M5.Lcd.printf("%.0fHz G%lu B%lu Q%lu", rate,
      (unsigned long)s.gap8Count, (unsigned long)s.badGapCount, (unsigned long)s.qDropCount);
  M5.Lcd.setTextColor(UI_WHITE, TFT_BLACK);
}

void drawMenuIcon(int16_t x, int16_t y, uint8_t type, uint16_t color) {
  // Male, jednoznaczne ikony rysowane lokalnie: bez dodatkowych plikow i RAM.
  if (type == 0) {                       // NORMAL RECORD
    M5.Lcd.drawCircle(x + 11, y + 10, 9, color);
    M5.Lcd.fillTriangle(x + 8, y + 4, x + 8, y + 16, x + 18, y + 10, color);
  } else if (type == 1) {                // RAW LAB
    M5.Lcd.drawRect(x + 2, y + 2, 20, 15, color);
    M5.Lcd.drawLine(x + 5, y + 13, x + 9, y + 8, color);
    M5.Lcd.drawLine(x + 9, y + 8, x + 13, y + 12, color);
    M5.Lcd.drawLine(x + 13, y + 12, x + 18, y + 5, color);
    M5.Lcd.drawFastHLine(x + 4, y + 20, 16, color);
  } else if (type == 2) {                // EXPORT
    M5.Lcd.drawRect(x, y, 14, 17, color);
    M5.Lcd.drawFastHLine(x + 3, y + 5, 7, color);
    M5.Lcd.drawFastHLine(x + 3, y + 9, 5, color);
    M5.Lcd.drawLine(x + 17, y + 8, x + 24, y + 8, color);
    M5.Lcd.drawLine(x + 21, y + 5, x + 24, y + 8, color);
    M5.Lcd.drawLine(x + 21, y + 11, x + 24, y + 8, color);
  } else if (type == 3) {                // FILES
    M5.Lcd.drawRect(x + 3, y, 17, 14, color);
    M5.Lcd.drawRect(x, y + 4, 17, 14, color);
    M5.Lcd.drawFastHLine(x + 4, y + 9, 8, color);
    M5.Lcd.drawFastHLine(x + 4, y + 13, 6, color);
  } else if (type == 4) {                // MOUNT
    M5.Lcd.drawCircle(x + 11, y + 10, 7, color);
    M5.Lcd.drawFastVLine(x + 11, y, 20, color);
    M5.Lcd.drawFastHLine(x + 1, y + 10, 20, color);
    M5.Lcd.fillCircle(x + 11, y + 10, 2, color);
  } else {                               // DIAGNOSTICS
    M5.Lcd.drawCircle(x + 11, y + 10, 7, color);
    M5.Lcd.fillCircle(x + 11, y + 10, 2, color);
    M5.Lcd.drawFastVLine(x + 11, y, 3, color);
    M5.Lcd.drawFastVLine(x + 11, y + 17, 3, color);
    M5.Lcd.drawFastHLine(x, y + 10, 3, color);
    M5.Lcd.drawFastHLine(x + 19, y + 10, 3, color);
  }
}

void drawMenuCard(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t item, bool primary) {
  static const char* labels[] = { "REC", "RAW", "XPORT", "FILES", "MOUNT", "DIAG" };
  static const char* subtitles[] = { "NORM", "LAB", "USB", "DATA", "AXIS", "SYS" };
  const bool selected = item == menuIndex;
  const uint16_t fill = selected ? UI_RED : TFT_BLACK;
  const uint16_t accent = selected ? UI_WHITE : UI_RED;
  const uint16_t text = UI_WHITE;
  M5.Lcd.fillRoundRect(x, y, w, h, 5, fill);
  M5.Lcd.drawRoundRect(x, y, w, h, 5, selected ? UI_WHITE : UI_DARK_RED);
  if (selected) M5.Lcd.fillRect(x + 4, y + 5, 3, h - 10, UI_WHITE);

  drawMenuIcon(x + 10, y + (primary ? 8 : 6), item, accent);

  const int16_t textX = x + 37;
  M5.Lcd.setTextColor(text, fill);
  M5.Lcd.setTextSize(primary ? 2 : 1);
  M5.Lcd.setCursor(textX, y + (primary ? 6 : 7)); M5.Lcd.print(labels[item]);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setTextColor(selected ? UI_WHITE : TFT_LIGHTGREY, fill);
  M5.Lcd.setCursor(textX, y + h - 11); M5.Lcd.print(subtitles[item]);
  M5.Lcd.setTextColor(UI_WHITE, TFT_BLACK);
}

void drawMenuFrame() {
  drawHeader("HOME");

  // Pasek statusu pokazuje od razu zasieg NORMAL i RAW LAB oraz liczbe plikow.
  M5.Lcd.fillRoundRect(5, 28, 230, 11, 4, UI_DARK_RED);
  const uint32_t freeBytes = freeFlashBytes();
  const uint16_t normalMinutes = estimatedRecordingMinutes(freeBytes, SESSION_MODE_NORMAL);
  const uint16_t rawMinutes = estimatedRecordingMinutes(freeBytes, SESSION_MODE_RAW_LAB);
  M5.Lcd.setTextColor(TFT_LIGHTGREY, UI_DARK_RED);
  M5.Lcd.setCursor(10, 30); M5.Lcd.print(wifiPanelReady ? "AP" : "WIFI OFF");
  M5.Lcd.setTextColor(UI_WHITE, UI_DARK_RED);
  M5.Lcd.setCursor(54, 30); M5.Lcd.printf("N%um R%um", normalMinutes, rawMinutes);
  M5.Lcd.setTextColor(UI_RED, UI_DARK_RED);
  M5.Lcd.setCursor(180, 30); M5.Lcd.printf("%u FILES", countSessionFiles());
  M5.Lcd.setTextColor(UI_WHITE, TFT_BLACK);

  // Szesć równych kart: najpierw dwa tryby rejestracji, potem operacje i narzędzia.
  drawMenuCard(5,   44, 72, 32, 0, false);
  drawMenuCard(83,  44, 72, 32, 1, false);
  drawMenuCard(161, 44, 74, 32, 2, false);
  drawMenuCard(5,   81, 72, 32, 3, false);
  drawMenuCard(83,  81, 72, 32, 4, false);
  drawMenuCard(161, 81, 74, 32, 5, false);

  M5.Lcd.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  M5.Lcd.setCursor(10, 122); M5.Lcd.print("A: NEXT");
  M5.Lcd.setTextColor(UI_RED, TFT_BLACK);
  M5.Lcd.setCursor(126, 122); M5.Lcd.print("B: OPEN");
  M5.Lcd.setTextColor(UI_WHITE, TFT_BLACK);
}

void drawMountFrame() {
  drawHeader("MOUNT GUIDE");
  M5.Lcd.setCursor(10, 31); M5.Lcd.print("X+ -> TOE / TRAVEL");
  M5.Lcd.setCursor(10, 45); M5.Lcd.print("Y+ -> SKY / UP");
  M5.Lcd.setCursor(10, 59); M5.Lcd.print("Z+ -> LEFT SIDE");
  M5.Lcd.drawFastHLine(4, 74, 232, UI_DARK_RED);
  M5.Lcd.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  M5.Lcd.setCursor(10, 81); M5.Lcd.print("STILL: F0 L0 U+1g");
  M5.Lcd.setTextColor(UI_WHITE, TFT_BLACK);
  M5.Lcd.setCursor(10, 119); M5.Lcd.print("A/B: MENU");
}

void updateMountSnapshot() {
  // Nie wywoluj tej funkcji podczas rejestracji: imuTask ma wtedy wylacznosc
  // na odczyt BMI270. W stanie bezczynnosci ta sama migawka zasila LCD i WWW.
  if (state == STATE_RECORDING || state == STATE_STOPPING || state == STATE_START_COUNTDOWN || !M5.Imu.update()) return;
  const auto d = M5.Imu.getImuData();
  mountSnapshot.forwardG = pickAxis(d.accel.x, d.accel.y, d.accel.z, FORWARD_AXIS, FORWARD_SIGN);
  mountSnapshot.lateralG = pickAxis(d.accel.x, d.accel.y, d.accel.z, LATERAL_AXIS, LATERAL_SIGN);
  mountSnapshot.verticalG = pickAxis(d.accel.x, d.accel.y, d.accel.z, VERTICAL_AXIS, VERTICAL_SIGN);
  mountSnapshot.valid = true;
  lastMountSnapshotMs = millis();
}

void updateMountValues() {
  const float f = mountSnapshot.forwardG;
  const float l = mountSnapshot.lateralG;
  const float u = mountSnapshot.verticalG;
  M5.Lcd.fillRect(10, 95, 225, 16, TFT_BLACK);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(mountSnapshot.valid && u > 0.70f ? UI_WHITE : UI_RED, TFT_BLACK);
  M5.Lcd.setCursor(10, 95); M5.Lcd.printf("F%+.1f L%+.1f U%+.1f", f, l, u);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setTextColor(UI_WHITE, TFT_BLACK);
}

void drawSavedScreen() {
  const RuntimeStats s = snapshotStats();
  const float rate = (s.recordsQueued > 1 && s.lastSampleUs > s.firstSampleUs)
      ? (s.recordsQueued - 1) * 1000000.0f / (s.lastSampleUs - s.firstSampleUs) : 0.0f;
  const float active = s.samplesAcquired ? 100.0f * s.activeSamples / s.samplesAcquired : 0.0f;
  drawHeader(activeSessionMode == SESSION_MODE_RAW_LAB ? "RAW SAVED" : "SAVED");
  M5.Lcd.setCursor(10, 31); M5.Lcd.print(currentFilename);
  M5.Lcd.setCursor(10, 47); M5.Lcd.printf("INT A%.0f M%.0f", s.intensityAverage, s.intensityMax);
  M5.Lcd.setCursor(10, 63); M5.Lcd.printf("ACT %.0f%% ODB %lu", active, (unsigned long)s.strokeCount);
  M5.Lcd.setCursor(10, 79); M5.Lcd.printf("LOG %.0fHz Q%lu", rate, (unsigned long)s.qDropCount);
  M5.Lcd.setTextColor((s.badGapCount || s.writerErrorCount) ? UI_RED : UI_WHITE, TFT_BLACK);
  M5.Lcd.setCursor(10, 95); M5.Lcd.printf("G%lu B%lu SAT%lu", (unsigned long)s.gap8Count,
      (unsigned long)s.badGapCount, (unsigned long)s.saturationCount);
  M5.Lcd.setTextColor(UI_WHITE, TFT_BLACK);
  M5.Lcd.setCursor(10, 115); M5.Lcd.print("A/B: MENU");
}

void drawDiagnosticsScreen() {
  const RuntimeStats s = snapshotStats();
  drawHeader("DIAGNOSTICS");
  M5.Lcd.setCursor(10, 34); M5.Lcd.printf("FS %s %luK", storageReady ? "OK" : "NO", (unsigned long)(freeFlashBytes()/1024));
  M5.Lcd.setCursor(10, 51); M5.Lcd.printf("FILES %u  MODE %s", countSessionFiles(), sessionModeName(activeSessionMode));
  M5.Lcd.setCursor(10, 68); M5.Lcd.printf("Q MAX %lu / %u", (unsigned long)s.maxQueueDepth, SAMPLE_QUEUE_LENGTH);
  M5.Lcd.setCursor(10, 85); M5.Lcd.printf("DROP %lu WRITE %lu", (unsigned long)s.qDropCount, (unsigned long)s.writerErrorCount);
  M5.Lcd.setCursor(10, 102); M5.Lcd.printf("WIFI %s C%u", wifiPanelReady ? "AP" : "OFF", wifiPanelReady ? WiFi.softAPgetStationNum() : 0);
  M5.Lcd.setCursor(10, 115); M5.Lcd.print("A/B: MENU");
}

void drawExportSelectScreen() {
  drawHeader("EXPORT SELECT");
  const uint16_t count = countSessionFiles();
  M5.Lcd.setCursor(10, 36); M5.Lcd.printf("File %u/%u", selectedFileIndex, count);
  M5.Lcd.setCursor(10, 54); M5.Lcd.print(selectedFilename[0] ? selectedFilename : "NO SESSIONS");
  M5.Lcd.setCursor(10, 72); M5.Lcd.printf("Size %lu B", (unsigned long)selectedFileSize());
  M5.Lcd.setCursor(10, 94); M5.Lcd.print("A: NEXT   B: EXPORT");
  M5.Lcd.setCursor(10, 112); M5.Lcd.print("HOLD B: MENU");
}

void drawFileManagerScreen() {
  drawHeader("MANAGE FILES");
  const uint16_t count = countSessionFiles();
  M5.Lcd.setCursor(10, 34); M5.Lcd.printf("PNT/RWL/RIM FILES: %u", count);
  const char* actions[] = { "DELETE ONE", "DELETE ALL" };
  for (uint8_t i = 0; i < 2; ++i) {
    const int16_t y = 52 + 22 * i;
    const bool selected = i == fileMenuIndex;
    M5.Lcd.fillRoundRect(10, y, 220, 19, 4, selected ? UI_RED : UI_DARK_RED);
    M5.Lcd.setTextColor(UI_WHITE, selected ? UI_RED : UI_DARK_RED);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setCursor(18, y + 2); M5.Lcd.print(actions[i]);
    M5.Lcd.setTextSize(1);
  }
  M5.Lcd.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  M5.Lcd.setCursor(10, 119); M5.Lcd.print("A: NEXT  B: SELECT  HOLD B: MENU");
  M5.Lcd.setTextColor(UI_WHITE, TFT_BLACK);
}

void drawDeleteOneSelectScreen() {
  drawHeader("DELETE ONE");
  const uint16_t count = countSessionFiles();
  if (!count) {
    M5.Lcd.setCursor(10, 51); M5.Lcd.print("NO SESSION FILES");
    M5.Lcd.setCursor(10, 112); M5.Lcd.print("A/B: FILE MENU");
    return;
  }
  M5.Lcd.setCursor(10, 36); M5.Lcd.printf("File %u/%u", deleteFileIndex, count);
  M5.Lcd.setTextColor(UI_RED, TFT_BLACK);
  M5.Lcd.setCursor(10, 56); M5.Lcd.print(deleteFilename);
  M5.Lcd.setTextColor(UI_WHITE, TFT_BLACK);
  M5.Lcd.setCursor(10, 82); M5.Lcd.print("A: NEXT   B: CONFIRM");
  M5.Lcd.setCursor(10, 112); M5.Lcd.print("HOLD B: FILE MENU");
}

void drawDeleteOneConfirmScreen() {
  drawHeader("CONFIRM DELETE");
  M5.Lcd.setTextColor(UI_RED, TFT_BLACK);
  M5.Lcd.setCursor(10, 40); M5.Lcd.print("DELETE THIS FILE?");
  M5.Lcd.setCursor(10, 61); M5.Lcd.print(deleteFilename);
  M5.Lcd.setTextColor(UI_WHITE, TFT_BLACK);
  M5.Lcd.setCursor(10, 88); M5.Lcd.print("A: CANCEL");
  M5.Lcd.setTextColor(UI_RED, TFT_BLACK);
  M5.Lcd.setCursor(10, 108); M5.Lcd.print("HOLD B: DELETE");
  M5.Lcd.setTextColor(UI_WHITE, TFT_BLACK);
}

void drawDeleteAllConfirmScreen() {
  drawHeader("DELETE ALL");
  M5.Lcd.setTextColor(UI_RED, TFT_BLACK);
  M5.Lcd.setCursor(10, 38); M5.Lcd.print("ERASE ALL PNT/RWL/RIM?");
  M5.Lcd.setCursor(10, 59); M5.Lcd.printf("FILES: %u", countSessionFiles());
  M5.Lcd.setTextColor(UI_WHITE, TFT_BLACK);
  M5.Lcd.setCursor(10, 87); M5.Lcd.print("A: CANCEL");
  M5.Lcd.setTextColor(UI_RED, TFT_BLACK);
  M5.Lcd.setCursor(10, 108); M5.Lcd.print("HOLD B: ERASE ALL");
  M5.Lcd.setTextColor(UI_WHITE, TFT_BLACK);
}

void drawStatusScreen(bool force = false) {
  const uint32_t now = millis();
  const bool changed = state != lastDrawnState;
  if (!force && !changed && (now - lastDisplayMs) < DISPLAY_PERIOD_MS) return;
  lastDisplayMs = now;

  if (state == STATE_START_COUNTDOWN) return;
  if (state == STATE_RECORDING) {
    // Pozostaw ekran START przez chwile po podwojnym sygnale; dopiero potem
    // narysuj standardowy ekran LIVE bez przerywania odczytu IMU.
    if (startBannerUntilMs && (int32_t)(now - startBannerUntilMs) < 0) return;
    if (force || changed) drawLiveFrame();
    updateLiveValues();
    lastDrawnState = state;
    return;
  }
  if (state == STATE_MOUNT_GUIDE) {
    if (force || changed) drawMountFrame();
    updateMountValues();
    lastDrawnState = state;
    return;
  }
  if (state == STATE_EXPORTING) return;
  if (!force && !changed) return;

  if (state == STATE_BOOT) {
    drawHeader("STARTING"); M5.Lcd.setCursor(10, 44); M5.Lcd.print("Initializing...");
  } else if (state == STATE_ERROR) {
    drawHeader("ERROR"); M5.Lcd.setCursor(10, 44); M5.Lcd.print(errorText); M5.Lcd.setCursor(10, 68); M5.Lcd.print("Reset after fix");
  } else if (state == STATE_VOLUME_SELECT) {
    drawVolumeSelectScreen();
  } else if (state == STATE_STORAGE_INIT_REQUIRED) {
    drawHeader("INIT STORAGE");
    M5.Lcd.setCursor(10, 38); M5.Lcd.print("LittleFS not ready");
    M5.Lcd.setCursor(10, 58); M5.Lcd.printf("Partition %luK", (unsigned long)(fsPartitionBytes/1024));
    M5.Lcd.setCursor(10, 80); M5.Lcd.print("HOLD A: FORMAT");
    M5.Lcd.setCursor(10, 96); M5.Lcd.print("ERASES ALL LOGS");
  } else if (state == STATE_STOPPING) {
    drawHeader("FINALIZING"); M5.Lcd.setCursor(10, 48); M5.Lcd.print("Draining RAM queue...");
  } else if (state == STATE_SAVED) {
    drawSavedScreen();
  } else if (state == STATE_DIAGNOSTICS) {
    drawDiagnosticsScreen();
  } else if (state == STATE_EXPORT_SELECT) {
    drawExportSelectScreen();
  } else if (state == STATE_FILE_MANAGER) {
    drawFileManagerScreen();
  } else if (state == STATE_DELETE_ONE_SELECT) {
    drawDeleteOneSelectScreen();
  } else if (state == STATE_DELETE_ONE_CONFIRM) {
    drawDeleteOneConfirmScreen();
  } else if (state == STATE_DELETE_ALL_CONFIRM) {
    drawDeleteAllConfirmScreen();
  } else if (state == STATE_MAIN_MENU) {
    drawMenuFrame();
  }
  lastDrawnState = state;
}

void enterMainMenu() {
  state = STATE_MAIN_MENU;
  drawStatusScreen(true);
}

// ---------------------------------------------------------------------------
// Kalibracja sesji podczas odliczania 10, 9, 8.
// ---------------------------------------------------------------------------
void resetCountdownCalibration(bool preserveRetryCount = false) {
  const uint8_t retryCount = preserveRetryCount ? countdownCalibration.retryCount : 0;
  memset(&countdownCalibration, 0, sizeof(countdownCalibration));
  countdownCalibration.retryCount = retryCount;
}

void collectCountdownCalibrationSample() {
  if (!M5.Imu.update()) return;
  const auto d = M5.Imu.getImuData();
  const float accNorm = vectorNorm(d.accel.x, d.accel.y, d.accel.z);
  const float gyroNorm = vectorNorm(d.gyro.x, d.gyro.y, d.gyro.z);
  const bool still = fabsf(accNorm - 1.0f) <= CALIBRATION_STABLE_ACC_TOLERANCE_G &&
                     gyroNorm <= CALIBRATION_STABLE_GYRO_DPS;
  if (!still) {
    countdownCalibration.rejectedSamples++;
    return;
  }
  countdownCalibration.gyroSumX += d.gyro.x;
  countdownCalibration.gyroSumY += d.gyro.y;
  countdownCalibration.gyroSumZ += d.gyro.z;
  countdownCalibration.accelSumX += d.accel.x;
  countdownCalibration.accelSumY += d.accel.y;
  countdownCalibration.accelSumZ += d.accel.z;
  countdownCalibration.stableSamples++;
}

bool finishCountdownCalibration() {
  const uint32_t stable = countdownCalibration.stableSamples;
  const uint32_t total = stable + countdownCalibration.rejectedSamples;
  const bool stableEnough = stable >= CALIBRATION_MIN_STABLE_SAMPLES && total > 0 &&
                            stable * 4U >= total * 3U;
  if (!stableEnough) {
    countdownCalibration.complete = false;
    countdownCalibration.lastAttemptMoved = true;
    return false;
  }
  gyroBiasX = countdownCalibration.gyroSumX / stable;
  gyroBiasY = countdownCalibration.gyroSumY / stable;
  gyroBiasZ = countdownCalibration.gyroSumZ / stable;
  const float al = pickAxis(countdownCalibration.accelSumX / stable, countdownCalibration.accelSumY / stable,
                            countdownCalibration.accelSumZ / stable, LATERAL_AXIS, LATERAL_SIGN);
  const float au = pickAxis(countdownCalibration.accelSumX / stable, countdownCalibration.accelSumY / stable,
                            countdownCalibration.accelSumZ / stable, VERTICAL_AXIS, VERTICAL_SIGN);
  calibrationLeanAbsDeg = atan2f(al, au) * 180.0f / PI;
  countdownCalibration.complete = true;
  countdownCalibration.lastAttemptMoved = false;
  Serial.printf("# COUNTDOWN_CAL_OK,bias=%.4f,%.4f,%.4f,lean0=%.2f,stable=%lu,rejected=%lu,retry=%u\n",
                gyroBiasX, gyroBiasY, gyroBiasZ, calibrationLeanAbsDeg, (unsigned long)stable,
                (unsigned long)countdownCalibration.rejectedSamples, countdownCalibration.retryCount);
  return true;
}

// ---------------------------------------------------------------------------
// Zadanie zapisu LittleFS: jedyny kontekst dotykajacy logFile po starcie.
// ---------------------------------------------------------------------------
void writerTask(void*) {
  // Jedna kolejka i jeden writer utrzymuja atomowy porzadek probek. NORMAL
  // serializuje tylko czesc PNT, RAW LAB serializuje caly rozszerzony rekord.
  // Bloki sa globalne, aby task nie alokowal ponad 4 KiB na swoim stosie przy starcie.
  size_t count = 0;
  writerTaskDone = false;
  const uint32_t writerStackStartFreeBytes =
      (uint32_t)uxTaskGetStackHighWaterMark(nullptr) * sizeof(StackType_t);
  portENTER_CRITICAL(&statsMux);
  stats.writerStackMinFreeBytes = writerStackStartFreeBytes;
  portEXIT_CRITICAL(&statsMux);

  auto writeBlock = [&](size_t records) -> bool {
    if (!records) return true;
    const bool rawLab = activeSessionMode == SESSION_MODE_RAW_LAB;
    const size_t expected = records * (rawLab ? sizeof(RawLabRecord) : sizeof(SampleRecord));
    const uint8_t* payload = rawLab ? (const uint8_t*)writerRawBlock : (const uint8_t*)writerNormalBlock;
    if (activeCompression) {
      // Blok skompresowany: [u16 rawLen][u16 compLen][u8 flags][dane].
      // flags bit0: 1=LZSS, 0=surowy (fallback gdy niescisliwe / nie miesci sie).
      const uint32_t rawLen = (uint32_t)expected;
      uint32_t compLen = lzssEncodeBlock(payload, rawLen, streamCompDst, sizeof(streamCompDst));
      uint8_t flags = 1;
      const uint8_t* dataPtr = streamCompDst;
      if (compLen == 0 || compLen >= rawLen) { // fallback: zapisz surowo
        flags = 0; compLen = rawLen; dataPtr = payload;
      }
      uint8_t bh[5] = { (uint8_t)(rawLen & 0xFF), (uint8_t)((rawLen >> 8) & 0xFF),
                        (uint8_t)(compLen & 0xFF), (uint8_t)((compLen >> 8) & 0xFF), flags };
      if (logFile.write(bh, 5) != 5 || logFile.write(dataPtr, compLen) != compLen) {
        portENTER_CRITICAL(&statsMux); stats.writerErrorCount++; portEXIT_CRITICAL(&statsMux);
        writerFailed = true;
        return false;
      }
      portENTER_CRITICAL(&statsMux); stats.recordsWritten += records; portEXIT_CRITICAL(&statsMux);
      return true;
    }
    const size_t written = logFile.write(payload, expected);
    if (written != expected) {
      portENTER_CRITICAL(&statsMux); stats.writerErrorCount++; portEXIT_CRITICAL(&statsMux);
      writerFailed = true;
      return false;
    }
    portENTER_CRITICAL(&statsMux); stats.recordsWritten += records; portEXIT_CRITICAL(&statsMux);
    return true;
  };

  while (!writerStopRequested || uxQueueMessagesWaiting(sampleQueue) > 0) {
    RawLabRecord record;
    if (xQueueReceive(sampleQueue, &record, pdMS_TO_TICKS(20)) == pdTRUE) {
      writerRawBlock[count] = record;
      writerNormalBlock[count].tUs = record.tUs;
      writerNormalBlock[count].af_mg = record.af_mg;
      writerNormalBlock[count].al_mg = record.al_mg;
      writerNormalBlock[count].au_mg = record.au_mg;
      writerNormalBlock[count].gf_dps10 = record.gf_dps10;
      writerNormalBlock[count].gl_dps10 = record.gl_dps10;
      writerNormalBlock[count].gu_dps10 = record.gu_dps10;
      writerNormalBlock[count].accNorm_mg = record.accNorm_mg;
      writerNormalBlock[count].lean_cdeg = record.lean_cdeg;
      writerNormalBlock[count].intensity_x10 = record.intensity_x10;
      writerNormalBlock[count].strokeStrength_x10 = record.strokeStrength_x10;
      writerNormalBlock[count].cadenceMs = record.cadenceMs;
      writerNormalBlock[count].surge_dps10 = record.surge_dps10;
      writerNormalBlock[count].strokePhaseMs = record.strokePhaseMs;
      writerNormalBlock[count].flags = record.flags;
      writerNormalBlock[count].side = record.side;
      ++count;
      if (count == WRITER_BLOCK_RECORDS) {
        if (!writeBlock(count)) break;
        count = 0;
      }
    }
  }

  if (!writerFailed && count) {
    writeBlock(count);
  }
  if (logFile) { logFile.flush(); logFile.close(); }
  const uint32_t writerStackFreeBytes =
      (uint32_t)uxTaskGetStackHighWaterMark(nullptr) * sizeof(StackType_t);
  portENTER_CRITICAL(&statsMux);
  stats.writerStackMinFreeBytes = writerStackFreeBytes;
  portEXIT_CRITICAL(&statsMux);
  if (discardCurrentLogOnClose && currentFilename[0] == '/') {
    const bool removed = LittleFS.remove(currentFilename);
    Serial.printf("# DISCARD_STARTUP_LOG,%s,ok=%u\n", currentFilename, removed ? 1 : 0);
    strncpy(currentFilename, "-", sizeof(currentFilename) - 1);
    currentFilename[sizeof(currentFilename) - 1] = '\0';
    discardCurrentLogOnClose = false;
  }
  writerTaskDone = true;
  vTaskDelete(nullptr);
}

// ---------------------------------------------------------------------------
// Kadencja z rytmu: autokorelacja obwiedni surge. Zwraca okres cyklu odepchniecia
// jednej nogi w sekundach (0 gdy brak wyraznego rytmu). Lekka wersja O(BUF*LAG),
// liczona raz na sekunde na rdzeniu writera — nie obciaza petli 200 Hz.
// ---------------------------------------------------------------------------
float cadencePeriodFromEnvelope() {
  if (cadenceEnvFilled < CADENCE_BUF) return 0.0f;
  // rozwin bufor kolowy do kolejnosci chronologicznej + srednia
  float seq[CADENCE_BUF];
  float mean = 0.0f;
  for (uint8_t k = 0; k < CADENCE_BUF; ++k) {
    seq[k] = cadenceEnv[(uint8_t)(cadenceEnvIdx + k) % CADENCE_BUF];
    mean += seq[k];
  }
  mean /= CADENCE_BUF;
  for (uint8_t k = 0; k < CADENCE_BUF; ++k) seq[k] -= mean;
  // autokorelacja: znajdz lag o najwyzszej korelacji
  float ac[CADENCE_LAG_MAX + 1];
  float best = -1e30f; uint8_t bestLag = 0;
  float ac0 = 0.0f; for (uint8_t k = 0; k < CADENCE_BUF; ++k) ac0 += seq[k]*seq[k];
  if (ac0 <= 1e-6f) return 0.0f; // cisza / brak sygnalu
  for (uint8_t lag = CADENCE_LAG_MIN; lag <= CADENCE_LAG_MAX; ++lag) {
    float s = 0.0f;
    for (uint8_t k = 0; k + lag < CADENCE_BUF; ++k) s += seq[k]*seq[k+lag];
    ac[lag] = s;
    if (s > best) { best = s; bestLag = lag; }
  }
  if (best <= 0.15f * ac0) return 0.0f; // rytm zbyt slaby/niepewny
  // korekta sub-harmoniczna: gdy 2*lag tez silnie koreluje, prawdziwy okres = 2*lag
  // (dwie fazy cyklu odepchniecia dawaly polowe okresu).
  uint16_t dbl = (uint16_t)bestLag * 2;
  if (dbl <= CADENCE_LAG_MAX && ac[dbl] > CADENCE_SUBHARM_RATIO * best) bestLag = (uint8_t)dbl;
  return (float)bestLag / (float)CADENCE_ENV_HZ; // okres w sekundach
}

// ---------------------------------------------------------------------------
// Zadanie IMU: wysokopriorytetowe, bez LittleFS, LCD i Serial.
// ---------------------------------------------------------------------------
void imuTask(void*) {
  OnePoleHighPass hp;
  OnePoleLowPass surgeFilter;
  OnePoleLowPass afLp;               // poprawka 2.6: LP przyspieszenia w przod dla surge
  OnePoleLowPass liveIntensityFilter;
  uint32_t previousUs = 0;
  uint32_t lastStrokeUs = 0;
  float previousAfLp = 0.0f;         // poprawka 2.6: poprzednia wartosc LP(af)
  float relLean = 0.0f;
  float intensityAccum = 0.0f;

  // Okno kandydata odepchniecia.
  bool inCandidate = false;
  float candPeakHp = 0.0f;        // szczyt przysp. hp w oknie (g)
  float candPeakSurge = 0.0f;     // max. uderzeniowosci w oknie (g/s)
  float candImpulse = 0.0f;       // calka |hp|*dt w oknie (g*s)
  uint32_t candStartUs = 0;
  uint32_t candPeakUs = 0;
  bool candConfirm = false;
  float strokeStrengthHeld = 0.0f;  // poprawka 2.2: pelna sila ostatniego odepchniecia (0..100), do rekordu
  uint32_t candLastActiveUs = 0;    // poprawka 2.8: ostatnia chwila aktywnego impulsu (do czasu trwania fazy)

  // Kadencja: ring bufor odstepow (ms) do sredniej ruchomej.
  uint16_t cadenceRing[STROKE_CADENCE_WINDOW];
  uint8_t cadenceHead = 0;
  uint8_t cadenceCount = 0;
  uint32_t cadenceSumMs = 0;

  // Kadencja z rytmu (autokorelacja): probkowanie obwiedni surge + timing.
  float cadenceEnvPeak = 0.0f;       // max surge od ostatniej probki obwiedni
  uint32_t lastEnvSampleUs = 0;
  uint32_t lastCadenceCalcUs = 0;
  float strideCadenceSmoothed = 0.0f; // kadencja KROKU (/min) — mediana z okien
  float cadenceWindow[CADENCE_MEDIAN_WINDOW] = {0}; // ostatnie pomiary kroku
  uint8_t cadenceWindowCount = 0;
  // wyzeruj bufor obwiedni na starcie sesji
  cadenceEnvIdx = 0; cadenceEnvFilled = 0;
  for (uint8_t k = 0; k < CADENCE_BUF; ++k) cadenceEnv[k] = 0.0f;

  uint32_t lastValidImuUs = 0;
  uint32_t lastStackSampleUs = 0;
  bool imuStallReported = false;
  TickType_t wake = xTaskGetTickCount();

  imuTaskDone = false;
  while (captureActive) {
    const uint32_t loopUs = micros();
    const bool imuReady = M5.Imu.update();
    if (imuReady) {
      const auto d = M5.Imu.getImuData();
      const uint32_t nowUs = loopUs;
      lastValidImuUs = nowUs;
      imuStallReported = false;
      if (previousUs == 0) { previousUs = nowUs; vTaskDelayUntil(&wake, pdMS_TO_TICKS(IMU_PERIOD_MS)); continue; }
      const float rawDt = (nowUs - previousUs) * 1.0e-6f;
      previousUs = nowUs;
      const bool gap8 = rawDt > GAP_DT_S;
      const bool badGap = rawDt < MIN_DT_S || rawDt > BAD_DT_S;
      const float dt = clampFloat(rawDt, MIN_DT_S, BAD_DT_S);

      const float af = pickAxis(d.accel.x, d.accel.y, d.accel.z, FORWARD_AXIS, FORWARD_SIGN);
      const float al = pickAxis(d.accel.x, d.accel.y, d.accel.z, LATERAL_AXIS, LATERAL_SIGN);
      const float au = pickAxis(d.accel.x, d.accel.y, d.accel.z, VERTICAL_AXIS, VERTICAL_SIGN);
      const float gf = pickAxis(d.gyro.x - gyroBiasX, d.gyro.y - gyroBiasY, d.gyro.z - gyroBiasZ, FORWARD_AXIS, FORWARD_SIGN);
      const float gl = pickAxis(d.gyro.x - gyroBiasX, d.gyro.y - gyroBiasY, d.gyro.z - gyroBiasZ, LATERAL_AXIS, LATERAL_SIGN);
      const float gu = pickAxis(d.gyro.x - gyroBiasX, d.gyro.y - gyroBiasY, d.gyro.z - gyroBiasZ, VERTICAL_AXIS, VERTICAL_SIGN);
      const float rollRate = pickAxis(d.gyro.x - gyroBiasX, d.gyro.y - gyroBiasY, d.gyro.z - gyroBiasZ, ROLL_GYRO_AXIS, ROLL_GYRO_SIGN);
      const float accNorm = vectorNorm(af, al, au);
      const float gyroNorm = vectorNorm(gf, gl, gu);

      // Sygnaly techniki v4: przysp. hp w przod, uderzeniowosc (szybka zmiana a),
      // intensywnosc i sila chwilowa (strefy B).
      const float hpAf = hp.update(af, dt, HP_CUTOFF_HZ);
      // Poprawka 2.6: surge = pochodna WYGLADZONEGO przyspieszenia w przod, bez
      // podwojnego rozniczkowania hpAf (ktore wzmacnialo szum). Najpierw LP na af,
      // potem jedna roznica, na koncu to samo wygladzanie surgeFilter co wczesniej.
      const float afLpNow = afLp.update(af, dt, SURGE_LP_CUTOFF_HZ);
      const float surgeRaw = (afLpNow - previousAfLp) / dt;
      previousAfLp = afLpNow;
      const float surge = surgeFilter.update(surgeRaw, dt, SURGE_LP_CUTOFF_HZ);

      // --- Kadencja z rytmu: obwiednia surge (max) probkowana co 50 ms do bufora ---
      const float surgeAbs = fabsf(surge);
      if (surgeAbs > cadenceEnvPeak) cadenceEnvPeak = surgeAbs;
      if (!lastEnvSampleUs) lastEnvSampleUs = nowUs;
      if ((uint32_t)(nowUs - lastEnvSampleUs) >= CADENCE_ENV_PERIOD_US) {
        cadenceEnv[cadenceEnvIdx] = cadenceEnvPeak;
        cadenceEnvIdx = (uint8_t)((cadenceEnvIdx + 1) % CADENCE_BUF);
        if (cadenceEnvFilled < CADENCE_BUF) cadenceEnvFilled++;
        cadenceEnvPeak = 0.0f;
        lastEnvSampleUs = nowUs;
      }

      const float intensity = 100.0f * fminf(1.0f,
          INTENSITY_ACC_WEIGHT * fmaxf(0.0f, hpAf) / INTENSITY_ACC_REF_G +
          INTENSITY_SURGE_WEIGHT * fmaxf(0.0f, surge) / INTENSITY_SURGE_REF_GPS);
      const float intensityLive = liveIntensityFilter.update(intensity, dt, LIVE_INTENSITY_CUTOFF_HZ);

      // Sila chwilowa (dla stref B) i calka impulsu kandydata (do pelnej sily).
      const float strengthInstant = 100.0f * fminf(1.0f,
          STROKE_W_PEAK * fmaxf(0.0f, hpAf) / STROKE_REF_PEAK_G +
          STROKE_W_SURGE * fmaxf(0.0f, surge) / STROKE_REF_SURGE_GPS);

      // Przechyl wzgledny z bardzo lagodna korekta grawitacyjna.
      const float leanAbs = atan2f(al, au) * 180.0f / PI;
      const float leanAccRel = wrapDeg180(leanAbs - calibrationLeanAbsDeg);
      const float linearMotion = clampFloat(fabsf(accNorm - 1.0f) / 0.50f, 0.0f, 1.0f);
      const float angularMotion = clampFloat(gyroNorm / 500.0f, 0.0f, 1.0f);
      const float correction = LEAN_BASE_CORRECTION + (1.0f - fmaxf(linearMotion, angularMotion)) *
                               (LEAN_MAX_CORRECTION - LEAN_BASE_CORRECTION);
      relLean += rollRate * dt;
      relLean = wrapDeg180(relLean);
      relLean += correction * wrapDeg180(leanAccRel - relLean);
      // Poprawka 2.7: przy bezruchu (brak ruchu liniowego i obrotowego) powoli
      // sciagaj integrator ku kalibracji, kasujac skumulowany dryf zyroskopu.
      const float restFactor = 1.0f - fmaxf(linearMotion, angularMotion);
      if (restFactor > 0.0f) {
        const float decay = clampFloat(LEAN_IDLE_DECAY_PER_S * restFactor * dt, 0.0f, 1.0f);
        relLean += decay * (leanAccRel - relLean);
      }
      relLean = clampFloat(relLean, -REL_LEAN_LIMIT_DEG, REL_LEAN_LIMIT_DEG);

      // --- DETEKTOR ODPCHNIECIA (STROKE) v4 ---
      // Okno kandydata otwierane po wejsciu (surge lub przysp. hp), potwierdzane
      // w oknie, zamykane po STROKE_WINDOW_US. Refrakcja odrzuca ten sam impuls.
      bool strokePeak = false;
      float strokeStrength = 0.0f;
      float strokePhaseMs = 0.0f;
      if (!inCandidate) {
        if (surge >= STROKE_ENTER_SURGE_GPS || hpAf >= STROKE_ENTER_HP_G) {
          inCandidate = true;
          candPeakHp = hpAf;
          candPeakSurge = surge;
          candImpulse = fabsf(hpAf) * dt;
          candStartUs = nowUs;
          candPeakUs = nowUs;
          candLastActiveUs = nowUs; // poprawka 2.8: start fazy aktywnej
          candConfirm = (surge >= STROKE_CONFIRM_SURGE_GPS || hpAf >= STROKE_CONFIRM_HP_G);
        }
      } else {
        if (hpAf > candPeakHp) { candPeakHp = hpAf; candPeakUs = nowUs; }
        if (surge > candPeakSurge) candPeakSurge = surge;
        candImpulse += fabsf(hpAf) * dt;
        // Poprawka 2.8: dopoki sygnal jest powyzej progu wejscia, faza trwa.
        if (surge >= STROKE_ENTER_SURGE_GPS || hpAf >= STROKE_ENTER_HP_G) candLastActiveUs = nowUs;
        if (!candConfirm && (surge >= STROKE_CONFIRM_SURGE_GPS || hpAf >= STROKE_CONFIRM_HP_G)) {
          candConfirm = true;
        }
        if ((uint32_t)(nowUs - candStartUs) >= STROKE_WINDOW_US) {
          if (candConfirm) {
            // Refrakcja: szczyty zbyt czeste traktujemy jako ten sam impuls.
            if (lastStrokeUs == 0 || (uint32_t)(candPeakUs - lastStrokeUs) >= STROKE_REFRACTORY_US) {
              strokePeak = true;
              // SILA odepchniecia: szczyt + calka impulsu + uderzeniowosc.
              const float normPeak = fminf(candPeakHp / STROKE_REF_PEAK_G, 2.0f);
              const float normImpulse = fminf(candImpulse / STROKE_REF_IMPULSE_GS, 2.0f);
              const float normSurge = fminf(candPeakSurge / STROKE_REF_SURGE_GPS, 2.0f);
              strokeStrength = 100.0f * (STROKE_W_PEAK * normPeak +
                                         STROKE_W_IMPULSE * normImpulse +
                                         STROKE_W_SURGE * normSurge);
              strokeStrength = clampFloat(strokeStrength, 0.0f, 100.0f);
              strokeStrengthHeld = strokeStrength; // poprawka 2.2: zapamietaj pelna sile do rekordu
              // Poprawka 2.8: rzeczywisty czas TRWANIA fazy aktywnej (start -> ostatnia
              // chwila powyzej progu), a nie tylko czas narastania do szczytu.
              strokePhaseMs = clampFloat((candLastActiveUs - candStartUs) * 0.001f, 0.0f, 250.0f);

              // Kadencja: srednia ruchoma z N ostatnich odstepow (ms).
              const uint16_t intervalMs =
                  (uint16_t)clampU16((float)(candPeakUs - lastStrokeUs) * 0.001f);
              if (lastStrokeUs != 0 && intervalMs > 0 && intervalMs < 4000) {
                if (cadenceCount == STROKE_CADENCE_WINDOW) {
                  cadenceSumMs -= cadenceRing[cadenceHead];
                } else {
                  cadenceCount++;
                }
                cadenceRing[cadenceHead] = intervalMs;
                cadenceHead = (cadenceHead + 1) % STROKE_CADENCE_WINDOW;
                cadenceSumMs += intervalMs;
              }
              lastStrokeUs = candPeakUs;
            }
          }
          inCandidate = false;
        }
      }

      const bool idle = fabsf(accNorm - 1.0f) < 0.12f && gyroNorm < 25.0f;
      const bool saturated = fabsf(af) > 15.8f || fabsf(al) > 15.8f || fabsf(au) > 15.8f ||
                             fabsf(gf) > 1990.0f || fabsf(gl) > 1990.0f || fabsf(gu) > 1990.0f;
      const bool active = intensityLive >= ACTIVE_INTENSITY_THRESHOLD || gyroNorm >= ACTIVE_GYRO_THRESHOLD_DPS;
      const bool strokeStrong = strokePeak && strokeStrength >= STROKE_STRONG_MIN;

      RawLabRecord record = {};
      record.tUs = nowUs - sessionStartUs;
      // Osie fizyczne przekazywane przez BMI270/M5Unified, przed mapowaniem i biasem.
      record.rawAx_mg = clampI16(d.accel.x * 1000.0f);
      record.rawAy_mg = clampI16(d.accel.y * 1000.0f);
      record.rawAz_mg = clampI16(d.accel.z * 1000.0f);
      record.rawGx_dps10 = clampI16(d.gyro.x * 10.0f);
      record.rawGy_dps10 = clampI16(d.gyro.y * 10.0f);
      record.rawGz_dps10 = clampI16(d.gyro.z * 10.0f);
      // Slad przetworzenia aktywny w tym samym firmware i tej samej probce.
      record.af_mg = clampI16(af * 1000.0f);
      record.al_mg = clampI16(al * 1000.0f);
      record.au_mg = clampI16(au * 1000.0f);
      record.gf_dps10 = clampI16(gf * 10.0f);
      record.gl_dps10 = clampI16(gl * 10.0f);
      record.gu_dps10 = clampI16(gu * 10.0f);
      record.accNorm_mg = clampU16(accNorm * 1000.0f);
      record.lean_cdeg = clampI16(relLean * 100.0f);
      record.intensity_x10 = clampU16(intensity * 10.0f);
      record.strokeStrength_x10 = clampU16(strokeStrengthHeld * 10.0f); // poprawka 2.2: pelna sila (peak+impuls+surge), nie chwilowa
      record.cadenceMs = clampU16(cadenceCount ? (float)(cadenceSumMs / cadenceCount) : 0.0f);
      record.surge_dps10 = clampI16(surge * 10.0f);
      record.intensitySmooth_x10 = clampU16(intensityLive * 10.0f); // poprawka 2.3: pole bez znaku (0..1000)
      record.strokePhaseMs = strokePeak ? (uint8_t)clampU16(strokePhaseMs) : 0;
      record.flags = 0;
      if (idle) record.flags |= FLAG_IDLE;
      if (strokePeak) record.flags |= FLAG_STROKE;
      if (strokeStrong) record.flags |= FLAG_STROKE_STRONG;
      if (saturated) record.flags |= FLAG_SATURATION;
      if (gap8 || badGap) record.flags |= FLAG_TIME_GAP;
      record.side = DEVICE_SIDE;

      const BaseType_t queued = xQueueSend(sampleQueue, &record, 0);
      const uint32_t depth = uxQueueMessagesWaiting(sampleQueue);
      const bool stackSampleDue = !lastStackSampleUs ||
          (uint32_t)(nowUs - lastStackSampleUs) >= IMU_STACK_SAMPLE_PERIOD_US;
      const uint32_t imuStackFreeBytes = stackSampleDue
          ? (uint32_t)uxTaskGetStackHighWaterMark(nullptr) * sizeof(StackType_t) : 0;
      if (stackSampleDue) lastStackSampleUs = nowUs;

      // Kadencja z rytmu: raz na sekunde policz okres (poza sekcja krytyczna — funkcja ciezsza).
      bool cadenceDue = !lastCadenceCalcUs || (uint32_t)(nowUs - lastCadenceCalcUs) >= CADENCE_CALC_PERIOD_US;
      float strideCadNow = strideCadenceSmoothed;
      if (cadenceDue) {
        lastCadenceCalcUs = nowUs;
        const float period = cadencePeriodFromEnvelope(); // okres cyklu nogi [s], 0 = brak rytmu
        if (period > 0.0f) {
          const float strideCad = (60.0f / period) * 2.0f; // KROK = obie nogi
          // przesun okno i policz mediane (odporna na sub/nad-harmoniczne wahniecia)
          for (uint8_t k = CADENCE_MEDIAN_WINDOW - 1; k > 0; --k) cadenceWindow[k] = cadenceWindow[k-1];
          cadenceWindow[0] = strideCad;
          if (cadenceWindowCount < CADENCE_MEDIAN_WINDOW) cadenceWindowCount++;
          float tmp[CADENCE_MEDIAN_WINDOW];
          for (uint8_t k = 0; k < cadenceWindowCount; ++k) tmp[k] = cadenceWindow[k];
          for (uint8_t a = 0; a < cadenceWindowCount; ++a)   // sortowanie babelkowe (max 5 el.)
            for (uint8_t bb = a + 1; bb < cadenceWindowCount; ++bb)
              if (tmp[bb] < tmp[a]) { float t = tmp[a]; tmp[a] = tmp[bb]; tmp[bb] = t; }
          strideCadenceSmoothed = tmp[cadenceWindowCount / 2];
        } else {
          // brak wyraznego rytmu (postoj/bezruch) — wyczysc okno
          cadenceWindowCount = 0;
          strideCadenceSmoothed = 0.0f;
        }
        strideCadNow = strideCadenceSmoothed;
      }

      portENTER_CRITICAL(&statsMux);
      stats.samplesAcquired++;
      if (!stats.firstSampleUs) stats.firstSampleUs = record.tUs;
      stats.lastSampleUs = record.tUs;
      if (gap8) stats.gap8Count++;
      if (badGap) stats.badGapCount++;
      if (saturated) stats.saturationCount++;
      if (idle) stats.idleSamples++;
      if (active) stats.activeSamples++;
      // Histogramy do P90/P95 (poprawka 2.9: indeks jawnie ograniczony do 0..100,
      // percentyle liczone tylko dla probek AKTYWNYCH, aby tlo bezruchu ich nie zanizalo).
      if (!idle) {
        stats.intensityHist[(uint8_t)clampFloat(intensityLive, 0.0f, 100.0f)]++;
        stats.strokeHist[(uint8_t)clampFloat(strengthInstant, 0.0f, 100.0f)]++;
      }
      if (strokePeak) {
        const uint32_t phaseMs = (uint32_t)lroundf(strokePhaseMs);
        stats.strokeCount++;
        stats.strokeSum100 += (uint32_t)lroundf(strokeStrength);
        stats.strokeMax100 = fmaxf(stats.strokeMax100, strokeStrength);
        stats.strokePhaseSumMs += phaseMs;
        if (phaseMs > stats.strokePhaseMaxMs) stats.strokePhaseMaxMs = phaseMs;
        if (strokeStrong) stats.strokeStrongCount++;
        stats.strokeStrengthNow = strokeStrength;
      }
      if (queued == pdTRUE) stats.recordsQueued++; else stats.qDropCount++;
      if (depth > stats.maxQueueDepth) stats.maxQueueDepth = depth;
      if (stackSampleDue) stats.imuStackMinFreeBytes = imuStackFreeBytes;
      // Strefy B — liczone z sily chwilowej.
      const uint8_t zone = strokeStrengthZone(strengthInstant);
      if (zone == 0) stats.zone0Count++;
      else if (zone == 1) stats.zone1Count++;
      else if (zone == 2) stats.zone2Count++;
      else if (zone == 3) stats.zone3Count++;
      else stats.zone4Count++;
      stats.intensityNow = intensityLive;
      stats.intensityMax = fmaxf(stats.intensityMax, intensityLive);
      intensityAccum += intensityLive;
      stats.intensityAverage = stats.samplesAcquired ? intensityAccum / stats.samplesAcquired : 0.0f;
      stats.surgeNow = surge;
      stats.surgeMax = fmaxf(stats.surgeMax, surge);
      stats.spinNow = gyroNorm;
      stats.gloadNow = accNorm;
      stats.relLeanDeg = relLean;
      // Kadencja i moc proxy.
      stats.cadenceMsNow = cadenceCount ? (uint16_t)(cadenceSumMs / cadenceCount) : 0;
      stats.cadencePerMin = stats.cadenceMsNow ? 60000.0f / stats.cadenceMsNow : 0.0f;
      // Kadencja z rytmu (dokladniejsza): KROK = obie nogi, oraz kadencja tej nogi.
      stats.strideCadencePerMin = strideCadNow;
      stats.legCadencePerMin = strideCadNow * 0.5f;
      // Moc proxy: uzyj kadencji krokowej z rytmu gdy dostepna, inaczej dawnej.
      const float cadForPower = strideCadNow > 0.0f ? strideCadNow : stats.cadencePerMin;
      stats.powerProxyNow = cadForPower * stats.strokeStrengthNow / 100.0f;
      stats.powerProxyMax = fmaxf(stats.powerProxyMax, stats.powerProxyNow);
      portEXIT_CRITICAL(&statsMux);
      appendLiveHistory(record.tUs, intensityLive, strengthInstant, (float)stats.cadenceMsNow,
                        gyroNorm, relLean, strokePeak, strokePhaseMs);
    } else if (lastValidImuUs && !imuStallReported &&
               (uint32_t)(loopUs - lastValidImuUs) >= IMU_STALL_THRESHOLD_US) {
      portENTER_CRITICAL(&statsMux);
      stats.imuStallCount++;
      portEXIT_CRITICAL(&statsMux);
      imuStallReported = true;
    }
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(IMU_PERIOD_MS));
  }
  const uint32_t imuStackFreeBytes =
      (uint32_t)uxTaskGetStackHighWaterMark(nullptr) * sizeof(StackType_t);
  portENTER_CRITICAL(&statsMux);
  stats.imuStackMinFreeBytes = imuStackFreeBytes;
  portEXIT_CRITICAL(&statsMux);
  imuTaskDone = true;
  vTaskDelete(nullptr);
}

// ---------------------------------------------------------------------------
// Start i stop sesji
// ---------------------------------------------------------------------------
void resetStats() {
  portENTER_CRITICAL(&statsMux);
  memset(&stats, 0, sizeof(stats));
  portEXIT_CRITICAL(&statsMux);
  resetLiveHistory();
}

bool startSessionNow() {
  if (!storageReady) { setError("FLASH NOT READY"); return false; }
  const uint32_t requiredFree = selectedSessionMode == SESSION_MODE_RAW_LAB
      ? RAW_LAB_MIN_FREE_BYTES_TO_START : MIN_FREE_BYTES_TO_START;
  if (freeFlashBytes() < requiredFree) { setError(selectedSessionMode == SESSION_MODE_RAW_LAB ? "RAW LAB NEEDS 192K" : "FLASH NEAR FULL"); return false; }
  if (sampleQueue || !imuTaskDone || !writerTaskDone) { setError("TASKS NOT READY"); return false; }

  activeSessionMode = selectedSessionMode;
  activeCompression = selectedCompression;
  sessionId = nextSessionId();
  if (!sessionId) { setError("SESSION ID WRITE FAIL"); return false; }
  const char* ext;
  if (activeCompression) {
    ext = activeSessionMode == SESSION_MODE_RAW_LAB ? STREAM_COMPRESSED_RAW_EXTENSION
                                                    : STREAM_COMPRESSED_EXTENSION;
  } else {
    ext = sessionExtensionForMode(activeSessionMode);
  }
  snprintf(currentFilename, sizeof(currentFilename), "/ses%05lu%s", (unsigned long)sessionId, ext);
  discardCurrentLogOnClose = false;
  logFile = LittleFS.open(currentFilename, FILE_WRITE);
  if (!logFile) { setError("OPEN LOG FAILED"); return false; }

  SessionHeader header = {};
  // Poprawka 2.1: pelne, bezpieczne kopiowanie 8-bajtowego magic (jawny NUL),
  // spojny formatVersion=4 dla obu formatow generacji v4 (format rozroznia magic).
  // Magic: skompresowane strumieniowo maja wlasny magic (RIMUZ1/RRAWZ1),
  // aby parser od razu wiedzial, ze plik jest ciagiem blokow LZSS.
  const char* magicStr = activeCompression
      ? (activeSessionMode == SESSION_MODE_RAW_LAB ? "RRAWZ1" : "RIMUZ1")
      : (activeSessionMode == SESSION_MODE_RAW_LAB ? "RRAW02" : "RIMU04");
  memset(header.magic, 0, sizeof(header.magic));
  for (size_t i = 0; i < sizeof(header.magic) && magicStr[i]; ++i) header.magic[i] = magicStr[i];
  header.formatVersion = 4;
  header.headerBytes = sizeof(SessionHeader);
  header.sessionId = sessionId;
  header.startMillis = millis();
  header.targetRateHz = 200;
  header.forwardAxis = FORWARD_AXIS; header.forwardSign = FORWARD_SIGN;
  header.lateralAxis = LATERAL_AXIS; header.lateralSign = LATERAL_SIGN;
  header.verticalAxis = VERTICAL_AXIS; header.verticalSign = VERTICAL_SIGN;
  header.rollGyroAxis = ROLL_GYRO_AXIS; header.rollGyroSign = ROLL_GYRO_SIGN;
  header.gyroBiasX = gyroBiasX; header.gyroBiasY = gyroBiasY; header.gyroBiasZ = gyroBiasZ;
  // Poprawka 2.1: rozmiar rekordu zapisany jawnie dla OBU formatow (dawniej 0 dla NORMAL).
  header.recordBytes = activeSessionMode == SESSION_MODE_RAW_LAB ? sizeof(RawLabRecord) : sizeof(SampleRecord);
  // Meta v4: strona rolki, strefy B, referencje sily.
  header.side = DEVICE_SIDE;
  header.zoneVersion = 2;
  // Pakowanie strumieniowe: reservedByte=flaga kompresji (bit0=1 LZSS blokowy),
  // reservedByte2=liczba rekordow na blok (parser nie musi zgadywac).
  header.reservedByte = activeCompression ? 1 : 0;
  header.reservedByte2 = activeCompression ? (uint8_t)WRITER_BLOCK_RECORDS : 0;
  header.refPeakG = STROKE_REF_PEAK_G;
  header.refImpulseGs = STROKE_REF_IMPULSE_GS;
  header.refSurgeGps = STROKE_REF_SURGE_GPS;
  if (logFile.write((const uint8_t*)&header, sizeof(header)) != sizeof(header)) {
    discardCurrentLogFileNow(); setError("WRITE HEADER FAIL"); return false;
  }
  logFile.flush();

  resetStats();
  writerFailed = false;
  writerStopRequested = false;
  stopWriterWhenImuDone = false;
  sessionStartUs = micros();
  sampleQueue = xQueueCreate(SAMPLE_QUEUE_LENGTH, sizeof(RawLabRecord));
  if (!sampleQueue) { discardCurrentLogFileNow(); setError("NO RAM QUEUE"); return false; }

  // Ustaw flagi na false dopiero przed faktycznym utworzeniem zadania.
  imuTaskDone = true;
  writerTaskDone = false;
  if (xTaskCreatePinnedToCore(writerTask, "rim_writer", WRITER_TASK_STACK_BYTES, nullptr, WRITER_TASK_PRIORITY,
                              &writerTaskHandle, WRITER_TASK_CORE) != pdPASS) {
    writerTaskDone = true;
    vQueueDelete(sampleQueue); sampleQueue = nullptr;
    discardCurrentLogFileNow(); setError("WRITER TASK FAIL"); return false;
  }
  captureActive = true;
  imuTaskDone = false;
  if (xTaskCreatePinnedToCore(imuTask, "imu_200hz", 6144, nullptr, IMU_TASK_PRIORITY,
                              &imuTaskHandle, IMU_TASK_CORE) != pdPASS) {
    // writerTask sam zamknie, a nastepnie usunie pusty plik startowy.
    captureActive = false;
    imuTaskDone = true;
    discardCurrentLogOnClose = true;
    writerStopRequested = true;
    const uint32_t deadline = millis() + 2000;
    while (!writerTaskDone && millis() < deadline) delay(2);
    cleanupSessionResources();
    setError("IMU TASK FAIL"); return false;
  }
  state = STATE_RECORDING;
  startBannerUntilMs = millis() + START_BANNER_MS;
  drawStartBanner();
  playStartSignal();
  Serial.printf("# SESSION_START_%s,%lu,%s,queue=%u,record=%u\n", sessionModeName(activeSessionMode),
                (unsigned long)sessionId, currentFilename, SAMPLE_QUEUE_LENGTH,
                activeSessionMode == SESSION_MODE_RAW_LAB ? (unsigned)sizeof(RawLabRecord) : (unsigned)sizeof(SampleRecord));
  return true;
}

void enterVolumeSelect(SessionMode mode = SESSION_MODE_NORMAL) {
  selectedSessionMode = mode;
  volumeSelection = 0;
  state = STATE_VOLUME_SELECT;
  drawStatusScreen(true);
}

void beginStartCountdown() {
  if (!storageReady) { setError("FLASH NOT READY"); return; }
  applySessionSpeakerVolume();
  resetCountdownCalibration();
  calibrationRetryUntilMs = 0;
  countdownStep = 0;
  nextCountdownMs = millis();
  startBannerUntilMs = 0;
  state = STATE_START_COUNTDOWN;
  drawCountdownScreen(countdownStep);
}

void updateStartCountdown() {
  if (state != STATE_START_COUNTDOWN) return;
  const uint32_t now = millis();

  if (calibrationRetryUntilMs) {
    if ((int32_t)(now - calibrationRetryUntilMs) < 0) return;
    calibrationRetryUntilMs = 0;
    resetCountdownCalibration(true);
    countdownStep = 0;
    nextCountdownMs = now;
    drawCountdownScreen(countdownStep);
    return;
  }

  if (countdownStep < CALIBRATION_COUNTDOWN_STEPS && !countdownCalibration.complete) {
    collectCountdownCalibrationSample();
  }
  if ((int32_t)(now - nextCountdownMs) < 0) return;

  if (countdownStep == CALIBRATION_COUNTDOWN_STEPS && !countdownCalibration.complete) {
    if (!finishCountdownCalibration()) {
      countdownCalibration.retryCount++;
      calibrationRetryUntilMs = now + CALIBRATION_RETRY_NOTICE_MS;
      countdownStep = 0;
      nextCountdownMs = calibrationRetryUntilMs;
      drawCountdownScreen(countdownStep);
      playTone(440.0f, 130, true);
      Serial.printf("# COUNTDOWN_CAL_RETRY,stable=%lu,rejected=%lu,retry=%u\n",
                    (unsigned long)countdownCalibration.stableSamples,
                    (unsigned long)countdownCalibration.rejectedSamples, countdownCalibration.retryCount);
      return;
    }
  }

  if (countdownStep < 10) {
    drawCountdownScreen(countdownStep);
    const bool calibrationPhase = countdownStep < CALIBRATION_COUNTDOWN_STEPS;
    playTone(calibrationPhase ? 1760.0f : COUNTDOWN_TONE_HZ,
             calibrationPhase ? CALIBRATION_FAST_BEEP_MS : COUNTDOWN_BEEP_MS, true);
    countdownStep++;
    nextCountdownMs += COUNTDOWN_PERIOD_MS;
    return;
  }
  if (!startSessionNow()) {
    if (speakerReady) M5.Speaker.stop();
    restoreDefaultSpeakerVolume();
    return;
  }
}

void stopSession() {
  if (state != STATE_RECORDING) return;
  state = STATE_STOPPING;
  drawStatusScreen(true);
  captureActive = false;
  const uint32_t imuDeadline = millis() + 1500;
  while (!imuTaskDone && millis() < imuDeadline) { M5.update(); delay(2); }
  // Nie zatrzymuj writerTask, gdy imuTask moze jeszcze dodac rekord do kolejki.
  // Glowna petla wykona odroczone zatrzymanie writerTask po zakonczeniu imuTask.
  if (!imuTaskDone) {
    stopWriterWhenImuDone = true;
    setError("IMU STOP TIMEOUT");
    return;
  }

  writerStopRequested = true;
  const uint32_t writerDeadline = millis() + 6000;
  while (!writerTaskDone && millis() < writerDeadline) { M5.update(); delay(5); }
  if (!writerTaskDone) { setError("WRITER STOP TIMEOUT"); return; }

  cleanupSessionResources();
  if (writerFailed) { setError("LOG WRITE FAILED"); return; }

  const RuntimeStats s = snapshotStats();
  playSavedSignal();
  // Pozwol dlugiemu sygnalowi wybrzmiec z glosnoscia sesji, potem wroc do domyslnej.
  volumeResetAtMs = millis() + SAVED_TONE_DURATION_MS + 50;
  state = STATE_SAVED;
  Serial.printf("# SESSION_STOP_V4,%s,ACQ,%lu,QUEUED,%lu,WRITTEN,%lu,GAP8,%lu,BAD,%lu,QDROP,%lu,SAT,%lu,STROKE,%lu,STRONG,%lu\n",
                currentFilename, (unsigned long)s.samplesAcquired, (unsigned long)s.recordsQueued,
                (unsigned long)s.recordsWritten, (unsigned long)s.gap8Count, (unsigned long)s.badGapCount,
                (unsigned long)s.qDropCount, (unsigned long)s.saturationCount, (unsigned long)s.strokeCount,
                (unsigned long)s.strokeStrongCount);
}

// ---------------------------------------------------------------------------
// Lokalny panel Wi-Fi dla telefonu
// ---------------------------------------------------------------------------
const char PHONE_DASHBOARD_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html lang="pl">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<meta name="theme-color" content="#090b10">
<meta name="color-scheme" content="dark">
<title>Rolki Silesia · IMU Live</title>
<style>
:root{--bg:#080a0f;--surface:#121722;--surface2:#1a2130;--line:#28313f;--line2:#323d4d;--text:#f4f7fb;--muted:#93a1b4;--muted2:#6c7a8c;--red:#ff2d55;--red2:#9b1124;--orange:#ff8c42;--cyan:#4cd6ea;--yellow:#ffd166;--green:#4ade9b;--danger:#ff5c5c;--z0:#c3d0de;--z1:#5ec7e8;--z2:#ffd166;--z3:#ff9f1c;--z4:#ff3d71;--accent:var(--red);--accentSoft:rgba(255,45,85,.14);--shadow:0 18px 46px rgba(0,0,0,.42);--radius:18px}
*{box-sizing:border-box}html{background:var(--bg)}
body{margin:0;background:radial-gradient(120% 60% at 100% -8%,rgba(255,45,85,.14) 0,transparent 46%),radial-gradient(90% 50% at -10% 4%,rgba(76,214,234,.08) 0,transparent 42%),var(--bg);color:var(--text);font-family:Inter,ui-sans-serif,system-ui,-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif;font-variant-numeric:tabular-nums;line-height:1.35;-webkit-text-size-adjust:100%}
.app{max-width:820px;margin:auto;padding:calc(12px + env(safe-area-inset-top)) 14px calc(150px + env(safe-area-inset-bottom))}
.hero{position:relative;overflow:hidden;background:linear-gradient(135deg,rgba(255,45,85,.16),rgba(18,23,34,.6) 60%),var(--surface);border:1px solid var(--line2);border-radius:var(--radius);padding:15px 16px;box-shadow:var(--shadow)}
.hero::after{content:"";position:absolute;inset:0;background:radial-gradient(60% 120% at 92% 0,rgba(255,45,85,.22),transparent 60%);pointer-events:none}
.eyebrow{color:var(--red);font-size:11px;font-weight:800;letter-spacing:.2em}
.heroRow{display:flex;justify-content:space-between;align-items:center;gap:10px;margin-top:3px;position:relative;z-index:1}
.hero h1{font-size:clamp(24px,7vw,32px);line-height:1;margin:0;letter-spacing:-.045em;font-weight:850}
.network{font-size:11px;color:var(--muted);text-align:right;line-height:1.5}
.network b{color:var(--text);font-weight:700}
.status{display:flex;align-items:center;gap:8px;margin-top:14px;padding:10px 12px;border-radius:12px;background:rgba(0,0,0,.34);border:1px solid var(--line);font-size:12px;font-weight:800;letter-spacing:.02em;position:relative;z-index:1}
.pulse{width:9px;height:9px;border-radius:50%;background:var(--green);box-shadow:0 0 0 5px rgba(74,222,155,.14);flex-shrink:0}
.status.recording{border-color:rgba(255,45,85,.4)}
.status.recording .pulse{background:var(--red);box-shadow:0 0 0 5px rgba(255,45,85,.16);animation:pulse 1.2s infinite}
.status.error .pulse{background:var(--danger);box-shadow:0 0 0 5px rgba(255,92,92,.16)}
@keyframes pulse{50%{transform:scale(1.4);opacity:.5}}
.nav{display:grid;grid-template-columns:repeat(6,1fr);gap:6px;margin:13px 0;position:sticky;top:calc(env(safe-area-inset-top));z-index:4}
.nav button{min-height:46px;padding:6px 2px;background:rgba(18,23,34,.86);backdrop-filter:blur(8px);border:1px solid var(--line);border-radius:12px;color:var(--muted);font-size:10px;font-weight:800;letter-spacing:.04em;transition:background .15s,color .15s,border-color .15s}
.nav button.active{background:linear-gradient(160deg,rgba(255,45,85,.32),rgba(255,45,85,.12));border-color:var(--red);color:#fff;box-shadow:0 6px 16px rgba(255,45,85,.18)}
button{font:inherit;cursor:pointer}
button:focus-visible{outline:3px solid var(--cyan);outline-offset:2px}
button:disabled{opacity:.36;cursor:not-allowed}
.view{display:none;animation:fade .22s ease}.view.active{display:block}
@keyframes fade{from{opacity:0;transform:translateY(4px)}to{opacity:1;transform:none}}
.sectionTitle{display:flex;justify-content:space-between;align-items:baseline;gap:8px;margin:18px 2px 9px}
.sectionTitle h2{font-size:12px;margin:0;letter-spacing:.12em;color:var(--muted);font-weight:800;text-transform:uppercase}
.sectionTitle span{font-size:11px;color:var(--muted2)}
/* HERO metryka SILY — dominujaca, kolor wg strefy */
.bigCard{position:relative;overflow:hidden;border-radius:var(--radius);border:1px solid var(--line2);padding:16px 18px 18px;background:linear-gradient(160deg,var(--accentSoft),rgba(18,23,34,.5) 62%),var(--surface);box-shadow:var(--shadow);transition:border-color .4s,background .4s}
.bigCard::before{content:"";position:absolute;left:0;top:0;bottom:0;width:5px;background:var(--accent);box-shadow:0 0 20px var(--accent);transition:background .4s,box-shadow .4s}
.bigTop{display:flex;justify-content:space-between;align-items:flex-start;gap:10px}
.bigLabel{color:var(--muted);font-size:11px;font-weight:800;letter-spacing:.14em}
.bigZone{font-size:12px;font-weight:850;letter-spacing:.06em;color:var(--accent);padding:5px 11px;border:1px solid var(--accent);border-radius:999px;white-space:nowrap;transition:color .4s,border-color .4s}
.bigValue{display:flex;align-items:baseline;gap:6px;margin-top:6px}
.bigValue #strokeStrength{font-size:clamp(64px,24vw,104px);font-weight:850;line-height:.9;letter-spacing:-.05em}
.bigValue .unit{font-size:20px;color:var(--muted);font-weight:700}
.gauge{height:12px;border-radius:999px;background:#0b0e14;border:1px solid var(--line);margin-top:12px;overflow:hidden;position:relative}
.gaugeFill{height:100%;width:0;border-radius:999px;background:linear-gradient(90deg,var(--z1),var(--z2) 45%,var(--z3) 72%,var(--z4));transition:width .3s ease}
/* pary metryk drugorzednych */
.duo{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-top:10px}
.metrics{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:10px}
.metric{position:relative;min-height:100px;padding:13px 14px;border:1px solid var(--line);border-radius:15px;background:linear-gradient(150deg,var(--surface2),var(--surface));box-shadow:0 10px 22px rgba(0,0,0,.16)}
.metric.primary{background:linear-gradient(150deg,rgba(255,45,85,.12),var(--surface));border-color:var(--line2)}
.label{display:block;color:var(--muted);font-size:10px;font-weight:800;letter-spacing:.12em}
.value{display:block;font-size:clamp(30px,10vw,44px);font-weight:850;line-height:1.02;letter-spacing:-.05em;margin-top:8px}
.unit{font-size:12px;color:var(--muted);font-weight:600;letter-spacing:0}
.metricSub{font-size:11px;color:var(--muted);margin-top:7px;min-height:15px}
.metricSub.strong{color:var(--orange);font-weight:800}
.wide{grid-column:1/-1}
.split{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:8px}
.miniMetric{background:rgba(0,0,0,.28);border:1px solid var(--line);border-radius:11px;padding:10px}
.miniMetric b{display:block;font-size:18px;margin-top:3px;font-weight:800}
.miniMetric span{font-size:10px;color:var(--muted);letter-spacing:.06em}
.statusGrid{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:7px}
.statusGrid div{padding:8px;background:rgba(0,0,0,.28);border:1px solid var(--line);border-radius:10px}
.statusGrid strong{display:block;font-size:16px}
.statusGrid small{font-size:9px;color:var(--muted);letter-spacing:.08em}
.zones{display:grid;grid-template-columns:repeat(5,1fr);gap:5px;margin-top:10px}
.zone{height:34px;display:grid;place-items:center;border-radius:8px;font-size:10px;font-weight:800;color:#0a0a0a}
.z0{background:var(--z0)}.z1{background:var(--z1)}.z2{background:var(--z2)}.z3{background:var(--z3)}.z4{background:var(--z4);color:#fff}
.zoneCard{padding:11px;background:rgba(0,0,0,.28);border:1px solid var(--line);border-radius:13px;margin-top:6px}
.zoneBar{position:relative;height:28px;display:flex;border-radius:9px;overflow:hidden;border:1px solid var(--line2)}
.zoneSeg{height:100%;display:flex;align-items:center;justify-content:center;font-size:9px;font-weight:800;color:#0a0a0a;min-width:0;overflow:hidden}
.zoneSeg.z0{flex:15}.zoneSeg.z1{flex:15}.zoneSeg.z2{flex:20}.zoneSeg.z3{flex:20}.zoneSeg.z4{flex:30;color:#fff}
.zoneCursor{position:absolute;top:-5px;bottom:-5px;width:3px;background:#fff;border-radius:2px;box-shadow:0 0 12px rgba(255,255,255,.98);transition:left .25s}
.zoneScale{display:flex;margin-top:5px;font-size:9px;color:var(--muted2)}
.zoneScale span{flex:1 1 0;width:0;text-align:center;overflow:hidden}
.zoneNow{margin-top:10px;display:flex;align-items:center;gap:6px;font-size:12px;color:var(--muted)}
.zoneNow .dot{width:11px;height:11px;border-radius:50%;flex-shrink:0}
.zoneNow strong{color:var(--text);font-size:15px}
.logCard{display:flex;gap:10px;align-items:stretch;background:rgba(0,0,0,.28);border:1px solid var(--line);border-radius:13px;padding:11px}
.logBadge{min-width:56px;display:grid;place-items:center;border-radius:10px;font-size:11px;font-weight:800;text-align:center;padding:4px 6px}
.logBadge.ok{background:rgba(74,222,155,.12);color:var(--green);border:1px solid rgba(74,222,155,.32)}
.logBadge.warn{background:rgba(255,209,102,.1);color:var(--yellow);border:1px solid rgba(255,209,102,.3)}
.logBadge.bad{background:rgba(255,45,85,.12);color:var(--red);border:1px solid rgba(255,45,85,.34)}
.logChips{display:flex;flex-wrap:wrap;gap:7px}
.logChips span{background:rgba(255,255,255,.03);border:1px solid var(--line);border-radius:9px;padding:5px 10px;font-size:10px;color:var(--muted)}
.logChips b{color:var(--text);font-size:14px}
.logWarns{display:flex;flex-wrap:wrap;gap:6px;margin-top:7px}
.logWarns span{padding:3px 8px;border-radius:7px;font-size:9px;font-weight:800}
.logWarns .warn{background:rgba(255,209,102,.12);color:var(--yellow)}
.logWarns .bad{background:rgba(255,45,85,.14);color:var(--red)}
.fileToolbar{display:flex;gap:6px;flex-wrap:wrap;margin-bottom:10px}
.hint.small{margin-top:8px}
.packedTag{display:inline-block;background:rgba(74,222,155,.12);color:var(--green);border:1px solid rgba(74,222,155,.32);border-radius:6px;padding:0 5px;font-size:9px;font-weight:800;margin-left:6px;vertical-align:1.5px}
.tabs{display:grid;grid-template-columns:repeat(3,1fr);gap:6px;margin:8px 0}
.tab{min-height:40px;background:rgba(18,23,34,.9);border:1px solid var(--line);border-radius:11px;color:var(--muted);font-size:10px;font-weight:800;letter-spacing:.04em;transition:background .15s,color .15s,border-color .15s}
.tab.active{background:linear-gradient(160deg,rgba(76,214,234,.2),rgba(76,214,234,.06));border-color:var(--cyan);color:#fff}
.chartPanel{display:none}.chartPanel.active{display:block}
.chart{border:1px solid var(--line);border-radius:15px;background:var(--surface);padding:12px;margin-bottom:10px;box-shadow:0 10px 22px rgba(0,0,0,.14)}
.chartHeader{display:flex;justify-content:space-between;align-items:center;gap:10px;margin-bottom:9px}
.chartHeader h3{font-size:11px;margin:0;letter-spacing:.1em;font-weight:800}
.chartHeader span{font-size:10px;color:var(--muted2)}
canvas{display:block;width:100%;height:180px;border-radius:11px;background:#0b0e14}
.legend{display:flex;flex-wrap:wrap;gap:10px;margin-top:9px;color:var(--muted);font-size:10px}
.legend i{display:inline-block;width:9px;height:9px;border-radius:9px;margin-right:4px;vertical-align:0}
.phaseNotice{border:1px solid #765424;background:#241d12;color:#ffe4a3;border-radius:10px;padding:9px;font-size:10px;margin-top:8px}
.panel{background:var(--surface);border:1px solid var(--line);border-radius:15px;padding:14px;box-shadow:0 10px 22px rgba(0,0,0,.14)}
.panelTitle{font-size:11px;font-weight:850;letter-spacing:.1em;color:var(--muted);margin-bottom:10px}
.fileList{display:grid;gap:7px}
.fileRow{display:flex;align-items:center;justify-content:space-between;gap:8px;padding:11px;background:rgba(0,0,0,.28);border:1px solid var(--line);border-radius:12px}
.fileName{font-size:12px;font-weight:800;overflow-wrap:anywhere}
.fileMeta,.hint{color:var(--muted);font-size:11px;margin-top:3px;line-height:1.45}
.fileActions{display:flex;gap:5px;flex-shrink:0}
.mini{min-height:34px;padding:0 10px;background:var(--line2);border:0;border-radius:9px;color:#fff;font-size:10px;font-weight:800}
.mini.danger,.danger{background:var(--red2)}
.warning{margin-top:11px;padding:11px;background:rgba(255,45,85,.08);border:1px solid rgba(255,45,85,.28);border-radius:11px;color:#ffc0c8;font-size:11px}
.diagGrid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:7px}
.diagGrid div{padding:10px;background:rgba(0,0,0,.28);border:1px solid var(--line);border-radius:11px;color:var(--muted);font-size:10px;letter-spacing:.04em}
.diagGrid b{display:block;color:var(--text);font-size:16px;margin-top:3px;letter-spacing:-.01em}
.mountValue{font-size:clamp(24px,8vw,34px);font-weight:850;letter-spacing:-.04em}
.actionDock{position:fixed;z-index:5;bottom:0;left:0;right:0;padding:10px max(14px,env(safe-area-inset-right)) calc(10px + env(safe-area-inset-bottom)) max(14px,env(safe-area-inset-left));background:linear-gradient(transparent,var(--bg) 34%)}
.actions{max-width:820px;margin:auto;display:grid;grid-template-columns:repeat(3,1fr);gap:8px}
.actions button{min-height:50px;border:1px solid var(--line2);border-radius:13px;background:rgba(26,33,48,.94);color:#fff;font-size:11px;font-weight:850;letter-spacing:.02em;line-height:1.15}
.actions .primary{background:linear-gradient(160deg,var(--red),var(--red2));border-color:transparent}
.actions .stop{background:linear-gradient(160deg,#8c1a2b,#5c0f1c);border-color:transparent}
.note{min-height:18px;margin:12px 2px 0;text-align:center;font-size:11px;color:var(--muted)}
.livestrip{display:flex;gap:2px;margin:4px 0 10px;padding:4px;background:rgba(0,0,0,.28);border:1px solid var(--line);border-radius:11px;overflow:hidden}
.err{color:var(--danger)}.good{color:var(--green)}
@media(min-width:600px){.metrics{grid-template-columns:repeat(4,minmax(0,1fr))}.metric.wide{grid-column:span 4}.bigValue #strokeStrength{font-size:clamp(72px,14vw,116px)}}
@media(prefers-reduced-motion:reduce){*{animation:none!important;transition:none!important;scroll-behavior:auto!important}}
</style>
</head>
<body>
<main class="app">
<header class="hero">
  <div class="eyebrow">ROLKI SILESIA · LOCAL TELEMETRY</div>
  <div class="heroRow"><h1>IMU LIVE</h1><div class="network">AP Wi‑Fi<br><span id="net">192.168.4.1</span></div></div>
  <div id="state" class="status" aria-live="polite"><i class="pulse"></i><span>LACZENIE Z URZADZENIEM...</span></div>
</header>
<nav class="nav" aria-label="Sekcje dashboardu">
  <button id="liveTab" class="active" type="button" onclick="showView('live')">LIVE</button>
  <button id="rhythmTab" type="button" onclick="showView('rhythm')">RYTM</button>
  <button id="zonesTab" type="button" onclick="showView('zones')">STREFY</button>
  <button id="filesTab" type="button" onclick="showView('files')">PLIKI</button>
  <button id="mountTab" type="button" onclick="showView('mount')">MONTAZ</button>
  <button id="diagTab" type="button" onclick="showView('diag')">DIAG</button>
</nav>

<section id="livePanel" class="view active">
  <div class="sectionTitle"><h2>Sila odepchniecia teraz</h2><span id="time">00:00</span></div>
  <div id="bigCard" class="bigCard">
    <div class="bigTop"><span class="bigLabel">SILA ODEPCHNIECIA</span><span id="strokeZoneCard" class="bigZone">--</span></div>
    <div class="bigValue"><span id="strokeStrength">--</span><span class="unit">/ 100</span></div>
    <div class="gauge"><div id="strengthGauge" class="gaugeFill"></div></div>
  </div>

  <div class="duo">
    <article class="metric primary"><span class="label">KADENCJA KROK</span><span class="value"><span id="cadence">--</span><span class="unit">/min</span></span><div class="metricSub">noga <span id="legCadence">--</span>/min · <span id="cadenceMs">--</span> ms</div></article>
    <article class="metric primary"><span class="label">MOC (proxy)</span><span class="value"><span id="power">--</span><span class="unit">pkt</span></span><div class="metricSub">sila x kadencja</div></article>
  </div>

  <div class="sectionTitle"><h2>Strefy sily</h2><span id="zonesText">rozklad sesji</span></div>
  <div id="zoneMap" class="zoneCard">
    <div class="zoneBar">
      <div class="zoneSeg z0"><b>--</b></div>
      <div class="zoneSeg z1"><b>--</b></div>
      <div class="zoneSeg z2"><b>--</b></div>
      <div class="zoneSeg z3"><b>--</b></div>
      <div class="zoneSeg z4"><b>--</b></div>
      <div id="zoneCursor" class="zoneCursor"></div>
    </div>
    <div class="zoneScale"><span>0</span><span>15</span><span>30</span><span>50</span><span>70</span><span>100</span></div>
    <div class="zoneNow"><span id="zoneDotNow" class="dot" style="background:#c3d0de"></span>TERAZ: <strong id="zoneNow">--</strong><span id="strokeZone" style="margin-left:auto;font-weight:800">--</span></div>
  </div>

  <div class="sectionTitle"><h2>Ostatnie 120 s</h2><span>1 pole = 1 s · kolor = strefa</span></div>
  <div id="liveStrip" class="livestrip"></div>

  <div class="sectionTitle"><h2>Rytm i faza</h2><span>metryki wzgledne</span></div>
  <div class="metrics">
    <article class="metric"><span class="label">ODEPCHNIECIA</span><span class="value" id="strokesPerMin">--</span><div class="metricSub">/ min · wszystkich <span id="strokes">0</span></div></article>
    <article class="metric"><span class="label">SILNE ODB.</span><span class="value" id="strokesStrongPct">--</span><div class="metricSub">% z wszystkich</div></article>
    <article class="metric"><span class="label">FAZA ODEPCHNIECIA</span><span class="value"><span id="phase">--</span><span class="unit">ms</span></span><div class="metricSub strong">czas trwania impulsu</div></article>
    <article class="metric"><span class="label">UDERZENIOWOSC</span><span class="value"><span id="surge">--</span><span class="unit">g/s</span></span><div class="metricSub">gwaltownosc ruchu</div></article>
  </div>

  <div class="sectionTitle"><h2>Aktywnosc sesji</h2><span>strona <span id="side">R</span></span></div>
  <div class="metrics">
    <article class="metric wide"><span class="label">SESJA</span><div class="split"><div class="miniMetric"><span>AKTYWNE</span><b id="active">--%</b></div><div class="miniMetric"><span>BEZRUCH</span><b id="static">--%</b></div><div class="miniMetric"><span>SILA P90/P95</span><b id="strPct">--</b></div><div class="miniMetric"><span>INT P90/P95</span><b id="intPct">--</b></div></div></article>
  </div>

  <div class="sectionTitle"><h2>Intensywnosc</h2><span id="intensityPct">chwilowa</span></div>
  <div class="metrics">
    <article class="metric wide"><span class="label">INTENSYWNOSC CHWILOWA</span><span class="value"><span id="intensity">--</span><span class="unit">/100</span></span></article>
  </div>

  <div class="sectionTitle"><h2>Jakosc logu</h2><span id="file">-</span></div>
  <div class="logCard">
    <div id="logBadge" class="logBadge ok">LOG OK</div>
    <div style="flex:1">
      <div class="logChips"><span>RATE <b id="rate">--</b></span><span>KOLEJKA <b id="queue">--</b></span><span>UTRACONE <b id="qdrop">0</b></span></div>
      <div id="logWarns" class="logWarns"></div>
    </div>
  </div>
  <div class="hint small"><span id="sessionMode">NORMAL · RIMU04</span> · <span id="files">0</span> plikow · <span id="free">--</span> KB wolne · telefon: <span id="clients">0</span> · LEAN: <span id="lean">--</span>°</div>
</section>

<section id="rhythmPanel" class="view">
  <div class="sectionTitle"><h2>WYKRESY NA ZYWO</h2><span id="historyInfo">BRAK DANYCH</span></div>
  <div class="tabs" role="tablist"><button id="dynTab" class="tab active" type="button" onclick="showCharts('intensity')">SILA / INT</button><button id="phaseTab" class="tab" type="button" onclick="showCharts('phase')">FAZA / KADENCJA</button><button id="paramsTab" class="tab" type="button" onclick="showCharts('motion')">MOTION / LEAN</button></div>
  <div id="intensityPanel" class="chartPanel active"><article class="chart"><div class="chartHeader"><h3>SILA ODPCHNIECIA · OSTATNIE 120 s</h3><span>0–100</span></div><canvas id="dynChart"></canvas><div class="legend"><span><i style="background:#ef233c"></i>SILA</span><span>1 punkt / s</span></div></article><article class="chart"><div class="chartHeader"><h3>INTENSYWNOSC</h3><span>0–100</span></div><canvas id="pushChart"></canvas><div class="legend"><span><i style="background:#ff8c42"></i>intensywnosc</span><span><i style="background:#fff"></i>odepchniecie</span></div></article></div>
  <div id="phasePanel" class="chartPanel"><article class="chart"><div class="chartHeader"><h3>CZAS IMPULSU ODPCHNIECIA</h3><span>ms</span></div><canvas id="phaseChart"></canvas><div class="legend"><span><i style="background:#ffd166"></i>czas impulsu / odepchniecie</span></div></article><article class="chart"><div class="chartHeader"><h3>KADENCJA</h3><span>odstep / rytm</span></div><canvas id="cadChart"></canvas><div class="legend"><span><i style="background:#56cfe1"></i>kadencja (ms) / kad</span></div></article></div>
  <div id="motionPanel" class="chartPanel"><article class="chart"><div class="chartHeader"><h3>RUCH (spin)</h3><span>dps</span></div><canvas id="motionChart"></canvas></article><article class="chart"><div class="chartHeader"><h3>PRZECHYL WZGLEDNY</h3><span>stopnie</span></div><canvas id="leanChart"></canvas><div class="hint">LEAN jest wskaźnikiem diagnostycznym; nie interpretuj go jako bezwzględnego kąta techniki.</div></article></div>
</section>

<section id="zonesPanel" class="view">
  <div class="sectionTitle"><h2>ANALIZA STREF SILY</h2><span id="zonesInfo">CALOSC SESJI + 120 s</span></div>
  <article class="chart"><div class="chartHeader"><h3>ROZKLAD CZASU W STREFACH</h3><span>CALOSC SESJI</span></div><canvas id="zoneShareChart"></canvas><div class="legend"><span><i style="background:#d9e2ec"></i>bardzo lekkie</span><span><i style="background:#7bdff2"></i>lekkie</span><span><i style="background:#ffd166"></i>srednie</span><span><i style="background:#ff9f1c"></i>mocne</span><span><i style="background:#ef476f"></i>bardzo mocne</span></div></article>
  <article class="chart"><div class="chartHeader"><h3>MAPA STREF SILY</h3><span>OSTATNIE 120 s</span></div><canvas id="zoneTimelineChart"></canvas><div class="legend"><span><i style="background:#ef476f"></i>gora = bardzo mocne</span><span>1 blok = 1 s</span></div></article>
  <article class="chart"><div class="chartHeader"><h3>UDZIAL MOCNE + BARDZO MOCNE</h3><span>OKNA CZASOWE</span></div><canvas id="zoneStrongChart"></canvas><div class="hint">Udzial probek o sile odepchniecia w strefach 3+4. To porownanie intensywnosci, nie moc fizyczna.</div></article>
</section>

<section id="filesPanel" class="view"><div class="sectionTitle"><h2>PLIKI SESJI</h2><span>POZA NAGRYWANIEM</span></div><div class="panel"><div class="fileToolbar"><button id="zipToggle" class="mini" type="button" onclick="toggleZip()">PAKOWANIE W LOCIE: --</button><button id="packAll" class="mini" type="button" onclick="packAll()">PAKUJ</button><button id="deleteAll" class="mini danger" type="button" onclick="deleteAllFiles()">USUN WSZYSTKIE</button></div><div id="fileList" class="fileList">Wczytywanie plikow...</div><div class="hint small"><strong>PAKOWANIE W LOCIE</strong> zapisuje od razu maly plik (.pzs/.rzs) i wydluza sesje; wlacz przed startem. PAKUJ kompresuje istniejace pliki po fakcie (.pz/.rz), a oryginal zostaje usuniety. Pobrany plik rozpakujesz na PC: <strong>rozpakuj.ps1</strong>.</div><div class="warning">Kasowanie jest nieodwracalne. Lista, pobieranie i pakowanie są blokowane podczas nagrywania, aby nie kolidować z zapisem do LittleFS.</div></div></section>

<section id="mountPanel" class="view"><div class="sectionTitle"><h2>MONTAZ CZUJNIKA</h2><span>TYLKO POZA SESJA</span></div><div class="panel"><div class="hint">X+ → PRZOD / JAZDA · Y+ → NIEBO · Z+ → LEWA STRONA</div><div id="mountValues" class="mountValue">F--.- L--.- U--.-</div><div id="mountHint" class="hint">Czekam na odczyt IMU...</div></div></section>

<section id="diagPanel" class="view"><div class="sectionTitle"><h2>DIAGNOSTYKA</h2><span>JAKOSC I SYSTEM</span></div><div class="panel"><div id="diagGrid" class="diagGrid">Wczytywanie diagnostyki...</div></div></section>
<div id="note" class="note" aria-live="polite">Lokalne polaczenie · brak dostepu do Internetu.</div>
</main>
<footer class="actionDock"><div class="actions"><button id="startDefault" class="primary" type="button" onclick="act('start_default')">NORMAL<br>DEFAULT</button><button id="startMax" type="button" onclick="act('start_max')">NORMAL<br>MAX</button><button id="startRawDefault" class="primary" type="button" onclick="act('start_raw_default')">RAW LAB<br>DEFAULT</button><button id="startRawMax" type="button" onclick="act('start_raw_max')">RAW LAB<br>MAX</button><button id="stop" class="stop" type="button" onclick="act('stop')">STOP<br>SESJI</button><button id="cancel" type="button" onclick="act('cancel')">ANULUJ<br>ODLICZANIE</button></div></footer>
<script>
const by=id=>document.getElementById(id);const set=(id,v)=>by(id).textContent=v;const two=v=>String(v).padStart(2,'0');const fmt=t=>{t=Math.max(0,Math.floor(t||0));return two(t/60|0)+':'+two(t%60)};const num=(v,d=0)=>Number(v||0).toFixed(d);let phoneStatus={},historyPoints=[],activeView='live',activeChart='intensity',historyBusy=false,filesBusy=false;
function calibrationStatusText(s){if(s.calibration_active)return ' · KALIBRACJA '+(s.calibration_stable||0)+' OK / '+(s.calibration_rejected||0)+' RUCH';if(s.countdown&&s.calibration_complete)return ' · KALIBRACJA OK';return ''}function mark(s){let e=by('state');e.className='status'+(s.recording?' recording':'');e.innerHTML='<i class="pulse"></i><span>'+s.state+(s.recording?' · REJESTRACJA AKTYWNA':calibrationStatusText(s))+'</span>'}
const zoneSoft=['rgba(195,208,222,.16)','rgba(94,199,232,.16)','rgba(255,209,102,.16)','rgba(255,159,28,.16)','rgba(255,61,113,.18)'];
function updateZones(z,str){let values=z||[];by('zoneMap').querySelectorAll('.zoneSeg b').forEach((el,i)=>el.textContent=num(values[i],0)+'%');let v=Math.min(100,Math.max(0,Number(str||0)));let cur=by('zoneCursor');cur.style.left=v+'%';let c=zoneOf(str);by('zoneDotNow').style.background=zoneColors[c];by('zoneNow').textContent=num(str||0,0)+' / 100';let g=by('strengthGauge');if(g)g.style.width=v+'%';let root=document.documentElement;root.style.setProperty('--accent',zoneColors[c]);root.style.setProperty('--accentSoft',zoneSoft[c])}function updateQuality(s){let grade='ok',label='LOG OK',warns=[];if((s.qdrop||0)>0){grade='bad';label='UTRATA PROBEK';warns.push('qdrop '+s.qdrop)}if((s.gap8||0)>0){if(grade!=='bad'){grade='warn';label='UWAGI'}warns.push('gap8 '+s.gap8)}if((s.bad||0)>0){grade='bad';label='UTRATA PROBEK';warns.push('bad '+s.bad)}if((s.sat||0)>0){if(grade!=='bad'){grade='warn';label='UWAGI'}warns.push('sat '+s.sat)}if(!warns.length&&(s.queue||0)>448){grade='warn';label='KOLJKA PELNA';warns.push('kolejka '+s.queue+'/512')}let badge=by('logBadge');badge.className='logBadge '+grade;badge.textContent=label;by('logWarns').innerHTML=warns.map(x=>{let c=(x.indexOf('qdrop')===0||x.indexOf('bad')===0)?'bad':'warn';return '<span class="'+c+'">'+x+'</span>'}).join('')}
function zoneFromStrength(v){let n=Number(v||0);return n<15?0:n<30?1:n<50?2:n<70?3:4}
function drawLiveStrip(){let el=by('liveStrip');if(!el){return}let html='';let pts=historyPoints.slice(-120);pts.forEach(p=>{let z=zoneFromStrength(p[2]);let c=zoneColors[z];let mark=p[6]&&p[6]>0?';outline:1px solid #fff':'';html+='<span style="background:'+c+';flex:1;min-width:3px;height:26px;border-radius:2px;'+mark+';display:inline-block" title="'+p[0]+'s · sily '+p[2]+'"></span>'});el.innerHTML=html||'<div class="hint">Brak danych 1 s...</div>'}
async function getStatus(){try{let r=await fetch('/api/status',{cache:'no-store'});if(!r.ok)throw new Error();let s=await r.json();phoneStatus=s;mark(s);set('time',fmt(s.time_s));set('strokeStrength',num(s.stroke_strength,0));set('strokeZone',s.stroke_zone_name||'--');set('cadence',num(s.stride_cadence!=null?s.stride_cadence:s.cadence_per_min,0));set('legCadence',num(s.leg_cadence,0));set('cadenceMs',num(s.cadence_ms,0));set('power',num(s.power_proxy,1));set('intensity',num(s.intensity,0));set('intensityPct','P90 '+num(s.intensity_p90,0));set('strokesPerMin',num(s.strokes_per_min,1));set('strokes',s.strokes||0);set('strokesStrongPct',num(s.strokes_strong_pct,0)+'%');set('phase',num(s.strength_phase_ms,0));set('surge',num(s.surge_gps,1));set('active',num(s.active_pct,0)+'%');set('static',num(s.idle_pct,0)+'%');set('side',s.side||'R');set('strPct',num(s.stroke_p90,0)+' / '+num(s.stroke_p95,0));set('intPct',num(s.intensity_p90,0)+' / '+num(s.intensity_p95,0));updateZones(s.zones,s.stroke_strength);set('rate',num(s.rate_hz,0)+' Hz');set('queue',s.queue+' / 512');set('qdrop',s.qdrop);set('strokeZoneCard',s.stroke_zone_name||'--');set('lean',num(s.lean_deg,1));updateQuality(s);set('file',s.file||'-');set('sessionMode',(s.session_mode||'NORMAL')+' · '+(s.session_format||'RIMU04'));set('files',s.files);set('free',s.free_kb);set('clients',s.clients);by('startDefault').disabled=!s.can_start;by('startMax').disabled=!s.can_start;by('startRawDefault').disabled=!s.can_start;by('startRawMax').disabled=!s.can_start;by('stop').disabled=!s.recording;by('cancel').disabled=!s.countdown;updateWebAvailability(s);set('note','Odświeżono · '+new Date().toLocaleTimeString())}catch(e){let n=by('note');n.textContent='Brak polaczenia z urzadzeniem';n.className='note err'}}
async function act(cmd){if(cmd==='stop'&&!confirm('Zatrzymac bieżąca sesje?'))return;try{let r=await fetch('/api/action?cmd='+cmd,{method:'POST',cache:'no-store'}),j=await r.json();set('note',j.message||'Polecenie wyslane');by('note').className=r.ok&&j.ok?'note good':'note err';setTimeout(getStatus,250)}catch(e){set('note','Nie mozna wyslac polecenia');by('note').className='note err'}}
function showView(view){activeView=view;['live','rhythm','zones','files','mount','diag'].forEach(name=>{by(name+'Panel').classList.toggle('active',name===view);by(name+'Tab').classList.toggle('active',name===view)});if(view==='files')loadFiles();if(view==='mount')loadMount();if(view==='diag')loadDiagnostics();if(view==='rhythm')setTimeout(drawActiveCharts,25);if(view==='zones')setTimeout(drawZoneCharts,25)}
function showCharts(which){activeChart=which;['intensity','phase','motion'].forEach(name=>{by(name+'Panel').classList.toggle('active',name===which);by(name==='intensity'?'dynTab':name==='phase'?'phaseTab':'paramsTab').classList.toggle('active',name===which)});setTimeout(drawActiveCharts,25)}
function chartContext(id){let c=by(id),d=Math.max(1,window.devicePixelRatio||1),w=c.clientWidth||300,h=c.clientHeight||178;if(c.width!==Math.round(w*d)||c.height!==Math.round(h*d)){c.width=Math.round(w*d);c.height=Math.round(h*d)}let x=c.getContext('2d');x.setTransform(d,0,0,d,0,0);return{x,w,h}}
function drawLine(id,index,low,high,color,markers=false){let box=chartContext(id),x=box.x,w=box.w,h=box.h,p={l:31,r:8,t:15,b:20};x.clearRect(0,0,w,h);x.fillStyle='#0c1016';x.fillRect(0,0,w,h);x.strokeStyle='#27303d';x.lineWidth=1;for(let i=0;i<4;i++){let y=p.t+(h-p.t-p.b)*i/3;x.beginPath();x.moveTo(p.l,y);x.lineTo(w-p.r,y);x.stroke()}x.fillStyle='#9ba7b7';x.font='10px system-ui';x.fillText(String(Math.round(high)),2,p.t+3);x.fillText(String(Math.round(low)),2,h-p.b+3);if(historyPoints.length<2){x.fillText('Czekam na dane aktywnej sesji...',p.l+10,h/2);return}let first=historyPoints[0][0],last=historyPoints[historyPoints.length-1][0],span=Math.max(1,last-first),val=v=>index===5?v[5]/100:v[index],py=v=>p.t+(h-p.t-p.b)*(1-(val(v)-low)/(high-low));x.strokeStyle=color;x.lineWidth=2;x.lineJoin='round';x.beginPath();historyPoints.forEach((v,i)=>{let px=p.l+(w-p.l-p.r)*(v[0]-first)/span,y=Math.max(p.t,Math.min(h-p.b,py(v)));i?x.lineTo(px,y):x.moveTo(px,y)});x.stroke();if(markers){historyPoints.forEach(v=>{if(v[6]){let px=p.l+(w-p.l-p.r)*(v[0]-first)/span,y=Math.max(p.t,Math.min(h-p.b,py(v)));x.fillStyle='#fff';x.beginPath();x.arc(px,y,3,0,Math.PI*2);x.fill()}})}x.fillStyle='#9ba7b7';x.fillText(first+'s',p.l,h-4);x.fillText(last+'s',w-p.r-18,h-4)}
function drawPhase(){let box=chartContext('phaseChart'),x=box.x,w=box.w,h=box.h,p={l:31,r:8,t:15,b:20};x.clearRect(0,0,w,h);x.fillStyle='#0c1016';x.fillRect(0,0,w,h);x.strokeStyle='#27303d';for(let i=0;i<4;i++){let y=p.t+(h-p.t-p.b)*i/3;x.beginPath();x.moveTo(p.l,y);x.lineTo(w-p.r,y);x.stroke()}x.fillStyle='#9ba7b7';x.font='10px system-ui';x.fillText('150',2,p.t+3);x.fillText('0',12,h-p.b+3);if(historyPoints.length<2){x.fillText('Czekam na zaakceptowane odepchniecie...',p.l+8,h/2);return;}let first=historyPoints[0][0],last=historyPoints[historyPoints.length-1][0],span=Math.max(1,last-first);historyPoints.forEach(v=>{if(v[6]&&v[7]>0){let px=p.l+(w-p.l-p.r)*(v[0]-first)/span,bar=(h-p.t-p.b)*Math.min(150,v[7])/150;x.fillStyle='#ffd166';x.fillRect(px-2,h-p.b-bar,4,bar)}});x.fillStyle='#9ba7b7';x.fillText(first+'s',p.l,h-4);x.fillText(last+'s',w-p.r-18,h-4)}
function drawCad(){let box=chartContext('cadChart'),x=box.x,w=box.w,h=box.h,p={l:31,r:8,t:15,b:20};x.clearRect(0,0,w,h);x.fillStyle='#0c1016';x.fillRect(0,0,w,h);x.strokeStyle='#27303d';for(let i=0;i<4;i++){let y=p.t+(h-p.t-p.b)*i/3;x.beginPath();x.moveTo(p.l,y);x.lineTo(w-p.r,y);x.stroke()}x.fillStyle='#9ba7b7';x.font='10px system-ui';x.fillText('1000',2,p.t+3);x.fillText('0',12,h-p.b+3);if(historyPoints.length<2){x.fillText('Czekam na kadencje...',p.l+8,h/2);return;}let first=historyPoints[0][0],last=historyPoints[historyPoints.length-1][0],span=Math.max(1,last-first);x.strokeStyle='#56cfe1';x.lineWidth=2;x.lineJoin='round';x.beginPath();historyPoints.forEach((v,i)=>{let px=p.l+(w-p.l-p.r)*(v[0]-first)/span,y=p.t+(h-p.t-p.b)*(1-Math.min(1000,v[3])/1000);i?x.lineTo(px,y):x.moveTo(px,y)});x.stroke();x.fillStyle='#9ba7b7';x.fillText(first+'s',p.l,h-4);x.fillText(last+'s',w-p.r-18,h-4)}
function drawActiveCharts(){if(activeChart==='intensity'){drawLine('dynChart',2,0,100,'#ef233c',true);drawLine('pushChart',1,0,100,'#ff8c42')}else if(activeChart==='phase'){drawPhase();drawCad()}else{let max=100;historyPoints.forEach(v=>max=Math.max(max,v[4]));max=Math.ceil(max/100)*100;drawLine('motionChart',4,0,max,'#56cfe1');drawLine('leanChart',5,-75,75,'#ffd166')}}
const zoneColors=['#d9e2ec','#7bdff2','#ffd166','#ff9f1c','#ef476f'],zoneNames=['BARDZO LEKKIE','LEKKIE','SREDNIE','MOCNE','BARDZO MOCNE'];
function zoneOf(v){let n=Number(v||0);return n<15?0:n<30?1:n<50?2:n<70?3:4}
function setupChart(id){let box=chartContext(id),x=box.x,w=box.w,h=box.h;x.clearRect(0,0,w,h);x.fillStyle='#0c1016';x.fillRect(0,0,w,h);return{box,x,w,h}}
function drawZoneShare(){let a=setupChart('zoneShareChart'),x=a.x,w=a.w,h=a.h,p={l:94,r:35,t:14,b:14},z=phoneStatus.zones||[];if(!z.length){x.fillStyle='#9ba7b7';x.font='11px system-ui';x.fillText('Czekam na dane sesji...',p.l,h/2);return}z.forEach((raw,i)=>{let value=Math.max(0,Number(raw||0)),y=p.t+i*(h-p.t-p.b)/5+4,bw=(w-p.l-p.r)*Math.min(100,value)/100;x.fillStyle='#9ba7b7';x.font='10px system-ui';x.fillText(zoneNames[i],4,y+10);x.fillStyle='#202938';x.fillRect(p.l,y,w-p.l-p.r,14);x.fillStyle=zoneColors[i];x.fillRect(p.l,y,bw,14);x.fillStyle=i===0?'#11151d':'#f6f8fb';x.fillText(num(value,0)+'%',p.l+bw+5,y+11)})}
function drawZoneTimeline(){let a=setupChart('zoneTimelineChart'),x=a.x,w=a.w,h=a.h,p={l:31,r:8,t:14,b:22};for(let i=0;i<5;i++){let y=p.t+(h-p.t-p.b)*(4-i)/4;x.strokeStyle='#27303d';x.beginPath();x.moveTo(p.l,y);x.lineTo(w-p.r,y);x.stroke();x.fillStyle='#9ba7b7';x.font='9px system-ui';x.fillText(String(i),13,y+3)}if(!historyPoints.length){x.fillStyle='#9ba7b7';x.font='11px system-ui';x.fillText('Czekam na dane aktywnej sesji...',p.l+8,h/2);return}let first=historyPoints[0][0],last=historyPoints[historyPoints.length-1][0],span=Math.max(1,last-first);historyPoints.forEach(v=>{let zone=zoneOf(v[2]),px=p.l+(w-p.l-p.r)*(v[0]-first)/span,next=Math.max(3,(w-p.l-p.r)/Math.max(1,historyPoints.length));let y=p.t+(h-p.t-p.b)*(4-zone)/4-(h-p.t-p.b)/8;x.fillStyle=zoneColors[zone];x.fillRect(px,y,next,(h-p.t-p.b)/4) });x.fillStyle='#9ba7b7';x.font='10px system-ui';x.fillText(first+'s',p.l,h-5);x.fillText(last+'s',w-p.r-18,h-5)}
function drawZoneStrong(){let a=setupChart('zoneStrongChart'),x=a.x,w=a.w,h=a.h,p={l:30,r:12,t:18,b:28},windows=[30,60,120];if(!historyPoints.length){x.fillStyle='#9ba7b7';x.font='11px system-ui';x.fillText('Czekam na dane aktywnej sesji...',p.l+8,h/2);return}let last=historyPoints[historyPoints.length-1][0],bw=(w-p.l-p.r)/windows.length*.52;windows.forEach((seconds,i)=>{let data=historyPoints.filter(v=>v[0]>last-seconds),pct=data.length?100*data.filter(v=>zoneOf(v[2])>=3).length/data.length:0,cx=p.l+(i+.5)*(w-p.l-p.r)/windows.length,bar=(h-p.t-p.b)*pct/100;x.fillStyle='#ef476f';x.fillRect(cx-bw/2,h-p.b-bar,bw,bar);x.fillStyle='#f6f8fb';x.font='11px system-ui';x.fillText(num(pct,0)+'%',cx-bw/2,h-p.b-bar-5);x.fillStyle='#9ba7b7';x.fillText(seconds+' s',cx-bw/2,h-8)});x.strokeStyle='#27303d';x.beginPath();x.moveTo(p.l,h-p.b);x.lineTo(w-p.r,h-p.b);x.stroke();x.fillStyle='#9ba7b7';x.font='10px system-ui';x.fillText('100%',1,p.t+3);x.fillText('0%',8,h-p.b+3)}
function drawZoneCharts(){drawZoneShare();drawZoneTimeline();drawZoneStrong();set('zonesInfo',historyPoints.length?historyPoints.length+' / 120 s':'BRAK DANYCH')}
async function getHistory(){if(historyBusy)return;historyBusy=true;try{let r=await fetch('/api/history',{cache:'no-store'});if(!r.ok)throw new Error();let d=await r.json();historyPoints=d.points||[];set('historyInfo',historyPoints.length?historyPoints.length+' / 120 s':'BRAK DANYCH');if(activeView==='rhythm')drawActiveCharts();if(activeView==='zones')drawZoneCharts();if(activeView==='live')drawLiveStrip()}catch(e){}finally{historyBusy=false}}
function esc(v){return String(v).replace(/[&<>"]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]))}function sizeText(b){return b>=1048576?(b/1048576).toFixed(2)+' MB':(b/1024).toFixed(1)+' KB'}
function updateWebAvailability(s){by('deleteAll').disabled=!s.can_manage_files;if(by('packAll'))by('packAll').disabled=!s.can_manage_files;document.querySelectorAll('[data-pack]').forEach(b=>b.disabled=!s.can_manage_files);let zt=by('zipToggle');if(zt){zt.textContent='PAKOWANIE W LOCIE: '+(s.compression?'WL':'WYL');zt.style.background=s.compression?'#12301f':'';zt.disabled=!s.can_start}if(activeView==='files'&&s.can_files)loadFiles()}
async function toggleZip(){try{await postAction('toggle_zip',{});setTimeout(getStatus,150)}catch(e){}}
async function postAction(cmd,extra){let q=new URLSearchParams(Object.assign({cmd},extra||{})),r=await fetch('/api/action?'+q.toString(),{method:'POST',cache:'no-store'}),j=await r.json();set('note',j.message||'Brak odpowiedzi');by('note').className=r.ok&&j.ok?'note good':'note err';if(!r.ok||!j.ok)throw new Error(j.message||'Blad');return j}
async function loadFiles(){if(filesBusy)return;let list=by('fileList');if(!phoneStatus.can_files){list.textContent='Lista plikow jest zablokowana podczas nagrywania.';return}filesBusy=true;try{let r=await fetch('/api/files',{cache:'no-store'});if(!r.ok)throw new Error();let d=await r.json(),files=d.files||[];if(!files.length){list.textContent='Brak zapisanych sesji.';return}list.innerHTML=files.map(f=>{let n=esc(f.name);return '<div class="fileRow"><div><div class="fileName">'+n+(f.packed?' <i class="packedTag">PAK</i>':'')+'</div><div class="fileMeta">'+sizeText(f.bytes)+'</div></div><div class="fileActions">'+(f.packed?'':'<button class="mini" data-pack="'+n+'">PAKUJ</button>')+'<button class="mini" data-download="'+n+'">POBIERZ</button><button class="mini danger" data-delete="'+n+'">USUN</button></div></div>'}).join('')+(d.truncated?'<div class="hint">Pokazano pierwsze '+files.length+' z '+d.total+' plikow.</div>':'');list.querySelectorAll('[data-download]').forEach(b=>b.onclick=()=>downloadFile(b.dataset.download));list.querySelectorAll('[data-delete]').forEach(b=>b.onclick=()=>deleteFile(b.dataset.delete));list.querySelectorAll('[data-pack]').forEach(b=>b.onclick=()=>packFile(b.dataset.pack))}catch(e){list.textContent='Nie mozna pobrac listy plikow.'}finally{filesBusy=false}}
function downloadFile(name){if(phoneStatus.can_files)window.location='/api/download?file='+encodeURIComponent(name)}async function deleteFile(name){if(!phoneStatus.can_manage_files||!confirm('Usunac bezpowrotnie '+name+'?'))return;try{await postAction('delete_one',{file:name});setTimeout(()=>{getStatus();loadFiles()},350)}catch(e){}}async function deleteAllFiles(){if(!phoneStatus.can_manage_files||!confirm('USUNAC WSZYSTKIE sesje? Operacji nie mozna cofnac.'))return;let code=prompt('Wpisz ERASE aby potwierdzic:');if(code!=='ERASE')return;try{await postAction('delete_all',{confirm:'ERASE'});setTimeout(()=>{getStatus();loadFiles()},500)}catch(e){}}function packFile(name){if(!confirm('Spakowac (LZSS) plik '+name+'?'))return;postAction('pack_one',{file:name}).then(()=>{getStatus();loadFiles()}).catch(()=>{})}async function packAll(){if(!confirm('Spakowac wszystkie sesje (LZSS)? Oryginaly zostana usuniete po sukcesie.'))return;try{await postAction('pack_all',{});setTimeout(()=>{getStatus();loadFiles()},600)}catch(e){}}
async function loadMount(){try{let r=await fetch('/api/mount',{cache:'no-store'}),d=await r.json();if(!d.available){set('mountValues','ODCZYT NIEDOSTEPNY');set('mountHint','Montaz sprawdzaj poza nagrywaniem.');return}set('mountValues','F'+d.forward_g.toFixed(1)+' L'+d.lateral_g.toFixed(1)+' U'+d.vertical_g.toFixed(1));set('mountHint',d.vertical_g>0.70?'Prawidlowo: gora czujnika wskazuje niebo.':'Ustaw rolke nieruchomo: oczekiwane U blisko +1.0 g.')}catch(e){set('mountHint','Brak odczytu montazu.')}}
function diagCell(label,value){return '<div>'+label+'<b>'+value+'</b></div>'}function batteryLevelText(d){return d.battery_valid?d.battery_pct+'% / '+(d.battery_mv>0?(d.battery_mv/1000).toFixed(2)+' V':'--'):'BRAK ODCZYTU'}function batteryTimeText(d){if(!d.battery_valid)return 'BRAK DANYCH';if(d.battery_charging)return 'LADOWANIE';return d.battery_estimate_valid?fmt(d.battery_remaining_min*60):'UCZY SIE'}function stackText(v){let n=Number(v||0);return n>0?Math.round(n)+' B':'N/A'}function calibrationDiagText(d){return d.calibration_active?'TRWA':d.calibration_complete?'GOTOWA':d.calibration_last_moved?'RUCH':'OCZEKUJE'}async function loadDiagnostics(){try{let r=await fetch('/api/diagnostics',{cache:'no-store'}),d=await r.json(),lag=Math.max(0,(d.queued||0)-(d.written||0));by('diagGrid').innerHTML=diagCell('FLASH',d.storage_ready?'OK':'BRAK')+diagCell('WOLNE',d.free_kb+' KB')+diagCell('PLIKI',d.files)+diagCell('WIFI',d.ap_ip+' / C'+d.clients)+diagCell('AKUMULATOR',batteryLevelText(d))+diagCell('PRACA EST.',batteryTimeText(d))+diagCell('SPADEK BATERII',d.battery_estimate_valid?num(d.battery_drain_pct_h,1)+'% / h':'--')+diagCell('POMIAR BATERII',d.battery_age_s+' s temu')+diagCell('SAMPLES',d.samples)+diagCell('ZAPISANE',d.written)+diagCell('LAG WRITER',lag)+diagCell('Q MAX',d.queue_max)+'<div>QDROP<b>'+d.qdrop+'</b></div>'+diagCell('WRITE',d.writer_errors)+diagCell('GAP8 / BAD',d.gap8+' / '+d.bad)+diagCell('SAT',d.saturation)+diagCell('INT SRED / MAX',num(d.intensity_avg)+' / '+num(d.intensity_max))+diagCell('INT P90/P95',num(d.intensity_p90)+' / '+num(d.intensity_p95))+diagCell('SILA SR / MAX',num(d.stroke_avg)+' / '+num(d.stroke_max))+diagCell('SILA P90/P95',num(d.stroke_p90)+' / '+num(d.stroke_p95))+diagCell('ODPCHNIEC',d.stroke_count)+diagCell('Z TEGO SILNE %',num(d.stroke_strong_pct,0)+'%')+diagCell('FAZA SR. / MAX',num(d.stroke_phase_avg_ms,0)+' / '+num(d.stroke_phase_max_ms,0)+' ms')+diagCell('KADENCJA KROK',num(d.stride_cadence,0)+' /min (noga '+num(d.leg_cadence,0)+')')+diagCell('MOC PROXY',num(d.power_proxy,1)+' szcz. '+num(d.power_proxy_max,1))+diagCell('UDERZEN.',num(d.surge_gps,0)+' g/s')+diagCell('RUCH / GLOAD',num(d.spin_dps,0)+' dps / '+num(d.gload_g,2)+' g')+diagCell('AKTYWNE / BEZRUCH',num(d.active_pct,0)+' / '+num(d.idle_pct,0)+'%')+diagCell('STREFY SILY',(d.zones||[]).map(v=>num(v,0)+'%').join(' / '))+diagCell('TRYB / FORMAT',(d.session_mode||'NORMAL')+' / '+(d.session_format||'RIMU04'))+diagCell('KAL SESJA',calibrationDiagText(d))+diagCell('CAL OK / RUCH',(d.calibration_stable||0)+' / '+(d.calibration_rejected||0))+diagCell('CAL PONOWIENIA',d.calibration_retries||0)+diagCell('IMU STALL',d.imu_stalls||0)+diagCell('STOS IMU MIN',stackText(d.imu_stack_min_free))+diagCell('STOS WRITER MIN',stackText(d.writer_stack_min_free))+diagCell('ODBCIENIA',d.stroke_count)+diagCell('LOG',num(d.rate_hz,0)+' Hz')+diagCell('IMU',d.imu_done?'GOTOWE':'AKTYWNE')+diagCell('WRITER',d.writer_done?'GOTOWY':'AKTYWNY')}catch(e){by('diagGrid').textContent='Nie mozna pobrac diagnostyki.'}}
getStatus();getHistory();setInterval(getStatus,1000);setInterval(getHistory,1000);setInterval(()=>{if(activeView==='files')loadFiles();else if(activeView==='mount')loadMount();else if(activeView==='diag')loadDiagnostics()},3000);window.addEventListener('resize',()=>{if(activeView==='rhythm')drawActiveCharts();if(activeView==='zones')drawZoneCharts()});
</script>
</body>
</html>
)HTML";

const char* deviceStateName(DeviceState value) {
  switch (value) {
    case STATE_MAIN_MENU: return "GOTOWY";
    case STATE_VOLUME_SELECT: return "WYBOR GLOSNOSCI";
    case STATE_START_COUNTDOWN: return "ODLICZANIE";
    case STATE_RECORDING: return "NAGRYWANIE";
    case STATE_STOPPING: return "ZATRZYMYWANIE";
    case STATE_SAVED: return "ZAPISANO";
    case STATE_EXPORT_SELECT: return "WYBOR EKSPORTU";
    case STATE_EXPORTING: return "EKSPORT USB";
    case STATE_FILE_MANAGER: return "PLIKI";
    case STATE_DIAGNOSTICS: return "DIAGNOSTYKA";
    case STATE_MOUNT_GUIDE: return "MONTAZ";
    case STATE_STORAGE_INIT_REQUIRED: return "PAMIEC WYMAGA INICJALIZACJI";
    case STATE_ERROR: return "BLAD";
    default: return "URUCHAMIANIE";
  }
}

bool phoneCanStartSession() {
  return storageReady && (state == STATE_MAIN_MENU || state == STATE_SAVED ||
                          state == STATE_DIAGNOSTICS || state == STATE_MOUNT_GUIDE);
}

bool phoneCanListFiles() {
  return storageReady && state != STATE_RECORDING && state != STATE_STOPPING &&
         state != STATE_EXPORTING;
}

bool phoneCanManageFiles() {
  return storageReady && (state == STATE_MAIN_MENU || state == STATE_SAVED ||
                          state == STATE_DIAGNOSTICS || state == STATE_MOUNT_GUIDE ||
                          state == STATE_FILE_MANAGER || state == STATE_EXPORT_SELECT);
}

bool phoneSessionPathFromArg(const String& arg, char* destination, size_t size) {
  if (!arg.length() || arg.length() >= size) return false;
  copyLittleFsPath(destination, size, arg.c_str());
  // WWW moze operowac tylko na nazwach sesji bez dodatkowych katalogow.
  return strncmp(destination, "/ses", 4) == 0 &&
         strchr(destination + 1, '/') == nullptr &&
         hasSessionExtension(destination) && LittleFS.exists(destination);
}

bool phoneJsonFits(char* response, size_t size, int written) {
  if (written >= 0 && (size_t)written < size) return true;
  const char* fallback = "{\"ok\":false,\"message\":\"JSON response too long\"}";
  strncpy(response, fallback, size - 1);
  response[size - 1] = '\0';
  return false;
}

void sendPhoneJson(int code, bool ok, const char* message) {
  char response[176];
  const int written = snprintf(response, sizeof(response), "{\"ok\":%s,\"message\":\"%s\"}", ok ? "true" : "false", message);
  const bool fits = phoneJsonFits(response, sizeof(response), written);
  webServer.sendHeader("Cache-Control", "no-store");
  webServer.send(fits ? code : 500, "application/json", response);
}

void sendPhoneStatus() {
  const RuntimeStats s = snapshotStats();
  const uint32_t elapsed = sessionStartUs ? (micros() - sessionStartUs) / 1000000UL : 0;
  const float rate = (s.recordsQueued > 1 && s.lastSampleUs > s.firstSampleUs)
      ? (s.recordsQueued - 1) * 1000000.0f / (s.lastSampleUs - s.firstSampleUs) : 0.0f;
  const float strokesPerMin = elapsed > 2 ? s.strokeCount * 60.0f / elapsed : 0.0f;
  const float strokeStrongPct = s.strokeCount ? 100.0f * s.strokeStrongCount / s.strokeCount : 0.0f;
  const float strokeAvg = s.strokeCount ? (float)s.strokeSum100 / s.strokeCount : 0.0f;
  const float strokePhaseAvg = s.strokeCount ? (float)s.strokePhaseSumMs / s.strokeCount : 0.0f;
  const float sampleTotal = s.samplesAcquired ? (float)s.samplesAcquired : 1.0f;
  const uint8_t zone = strokeStrengthZone(s.strokeStrengthNow);
  const uint8_t intP90 = histogramPercentile(s.intensityHist, 90);
  const uint8_t intP95 = histogramPercentile(s.intensityHist, 95);
  const uint8_t strP90 = histogramPercentile(s.strokeHist, 90);
  const uint8_t strP95 = histogramPercentile(s.strokeHist, 95);
  const uint32_t queueDepth = sampleQueue ? uxQueueMessagesWaiting(sampleQueue) : 0;
  const bool recording = state == STATE_RECORDING;
  const bool storageBusy = recording || state == STATE_STOPPING;
  const bool countdown = state == STATE_START_COUNTDOWN;
  if (!storageBusy && storageReady) {
    phoneKnownSessionFiles = countSessionFiles();
    phoneKnownFreeKBytes = freeFlashBytes() / 1024UL;
  }
  char response[1920];
  const int written = snprintf(response, sizeof(response),
      "{\"state\":\"%s\",\"recording\":%s,\"countdown\":%s,\"can_start\":%s,"
      "\"can_files\":%s,\"can_manage_files\":%s,"
      "\"calibration_active\":%s,\"calibration_complete\":%s,\"calibration_stable\":%lu,"
      "\"calibration_rejected\":%lu,\"calibration_retries\":%u,\"calibration_last_moved\":%s,"
      "\"time_s\":%lu,"
      "\"stroke_strength\":%.2f,\"stroke_avg\":%.2f,\"stroke_p90\":%u,\"stroke_p95\":%u,"
      "\"stroke_zone\":%u,\"stroke_zone_name\":\"%s\",\"strokes_per_min\":%.2f,\"strokes\":%lu,\"strokes_strong_pct\":%.1f,"
      "\"cadence_per_min\":%.1f,\"cadence_ms\":%u,\"stride_cadence\":%.1f,\"leg_cadence\":%.1f,\"power_proxy\":%.2f,\"power_proxy_max\":%.2f,"
      "\"strength_phase_ms\":%.1f,\"strength_phase_avg_ms\":%.1f,\"strength_phase_max_ms\":%lu,"
      "\"intensity\":%.2f,\"intensity_avg\":%.2f,\"intensity_max\":%.2f,\"intensity_p90\":%u,\"intensity_p95\":%u,"
      "\"surge_gps\":%.1f,\"surge_max\":%.1f,\"spin_dps\":%.2f,\"gload_g\":%.3f,\"lean_deg\":%.2f,"
      "\"active_pct\":%.1f,\"idle_pct\":%.1f,\"zones\":[%.1f,%.1f,%.1f,%.1f,%.1f],"
      "\"rate_hz\":%.2f,\"queue\":%lu,\"qdrop\":%lu,\"gap8\":%lu,\"bad\":%lu,\"sat\":%lu,"
      "\"imu_stalls\":%lu,\"imu_stack_min_free\":%lu,\"writer_stack_min_free\":%lu,"
      "\"side\":\"%s\","
      "\"session_mode\":\"%s\",\"session_format\":\"%s\",\"file\":\"%s\",\"files\":%u,\"free_kb\":%lu,\"clients\":%u,\"compression\":%s}",
      deviceStateName(state), recording ? "true" : "false", countdown ? "true" : "false",
      phoneCanStartSession() ? "true" : "false", phoneCanListFiles() ? "true" : "false",
      phoneCanManageFiles() ? "true" : "false", (countdown && !countdownCalibration.complete) ? "true" : "false",
      countdownCalibration.complete ? "true" : "false", (unsigned long)countdownCalibration.stableSamples,
      (unsigned long)countdownCalibration.rejectedSamples, countdownCalibration.retryCount,
      countdownCalibration.lastAttemptMoved ? "true" : "false", (unsigned long)elapsed,
      s.strokeStrengthNow, strokeAvg, strP90, strP95,
      zone, strokeStrengthZoneName(zone), strokesPerMin, (unsigned long)s.strokeCount, strokeStrongPct,
      s.cadencePerMin, s.cadenceMsNow, s.strideCadencePerMin, s.legCadencePerMin, s.powerProxyNow, s.powerProxyMax,
      s.strokePhaseSumMs && s.strokeCount ? (float)s.strokePhaseSumMs / s.strokeCount : 0.0f, strokePhaseAvg,
      (unsigned long)s.strokePhaseMaxMs,
      s.intensityNow, s.intensityAverage, s.intensityMax, intP90, intP95,
      s.surgeNow, s.surgeMax, s.spinNow, s.gloadNow, s.relLeanDeg,
      100.0f * s.activeSamples / sampleTotal, 100.0f * s.idleSamples / sampleTotal,
      100.0f * s.zone0Count / sampleTotal, 100.0f * s.zone1Count / sampleTotal,
      100.0f * s.zone2Count / sampleTotal, 100.0f * s.zone3Count / sampleTotal,
      100.0f * s.zone4Count / sampleTotal, rate, (unsigned long)queueDepth,
      (unsigned long)s.qDropCount, (unsigned long)s.gap8Count, (unsigned long)s.badGapCount,
      (unsigned long)s.saturationCount, (unsigned long)s.imuStallCount,
      (unsigned long)s.imuStackMinFreeBytes, (unsigned long)s.writerStackMinFreeBytes,
      DEVICE_SIDE_NAME,
      sessionModeName(state == STATE_RECORDING || state == STATE_STOPPING ? activeSessionMode : selectedSessionMode),
      (state == STATE_RECORDING || state == STATE_STOPPING || state == STATE_START_COUNTDOWN) &&
              (state == STATE_START_COUNTDOWN ? selectedSessionMode : activeSessionMode) == SESSION_MODE_RAW_LAB ? "RRAW02" : "RIMU04",
      currentFilename, phoneKnownSessionFiles, (unsigned long)phoneKnownFreeKBytes, WiFi.softAPgetStationNum(),
      ((state == STATE_RECORDING || state == STATE_STOPPING) ? activeCompression : selectedCompression) ? "true" : "false");
  if (!phoneJsonFits(response, sizeof(response), written)) { sendPhoneJson(500, false, "Status JSON too long"); return; }
  webServer.sendHeader("Cache-Control", "no-store");
  webServer.send(200, "application/json", response);
}

void sendPhoneHistory() {
  uint16_t count = 0;
  portENTER_CRITICAL(&historyMux);
  count = liveHistoryCount;
  const uint16_t first = (liveHistoryHead + LIVE_HISTORY_POINTS - count) % LIVE_HISTORY_POINTS;
  for (uint16_t i = 0; i < count; ++i) {
    webHistoryCopy[i] = liveHistory[(first + i) % LIVE_HISTORY_POINTS];
  }
  portEXIT_CRITICAL(&historyMux);

  size_t position = 0;
  int written = snprintf(webHistoryJson, sizeof(webHistoryJson), "{\"points\":[");
  if (written < 0) { webServer.send(500, "application/json", "{\"ok\":false}"); return; }
  position = (size_t)written;
  bool truncated = false;
  for (uint16_t i = 0; i < count; ++i) {
    const LiveHistoryPoint& point = webHistoryCopy[i];
    // Punkt: [s, intensity, strokeStrength, cadenceMs, motionDps, leanCdeg, event, strokePhaseMs, side]
    written = snprintf(webHistoryJson + position, sizeof(webHistoryJson) - position,
                       "%s[%u,%u,%u,%u,%u,%d,%u,%u,%u]", i ? "," : "", point.elapsedSeconds, point.intensity,
                       point.strokeStrength, point.cadenceMs, point.motionDps, point.leanCdeg, point.event,
                       point.strokePhaseMs, point.side);
    if (written < 0 || (size_t)written >= sizeof(webHistoryJson) - position) { truncated = true; break; }
    position += (size_t)written;
  }
  written = snprintf(webHistoryJson + position, sizeof(webHistoryJson) - position,
                     "],\"truncated\":%s}", truncated ? "true" : "false");
  if (!phoneJsonFits(webHistoryJson + position, sizeof(webHistoryJson) - position, written)) {
    sendPhoneJson(500, false, "History JSON too long"); return;
  }
  webServer.sendHeader("Cache-Control", "no-store");
  webServer.send(200, "application/json", webHistoryJson);
}

void sendPhoneFiles() {
  if (!phoneCanListFiles()) { sendPhoneJson(409, false, "Pliki sa niedostepne podczas nagrywania"); return; }
  size_t position = 0;
  int written = snprintf(webFilesJson, sizeof(webFilesJson), "{\"files\":[");
  if (written < 0) { sendPhoneJson(500, false, "Blad listy plikow"); return; }
  position = (size_t)written;
  uint16_t total = 0;
  uint16_t sent = 0;
  bool truncated = false;
  File root = LittleFS.open("/");
  if (!root || !root.isDirectory()) { sendPhoneJson(500, false, "Nie mozna otworzyc katalogu"); return; }
  for (File file = root.openNextFile(); file; file = root.openNextFile()) {
    if (file.isDirectory() || !hasSessionExtension(file.name())) continue;
    ++total;
    if (sent >= WEB_FILE_LIST_LIMIT || truncated) continue;
    const char* name = file.name();
    const char* visibleName = name[0] == '/' ? name + 1 : name;
    written = snprintf(webFilesJson + position, sizeof(webFilesJson) - position,
                       "%s{\"name\":\"%s\",\"bytes\":%lu,\"packed\":%s}", sent ? "," : "", visibleName,
                       (unsigned long)file.size(), isPackedFilename(name) ? "true" : "false");
    if (written < 0 || (size_t)written >= sizeof(webFilesJson) - position) { truncated = true; continue; }
    position += (size_t)written;
    ++sent;
  }
  root.close();
  written = snprintf(webFilesJson + position, sizeof(webFilesJson) - position,
                     "],\"total\":%u,\"truncated\":%s}", total, (truncated || total > sent) ? "true" : "false");
  if (!phoneJsonFits(webFilesJson + position, sizeof(webFilesJson) - position, written)) {
    sendPhoneJson(500, false, "Files JSON too long"); return;
  }
  webServer.sendHeader("Cache-Control", "no-store");
  webServer.send(200, "application/json", webFilesJson);
}

void sendPhoneDiagnostics() {
  const RuntimeStats s = snapshotStats();
  const uint32_t queueDepth = sampleQueue ? uxQueueMessagesWaiting(sampleQueue) : 0;
  const bool storageBusy = state == STATE_RECORDING || state == STATE_STOPPING;
  const uint16_t fileCount = storageBusy ? phoneKnownSessionFiles : countSessionFiles();
  const uint32_t freeKBytes = storageBusy ? phoneKnownFreeKBytes : freeFlashBytes() / 1024UL;
  const float activePct = s.samplesAcquired ? 100.0f * s.activeSamples / s.samplesAcquired : 0.0f;
  const float idlePct = s.samplesAcquired ? 100.0f * s.idleSamples / s.samplesAcquired : 0.0f;
  const float strokeAvg = s.strokeCount ? (float)s.strokeSum100 / s.strokeCount : 0.0f;
  const float strokePhaseAvg = s.strokeCount ? (float)s.strokePhaseSumMs / s.strokeCount : 0.0f;
  const float sampleTotal = s.samplesAcquired ? (float)s.samplesAcquired : 1.0f;
  const uint8_t intP90 = histogramPercentile(s.intensityHist, 90);
  const uint8_t intP95 = histogramPercentile(s.intensityHist, 95);
  const uint8_t strP90 = histogramPercentile(s.strokeHist, 90);
  const uint8_t strP95 = histogramPercentile(s.strokeHist, 95);
  const float rateHz = (s.recordsQueued > 1 && s.lastSampleUs > s.firstSampleUs)
      ? (s.recordsQueued - 1) * 1000000.0f / (s.lastSampleUs - s.firstSampleUs) : 0.0f;
  const PowerSnapshot battery = powerSnapshot;
  const uint32_t batteryAgeSeconds = battery.lastUpdatedMs ? (millis() - battery.lastUpdatedMs) / 1000UL : 0;
  char response[2048];
  const int written = snprintf(response, sizeof(response),
      "{\"storage_ready\":%s,\"free_kb\":%lu,\"files\":%u,\"queue\":%lu,"
      "\"queue_max\":%lu,\"queue_capacity\":%u,\"qdrop\":%lu,\"writer_errors\":%lu,"
      "\"gap8\":%lu,\"bad\":%lu,\"saturation\":%lu,\"imu_stalls\":%lu,"
      "\"imu_stack_min_free\":%lu,\"writer_stack_min_free\":%lu,\"clients\":%u,\"ap_ip\":\"%s\","
      "\"calibration_active\":%s,\"calibration_complete\":%s,\"calibration_stable\":%lu,"
      "\"calibration_rejected\":%lu,\"calibration_retries\":%u,\"calibration_last_moved\":%s,"
      "\"battery_valid\":%s,\"battery_pct\":%d,\"battery_mv\":%d,\"battery_charging\":%s,\"battery_charge_known\":%s,"
      "\"battery_estimate_valid\":%s,\"battery_remaining_min\":%u,\"battery_drain_pct_h\":%.2f,\"battery_age_s\":%lu,"
      "\"intensity_avg\":%.2f,\"intensity_max\":%.2f,\"intensity_p90\":%u,\"intensity_p95\":%u,"
      "\"intensity\":%.2f,\"surge_gps\":%.1f,\"surge_max\":%.1f,\"spin_dps\":%.2f,\"gload_g\":%.3f,"
      "\"stroke_count\":%lu,\"stroke_strong_pct\":%.1f,\"stroke_avg\":%.2f,\"stroke_p90\":%u,\"stroke_p95\":%u,"
      "\"stroke_max\":%lu,\"stroke_phase_avg_ms\":%.1f,\"stroke_phase_max_ms\":%lu,"
      "\"cadence_ms\":%u,\"cadence_per_min\":%.1f,\"stride_cadence\":%.1f,\"leg_cadence\":%.1f,\"power_proxy\":%.2f,\"power_proxy_max\":%.2f,"
      "\"active_pct\":%.2f,\"idle_pct\":%.2f,\"samples\":%lu,\"queued\":%lu,\"written\":%lu,"
      "\"zones\":[%.1f,%.1f,%.1f,%.1f,%.1f],\"rate_hz\":%.2f,\"session_mode\":\"%s\",\"session_format\":\"%s\","
      "\"record_bytes\":%u,\"imu_done\":%s,\"writer_done\":%s}",
      storageReady ? "true" : "false", (unsigned long)freeKBytes, fileCount,
      (unsigned long)queueDepth, (unsigned long)s.maxQueueDepth, SAMPLE_QUEUE_LENGTH,
      (unsigned long)s.qDropCount, (unsigned long)s.writerErrorCount, (unsigned long)s.gap8Count,
      (unsigned long)s.badGapCount, (unsigned long)s.saturationCount, (unsigned long)s.imuStallCount,
      (unsigned long)s.imuStackMinFreeBytes, (unsigned long)s.writerStackMinFreeBytes, WiFi.softAPgetStationNum(),
      WiFi.softAPIP().toString().c_str(), (state == STATE_START_COUNTDOWN && !countdownCalibration.complete) ? "true" : "false",
      countdownCalibration.complete ? "true" : "false", (unsigned long)countdownCalibration.stableSamples,
      (unsigned long)countdownCalibration.rejectedSamples, countdownCalibration.retryCount,
      countdownCalibration.lastAttemptMoved ? "true" : "false", battery.valid ? "true" : "false", battery.levelPercent,
      battery.voltageMv, battery.charging ? "true" : "false", battery.chargeKnown ? "true" : "false",
      battery.estimateValid ? "true" : "false", battery.estimateMinutes, battery.dischargePctPerHour,
      (unsigned long)batteryAgeSeconds, s.intensityAverage, s.intensityMax, intP90, intP95,
      s.intensityNow, s.surgeNow, s.surgeMax, s.spinNow, s.gloadNow,
      (unsigned long)s.strokeCount, s.strokeCount ? 100.0f * s.strokeStrongCount / s.strokeCount : 0.0f,
      strokeAvg, strP90, strP95, (unsigned long)s.strokeMax100, strokePhaseAvg,
      (unsigned long)s.strokePhaseMaxMs, s.cadenceMsNow, s.cadencePerMin, s.strideCadencePerMin, s.legCadencePerMin, s.powerProxyNow, s.powerProxyMax,
      activePct, idlePct,
      (unsigned long)s.samplesAcquired, (unsigned long)s.recordsQueued, (unsigned long)s.recordsWritten,
      100.0f * s.zone0Count / sampleTotal, 100.0f * s.zone1Count / sampleTotal,
      100.0f * s.zone2Count / sampleTotal, 100.0f * s.zone3Count / sampleTotal,
      100.0f * s.zone4Count / sampleTotal, rateHz,
      sessionModeName(state == STATE_RECORDING || state == STATE_STOPPING ? activeSessionMode : selectedSessionMode),
      (state == STATE_RECORDING || state == STATE_STOPPING) && activeSessionMode == SESSION_MODE_RAW_LAB ? "RRAW02" : "RIMU04",
      (unsigned)(activeSessionMode == SESSION_MODE_RAW_LAB ? sizeof(RawLabRecord) : sizeof(SampleRecord)),
      imuTaskDone ? "true" : "false", writerTaskDone ? "true" : "false");
  if (!phoneJsonFits(response, sizeof(response), written)) { sendPhoneJson(500, false, "Diagnostics JSON too long"); return; }
  webServer.sendHeader("Cache-Control", "no-store");
  webServer.send(200, "application/json", response);
}

void sendPhoneMount() {
  const bool available = mountSnapshot.valid && state != STATE_RECORDING && state != STATE_STOPPING;
  char response[256];
  const int written = snprintf(response, sizeof(response), "{\"available\":%s,\"forward_g\":%.3f,\"lateral_g\":%.3f,\"vertical_g\":%.3f}",
                               available ? "true" : "false", mountSnapshot.forwardG, mountSnapshot.lateralG, mountSnapshot.verticalG);
  if (!phoneJsonFits(response, sizeof(response), written)) { sendPhoneJson(500, false, "Mount JSON too long"); return; }
  webServer.sendHeader("Cache-Control", "no-store");
  webServer.send(200, "application/json", response);
}

void downloadPhoneSession() {
  if (!phoneCanListFiles()) { sendPhoneJson(409, false, "Pobieranie tylko poza nagrywaniem"); return; }
  if (!webServer.hasArg("file")) { sendPhoneJson(400, false, "Brak pliku"); return; }
  char path[32] = {};
  if (!phoneSessionPathFromArg(webServer.arg("file"), path, sizeof(path))) {
    sendPhoneJson(404, false, "Nieprawidlowy plik sesji"); return;
  }
  File input = LittleFS.open(path, FILE_READ);
  if (!input) { sendPhoneJson(404, false, "Nie mozna otworzyc pliku"); return; }
  const char* visibleName = path + 1;
  webServer.sendHeader("Content-Disposition", String("attachment; filename=\"") + visibleName + "\"");
  webServer.sendHeader("Cache-Control", "no-store");
  webServer.streamFile(input, "application/octet-stream");
  input.close();
}

void handlePhoneAction() {
  if (!webServer.hasArg("cmd")) { sendPhoneJson(400, false, "Brak polecenia"); return; }
  if (pendingPhoneCommand != PHONE_COMMAND_NONE) { sendPhoneJson(409, false, "Polecenie oczekuje"); return; }
  const String command = webServer.arg("cmd");
  if (command == "start_default" || command == "start_max" ||
      command == "start_raw_default" || command == "start_raw_max") {
    if (!phoneCanStartSession()) { sendPhoneJson(409, false, "Nie mozna teraz rozpoczac sesji"); return; }
    if (command == "start_raw_max") pendingPhoneCommand = PHONE_COMMAND_START_RAW_MAX;
    else if (command == "start_raw_default") pendingPhoneCommand = PHONE_COMMAND_START_RAW_DEFAULT;
    else pendingPhoneCommand = command == "start_max" ? PHONE_COMMAND_START_MAX : PHONE_COMMAND_START_DEFAULT;
    sendPhoneJson(202, true, command.startsWith("start_raw") ? "Rozpoczynam RAW LAB" : "Rozpoczynam odliczanie");
  } else if (command == "toggle_zip") {
    // Przelacznik pakowania strumieniowego "w locie". Obowiazuje od nastepnej sesji.
    if (!phoneCanStartSession()) { sendPhoneJson(409, false, "Zmiana tylko poza sesja"); return; }
    selectedCompression = !selectedCompression;
    sendPhoneJson(200, true, selectedCompression ? "Pakowanie w locie: WLACZONE" : "Pakowanie w locie: WYLACZONE");
  } else if (command == "stop") {
    if (state != STATE_RECORDING) { sendPhoneJson(409, false, "Brak aktywnej sesji"); return; }
    pendingPhoneCommand = PHONE_COMMAND_STOP;
    sendPhoneJson(202, true, "Zatrzymuje sesje");
  } else if (command == "cancel") {
    if (state != STATE_START_COUNTDOWN) { sendPhoneJson(409, false, "Nie trwa odliczanie"); return; }
    pendingPhoneCommand = PHONE_COMMAND_CANCEL;
    sendPhoneJson(202, true, "Anuluje odliczanie");
  } else if (command == "delete_one") {
    if (!phoneCanManageFiles() || !webServer.hasArg("file")) { sendPhoneJson(409, false, "Nie mozna teraz usunac pliku"); return; }
    if (!phoneSessionPathFromArg(webServer.arg("file"), phonePendingFilename, sizeof(phonePendingFilename))) {
      sendPhoneJson(404, false, "Nieprawidlowy plik sesji"); return;
    }
    pendingPhoneCommand = PHONE_COMMAND_DELETE_ONE;
    sendPhoneJson(202, true, "Usuwam wybrany plik");
  } else if (command == "delete_all") {
    if (!phoneCanManageFiles() || !webServer.hasArg("confirm") || webServer.arg("confirm") != "ERASE") {
      sendPhoneJson(409, false, "Brak potwierdzenia ERASE"); return;
    }
    pendingPhoneCommand = PHONE_COMMAND_DELETE_ALL;
    sendPhoneJson(202, true, "Usuwam wszystkie pliki sesji");
  } else if (command == "pack_one") {
    if (!phoneCanManageFiles() || !webServer.hasArg("file")) { sendPhoneJson(409, false, "Nie mozna teraz pakowac pliku"); return; }
    if (!phoneSessionPathFromArg(webServer.arg("file"), phonePendingFilename, sizeof(phonePendingFilename))) {
      sendPhoneJson(404, false, "Nieprawidlowy plik sesji"); return;
    }
    if (isPackedFilename(phonePendingFilename)) { sendPhoneJson(409, false, "Plik juz jest spakowany"); return; }
    pendingPhoneCommand = PHONE_COMMAND_PACK_ONE;
    sendPhoneJson(202, true, "Pakuje wybrany plik");
  } else if (command == "pack_all") {
    if (!phoneCanManageFiles()) { sendPhoneJson(409, false, "Nie mozna teraz pakowac plikow"); return; }
    pendingPhoneCommand = PHONE_COMMAND_PACK_ALL;
    sendPhoneJson(202, true, "Pakuje wszystkie sesje");
  } else {
    sendPhoneJson(400, false, "Nieznane polecenie");
  }
}

void processPhoneCommand() {
  const PhoneCommand command = pendingPhoneCommand;
  if (command == PHONE_COMMAND_NONE) return;
  pendingPhoneCommand = PHONE_COMMAND_NONE;
  if (command == PHONE_COMMAND_START_DEFAULT || command == PHONE_COMMAND_START_MAX ||
      command == PHONE_COMMAND_START_RAW_DEFAULT || command == PHONE_COMMAND_START_RAW_MAX) {
    if (!phoneCanStartSession()) return;
    selectedSessionMode = (command == PHONE_COMMAND_START_RAW_DEFAULT || command == PHONE_COMMAND_START_RAW_MAX)
        ? SESSION_MODE_RAW_LAB : SESSION_MODE_NORMAL;
    volumeSelection = (command == PHONE_COMMAND_START_MAX || command == PHONE_COMMAND_START_RAW_MAX) ? 1 : 0;
    beginStartCountdown();
  } else if (command == PHONE_COMMAND_STOP) {
    if (state == STATE_RECORDING) stopSession();
  } else if (command == PHONE_COMMAND_CANCEL && state == STATE_START_COUNTDOWN) {
    if (speakerReady) M5.Speaker.stop();
    restoreDefaultSpeakerVolume();
    enterMainMenu();
  } else if (command == PHONE_COMMAND_DELETE_ONE && phoneCanManageFiles()) {
    copyLittleFsPath(deleteFilename, sizeof(deleteFilename), phonePendingFilename);
    phonePendingFilename[0] = '\0';
    if (!deleteOneSessionFile()) { setError("DELETE ONE FAIL"); return; }
    phoneKnownSessionFiles = countSessionFiles();
    phoneKnownFreeKBytes = freeFlashBytes() / 1024UL;
    enterMainMenu();
  } else if (command == PHONE_COMMAND_DELETE_ALL && phoneCanManageFiles()) {
    if (!deleteAllSessionFiles()) { setError("DELETE ALL FAIL"); return; }
    phoneKnownSessionFiles = countSessionFiles();
    phoneKnownFreeKBytes = freeFlashBytes() / 1024UL;
    enterMainMenu();
  } else if (command == PHONE_COMMAND_PACK_ONE && phoneCanManageFiles()) {
    char source[32] = "";
    copyLittleFsPath(source, sizeof(source), phonePendingFilename);
    phonePendingFilename[0] = '\0';
    char packedPath[32] = "";
    if (!packedNameForPath(source, packedPath, sizeof(packedPath))) { setError("PACK PATH FAIL"); return; }
    if (!packOneFile(source, packedPath)) { setError("PACK FAIL"); return; }
    phoneKnownSessionFiles = countSessionFiles();
    phoneKnownFreeKBytes = freeFlashBytes() / 1024UL;
    enterMainMenu();
  } else if (command == PHONE_COMMAND_PACK_ALL && phoneCanManageFiles()) {
    uint16_t packedCount = 0; uint32_t freedBytes = 0;
    if (!packAllSessionFiles(packedCount, freedBytes)) { setError("PACK ALL FAIL"); return; }
    phoneKnownSessionFiles = countSessionFiles();
    phoneKnownFreeKBytes = freeFlashBytes() / 1024UL;
    Serial.printf("# PACK_ALL,count=%u,freed=%luB\n", packedCount, (unsigned long)freedBytes);
    enterMainMenu();
  }
}

void setupWifiPanel() {
  WiFi.mode(WIFI_AP);
  if (!WiFi.softAPConfig(WIFI_AP_IP, WIFI_AP_GATEWAY, WIFI_AP_SUBNET)) {
    Serial.println("# WIFI AP IP CONFIG FAIL");
    return;
  }
  if (!WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD, 1, false, WIFI_AP_MAX_CLIENTS)) {
    Serial.println("# WIFI AP START FAIL");
    return;
  }
  webServer.enableDelay(false);
  webServer.on("/", HTTP_GET, []() {
    webServer.sendHeader("Cache-Control", "no-store");
    webServer.send_P(200, PSTR("text/html; charset=utf-8"), PHONE_DASHBOARD_HTML);
  });
  webServer.on("/api/status", HTTP_GET, sendPhoneStatus);
  webServer.on("/api/history", HTTP_GET, sendPhoneHistory);
  webServer.on("/api/files", HTTP_GET, sendPhoneFiles);
  webServer.on("/api/diagnostics", HTTP_GET, sendPhoneDiagnostics);
  webServer.on("/api/mount", HTTP_GET, sendPhoneMount);
  webServer.on("/api/download", HTTP_GET, downloadPhoneSession);
  webServer.on("/api/action", HTTP_POST, handlePhoneAction);
  webServer.onNotFound([]() { webServer.send(404, "application/json", "{\"ok\":false,\"message\":\"Nie znaleziono\"}"); });
  webServer.begin();
  wifiPanelReady = true;
  Serial.printf("# WIFI AP READY,SSID,%s,IP,%s\n", WIFI_AP_SSID, WiFi.softAPIP().toString().c_str());
}

// ---------------------------------------------------------------------------
// Eksport USB: format zgodny z poprzednim skryptem PowerShell.
// ---------------------------------------------------------------------------
void drawExportProgress(uint32_t sent, uint32_t total) {
  const uint32_t percent = total ? 100UL * sent / total : 100;
  if (percent == lastExportProgressPercent) return;
  lastExportProgressPercent = percent;
  if (percent == 0) {
    drawHeader("EXPORTING");
    M5.Lcd.setCursor(10, 35); M5.Lcd.print(selectedFilename);
    M5.Lcd.setCursor(10, 88); M5.Lcd.print("USB Serial active");
  }
  M5.Lcd.fillRect(10, 52, 220, 11, TFT_BLACK);
  M5.Lcd.setCursor(10, 52); M5.Lcd.printf("%lu / %lu B", (unsigned long)sent, (unsigned long)total);
  M5.Lcd.fillRect(10, 68, 220, 10, UI_DARK_RED);
  M5.Lcd.fillRect(10, 68, 220 * percent / 100UL, 10, UI_RED);
}

void exportSelectedFile() {
  if (state == STATE_RECORDING || state == STATE_STOPPING) { Serial.println("ERR stop recording before export"); return; }
  if (!selectedFilename[0] && !selectSessionFile(selectedFileIndex)) { Serial.println("ERR no session"); return; }
  File input = LittleFS.open(selectedFilename, FILE_READ);
  if (!input) { Serial.printf("ERR cannot open %s\n", selectedFilename); return; }
  state = STATE_EXPORTING;
  uint8_t raw[EXPORT_RAW_CHUNK_BYTES];
  unsigned char encoded[4 * ((EXPORT_RAW_CHUNK_BYTES + 2) / 3) + 1];
  const uint32_t total = input.size();
  uint32_t sent = 0, sum32 = 0;
  lastExportProgressPercent = 101;
  drawExportProgress(0, total);
  Serial.printf("BEGIN_RIM filename=%s bytes=%lu encoding=base64\n", selectedFilename, (unsigned long)total);
  while (input.available()) {
    const size_t n = input.read(raw, sizeof(raw));
    if (!n) break;
    for (size_t i = 0; i < n; ++i) sum32 += raw[i];
    size_t encodedLength = 0;
    if (mbedtls_base64_encode(encoded, sizeof(encoded), &encodedLength, raw, n) != 0) {
      input.close(); setError("BASE64 ENCODE FAIL"); return;
    }
    encoded[encodedLength] = '\0';
    Serial.printf("B64 %s\n", (char*)encoded);
    sent += n;
    drawExportProgress(sent, total);
    M5.update();
  }
  input.close();
  Serial.printf("END_RIM filename=%s bytes=%lu sum32=%lu\n", selectedFilename, (unsigned long)sent, (unsigned long)sum32);
  state = STATE_EXPORT_SELECT;
  drawStatusScreen(true);
}

void printSerialHelp() {
  Serial.println("HELP: LIST | SELECT <number> | EXPORT | MENU | HELP");
  Serial.println("USB protocol: BEGIN_RIM, B64, END_RIM");
}

void processSerialCommand(char* command) {
  while (*command == ' ') command++;
  for (char* p = command; *p; ++p) *p = toupper((unsigned char)*p);
  if (!strcmp(command, "LIST")) listSessionFilesToSerial();
  else if (!strncmp(command, "SELECT ", 7)) {
    const uint16_t number = (uint16_t)atoi(command + 7);
    if (state == STATE_RECORDING || state == STATE_STOPPING) Serial.println("ERR stop recording before selection");
    else if (!number || !selectSessionFile(number)) Serial.println("ERR invalid session number; use LIST");
    else { state = STATE_EXPORT_SELECT; drawStatusScreen(true); }
  } else if (!strcmp(command, "EXPORT")) exportSelectedFile();
  else if (!strcmp(command, "MENU") && state != STATE_RECORDING && state != STATE_STOPPING) enterMainMenu();
  else if (!strcmp(command, "HELP")) printSerialHelp();
  else if (*command) Serial.println("ERR unknown command; type HELP");
}

void readSerialCommands() {
  while (Serial.available()) {
    const char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') { commandBuffer[commandLength] = '\0'; processSerialCommand(commandBuffer); commandLength = 0; }
    else if (commandLength < sizeof(commandBuffer) - 1) commandBuffer[commandLength++] = c;
    else { commandLength = 0; Serial.println("ERR command too long"); }
  }
}

// ---------------------------------------------------------------------------
// Przyciski, setup i glowna petla
// ---------------------------------------------------------------------------
void enterExportSelection() {
  const uint16_t count = countSessionFiles();
  if (count) {
    if (!selectedFileIndex || selectedFileIndex > count) selectedFileIndex = 1;
    selectSessionFile(selectedFileIndex);
  } else selectedFilename[0] = '\0';
  state = STATE_EXPORT_SELECT;
  drawStatusScreen(true);
}

void enterFileManager() {
  fileMenuIndex = 0;
  state = STATE_FILE_MANAGER;
  drawStatusScreen(true);
}

void enterDeleteOneSelection() {
  const uint16_t count = countSessionFiles();
  if (count) {
    if (!deleteFileIndex || deleteFileIndex > count) deleteFileIndex = 1;
    selectDeleteFile(deleteFileIndex);
  } else {
    deleteFilename[0] = '\0';
  }
  state = STATE_DELETE_ONE_SELECT;
  drawStatusScreen(true);
}

void handleButtons() {
  if (state == STATE_STORAGE_INIT_REQUIRED && M5.BtnA.wasHold()) {
    storageReady = formatAndMountLittleFS();
    if (storageReady) enterMainMenu();
    else setError("LITTLEFS FORMAT FAIL");
    return;
  }
  if (state == STATE_MAIN_MENU) {
    if (M5.BtnA.wasClicked()) { menuIndex = (menuIndex + 1) % MENU_ITEM_COUNT; drawStatusScreen(true); }
    if (M5.BtnB.wasClicked()) {
      if (menuIndex == 0) enterVolumeSelect(SESSION_MODE_NORMAL);
      else if (menuIndex == 1) enterVolumeSelect(SESSION_MODE_RAW_LAB);
      else if (menuIndex == 2) enterExportSelection();
      else if (menuIndex == 3) enterFileManager();
      else if (menuIndex == 4) { state = STATE_MOUNT_GUIDE; drawStatusScreen(true); }
      else { state = STATE_DIAGNOSTICS; drawStatusScreen(true); }
      drawStatusScreen(true);
    }
    return;
  }
  if (state == STATE_FILE_MANAGER) {
    if (M5.BtnB.wasHold()) { enterMainMenu(); return; }
    if (M5.BtnA.wasClicked()) {
      fileMenuIndex = (fileMenuIndex + 1) % 2;
      drawStatusScreen(true);
      return;
    }
    if (M5.BtnB.wasClicked()) {
      if (fileMenuIndex == 0) enterDeleteOneSelection();
      else { state = STATE_DELETE_ALL_CONFIRM; drawStatusScreen(true); }
      return;
    }
    return;
  }
  if (state == STATE_DELETE_ONE_SELECT) {
    const uint16_t count = countSessionFiles();
    if (M5.BtnB.wasHold()) { enterFileManager(); return; }
    if (count == 0) {
      if (M5.BtnA.wasClicked() || M5.BtnB.wasClicked()) enterFileManager();
      return;
    }
    if (M5.BtnA.wasClicked()) {
      deleteFileIndex = (deleteFileIndex % count) + 1;
      selectDeleteFile(deleteFileIndex);
      drawStatusScreen(true);
      return;
    }
    if (M5.BtnB.wasClicked()) { state = STATE_DELETE_ONE_CONFIRM; drawStatusScreen(true); return; }
    return;
  }
  if (state == STATE_DELETE_ONE_CONFIRM) {
    if (M5.BtnA.wasClicked()) { state = STATE_DELETE_ONE_SELECT; drawStatusScreen(true); return; }
    if (M5.BtnB.wasHold()) {
      if (!deleteOneSessionFile()) { setError("DELETE ONE FAIL"); return; }
      enterFileManager();
      return;
    }
    return;
  }
  if (state == STATE_DELETE_ALL_CONFIRM) {
    if (M5.BtnA.wasClicked()) { enterFileManager(); return; }
    if (M5.BtnB.wasHold()) {
      if (!deleteAllSessionFiles()) { setError("DELETE ALL FAIL"); return; }
      enterFileManager();
      return;
    }
    return;
  }
  if (state == STATE_VOLUME_SELECT) {
    if (M5.BtnB.wasHold()) { restoreDefaultSpeakerVolume(); enterMainMenu(); return; }
    if (M5.BtnA.wasClicked()) { volumeSelection = 1 - volumeSelection; drawStatusScreen(true); return; }
    if (M5.BtnB.wasClicked()) { beginStartCountdown(); return; }
    return;
  }
  if (state == STATE_START_COUNTDOWN) {
    if (M5.BtnA.wasClicked() || M5.BtnB.wasClicked()) {
      if (speakerReady) M5.Speaker.stop();
      restoreDefaultSpeakerVolume();
      enterMainMenu();
    }
    return;
  }
  if (state == STATE_RECORDING) { if (M5.BtnA.wasClicked()) stopSession(); return; }
  if (state == STATE_SAVED || state == STATE_MOUNT_GUIDE || state == STATE_DIAGNOSTICS) {
    if (M5.BtnA.wasClicked() || M5.BtnB.wasClicked()) enterMainMenu();
    return;
  }
  if (state == STATE_EXPORT_SELECT) {
    if (M5.BtnB.wasHold()) { enterMainMenu(); return; }
    if (M5.BtnA.wasClicked()) {
      const uint16_t count = countSessionFiles();
      if (count) { selectedFileIndex = (selectedFileIndex % count) + 1; selectSessionFile(selectedFileIndex); }
      drawStatusScreen(true);
    }
    if (M5.BtnB.wasClicked()) exportSelectedFile();
  }
}

void setup() {
  auto cfg = M5.config();
  cfg.internal_spk = true;
  M5.begin(cfg);
  // M5StickS3 ma wewnetrzny glosnik I2S; nie zatrzymuj loggera, jezeli
  // audio nie wystartuje, lecz raportuj to przez Serial.
  speakerReady = M5.Speaker.isEnabled() || M5.Speaker.begin();
  if (speakerReady) M5.Speaker.setVolume(SPEAKER_VOLUME);
  else Serial.println("# SPEAKER NOT AVAILABLE");
  M5.Lcd.setRotation(1);
  M5.Lcd.setTextWrap(false);
  Serial.begin(SERIAL_BAUD);
  delay(300);
  // Animowany baner marki przed inicjalizacją sesji i menu.
  state = STATE_SPLASH;
  playSplashAnimation();
  state = STATE_BOOT;
  drawStatusScreen(true);
  if (!M5.Imu.isEnabled()) { setError("IMU NOT AVAILABLE"); drawStatusScreen(true); return; }

  storageReady = initStorage();
  if (!storageReady) {
    if (!fsPartitionBytes) setError("NO SPIFFS PARTITION");
    else state = STATE_STORAGE_INIT_REQUIRED;
    drawStatusScreen(true);
    return;
  }
  // Bias zyroskopu i punkt odniesienia REL LEAN sa teraz kalibrowane
  // bezposrednio przed kazda sesja, podczas odliczania 10, 9, 8.
  updateBatteryTelemetry(true);
  phoneKnownSessionFiles = countSessionFiles();
  phoneKnownFreeKBytes = freeFlashBytes() / 1024UL;
  setupWifiPanel();
  enterMainMenu();
  Serial.println("# READY V3: async IMU + RAM queue + writer task. New logs: .pnt");
  printSerialHelp();
}

void loop() {
  M5.update();
  updateBatteryTelemetry();
  if (state != STATE_RECORDING && state != STATE_STOPPING &&
      millis() - lastMountSnapshotMs >= MOUNT_SNAPSHOT_PERIOD_MS) {
    updateMountSnapshot();
  }
  // W stanie bledu po timeout IMU utrzymaj writerTask przy zyciu tak dlugo,
  // jak imuTask moze jeszcze wyslac rekord. Dopiero wtedy rozpocznij drenaz.
  if (stopWriterWhenImuDone && imuTaskDone) {
    writerStopRequested = true;
    stopWriterWhenImuDone = false;
    Serial.println("# DEFERRED_WRITER_STOP");
  }
  // Obsluguje takze opoznione zakonczenie zadan po stanie bledu, bez ryzyka
  // skasowania kolejki uzywanej jeszcze przez imuTask lub writerTask.
  cleanupSessionResources();
  readSerialCommands();
  if (wifiPanelReady) webServer.handleClient();
  // Komenda HTTP zostaje wykonana wyłącznie tutaj, w tej samej maszynie
  // stanów co przyciski urządzenia; handler HTTP tylko ją kolejkował.
  processPhoneCommand();
  handleButtons();
  updateStartCountdown();
  updateSpeakerVolumeReset();
  if (state == STATE_RECORDING && writerFailed) {
    stopSession();
    if (state != STATE_ERROR) setError("WRITER FAILED");
  }
  drawStatusScreen();
}
