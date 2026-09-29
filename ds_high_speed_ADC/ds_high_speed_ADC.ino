/** @file ds_high_speed_ADC.ino
 *  @description Two-channel ADC capture on Teensy 4.x. V1 on ADC0, V2 on ADC1,
 *  sampled simultaneously (analogSyncRead).
 *  RUN_PIN  (switch to GND, active LOW):
 *    CONTINUOUS: samples while RUN is LOW, stops when it goes HIGH
 *    BURST     : one burst per RUN falling edge (HIGH -> LOW); RUN may stay LOW afterwards
 *  MODE_PIN (switch to GND, pullup): open = CONTINUOUS, closed = BURST
 *    mode is latched at the RUN falling edge; changing it mid-run takes effect at next run
 *  CONTINUOUS: IntervalTimer ISR samples into a ring buffer; loop() prints continuously
 *  BURST     : tight polling loop on the CPU cycle counter, phase-accumulator pacing,
 *              capture to RAM2, then print
 *  Data lines are CSV, volts to 3 decimals: V1,V2[,V1-V2]  (burst adds index,time_us,dt_us)
 *  A bad ADC read prints as "nan". Status lines start with '#' so a parser can skip them.
 *  CONTINUOUS lines carry no timestamp; lost samples are marked "# gap: N samples lost".
 *
 *  @reference Teensy Examples: analogContinuousRead.ino, adc_dma.ino
 *  @author David Smith
 *
 *  0.9.7  2026-09-29
 *   - ENERGIZE/PWR pins and elapsed* timers removed (HV enable to be tied to RUN)
 *   - burst starts on RUN falling edge; repeating burst mode removed
 *   - burst pacing: cycle-counter phase accumulator (no drift, exact for any F_CPU)
 *   - continuous: sequence-numbered ring entries -> "# gap" markers; bad reads -> nan
 *   - integer millivolt formatting (no float printf); snprintf; burst buffers in RAM2
 *  0.9.6  2026-09-29
 *   - RUN_PIN switch (debounced) starts/stops capture; MODE_PIN selects continuous/burst
 *  0.9.5  2026-09-29
 *   - continuous mode (IntervalTimer + ring buffer); Serial1 2 Mbaud + TX buffer; CSV output
 *  0.9.4  2026-09-29
 *   - 12-bit T4 scaling; 2 ch sync read; fixed-grid burst timing
 */

#include <ADC.h>
#include <ADC_util.h>

#define HWSERIAL    Serial1
#define HW_BAUDRATE 2000000   // [baud] T4 UART ok to ~6M; adapter limit: CP2102N ~3M, FT232R 3M, FT232H 12M

// --------------------------------------------------------------------------------------
const char    *VERSION            = "0.9.7";
const uint32_t V_REF_MV           = 3297;    // [mV] measured reference (3300 nominal)
const uint32_t SAMPLE_INTERVAL_US = 100;     // [us] sample period
// approx min SAMPLE_INTERVAL_US:
//   BURST     : ADC limited     ~5 us (ADC_AVERAGING 1), ~15 us (ADC_AVERAGING 4)
//   CONTINUOUS: serial limited  bytes/line = 12 (V1,V2) or 19 (+V1-V2); UART 10 bits/byte; keep <= 70% load
//               2 Mbaud: ~90 us (V1,V2), ~140 us (+diff)    3 Mbaud: ~60 us, ~90 us
//               USB Serial: ~10-20 us (formatting limited)
//   check: BURST "max dt" = SAMPLE_INTERVAL_US;  CONTINUOUS no "# gap" lines
const bool     PRINT_DIFF         = false;   // add V1-V2 column (continuous at 100 us: needs ~3 Mbaud or USB)
const uint8_t  ADC_AVERAGING      = 4;       // 1, 4, 8, 16, 32
const uint32_t BURST_INTERVAL_MS  = 500;     // [ms] burst length
const uint32_t DEBOUNCE_MS        = 20;      // [ms] RUN switch debounce
uint16_t CONSOLE_FLAG = 1;                   // print to USB Serial
uint16_t HWSER1_FLAG  = 1;                   // print to Serial1 (continuous: every enabled port must keep up)
// --------------------------------------------------------------------------------------

const uint16_t ADC_MAX = 4095;        // 12-bit T4
const uint16_t ADC_BAD = 0xFFFF;      // marks a failed read (prints "nan")

// burst buffer in RAM2 (DMAMEM): every slot rewritten each burst before printing, no clear needed
const uint32_t A_SIZE = BURST_INTERVAL_MS * 1000 / SAMPLE_INTERVAL_US;  // [samples] per burst
const uint32_t MEM    = A_SIZE * (4 + 2 * 2);                          // [bytes] time 4 B + 2 values 2 B
static_assert(MEM < 400000, "burst buffer too large: shorten burst or lengthen sample interval");
DMAMEM uint32_t times[A_SIZE];        // [cycles] sample time rel. to first sample
DMAMEM uint16_t ana_data[2][A_SIZE];  // raw counts 0..ADC_MAX, or ADC_BAD

// continuous ring buffer (single producer = ISR, single consumer = loop)
const uint32_t RB_SIZE = 4096;        // [samples] power of 2; 4096 x 100 us = 410 ms of slack
static_assert((RB_SIZE & (RB_SIZE - 1)) == 0, "RB_SIZE must be a power of 2");
struct Sample { uint32_t seq; uint16_t v1, v2; };
volatile Sample   rb[RB_SIZE];        // volatile: ISR/loop ordering
volatile uint32_t rb_head     = 0;    // next write index (ISR only)
volatile uint32_t rb_tail     = 0;    // next read index (loop only)
volatile uint32_t rb_overflow = 0;    // samples dropped, buffer full
volatile uint32_t n_stream    = 0;    // samples taken this run (also the sequence number)
volatile uint32_t n_bad       = 0;    // failed ADC reads this run
uint32_t next_seq = 0;                // loop side: sequence number expected next
uint8_t tx_buf1[8192];                // extra Serial1 TX buffer [bytes]
IntervalTimer sample_itimer;

const uint8_t V1_PIN   = A0;          // -> ADC0
const uint8_t V2_PIN   = A1;          // -> ADC1
const uint8_t RUN_PIN  = 23;          // switch to GND: LOW = run
const uint8_t MODE_PIN = 22;          // switch to GND: open = CONTINUOUS, LOW = BURST
bool run_on   = false;                // debounced RUN switch state (true = LOW)
bool run_fell = false;                // set on debounced falling edge, cleared by loop()

enum CaptureMode { CM_CONTINUOUS, CM_BURST };
CaptureMode capture_mode = CM_CONTINUOUS;   // latched at run start
enum State { ST_IDLE, ST_STREAM };
State state = ST_IDLE;
uint32_t burst_cycles = 0;            // [cycles] actual burst duration
char msg[96] = "";

ADC *adc = new ADC();  // adc object

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RUN_PIN, INPUT_PULLUP);
  pinMode(MODE_PIN, INPUT_PULLUP);
  pinMode(V1_PIN, INPUT);
  pinMode(V2_PIN, INPUT);
  ARM_DEMCR |= ARM_DEMCR_TRCENA;              // cycle counter (already on in the core; harmless)
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;
  initADC();
  initSerialPorts();
  snprintf(msg, sizeof(msg), "# Version: %s  ready: RUN switch on (LOW) to start\n", VERSION);
  echo(msg);
}

void loop() {
  updateRun();
  digitalWriteFast(LED_BUILTIN, run_on);

  switch (state) {
    case ST_IDLE:
      if (run_fell) {                 // run start: latch mode, print header, start
        run_fell = false;
        capture_mode = digitalReadFast(MODE_PIN) ? CM_CONTINUOUS : CM_BURST;
        printHeader();
        if (capture_mode == CM_CONTINUOUS) {
          startStream();
        } else {
          captureBurst();             // blocking; edges during capture/print are ignored
          printBurst();
          run_fell = false;
        }
      }
      break;
    case ST_STREAM:
      streamOut();
      if (!run_on) stopStream();
      break;
  }
}

void updateRun() {
  // debounce RUN switch: state accepted after DEBOUNCE_MS stable
  static bool     last_raw = HIGH;
  static uint32_t t_change = 0;
  bool raw = digitalReadFast(RUN_PIN);
  if (raw != last_raw) {
    last_raw = raw;
    t_change = millis();
  } else if (millis() - t_change >= DEBOUNCE_MS) {
    bool on = (raw == LOW);
    if (on && !run_on) run_fell = true;
    run_on = on;
  }
}

// ---------------------------------------------------------------- continuous mode
void startStream() {
  rb_head = rb_tail = rb_overflow = n_stream = n_bad = 0;   // ISR not running: safe to reset
  next_seq = 0;
  echo(PRINT_DIFF ? "V1,V2,V1-V2\n" : "V1,V2\n");
  sample_itimer.priority(32);         // above UART/USB ISRs: low sample jitter
  sample_itimer.begin(sampleISR, SAMPLE_INTERVAL_US);
  state = ST_STREAM;
}

void stopStream() {
  sample_itimer.end();                // no ISR after this: drain is race-free
  while (rb_tail != rb_head) streamOut();
  snprintf(msg, sizeof(msg), "# stopped: %lu samples, %lu dropped, %lu bad reads\n",
           (unsigned long)n_stream, (unsigned long)rb_overflow, (unsigned long)n_bad);
  echo(msg);
  reportADCErrors();
  state = ST_IDLE;
}

uint16_t rawOrBad(int r) {            // failed reads come back as a sentinel outside 0..ADC_MAX
  return ((uint32_t)r > ADC_MAX) ? ADC_BAD : (uint16_t)r;
}

void sampleISR() {
  ADC::Sync_result r = adc->analogSyncRead(V1_PIN, V2_PIN);
  uint32_t seq = n_stream++;
  uint16_t c1 = rawOrBad(r.result_adc0);
  uint16_t c2 = rawOrBad(r.result_adc1);
  if (c1 == ADC_BAD || c2 == ADC_BAD) n_bad++;
  uint32_t next = (rb_head + 1) & (RB_SIZE - 1);
  if (next == rb_tail) {              // full: drop newest, count it (seq gap shows in output)
    rb_overflow++;
    return;
  }
  rb[rb_head].seq = seq;
  rb[rb_head].v1  = c1;
  rb[rb_head].v2  = c2;
  rb_head = next;                     // publish after data written
}

void streamOut() {
  uint32_t n = 0;
  while (rb_tail != rb_head && n++ < 64) {       // bounded: loop() stays responsive
    uint32_t seq = rb[rb_tail].seq;
    uint16_t c1  = rb[rb_tail].v1;
    uint16_t c2  = rb[rb_tail].v2;
    rb_tail = (rb_tail + 1) & (RB_SIZE - 1);     // release slot after data read
    if (seq != next_seq) {                       // samples were dropped before this one
      snprintf(msg, sizeof(msg), "# gap: %lu samples lost\n", (unsigned long)(seq - next_seq));
      echo(msg);
    }
    next_seq = seq + 1;
    formatV(msg, sizeof(msg), c1, c2);
    echo(msg);
  }
}

// ---------------------------------------------------------------- burst mode
// Phase accumulator: sample period in CPU cycles = whole + frac/1e6, so the grid stays exact
// even when F_CPU * interval is not a whole number of cycles. No drift, no elapsed* timers.
void captureBurst() {
  const uint64_t q     = (uint64_t)SAMPLE_INTERVAL_US * F_CPU_ACTUAL;   // cycles * 1e6
  const uint32_t whole = (uint32_t)(q / 1000000ULL);
  const uint32_t frac  = (uint32_t)(q % 1000000ULL);
  uint32_t acc  = 0;                  // phase accumulator [1e-6 cycle]
  uint32_t next = ARM_DWT_CYCCNT;     // first sample now, then fixed grid
  uint32_t c0   = 0;
  for (uint32_t i = 0; i < A_SIZE; i++) {
    while ((int32_t)(ARM_DWT_CYCCNT - next) < 0) { }   // wrap-safe wait
    uint32_t c = ARM_DWT_CYCCNT;
    ADC::Sync_result r = adc->analogSyncRead(V1_PIN, V2_PIN);
    if (i == 0) c0 = c;
    times[i]       = c - c0;
    ana_data[0][i] = rawOrBad(r.result_adc0);
    ana_data[1][i] = rawOrBad(r.result_adc1);
    next += whole;
    acc  += frac;
    if (acc >= 1000000UL) { acc -= 1000000UL; next++; }
  }
  burst_cycles = ARM_DWT_CYCCNT - c0;
}

uint32_t cyclesToUs(uint32_t cyc) {
  return (uint32_t)(((uint64_t)cyc * 1000000ULL + F_CPU_ACTUAL / 2) / F_CPU_ACTUAL);
}

void printBurst() {
  uint32_t dt_max = 0;
  echo(PRINT_DIFF ? "index,time_us,dt_us,V1,V2,V1-V2\n" : "index,time_us,dt_us,V1,V2\n");
  uint32_t t_prev = 0;
  for (uint32_t i = 0; i < A_SIZE; i++) {
    uint32_t t  = cyclesToUs(times[i]);
    uint32_t dt = (i == 0) ? 0 : t - t_prev;
    t_prev = t;
    if (dt > dt_max) dt_max = dt;
    int k = snprintf(msg, sizeof(msg), "%lu,%lu,%lu,", (unsigned long)i, (unsigned long)t, (unsigned long)dt);
    formatV(msg + k, sizeof(msg) - k, ana_data[0][i], ana_data[1][i]);
    echo(msg);
  }
  // N samples = N-1 intervals
  snprintf(msg, sizeof(msg), "# expected burst [ms]: %lu.%03lu\n",
           (unsigned long)((A_SIZE - 1) * SAMPLE_INTERVAL_US / 1000),
           (unsigned long)((A_SIZE - 1) * SAMPLE_INTERVAL_US % 1000));              echo(msg);
  uint32_t b_us = cyclesToUs(burst_cycles);
  snprintf(msg, sizeof(msg), "# actual   burst [ms]: %lu.%03lu\n",
           (unsigned long)(b_us / 1000), (unsigned long)(b_us % 1000));             echo(msg);
  snprintf(msg, sizeof(msg), "# max dt        [us]: %lu\n", (unsigned long)dt_max); echo(msg);
  reportADCErrors();
}

// ---------------------------------------------------------------- common
void reportADCErrors() {
  if (adc->adc0->fail_flag != ADC_ERROR::CLEAR || adc->adc1->fail_flag != ADC_ERROR::CLEAR) {
    echo("# ADC0 error: "); echo(getStringADCError(adc->adc0->fail_flag));
    echo("\n# ADC1 error: "); echo(getStringADCError(adc->adc1->fail_flag)); echo("\n");
    adc->resetError();
  }
}

// counts -> millivolts, rounded
int32_t countsToMv(uint16_t c) {
  return (int32_t)(((uint32_t)c * V_REF_MV + ADC_MAX / 2) / ADC_MAX);
}

// millivolts -> "[-]V.mmm"; returns chars written
int fmtMv(char *buf, size_t n, int32_t mv) {
  uint32_t a = (mv < 0) ? -mv : mv;
  return snprintf(buf, n, "%s%lu.%03lu", (mv < 0) ? "-" : "", (unsigned long)(a / 1000), (unsigned long)(a % 1000));
}

void formatV(char *buf, size_t n, uint16_t c1, uint16_t c2) {
  // counts -> volts, CSV, 3 decimals (1 count = 0.8 mV); integer math, no float printf
  bool ok = (c1 != ADC_BAD && c2 != ADC_BAD);
  if (!ok) {
    snprintf(buf, n, PRINT_DIFF ? "nan,nan,nan\n" : "nan,nan\n");
    return;
  }
  int32_t m1 = countsToMv(c1), m2 = countsToMv(c2);
  int k = fmtMv(buf, n, m1);
  k += snprintf(buf + k, n - k, ",");
  k += fmtMv(buf + k, n - k, m2);
  if (PRINT_DIFF) {
    k += snprintf(buf + k, n - k, ",");
    k += fmtMv(buf + k, n - k, m1 - m2);
  }
  snprintf(buf + k, n - k, "\n");
}

void printHeader() {
  snprintf(msg, sizeof(msg), "# Version:         %s\n", VERSION);                     echo(msg);
  echo(capture_mode == CM_CONTINUOUS ? "# Capture mode:    CONTINUOUS\n" : "# Capture mode:    BURST\n");
  snprintf(msg, sizeof(msg), "# Sample rate:     %lu.%03lu kHz  (%lu us)\n",
           (unsigned long)(1000 / SAMPLE_INTERVAL_US),
           (unsigned long)((1000000 / SAMPLE_INTERVAL_US) % 1000),
           (unsigned long)SAMPLE_INTERVAL_US);                                        echo(msg);
  if (capture_mode == CM_BURST) {
    snprintf(msg, sizeof(msg), "# Burst interval:  %7lu ms\n", (unsigned long)BURST_INTERVAL_MS); echo(msg);
    snprintf(msg, sizeof(msg), "# Samples/burst:   %7lu\n",    (unsigned long)A_SIZE);            echo(msg);
    snprintf(msg, sizeof(msg), "# Memory:          %7lu bytes\n", (unsigned long)MEM);            echo(msg);
  }
  snprintf(msg, sizeof(msg), "# ADC bits / avg:  %4u / %u\n", adc->adc0->getResolution(), ADC_AVERAGING);  echo(msg);
  snprintf(msg, sizeof(msg), "# HW baud:         %7lu\n", (unsigned long)HW_BAUDRATE);  echo(msg);
}

void initADC() {
  // both ADCs configured identically so V1 and V2 match
  ADC_Module *mods[2] = { adc->adc0, adc->adc1 };
  for (ADC_Module *m : mods) {
    m->setAveraging(ADC_AVERAGING);
    m->setResolution(12);             // T4 max; values 0..ADC_MAX in uint16_t
    m->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED);
    m->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED);
  }
}

void initSerialPorts() {
  HWSERIAL.begin(HW_BAUDRATE);
  HWSERIAL.addMemoryForWrite(tx_buf1, sizeof(tx_buf1));
  const uint16_t TIMEOUT_INTERVAL = 5000;  // [ms]
  uint32_t t_start = millis();
  Serial.begin(115200);               // USB: baud ignored, runs at 480 Mbit/s
  while (!Serial && (millis() - t_start < TIMEOUT_INTERVAL)) {
    ;  // wait for a USB host program to open the port, or skip after timeout
  }
}

void echo(const char *s) {
  // print to ports selected by global flags
  if (HWSER1_FLAG)  HWSERIAL.print(s);
  if (CONSOLE_FLAG) Serial.print(s);
}

// EOF
