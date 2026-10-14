/*
  HEXAGONE GAME - MASTER firmware (ESP32)
  ---------------------------------------
  Board : any ESP32 dev board (works with ESP32 Arduino core 2.x and 3.x)
  Libs  : FastLED (for the WS2812 7-segment scoreboard on GPIO27)

  - Discovers nodes automatically (they send HELLO every second) and gives each a slot.
  - START button (BOOT button on GPIO0 by default) starts the game. Press again during
    the game to abort.
  - Runs the game: random red targets (player 1) / blue targets (player 2), decoys and
    "confuse" phases. Nodes report hits; master keeps the score.
  - Scores are shown on the WS2812 7-segment scoreboard and printed on Serial (115200).

  ALL SETTINGS LIVE HERE (game timing, hit sensitivity, beeps, idle patterns, decoys ...).
  Change them on the Serial Monitor with  set <name> <value>  -> saved in flash and sent
  to every node within ~1 s. Nodes never need re-uploading for a settings change.
  Type "config" to see every setting with its value, range and meaning.

  Wall positions (optional, idle only): type "learn" (or hold START 3 s), then hit every
  connected hexagon with a ball in reading order; it finishes by itself when all connected
  nodes are done. Waiting hexagons blink white, learned ones turn green. "forget" clears it.

        left   middle   right
        [ 1]            [ 2]
               [ 3]
        [ 4]            [ 5]
               [ 6]
         ...    ...      ...

  Serial commands:
    start | stop                     start / abort a game
    config                           list all settings
    set <name> <value>               change a setting (hex 0x.. and binary 0b.. allowed)
    defaults                         all settings back to default
    patterns                         list idle patterns and decoys with their numbers
    wall <n> on|off                  enable / disable wall idle pattern n
    free <n> on|off                  enable / disable free idle pattern n
    decoy <n> on|off                 enable / disable in-game decoy n
    nodes | learn | forget           node list, wall position learning
    time <s> | thr <n> | bri <n>     shortcuts for game_time / hit_thr / led_bri
*/

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <FastLED.h>
#include <Preferences.h>
#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif

// ============================ HARDWARE CONFIG ============================
#define ESPNOW_CHANNEL        1        // must match nodes
#define START_BUTTON_PIN      0        // BOOT button, active LOW
#define STATUS_LED_PIN        2        // on-board LED (-1 to disable)

// WS2812 7-segment scoreboard: 3 displays x 3 digits x 7 LEDs (1 LED per segment)
//   LEDs  0-20 : player 1 score (red)    LEDs 21-41 : player 2 score (blue)
//   LEDs 42-62 : seconds left (green)
//   Inside each display: hundreds digit first, then tens, then ones. Segment order a,b,c,d,e,f,g.
#define SCORE_LED_PIN         27
#define SCORE_DIGIT_LEDS      7
#define SCORE_DISPLAY_LEDS    (3 * SCORE_DIGIT_LEDS)
#define NUM_SCORE_LEDS        (3 * SCORE_DISPLAY_LEDS)

#define NODE_TIMEOUT_MS       4000     // node considered offline after this
#define LEARN_HOLD_MS         3000     // hold START this long to enter learn mode

// ============================ PROTOCOL (keep identical in node) ============================
#define HEX_MAGIC  0xA7
#define MAX_NODES  20

enum MsgType : uint8_t {
  MSG_HELLO = 1, MSG_WELCOME = 2, MSG_IDLE = 3, MSG_START = 4,
  MSG_ROUND = 5, MSG_END = 6, MSG_HIT = 7
};
enum Role : uint8_t { ROLE_NONE = 0, ROLE_P1 = 1, ROLE_P2 = 2 };
enum Seg : uint8_t { SEG_OFF = 0, SEG_RED = 1, SEG_BLUE = 2, SEG_BOTH = 3 };
enum CColor : uint8_t { C_OFF, C_RED, C_BLUE, C_GREEN, C_PURPLE, C_YELLOW, C_CYAN, C_WHITE };
// look byte: bits0-1 outer ring, bits2-3 middle ring, bits4-6 center color, bit7 blink
#define MAKE_LOOK(outer, middle, center, blink) \
  (uint8_t)(((outer) & 3) | (((middle) & 3) << 2) | (((center) & 7) << 4) | ((blink) ? 0x80 : 0))

// Every node setting, sent by the master inside each WELCOME (heartbeat)
struct __attribute__((packed)) NodeConfig {
  int32_t  hitThreshold;   // load cell counts above baseline = hit
  uint16_t hitLockoutMs;   // ignore load cell after a hit
  uint8_t  hitRearmPct;    // re-arm when signal falls below this % of threshold
  uint8_t  baselineShift;  // baseline drift follow speed (EMA 1/2^n)
  uint16_t hitStuckMs;     // re-learn baseline if signal stays high this long
  uint8_t  brightness;     // center LED brightness
  uint8_t  buzzer;         // 0 = silent
  uint16_t hitBeepMs;
  uint16_t wrongBeepMs;
  uint16_t startBeepMs;
  uint16_t endBeepMs;
  uint16_t hitFlashMs;     // center green after correct hit
  uint16_t endDarkMs;      // game over: all dark this long
  uint16_t winnerBlinkMs;  // game over: outer ring blink speed
  uint16_t idleSlotMs;     // learned wall: each pattern runs this long
  uint16_t freeIdleMinMs;  // not learned: each node changes pattern after min..max
  uint16_t freeIdleMaxMs;
  uint16_t idleWallMask;   // enabled wall patterns (bit per pattern)
  uint8_t  idleFreeMask;   // enabled free patterns (bit per pattern)
  uint8_t  idleSpeedPct;   // animation speed, 100 = normal
};

struct __attribute__((packed)) MsgHeader  { uint8_t magic; uint8_t type; uint16_t seq; };
struct __attribute__((packed)) HelloMsg   { MsgHeader h; uint8_t slot; uint8_t state; uint8_t loadCellOk; int32_t loadRaw; };
struct __attribute__((packed)) WelcomeMsg { MsgHeader h; uint8_t mac[6]; uint8_t slot; uint32_t masterMs; uint8_t position; uint8_t flags; uint8_t wallCount; NodeConfig cfg; };
#define WFLAG_LEARN  0x01  // master is in position-learn mode
struct __attribute__((packed)) StartMsg   { MsgHeader h; uint16_t greenMs; uint16_t darkMs; };
struct __attribute__((packed)) RoundMsg   { MsgHeader h; uint16_t roundId; uint16_t durationMs; uint8_t look[MAX_NODES]; uint8_t role[MAX_NODES]; };
struct __attribute__((packed)) EndMsg     { MsgHeader h; uint8_t winner; uint16_t showMs; uint16_t score1; uint16_t score2; };
struct __attribute__((packed)) HitMsg     { MsgHeader h; uint8_t slot; uint16_t roundId; uint8_t role; int32_t strength; };
// ===========================================================================================

// ============================ SETTINGS ============================
// Every value here can be changed at runtime with "set <name> <value>" and is saved in flash.
struct Settings {
  // game (master)
  int32_t gameTime, startGreen, startDark, targets, targetMin, targetMax, gap,
          decoyPct, decoyMask, confusePct, confuseMin, confuseMax, confuseDecoy,
          endDark, endShow, scoreBri;
  // hit detection (nodes)
  int32_t hitThr, hitLockout, hitRearm, hitStuck, baseShift;
  // feedback (nodes)
  int32_t ledBri, buzzer, hitBeep, wrongBeep, startBeep, endBeep, hitFlash, winBlink;
  // idle (nodes)
  int32_t idleSlot, freeMin, freeMax, wallMask, freeMask, idleSpeed;
};
Settings S;

struct SettingDef {
  const char *name;   // also the flash key (max 15 chars)
  int32_t *val;       // nullptr = section title
  int32_t def, lo, hi;
  bool hex;           // print as hex (bit masks)
  const char *help;
};

const SettingDef SETTINGS[] = {
  {"GAME",          nullptr,         0, 0, 0, false, ""},
  {"game_time",     &S.gameTime,     60,   5,   999,     false, "game length (seconds)"},
  {"start_green",   &S.startGreen,   1000, 0,   10000,   false, "start: centers green (ms)"},
  {"start_dark",    &S.startDark,    2000, 0,   10000,   false, "start: all dark before first target (ms)"},
  {"targets",       &S.targets,      5,    1,   20,      false, "max targets per player per round"},
  {"target_min",    &S.targetMin,    800,  100, 10000,   false, "target lit time, min (ms)"},
  {"target_max",    &S.targetMax,    1000, 100, 10000,   false, "target lit time, max (ms)"},
  {"gap",           &S.gap,          200,  0,   5000,    false, "all dark between rounds (ms)"},
  {"decoy_pct",     &S.decoyPct,     50,   0,   100,     false, "% of free nodes showing a decoy in a target round"},
  {"decoys",        &S.decoyMask,    0x3,  0,   0x7FF,   true,  "enabled decoys, bit mask (see 'patterns')"},
  {"confuse_pct",   &S.confusePct,   40,   0,   100,     false, "% chance of a confuse round after a target round"},
  {"confuse_min",   &S.confuseMin,   500,  100, 10000,   false, "confuse round time, min (ms)"},
  {"confuse_max",   &S.confuseMax,   1200, 100, 10000,   false, "confuse round time, max (ms)"},
  {"confuse_decoy", &S.confuseDecoy, 75,   0,   100,     false, "% of nodes showing a decoy in a confuse round"},
  {"end_dark",      &S.endDark,      2000, 0,   20000,   false, "game over: all dark (ms)"},
  {"end_show",      &S.endShow,      6000, 0,   60000,   false, "game over: winner outer ring blinks (ms)"},
  {"score_bri",     &S.scoreBri,     255,  0,   255,     false, "scoreboard brightness"},
  {"HIT DETECTION", nullptr,         0, 0, 0, false, ""},
  {"hit_thr",       &S.hitThr,       800,  1,   1000000, false, "load cell counts for a hit (lower = more sensitive)"},
  {"hit_lockout",   &S.hitLockout,   250,  0,   5000,    false, "ignore load cell after a hit (ms)"},
  {"hit_rearm",     &S.hitRearm,     50,   5,   100,     false, "re-arm below this % of hit_thr"},
  {"hit_stuck",     &S.hitStuck,     2000, 200, 60000,   false, "re-zero if signal stays high this long (ms)"},
  {"baseline_shift",&S.baseShift,    4,    1,   8,       false, "drift tracking: 1 fast .. 8 slow"},
  {"NODE FEEDBACK", nullptr,         0, 0, 0, false, ""},
  {"led_bri",       &S.ledBri,       150,  1,   255,     false, "center LED brightness"},
  {"buzzer",        &S.buzzer,       1,    0,   1,       false, "1 = on, 0 = silent"},
  {"hit_beep",      &S.hitBeep,      80,   0,   5000,    false, "beep on correct hit (ms)"},
  {"wrong_beep",    &S.wrongBeep,    300,  0,   5000,    false, "beep on decoy hit (ms)"},
  {"start_beep",    &S.startBeep,    150,  0,   5000,    false, "beep at game start (ms)"},
  {"end_beep",      &S.endBeep,      500,  0,   5000,    false, "beep at game over (ms)"},
  {"hit_flash",     &S.hitFlash,     200,  0,   5000,    false, "center green after correct hit (ms)"},
  {"win_blink",     &S.winBlink,     300,  50,  5000,    false, "game over outer ring blink speed (ms)"},
  {"IDLE",          nullptr,         0, 0, 0, false, ""},
  {"idle_slot",     &S.idleSlot,     8000, 1000, 60000,  false, "learned wall: time per pattern (ms)"},
  {"free_min",      &S.freeMin,      6000, 1000, 60000,  false, "not learned: pattern change after min (ms)"},
  {"free_max",      &S.freeMax,      12000,1000, 60000,  false, "not learned: pattern change after max (ms)"},
  {"wall_mask",     &S.wallMask,     0x3FF,0,    0x3FF,  true,  "enabled wall patterns, bit mask (see 'patterns')"},
  {"free_mask",     &S.freeMask,     0x3F, 0,    0x3F,   true,  "enabled free patterns, bit mask (see 'patterns')"},
  {"idle_speed",    &S.idleSpeed,    100,  10,   400,    false, "idle animation speed % (100 = normal)"},
};
#define NUM_SETTINGS (sizeof(SETTINGS) / sizeof(SETTINGS[0]))

const char *WALL_NAMES[10] = {"look cycle (whole wall)", "looks flowing down", "column sweep", "checkerboard",
                              "rain", "color sweep down", "breathing", "ripple from middle", "heartbeat", "strobe"};
const char *FREE_NAMES[6]  = {"look cycle", "color sweep", "breathing", "ripple", "heartbeat", "strobe"};

const uint8_t LOOK_P1 = MAKE_LOOK(SEG_RED,  SEG_RED,  C_RED,  0);   // all red  -> player 1
const uint8_t LOOK_P2 = MAKE_LOOK(SEG_BLUE, SEG_BLUE, C_BLUE, 0);   // all blue -> player 2

// In-game decoys (outer, middle, center). None of them is a valid target.
// Enable / disable with "decoy <n> on|off" (default: 0 and 1).
const uint8_t DECOY_LOOKS[] = {
  MAKE_LOOK(SEG_RED,  SEG_BLUE, C_RED,    0),
  MAKE_LOOK(SEG_BLUE, SEG_RED,  C_BLUE,   0),
  MAKE_LOOK(SEG_RED,  SEG_RED,  C_BLUE,   0),
  MAKE_LOOK(SEG_BLUE, SEG_BLUE, C_RED,    0),
  MAKE_LOOK(SEG_RED,  SEG_BLUE, C_BLUE,   0),
  MAKE_LOOK(SEG_BLUE, SEG_RED,  C_RED,    0),
  MAKE_LOOK(SEG_RED,  SEG_OFF,  C_RED,    0),
  MAKE_LOOK(SEG_BLUE, SEG_OFF,  C_BLUE,   0),
  MAKE_LOOK(SEG_BOTH, SEG_BOTH, C_PURPLE, 0),
  MAKE_LOOK(SEG_RED,  SEG_RED,  C_RED,    1),
  MAKE_LOOK(SEG_BLUE, SEG_BLUE, C_BLUE,   1),
};
const char *DECOY_NAMES[] = {
  "red-blue-red", "blue-red-blue", "red-red-blue", "blue-blue-red", "red-blue-blue", "blue-red-red",
  "red-off-red", "blue-off-blue", "purple (both colors)", "all red BLINKING", "all blue BLINKING",
};
#define NUM_DECOYS (sizeof(DECOY_LOOKS) / sizeof(DECOY_LOOKS[0]))

const uint8_t BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

struct NodeInfo {
  bool used;
  uint8_t mac[6];
  uint32_t lastSeen;
  uint8_t state;
  bool loadCellOk;
  int32_t loadRaw;
  uint16_t lastScoredRound;
  bool scoredValid;
};
NodeInfo nodes[MAX_NODES];

enum GameState { G_IDLE, G_COUNTDOWN, G_RUNNING, G_ENDED };
enum Phase { PH_TARGET, PH_CONFUSE, PH_GAP };

GameState gState = G_IDLE;
Phase phase = PH_GAP;
bool lastPhaseWasTarget = false;
uint32_t countdownAt = 0, runAt = 0, endAt = 0, phaseEndAt = 0, endedAt = 0;
uint32_t endWaitMs = 0;  // game over show length, fixed when the game ends
uint16_t score1 = 0, score2 = 0, wrongHits = 0;
uint16_t roundId = 0;
uint16_t txSeq = 0;

// wall position -> node MAC (index 1..N, all zero = not learned), saved in flash
Preferences prefs;
uint8_t wallMac[MAX_NODES + 1][6];
bool learnMode = false;
uint8_t learnNext = 1;

// ---------------- RX queue (ESP-NOW callback runs in WiFi task) ----------------
struct RxPacket { uint8_t mac[6]; uint8_t len; uint8_t data[64]; };
QueueHandle_t rxQueue;

static void queueRx(const uint8_t *mac, const uint8_t *data, int len) {
  if (len <= 0 || len > (int)sizeof(RxPacket::data)) return;
  RxPacket p;
  memcpy(p.mac, mac, 6);
  memcpy(p.data, data, len);
  p.len = len;
  xQueueSend(rxQueue, &p, 0);
}

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
void onEspNowRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) { queueRx(info->src_addr, data, len); }
#else
void onEspNowRecv(const uint8_t *mac, const uint8_t *data, int len) { queueRx(mac, data, len); }
#endif

// random value in [a, b], works if a > b too
int32_t rndRange(int32_t a, int32_t b) {
  if (a > b) { int32_t t = a; a = b; b = t; }
  return random(a, b + 1);
}

// ============================ SETTINGS STORAGE ============================
int32_t clampSetting(const SettingDef &d, int32_t v) {
  return v < d.lo ? d.lo : v > d.hi ? d.hi : v;
}

void loadSettings() {
  prefs.begin("hexcfg", true);
  for (uint8_t i = 0; i < NUM_SETTINGS; i++) {
    const SettingDef &d = SETTINGS[i];
    if (d.val) *d.val = clampSetting(d, prefs.getInt(d.name, d.def));  // missing key -> default
  }
  prefs.end();
}

void saveSetting(const SettingDef &d) {
  prefs.begin("hexcfg", false);
  prefs.putInt(d.name, *d.val);
  prefs.end();
}

void resetSettings() {
  prefs.begin("hexcfg", false);
  prefs.clear();
  prefs.end();
  for (uint8_t i = 0; i < NUM_SETTINGS; i++)
    if (SETTINGS[i].val) *SETTINGS[i].val = SETTINGS[i].def;
}

int findSetting(const String &name) {
  for (uint8_t i = 0; i < NUM_SETTINGS; i++)
    if (SETTINGS[i].val && name.equalsIgnoreCase(SETTINGS[i].name)) return i;
  return -1;
}

void printSettings() {
  for (uint8_t i = 0; i < NUM_SETTINGS; i++) {
    const SettingDef &d = SETTINGS[i];
    if (!d.val) { Serial.printf("\n[%s]\n", d.name); continue; }
    char val[16], range[28];
    if (d.hex) {
      snprintf(val, sizeof(val), "0x%lX", (unsigned long)*d.val);
      snprintf(range, sizeof(range), "(0x%lX..0x%lX)", (unsigned long)d.lo, (unsigned long)d.hi);
    } else {
      snprintf(val, sizeof(val), "%ld", (long)*d.val);
      snprintf(range, sizeof(range), "(%ld..%ld)", (long)d.lo, (long)d.hi);
    }
    Serial.printf("  %-15s %-8s %-18s %s\n", d.name, val, range, d.help);
  }
  Serial.println();
}

void printPatterns() {
  Serial.println("\nWall idle patterns (after 'learn')   -> wall <n> on|off");
  for (uint8_t i = 0; i < 10; i++)
    Serial.printf("  %u  [%s]  %s\n", i, (S.wallMask >> i) & 1 ? "on " : "off", WALL_NAMES[i]);
  Serial.println("Free idle patterns (before 'learn')  -> free <n> on|off");
  for (uint8_t i = 0; i < 6; i++)
    Serial.printf("  %u  [%s]  %s\n", i, (S.freeMask >> i) & 1 ? "on " : "off", FREE_NAMES[i]);
  Serial.println("In-game decoys (outer-middle-center) -> decoy <n> on|off");
  for (uint8_t i = 0; i < NUM_DECOYS; i++)
    Serial.printf("  %2u  [%s]  %s\n", i, (S.decoyMask >> i) & 1 ? "on " : "off", DECOY_NAMES[i]);
  Serial.println();
}

// ============================ WALL POSITIONS ============================
uint8_t positionOf(const uint8_t *mac) {
  static const uint8_t ZERO[6] = {0};
  if (memcmp(mac, ZERO, 6) == 0) return 0;
  for (uint8_t p = 1; p <= MAX_NODES; p++)
    if (memcmp(wallMac[p], mac, 6) == 0) return p;
  return 0;
}

void loadPositions() {
  prefs.begin("hexagone", true);
  size_t n = prefs.getBytes("wall", wallMac, sizeof(wallMac));
  prefs.end();
  if (n != sizeof(wallMac)) memset(wallMac, 0, sizeof(wallMac));
}

void savePositions() {
  prefs.begin("hexagone", false);
  prefs.putBytes("wall", wallMac, sizeof(wallMac));
  prefs.end();
}

uint8_t learnedCount() {
  static const uint8_t ZERO[6] = {0};
  uint8_t n = 0;
  for (uint8_t p = 1; p <= MAX_NODES; p++) if (memcmp(wallMac[p], ZERO, 6) != 0) n++;
  return n;
}

// ============================ SENDING ============================
void fillHeader(MsgHeader &h, uint8_t type) {
  h.magic = HEX_MAGIC;
  h.type = type;
  h.seq = ++txSeq;
}

// Broadcast has no ACK, so important messages are sent a few times (nodes ignore copies)
void broadcast(const void *msg, size_t len, uint8_t copies) {
  for (uint8_t i = 0; i < copies; i++) {
    esp_now_send(BROADCAST_MAC, (const uint8_t *)msg, len);
    if (i + 1 < copies) delay(3);
  }
}

void sendWelcome(uint8_t slot) {
  WelcomeMsg m;
  fillHeader(m.h, MSG_WELCOME);
  memcpy(m.mac, nodes[slot].mac, 6);
  m.slot = slot;
  m.masterMs = millis();  // nodes sync their idle animation clock to this
  m.position = positionOf(nodes[slot].mac);
  m.flags = learnMode ? WFLAG_LEARN : 0;
  m.wallCount = learnedCount();

  NodeConfig &c = m.cfg;  // all node settings ride along with every heartbeat
  c.hitThreshold  = S.hitThr;
  c.hitLockoutMs  = S.hitLockout;
  c.hitRearmPct   = S.hitRearm;
  c.baselineShift = S.baseShift;
  c.hitStuckMs    = S.hitStuck;
  c.brightness    = S.ledBri;
  c.buzzer        = S.buzzer;
  c.hitBeepMs     = S.hitBeep;
  c.wrongBeepMs   = S.wrongBeep;
  c.startBeepMs   = S.startBeep;
  c.endBeepMs     = S.endBeep;
  c.hitFlashMs    = S.hitFlash;
  c.endDarkMs     = S.endDark;
  c.winnerBlinkMs = S.winBlink;
  c.idleSlotMs    = S.idleSlot;
  c.freeIdleMinMs = S.freeMin;
  c.freeIdleMaxMs = S.freeMax;
  c.idleWallMask  = S.wallMask;
  c.idleFreeMask  = S.freeMask;
  c.idleSpeedPct  = S.idleSpeed;
  broadcast(&m, sizeof(m), 1);
}

void sendConfigToAll() {
  for (uint8_t i = 0; i < MAX_NODES; i++)
    if (nodes[i].used) { sendWelcome(i); delay(3); }
}

void sendIdle() {
  MsgHeader m;
  fillHeader(m, MSG_IDLE);
  broadcast(&m, sizeof(m), 3);
}

// ============================ NODES ============================
bool isOnline(uint8_t i, uint32_t now) {
  return nodes[i].used && (now - nodes[i].lastSeen) < NODE_TIMEOUT_MS;
}

uint8_t onlineCount() {
  uint32_t now = millis();
  uint8_t n = 0;
  for (uint8_t i = 0; i < MAX_NODES; i++) if (isOnline(i, now)) n++;
  return n;
}

int findNode(const uint8_t *mac) {
  for (uint8_t i = 0; i < MAX_NODES; i++)
    if (nodes[i].used && memcmp(nodes[i].mac, mac, 6) == 0) return i;
  return -1;
}

int allocSlot(const uint8_t *mac) {
  uint32_t now = millis();
  int slot = -1;
  for (uint8_t i = 0; i < MAX_NODES && slot < 0; i++) if (!nodes[i].used) slot = i;
  if (slot < 0) {  // table full: reuse the longest-offline slot
    uint32_t oldest = 0;
    for (uint8_t i = 0; i < MAX_NODES; i++)
      if (!isOnline(i, now) && now - nodes[i].lastSeen >= oldest) { oldest = now - nodes[i].lastSeen; slot = i; }
  }
  if (slot < 0) return -1;
  memset(&nodes[slot], 0, sizeof(NodeInfo));
  nodes[slot].used = true;
  memcpy(nodes[slot].mac, mac, 6);
  Serial.printf("New node %02X:%02X:%02X:%02X:%02X:%02X -> slot %d\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], slot);
  return slot;
}

void printNodes() {
  uint32_t now = millis();
  Serial.printf("Nodes online: %u   wall positions learned: %u\n", onlineCount(), learnedCount());
  for (uint8_t i = 0; i < MAX_NODES; i++) {
    if (!nodes[i].used) continue;
    const uint8_t *m = nodes[i].mac;
    Serial.printf("  slot %2u  wall pos %2u  %02X:%02X:%02X:%02X:%02X:%02X  %s  loadcell:%s  raw:%ld\n", i,
                  positionOf(m), m[0], m[1], m[2], m[3], m[4], m[5],
                  isOnline(i, now) ? "online " : "OFFLINE",
                  nodes[i].loadCellOk ? "ok" : "NO DATA", (long)nodes[i].loadRaw);
  }
}

// ============================ SCORE DISPLAY ============================
CRGB scoreLeds[NUM_SCORE_LEDS];

const uint8_t SEG_DIGITS[10][7] = {
  {1, 1, 1, 1, 1, 1, 0},  // 0
  {0, 1, 1, 0, 0, 0, 0},  // 1
  {1, 1, 0, 1, 1, 0, 1},  // 2
  {1, 1, 1, 1, 0, 0, 1},  // 3
  {0, 1, 1, 0, 0, 1, 1},  // 4
  {1, 0, 1, 1, 0, 1, 1},  // 5
  {1, 0, 1, 1, 1, 1, 1},  // 6
  {1, 1, 1, 0, 0, 0, 0},  // 7
  {1, 1, 1, 1, 1, 1, 1},  // 8
  {1, 1, 1, 1, 0, 1, 1},  // 9
};

// display 0 = P1, 1 = P2, 2 = timer. Leading zeros are blanked.
void drawNumber(uint8_t display, int value, CRGB color) {
  if (value < 0) value = 0;
  if (value > 999) value = 999;
  const uint8_t d[3] = {(uint8_t)(value / 100), (uint8_t)(value / 10 % 10), (uint8_t)(value % 10)};
  const bool show[3] = {value >= 100, value >= 10, true};
  CRGB *base = &scoreLeds[display * SCORE_DISPLAY_LEDS];
  for (uint8_t pos = 0; pos < 3; pos++)
    for (uint8_t k = 0; k < SCORE_DIGIT_LEDS; k++)
      base[pos * SCORE_DIGIT_LEDS + k] = (show[pos] && SEG_DIGITS[d[pos]][k]) ? color : CRGB::Black;
}

void scoreboardTask() {
  static uint32_t nextFrame = 0;
  uint32_t now = millis();
  if ((int32_t)(now - nextFrame) < 0) return;
  nextFrame = now + 33;  // ~30 fps

  switch (gState) {
    case G_COUNTDOWN: {  // running dot that changes color: red -> blue -> green
      static const CRGB cols[3] = {CRGB::Red, CRGB::Blue, CRGB::Green};
      uint32_t step = (now - countdownAt) / 15;
      fill_solid(scoreLeds, NUM_SCORE_LEDS, CRGB::Black);
      scoreLeds[step % NUM_SCORE_LEDS] = cols[(step / NUM_SCORE_LEDS) % 3];
    } break;

    case G_RUNNING: {
      int32_t left = (int32_t)(endAt - now);
      drawNumber(0, score1, CRGB::Red);
      drawNumber(1, score2, CRGB::Blue);
      drawNumber(2, left > 0 ? (left + 999) / 1000 : 0, CRGB::Green);
    } break;

    case G_ENDED: {  // final score, winner's display blinks
      bool blinkOff = (now / 400) & 1;
      uint8_t winner = score1 > score2 ? 1 : score2 > score1 ? 2 : 0;
      drawNumber(0, score1, (winner == 1 && blinkOff) ? CRGB::Black : CRGB::Red);
      drawNumber(1, score2, (winner == 2 && blinkOff) ? CRGB::Black : CRGB::Blue);
      drawNumber(2, 0, CRGB::Green);
    } break;

    default:  // idle: last scores + configured game time
      drawNumber(0, score1, CRGB::Red);
      drawNumber(1, score2, CRGB::Blue);
      drawNumber(2, S.gameTime, CRGB::Green);
      break;
  }
  FastLED.setBrightness(S.scoreBri);
  FastLED.show();
}

// Serial scoreboard (the LED scoreboard updates itself in scoreboardTask)
void showScoreboard() {
  uint32_t now = millis();
  int32_t left = (gState == G_RUNNING) ? (int32_t)(endAt - now) : 0;
  if (left < 0) left = 0;
  Serial.printf("[SCORE] P1(RED): %u   P2(BLUE): %u   time left: %lus   wrong hits: %u\n",
                score1, score2, (unsigned long)((left + 999) / 1000), wrongHits);
}

// ============================ GAME ============================
void startGame() {
  if (onlineCount() == 0) { Serial.println("No nodes connected - cannot start"); return; }
  uint32_t now = millis();
  score1 = score2 = wrongHits = 0;
  for (uint8_t i = 0; i < MAX_NODES; i++) nodes[i].scoredValid = false;

  sendConfigToAll();
  StartMsg m;
  fillHeader(m.h, MSG_START);
  m.greenMs = S.startGreen;
  m.darkMs = S.startDark;
  broadcast(&m, sizeof(m), 3);

  countdownAt = now;
  runAt = now + S.startGreen + S.startDark;
  endAt = runAt + S.gameTime * 1000UL;
  gState = G_COUNTDOWN;
  Serial.printf("\n=== GAME START (%lds, %u nodes) ===\n", (long)S.gameTime, onlineCount());
}

void endGame() {
  uint8_t winner = score1 > score2 ? 1 : score2 > score1 ? 2 : 0;
  EndMsg m;
  fillHeader(m.h, MSG_END);
  m.winner = winner;
  m.showMs = S.endShow;
  m.score1 = score1;
  m.score2 = score2;
  broadcast(&m, sizeof(m), 3);

  gState = G_ENDED;
  endedAt = millis();
  endWaitMs = S.endDark + S.endShow + 500;
  Serial.println("\n=== GAME OVER ===");
  showScoreboard();
  Serial.println(winner == 1 ? "WINNER: PLAYER 1 (RED)" : winner == 2 ? "WINNER: PLAYER 2 (BLUE)" : "DRAW");
}

void abortGame() {
  sendIdle();
  gState = G_IDLE;
  Serial.println("Game aborted");
}

uint8_t randomDecoy() {
  uint8_t n = 0;
  for (uint8_t i = 0; i < NUM_DECOYS; i++) if ((S.decoyMask >> i) & 1) n++;
  if (n == 0) return 0;  // no decoys enabled -> node stays off
  uint8_t k = random(n);
  for (uint8_t i = 0; i < NUM_DECOYS; i++)
    if ((S.decoyMask >> i) & 1) { if (k == 0) return DECOY_LOOKS[i]; k--; }
  return 0;
}

// Sends one round: target round (targets + decoys) or confuse round (decoys only)
void sendRound(bool withTargets, uint16_t durationMs) {
  uint32_t now = millis();
  RoundMsg m;
  fillHeader(m.h, MSG_ROUND);
  m.roundId = ++roundId;
  m.durationMs = durationMs;
  memset(m.look, 0, sizeof(m.look));
  memset(m.role, ROLE_NONE, sizeof(m.role));

  uint8_t list[MAX_NODES];
  uint8_t n = 0;
  for (uint8_t i = 0; i < MAX_NODES; i++) if (isOnline(i, now)) list[n++] = i;
  for (int i = n - 1; i > 0; i--) {  // shuffle
    int j = random(i + 1);
    uint8_t t = list[i]; list[i] = list[j]; list[j] = t;
  }

  uint8_t n1 = 0, n2 = 0;
  if (withTargets) {
    uint8_t maxT = S.targets;
    n1 = (n / 2 < maxT) ? n / 2 : maxT;
    n2 = n1;
    if ((n & 1) && n1 < maxT) { if (random(2)) n1++; else n2++; }  // odd count: give extra to a random player
  }

  for (uint8_t k = 0; k < n; k++) {
    uint8_t s = list[k];
    if (k < n1) {
      m.look[s] = LOOK_P1; m.role[s] = ROLE_P1;
    } else if (k < n1 + n2) {
      m.look[s] = LOOK_P2; m.role[s] = ROLE_P2;
    } else if (random(100) < (withTargets ? S.decoyPct : S.confuseDecoy)) {
      m.look[s] = randomDecoy();
    }
  }
  broadcast(&m, sizeof(m), 2);
}

void gameTask() {
  uint32_t now = millis();

  if (gState == G_COUNTDOWN && (int32_t)(now - runAt) >= 0) {
    gState = G_RUNNING;
    phase = PH_GAP;
    phaseEndAt = now;
    lastPhaseWasTarget = false;
    showScoreboard();
  }

  if (gState == G_RUNNING) {
    if ((int32_t)(now - endAt) >= 0) { endGame(); return; }
    if ((int32_t)(now - phaseEndAt) >= 0) {
      uint16_t dur;
      if (phase != PH_GAP) {
        phase = PH_GAP;
        dur = S.gap;  // nodes go dark by themselves when the previous round expires
      } else if (lastPhaseWasTarget && random(100) < S.confusePct) {
        phase = PH_CONFUSE;
        lastPhaseWasTarget = false;
        dur = rndRange(S.confuseMin, S.confuseMax);
        sendRound(false, dur);
      } else {
        phase = PH_TARGET;
        lastPhaseWasTarget = true;
        dur = rndRange(S.targetMin, S.targetMax);
        sendRound(true, dur);
      }
      phaseEndAt = now + dur;
    }
    static uint32_t nextTick = 0;
    if ((int32_t)(now - nextTick) >= 0) { nextTick = now + 5000; showScoreboard(); }
  }

  if (gState == G_ENDED && now - endedAt > endWaitMs) {
    sendIdle();
    gState = G_IDLE;
  }
}

// ============================ LEARN MODE ============================
void startLearn() {
  if (gState == G_COUNTDOWN || gState == G_RUNNING) { Serial.println("Stop the game first"); return; }
  if (gState == G_ENDED) { sendIdle(); gState = G_IDLE; }
  memset(wallMac, 0, sizeof(wallMac));
  learnMode = true;
  learnNext = 1;
  sendConfigToAll();  // nodes start blinking white
  Serial.println("\n=== LEARN WALL POSITIONS ===");
  Serial.printf("Hit the %u connected hexagons with a ball in reading order (see drawing).\n", onlineCount());
  Serial.println("Hit hexagon 1 now. Type 'learn' or short-press START to finish early.");
}

void stopLearn() {
  learnMode = false;
  savePositions();
  sendConfigToAll();  // nodes back to idle patterns, with their new positions
  Serial.printf("Learn finished: %u positions saved\n", learnedCount());
}

void learnHit(uint8_t slot) {
  uint8_t already = positionOf(nodes[slot].mac);
  if (already) { Serial.printf("That hexagon is already position %u\n", already); return; }
  memcpy(wallMac[learnNext], nodes[slot].mac, 6);
  savePositions();
  sendWelcome(slot);  // node turns green right away
  Serial.printf("Position %u learned (slot %u)\n", learnNext, slot);
  learnNext++;
  if (learnNext > MAX_NODES || learnedCount() >= onlineCount()) stopLearn();  // every connected node done
  else Serial.printf("Hit hexagon %u now\n", learnNext);
}

// ============================ PACKET HANDLING ============================
void handlePacket(const RxPacket &p) {
  if (p.len < sizeof(MsgHeader)) return;
  const MsgHeader *h = (const MsgHeader *)p.data;
  if (h->magic != HEX_MAGIC) return;
  uint32_t now = millis();

  if (h->type == MSG_HELLO && p.len >= sizeof(HelloMsg)) {
    const HelloMsg *m = (const HelloMsg *)p.data;
    int slot = findNode(p.mac);
    if (slot < 0) slot = allocSlot(p.mac);
    if (slot < 0) { Serial.println("Node table full!"); return; }
    NodeInfo &n = nodes[slot];
    n.lastSeen = now;
    n.state = m->state;
    n.loadCellOk = m->loadCellOk;
    n.loadRaw = m->loadRaw;
    sendWelcome(slot);  // reply to every HELLO: heartbeat + all settings
    return;
  }

  if (h->type == MSG_HIT && p.len >= sizeof(HitMsg)) {
    const HitMsg *m = (const HitMsg *)p.data;
    if (m->slot >= MAX_NODES || findNode(p.mac) != m->slot) return;
    NodeInfo &n = nodes[m->slot];
    n.lastSeen = now;

    if (gState != G_RUNNING) {
      if (learnMode) { learnHit(m->slot); return; }
      Serial.printf("Node %u hit (not in game), strength %ld\n", m->slot, (long)m->strength);
      return;
    }
    if (m->role == ROLE_NONE) {
      wrongHits++;
      Serial.printf("Node %u: decoy hit (strength %ld)\n", m->slot, (long)m->strength);
      return;
    }
    // accept current or just-finished round; score each node once per round
    if ((uint16_t)(roundId - m->roundId) > 1) return;
    if (n.scoredValid && n.lastScoredRound == m->roundId) return;
    n.scoredValid = true;
    n.lastScoredRound = m->roundId;

    if (m->role == ROLE_P1) score1++;
    else if (m->role == ROLE_P2) score2++;
    Serial.printf("Node %u: %s scores (strength %ld)\n", m->slot,
                  m->role == ROLE_P1 ? "P1 RED" : "P2 BLUE", (long)m->strength);
    showScoreboard();
  }
}

// ============================ INPUTS ============================
// Short press: start / abort game (or finish learn). Hold 3 s while idle: learn wall positions.
void buttonTask() {
  static bool lastStable = HIGH, lastRead = HIGH, longDone = false;
  static uint32_t changedAt = 0, pressedAt = 0;
  uint32_t now = millis();
  bool r = digitalRead(START_BUTTON_PIN);
  if (r != lastRead) { lastRead = r; changedAt = now; }
  if (now - changedAt > 30 && r != lastStable) {
    lastStable = r;
    if (r == LOW) {  // pressed
      pressedAt = now;
      longDone = false;
    } else if (!longDone) {  // released after a short press
      if (learnMode) stopLearn();
      else if (gState == G_IDLE || gState == G_ENDED) startGame();
      else abortGame();
    }
  }
  if (lastStable == LOW && !longDone && now - pressedAt > LEARN_HOLD_MS && !learnMode &&
      (gState == G_IDLE || gState == G_ENDED)) {
    longDone = true;
    startLearn();
  }
}

// accepts decimal, 0x.. hex and 0b.. binary
bool parseNumber(String s, int32_t &out) {
  s.trim();
  if (s.length() == 0) return false;
  int base = 10;
  if (s.startsWith("0x")) { base = 16; s = s.substring(2); }
  else if (s.startsWith("0b")) { base = 2; s = s.substring(2); }
  char *end;
  long v = strtol(s.c_str(), &end, base);
  if (*end != '\0') return false;
  out = v;
  return true;
}

void setSetting(const String &name, const String &value) {
  int i = findSetting(name);
  if (i < 0) { Serial.printf("Unknown setting '%s' - type 'config' for the list\n", name.c_str()); return; }
  const SettingDef &d = SETTINGS[i];
  int32_t v;
  if (!parseNumber(value, v)) { Serial.println("Bad number"); return; }
  if (v < d.lo || v > d.hi) { Serial.printf("%s must be %ld..%ld\n", d.name, (long)d.lo, (long)d.hi); return; }
  *d.val = v;
  saveSetting(d);
  sendConfigToAll();  // nodes get it now (they also get it every heartbeat)
  Serial.printf("%s = %ld (saved)\n", d.name, (long)v);
}

// "wall 3 off", "decoy 4 on" ...
void toggleBit(const char *settingName, uint8_t count, const String &args) {
  int sp = args.indexOf(' ');
  if (sp < 0) { Serial.println("Usage: <wall|free|decoy> <n> on|off"); return; }
  int n = args.substring(0, sp).toInt();
  String onOff = args.substring(sp + 1);
  onOff.trim();
  if (n < 0 || n >= count || (onOff != "on" && onOff != "off")) {
    Serial.printf("Usage: %s <0..%u> on|off\n", settingName, count - 1);
    return;
  }
  const SettingDef &d = SETTINGS[findSetting(settingName)];
  if (onOff == "on") *d.val |= (1L << n); else *d.val &= ~(1L << n);
  saveSetting(d);
  sendConfigToAll();
  printPatterns();
}

void serialTask() {
  static String line;
  while (Serial.available()) {
    char c = Serial.read();
    if (c != '\n' && c != '\r') { line += c; continue; }
    line.trim();
    if (line.length() == 0) continue;
    String cmd = line;
    line = "";
    cmd.toLowerCase();

    if (cmd == "start") {
      if (learnMode) stopLearn();
      if (gState == G_IDLE || gState == G_ENDED) startGame();
    } else if (cmd == "stop") {
      abortGame();
    } else if (cmd == "config" || cmd == "settings") {
      printSettings();
    } else if (cmd.startsWith("set ")) {
      String rest = cmd.substring(4);
      rest.trim();
      int sp = rest.indexOf(' ');
      if (sp < 0) Serial.println("Usage: set <name> <value>");
      else setSetting(rest.substring(0, sp), rest.substring(sp + 1));
    } else if (cmd == "defaults") {
      resetSettings();
      sendConfigToAll();
      Serial.println("All settings back to defaults");
    } else if (cmd == "patterns") {
      printPatterns();
    } else if (cmd.startsWith("wall ")) {
      toggleBit("wall_mask", 10, cmd.substring(5));
    } else if (cmd.startsWith("free ")) {
      toggleBit("free_mask", 6, cmd.substring(5));
    } else if (cmd.startsWith("decoy ")) {
      toggleBit("decoys", NUM_DECOYS, cmd.substring(6));
    } else if (cmd.startsWith("time ")) {
      setSetting("game_time", cmd.substring(5));
    } else if (cmd.startsWith("thr ")) {
      setSetting("hit_thr", cmd.substring(4));
    } else if (cmd.startsWith("bri ")) {
      setSetting("led_bri", cmd.substring(4));
    } else if (cmd == "nodes") {
      printNodes();
    } else if (cmd == "learn") {
      if (learnMode) stopLearn(); else startLearn();
    } else if (cmd == "forget") {  // clear wall positions -> nodes go back to random idle
      learnMode = false;
      memset(wallMac, 0, sizeof(wallMac));
      savePositions();
      sendConfigToAll();
      Serial.println("Wall positions cleared, nodes back to random idle patterns");
    } else {
      Serial.println("Commands: start | stop | config | set <name> <value> | defaults | patterns");
      Serial.println("          wall|free|decoy <n> on|off | nodes | learn | forget | time <s> | thr <n> | bri <n>");
    }
  }
}

void statusLedTask() {
  if (STATUS_LED_PIN < 0) return;
  uint32_t now = millis();
  bool on;
  if (gState == G_RUNNING || gState == G_COUNTDOWN) on = true;
  else on = onlineCount() > 0 ? ((now / 1000) & 1) : ((now / 150) & 1);  // fast blink = no nodes
  digitalWrite(STATUS_LED_PIN, on);
}

// ============================ SETUP / LOOP ============================
void setup() {
  Serial.begin(115200);
  delay(100);
  pinMode(START_BUTTON_PIN, INPUT_PULLUP);
  if (STATUS_LED_PIN >= 0) pinMode(STATUS_LED_PIN, OUTPUT);

  loadSettings();
  loadPositions();

  FastLED.addLeds<WS2812B, SCORE_LED_PIN, GRB>(scoreLeds, NUM_SCORE_LEDS);
  FastLED.setBrightness(S.scoreBri);
  FastLED.clear(true);

  rxQueue = xQueueCreate(32, sizeof(RxPacket));

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.setSleep(false);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed, restarting");
    delay(1000);
    ESP.restart();
  }
  esp_now_register_recv_cb(onEspNowRecv);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BROADCAST_MAC, 6);
  peer.channel = ESPNOW_CHANNEL;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;
  esp_now_add_peer(&peer);

  randomSeed(esp_random());

  Serial.printf("\nHexagone MASTER  MAC %s  channel %d\n", WiFi.macAddress().c_str(), ESPNOW_CHANNEL);
  Serial.printf("Wall positions learned: %u%s\n", learnedCount(),
                learnedCount() ? "" : "  (optional: 'learn' for wall-wide idle patterns)");
  Serial.println("Press BOOT button or type 'start'. 'config' lists all settings, 'help' for commands.");
}

void loop() {
  RxPacket p;
  while (xQueueReceive(rxQueue, &p, 0) == pdTRUE) handlePacket(p);

  buttonTask();
  serialTask();
  gameTask();
  scoreboardTask();
  statusLedTask();

  static uint32_t lastCount = 255, nextCheck = 0;
  if (millis() > nextCheck) {
    nextCheck = millis() + 2000;
    uint8_t c = onlineCount();
    if (c != lastCount) { lastCount = c; Serial.printf("Nodes online: %u\n", c); }
  }
  delay(1);
}
