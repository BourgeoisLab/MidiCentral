#include <MIDI.h>
#include <Encoder.h>
#include <TimerOne.h>
#include <Adafruit_LEDBackpack.h>

// TODO:
// option for active-sensing (send at <300ms)
// select clock source

// pin definitions
#define PIN_BTN_START       11
#define PIN_BTN_TAP         12
#define PIN_ENCODER_A       21
#define PIN_ENCODER_B       20
#define PIN_ENCODER_BTN     6
#define PIN_CLK_IN          2   // not used yet
#define PIN_CLK_OUT         3
#define PIN_STST_IN         4
#define PIN_STST_OUT        5
#define PIN_LED_STARTED     22
#define PIN_LED_STOPPED     23

// I2C address of 7-segment display
#define DISP_I2C_ADDR       0x70

// MIDI definitions
#define CLOCKS_PER_BEAT     24
#define CLOCK_FROM_BPM(b)   (60000000UL/(CLOCKS_PER_BEAT*b))
#define BPM_FROM_US(b)      (60000000UL/(b))

// others
#define MINIMUM_US          BPM_FROM_US(MAXIMUM_BPM)
#define MAXIMUM_US          BPM_FROM_US(MINIMUM_BPM)

// minimal/maximal BPM
#define MINIMUM_BPM         30
#define MAXIMUM_BPM         300

// debounce time for start/stop button
#define BTN_START_DEBOUNCE  200

// global variables
static Adafruit_7segment disp;
static Encoder encoder(PIN_ENCODER_A, PIN_ENCODER_B);
static MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, midiA);
static MIDI_CREATE_INSTANCE(HardwareSerial, Serial2, midiB);
static volatile uint16_t bpm = 120;
static volatile bool started = false;
static volatile uint8_t clockCount = 0;
static volatile uint32_t btnStartPressedTime = 0;

static void sendMidi(midi::MidiType type, midi::DataByte data1 = 0, midi::DataByte data2 = 0, midi::Channel channel = 0) {
  if (type <= midi::MidiType::PitchBend) {
    midiA.send(type, data1, data2, channel);
    midiB.send(type, data1, data2, channel);
    usbMIDI.send(type, data1, data2, channel, 0);
  }
  else {
    midiA.sendRealTime(type);
    midiB.sendRealTime(type);
    usbMIDI.sendRealTime(type);
  }
}

void interruptSendMidiClock() {
  if (started) {
    if (clockCount == 0) {
      digitalWrite(PIN_CLK_OUT, LOW);
      digitalWrite(PIN_LED_STARTED, HIGH);
    } else if (clockCount == CLOCKS_PER_BEAT/2) {
      digitalWrite(PIN_CLK_OUT, HIGH);
      digitalWrite(PIN_LED_STARTED, LOW);
    }
    sendMidi(midi::MidiType::Clock);
  } else {
    if (clockCount == 0) {
      digitalWrite(PIN_LED_STOPPED, HIGH);
    } else if (clockCount == CLOCKS_PER_BEAT/2) {
      digitalWrite(PIN_LED_STOPPED, LOW);
    }
  }
  clockCount = (clockCount + 1) % CLOCKS_PER_BEAT;
}

static void start(midi::MidiType type) {
  if (!started) {
    noInterrupts();
    started = true;
    clockCount = 0;
    digitalWrite(PIN_LED_STOPPED, LOW);
    digitalWrite(PIN_STST_OUT, LOW);
    digitalWrite(PIN_CLK_OUT, HIGH);
    sendMidi(type);
    interrupts();
  }
}

static void stop() {
  if (started) {
    noInterrupts();
    started = false;
    digitalWrite(PIN_LED_STARTED, LOW);
    digitalWrite(PIN_STST_OUT, HIGH);
    digitalWrite(PIN_CLK_OUT, HIGH);
    sendMidi(midi::MidiType::Stop);
    interrupts();
  }
}

static void interruptStartButton() {
  if (!digitalRead(PIN_BTN_START)) {
    static uint32_t lastTime = 0;
    uint32_t now = millis();
    if (now - lastTime < BTN_START_DEBOUNCE)
      return;
    if (started)
      stop();
    else
      start(midi::MidiType::Start);
    lastTime = now;
    btnStartPressedTime = now;
  } else{
    btnStartPressedTime = 0;
  }
}

static void checkSystemReset() {
  uint32_t now = millis();
  if (btnStartPressedTime > 0 && now - btnStartPressedTime > 2000) {
    btnStartPressedTime = 0;
    disp.printError();
    disp.writeDisplay();
    stop();
    sendMidi(midi::MidiType::SystemReset);
    delay(500);
    disp.print(bpm, DEC);
    disp.writeDisplay();
  }
}

static void interruptStartExtern() {
  if (digitalRead(PIN_STST_IN))
    stop();
  else
    start(midi::MidiType::Start);
}

static void interruptTapButton() {
  static uint32_t tapInterval[2];
  static uint32_t lastTime = 0;
  noInterrupts();
  uint32_t now = micros();
  uint32_t dt = now - lastTime;
  if (dt < MINIMUM_US || dt > MAXIMUM_US) {
    tapInterval[0] = 0;
    tapInterval[1] = 0;
    lastTime = now;
  } else {
    tapInterval[1] = tapInterval[0];
    tapInterval[0] = dt;
    lastTime = now;
    if (tapInterval[0] != 0 && tapInterval[1] != 0) {
      uint32_t interval = (tapInterval[0] + tapInterval[1]) / 2;
      bpm = BPM_FROM_US(interval);
    }
  }
  interrupts();
}

static void interruptEncoderButton() {
  static uint32_t lastTime = 0;
  uint32_t now = millis();
  if (now - lastTime < BTN_START_DEBOUNCE)
    return;
  if (started)
    stop();
  else
    start(midi::MidiType::Continue);
  lastTime = now;
}

static inline void readEncoder() {
  static int32_t lastEncoderValue = 0;
  int32_t encoderValue = encoder.read();
  if (encoderValue - lastEncoderValue <= -4) {
    bpm--;
    lastEncoderValue = encoderValue;
  } else if (encoderValue - lastEncoderValue >= 4) {
    bpm++;
    lastEncoderValue = encoderValue;
  }
}

static inline void updateBpm() {
  static int lastBpm = 0;
  bpm = constrain(bpm, MINIMUM_BPM, MAXIMUM_BPM);
  if (bpm != lastBpm) {
    Timer1.setPeriod(CLOCK_FROM_BPM(bpm));
    disp.print(bpm, DEC);
    disp.writeDisplay();
    lastBpm = bpm;
  }
}

static inline bool filterMidiInput(midi::MidiType type) {
  switch (type) {
    case midi::MidiType::Start:
    case midi::MidiType::Continue:
      start(type);
      return false;
    case midi::MidiType::Stop:
      stop();
      return false;
    case midi::MidiType::Clock:
      return false;
    default:
      return true;
  }
}

static inline void forwardMidi() {
  if (midiA.read()) {
    midi::MidiType type = midiA.getType();
    if (filterMidiInput(type))
      sendMidi(type, midiA.getData1(), midiA.getData2(), midiA.getChannel());
  }
  if (midiB.read()) {
    midi::MidiType type = midiB.getType();
    if (filterMidiInput(type))
      sendMidi(type, midiB.getData1(), midiB.getData2(), midiB.getChannel());
  }
  if (usbMIDI.read()) {
    midi::MidiType type = (midi::MidiType)usbMIDI.getType();
    if (filterMidiInput(type))
      sendMidi(type, usbMIDI.getData1(), usbMIDI.getData2(), usbMIDI.getChannel());
  }
}

void setup() {
  // configure pins
  pinMode(PIN_BTN_START, INPUT_PULLUP);
  pinMode(PIN_BTN_TAP, INPUT_PULLUP);
  pinMode(PIN_ENCODER_BTN, INPUT_PULLUP);
  pinMode(PIN_CLK_IN, INPUT);
  pinMode(PIN_CLK_OUT, OUTPUT);
  pinMode(PIN_STST_IN, INPUT);
  pinMode(PIN_STST_OUT, OUTPUT);
  pinMode(PIN_LED_STARTED, OUTPUT);
  pinMode(PIN_LED_STOPPED, OUTPUT);

  // attach interrupts
  attachInterrupt(digitalPinToInterrupt(PIN_BTN_START), interruptStartButton, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_BTN_TAP), interruptTapButton, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_BTN), interruptEncoderButton, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_STST_IN), interruptStartExtern, CHANGE);

  // initialize display
  disp.begin(DISP_I2C_ADDR);

  // initialize MIDI A
  midiA.begin(MIDI_CHANNEL_OMNI);
  midiA.turnThruOff();

  // initialize MIDI B
  midiB.begin(MIDI_CHANNEL_OMNI);
  midiB.turnThruOff();

  // initialize timer to send the MIDI clock
  Timer1.initialize(CLOCK_FROM_BPM(bpm));
  Timer1.attachInterrupt(interruptSendMidiClock);
}

void loop() {
  // check for system reset
  checkSystemReset();
    
  // read encoder
  readEncoder();

  // update bpm
  updateBpm();

  // forward MIDI messages
  forwardMidi();
}
