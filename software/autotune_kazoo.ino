// BUILDING FOR SEEED STUDIO XIAO ESP-32-S3

#include <driver/i2s.h>
#include "Yin.h"
#include "SawtoothWaveGenerator.h"
#include <BLEMIDI_Transport.h>
#include <hardware/BLEMIDI_ESP32_NimBLE.h>

#define I2S_BUFFER_SIZE 128
#define PLAYBACK_SPEED_ADJUST 5
#define PLAYBACK_RATE YIN_SAMPLING_RATE *PLAYBACK_SPEED_ADJUST
#define TONE_DETECT_BUFFER_SIZE 3
#define NUM_SCALES 3
#define SCALE_SIZE 7
#define DEBOUNCE_MAX 10

BLEMIDI_CREATE_INSTANCE("Kazoo MIDI", MIDI);

/**
 * Four buffers hold copies of latest audio data, but current index is offset for each
 * Lazy approach, but we have plenty of RAM to work with
 */
uint8_t yinBuffers[4][BUFFER_SIZE];
uint16_t yindicies[4] = {0, 128, 256, 384};

const i2s_port_t I2S_PORT_IN = I2S_NUM_0;
const i2s_port_t I2S_PORT_OUT = I2S_NUM_1;
int32_t samples[I2S_BUFFER_SIZE];
int16_t samplesOut[I2S_BUFFER_SIZE];
float toneBuffer[TONE_DETECT_BUFFER_SIZE];
uint8_t toneIndex = 0;
uint32_t sampleIndex = 0;
uint16_t yindex = 0;
uint16_t offsetYindex = BUFFER_SIZE / 2;
int16_t debounceCount = 0;
float activePitch = -1;
Yin yInMethod;
SawtoothWaveGenerator *sawtooth;
int scaleMap[NUM_SCALES][SCALE_SIZE] = {
  // major
  { 0, 2, 4, 5, 7, 9, 11 },
  // minor
  { 0, 2, 3, 5, 7, 8, 10 },
  // blues
  { 0, 3, 5, 8, 10, 11, -1 }
};

bool muteSpeaker = false;
int rootNote = 6;
uint8_t scaleMapIndex = 0;
bool bleConnected = false;
int bleOutputNote = -1;
int queuedBleInputNote = -1;
int bleInputNote = -1;

bool isInScale(uint8_t scaleIndex, int rootNoteNumber, int note) {
  int noteOffset = (note - rootNoteNumber) % 12;
  for (size_t i = 0; i < SCALE_SIZE; i++) {
    int currentNote = scaleMap[scaleIndex][i];
    if (currentNote < 0) {
      return false;
    }
    if (noteOffset == currentNote) {
      return true;
    }
  }
  return false;
}

uint8_t frequencyToMIDINumber(double frequency) {
  return uint8_t(round(69 + 12 * log2(frequency / 440.0)));
}

float midiNoteToFrequency(uint8_t midiNote) {
  return 440.0 * pow(2, (midiNote - 69) / 12.0);
}

void updateFrequency(float frequency) {
  if (queuedBleInputNote != bleInputNote) {
    if (queuedBleInputNote < 0) {
      activePitch = -1;
      bleInputNote = -1;
    } else {
      activePitch = midiNoteToFrequency(queuedBleInputNote);
      sawtooth->setFrequency(activePitch);
      bleInputNote = queuedBleInputNote;
    }
  } else if (bleInputNote < 0) {
    if (frequency < 0 && activePitch >= 0) {
      Serial.println("off");
      activePitch = -1.0;
      debounceCount = DEBOUNCE_MAX / 2;
      MIDI.sendNoteOff(bleOutputNote, 0, 1);
      bleOutputNote = -1;
    } else if (frequency >= 0 && activePitch != frequency) {
      activePitch = frequency;
      sawtooth->setFrequency(frequency);
      Serial.println(frequency);
      int previousNote = bleOutputNote;
      bleOutputNote = frequencyToMIDINumber(frequency);
      if (previousNote > 0) {
        MIDI.sendNoteOff(previousNote, 0, 1);
      }
      MIDI.sendNoteOn(bleOutputNote, 120, 1);
      debounceCount = DEBOUNCE_MAX;
    }
  }
}

void runPitchDetect(uint8_t *buffer) {
  float frequency = Yin_getPitch(&yInMethod, buffer) * PLAYBACK_SPEED_ADJUST;
  if (frequency > 1500) {
    frequency = -1.0;
  }
  toneBuffer[toneIndex++] = frequency;
  if (toneIndex == TONE_DETECT_BUFFER_SIZE) {
    toneIndex = 0;
  }
  float max = -1.0;
  float min = 10000.0;
  float average = 0.0;
  uint8_t countedSamples = 0;
  for (size_t i = 0; i < TONE_DETECT_BUFFER_SIZE; i++) {
    // average last few detected pitches. 
    float current = toneBuffer[i];
    if (current > max) {
      max = current;
    }
    if (current < min) {
      min = current;
    }
    average += toneBuffer[i];
  }
  // If they are too far apart don't do anything, wait for consistent readings
  if (max - min > 90.0) {
    return;
  }
  frequency = average / TONE_DETECT_BUFFER_SIZE; 
  if (frequency < 50.0 || isnan(frequency)) {
    updateFrequency(-1.0);
  } else {
    float nearestSemitoneFromC2f = 12.0 * log2(frequency / 65.41);
    uint16_t nearestSemitoneFromC2 = round(nearestSemitoneFromC2f);
    auto nearest = pow(2, nearestSemitoneFromC2 / 12.0f) * 65.41f;
    float noteRatio = frequency / nearest;
    uint8_t midiNote = frequencyToMIDINumber(frequency);
    bool jumpDirection = noteRatio < 0.0;
    uint8_t jumpAmount = 1;
    while (!isInScale(scaleMapIndex, rootNote, midiNote)) {
      if (jumpDirection) {
        midiNote += jumpAmount++;
      } else {
        midiNote -= jumpAmount++;
      }
      jumpDirection = !jumpDirection;
    }
    updateFrequency(midiNoteToFrequency(midiNote));
  }
}

void OnConnected() {
  Serial.println("BLE connected");
  bleConnected = true;
}

void OnDisconnected() {
  Serial.println("BLE disconnected");
  bleConnected = false;
  queuedBleInputNote = -1;
  bleInputNote = -1;
  muteSpeaker = 0;
}

void onNoteOn(byte channel, byte note, byte velocity) {
  Serial.print("Note on ");
  Serial.println(note);
  queuedBleInputNote = note;
}

void onNoteOff(byte channel, byte note, byte velocity) {
  queuedBleInputNote = -1;
}

void onCC(byte channel, byte control, byte value) {
  if (control == 100) {
    muteSpeaker = value == 0;
  } else if (control == 101) {
    rootNote = value;
  } else if (control == 102) {
    if (value < NUM_SCALES) {
      scaleMapIndex = value;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  sawtooth = new SawtoothWaveGenerator(PLAYBACK_RATE);
  sawtooth->setFrequency(220);
  Yin_init(&yInMethod, 0.2f);

  BLEMIDI.setHandleConnected(OnConnected);
  BLEMIDI.setHandleDisconnected(OnDisconnected);
  MIDI.setHandleNoteOn(onNoteOn);
  MIDI.setHandleNoteOff(onNoteOff);
  MIDI.setHandleControlChange(onCC);
  MIDI.begin();

  Serial.println("Configuring I2S...");
  esp_err_t err;

  const i2s_config_t i2s_config_in = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = PLAYBACK_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = I2S_BUFFER_SIZE
  };

  const i2s_config_t i2s_config_out = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = PLAYBACK_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = I2S_BUFFER_SIZE
  };

  // SEE WIRING DIAGRAM
  i2s_pin_config_t pin_config_in = {
    .bck_io_num = 5,    
    .ws_io_num = 6,     
    .data_out_num = -1, 
    .data_in_num = 4    
  };

  i2s_pin_config_t pin_config_out = {
    .bck_io_num = 9,   
    .ws_io_num = 8,    
    .data_out_num = 7, 
    .data_in_num = -1  
  };

  err = i2s_driver_install(I2S_PORT_IN, &i2s_config_in, 0, NULL);
  if (err != ESP_OK) {
    Serial.printf("Failed installing INPUT driver: %d\n", err);
    while (true)
      ;
  }

  err = i2s_driver_install(I2S_PORT_OUT, &i2s_config_out, 0, NULL);
  if (err != ESP_OK) {
    Serial.printf("Failed installing OUTPUT driver: %d\n", err);
    while (true)
      ;
  }

  err = i2s_set_pin(I2S_PORT_IN, &pin_config_in);
  if (err != ESP_OK) {
    Serial.printf("Failed setting DIN pin: %d\n", err);
    while (true)
      ;
  }

  err = i2s_set_pin(I2S_PORT_OUT, &pin_config_out);
  if (err != ESP_OK) {
    Serial.printf("Failed setting  DOUT pin: %d\n", err);
    while (true)
      ;
  }

  Serial.println("I2S driver installed and running.");
}

void loop() {
  int bytesRead;
  i2s_read(I2S_PORT_IN, (char *)samples, I2S_BUFFER_SIZE * 4, (size_t *)&bytesRead, portMAX_DELAY);

  size_t samplesRead = bytesRead / 4;
  for (size_t i = 0; i < samplesRead; i++) {
    int32_t sample = samples[i];
    // dividend is not meaningful, just changing the gain of the audio a bit
    float adjSample = ((float)sample / 4147483647.0);  //2147483647 is max for 32 bit signed
    // put this back if we end up boosting the audio
    // if (adjSample > 1.0) {
    //   adjSample = 1.0;
    // } else if (adjSample < -1.0) {
    //   adjSample = -1.0;
    // }
    uint8_t normalizedInput = uint8_t((adjSample + 1.0) * 127);
    for (size_t y = 0; y < 4; y++) {
      auto index = yindicies[y];
      yinBuffers[y][index] = normalizedInput;
      if (++index == BUFFER_SIZE) {
        index = 0;
        runPitchDetect(yinBuffers[y]);
      }
      yindicies[y] = index;
    }

    int32_t sawtoothSample = sawtooth->readSample();
    samples[i] = (muteSpeaker || activePitch < 0) ? 0 : sawtoothSample;
  }

  int bytesWritten;
  i2s_write(I2S_PORT_OUT, (char *)samples, bytesRead, (size_t *)&bytesWritten, portMAX_DELAY);
  MIDI.read();
}