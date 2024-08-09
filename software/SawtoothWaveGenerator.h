
class SawtoothWaveGenerator {
public:
    SawtoothWaveGenerator(float sampleRate)
        : sampleRate(sampleRate), frequency(440.0f), phase(0.0f), phaseIncrement(0.0f) {
        updatePhaseIncrement();
    }

    void setFrequency(float freq) {
        frequency = freq;
        updatePhaseIncrement();
    }

    int32_t readSample() {
        float value = 2.0f * (phase - floor(phase)) - 1.0f;
        int32_t sample = static_cast<int32_t>(value * 2147483647.0f);

        phase += phaseIncrement;
        if (phase >= 1.0f) {
            phase -= 1.0f;
        }

        return sample / 2;
    }

private:
    void updatePhaseIncrement() {
        phaseIncrement = frequency / sampleRate;
    }

    float sampleRate;
    float frequency;
    float phase;
    float phaseIncrement;
};