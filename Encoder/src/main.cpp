#include <driver/pcnt.h>
#include <esp_timer.h>

#include <Wire.h>

const int ENC_A_PIN = 11;
const int ENC_B_PIN = 12;
const float PULSES_PER_REV = 5281.1;
const int SAMPLE_PERIOD_MS = 100; // 100 milliseconds

const int I2C_ADDRESS = 0x04; // pick something

// const int SDA_PIN = 21;  // Default SDA pin for ESP32
// const int SCL_PIN = 22;  // Default SCL pin for ESP32

// RPM calculation
volatile int64_t lastMeasurementTime = 0;
volatile int32_t lastCount = 0;
volatile float currentRPM = 0.0;

// PCNT unit configuration from docs
pcnt_config_t pcnt_config = {
    .pulse_gpio_num = ENC_A_PIN,     // Pin A
    .ctrl_gpio_num = ENC_B_PIN,      // Pin B
    .lctrl_mode = PCNT_MODE_REVERSE, // Reverse counting direction if B is high
    .hctrl_mode = PCNT_MODE_KEEP,    // Keep counting direction if B is low
    .pos_mode = PCNT_COUNT_INC,      // Count up on rising edge of A
    .neg_mode = PCNT_COUNT_DEC,      // Count down on falling edge of A
    .counter_h_lim = INT16_MAX,
    .counter_l_lim = INT16_MIN,
    .unit = PCNT_UNIT_0,
    .channel = PCNT_CHANNEL_0};

// Timer callback for regular RPM updates, using demo code from docs
void IRAM_ATTR updateRPM(void *arg)
{
    int16_t count;
    pcnt_get_counter_value(PCNT_UNIT_0, &count);

    int64_t currentTime = esp_timer_get_time();
    int64_t timeDiff = currentTime - lastMeasurementTime;

    // Calculate pulse count difference
    int32_t countDiff = count - lastCount;

    // Reset counter to prevent overflow
    pcnt_counter_clear(PCNT_UNIT_0);

    // Calculate RPM
    // (countDiff * 60 * 1,000,000) / (PULSES_PER_REV * timeDiff)
    currentRPM = (countDiff * 60.0 * 1000000.0 * 3) / (PULSES_PER_REV * timeDiff); // * 3 for some reason??
    // (1+(46/17) )* (1+(46/17)) * (1+(46/17) )* (1+(46/17)) * 28
    lastCount = 0; // Counter was cleared, can also not reset it, but for us it is better to reset it to avoid overflow
    lastMeasurementTime = currentTime;
}

void setup()
{
    // Initialize PCNT unit
    pcnt_unit_config(&pcnt_config);

    // Set up input filters to debounce
    /* From docs: filter_val is a 10-bit value, so the maximum filter_val should be limited to 1023.
        Any pulses lasting shorter than this will be ignored when the filter is enabled.

        I assume it is microseconds, so 1000 is 1ms, but not sure
     */
    pcnt_set_filter_value(PCNT_UNIT_0, 1000); // Adjust filter value based on noise, not sure what the unit is here
    pcnt_filter_enable(PCNT_UNIT_0);

    // Enable counter
    pcnt_counter_pause(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_resume(PCNT_UNIT_0); // initialize to 0

    // Create timer for regular RPM updates
    esp_timer_handle_t timer;
    esp_timer_create_args_t timer_args = {
        .callback = &updateRPM,
        .name = "rpm_timer"};
    esp_timer_create(&timer_args, &timer);
    esp_timer_start_periodic(timer, SAMPLE_PERIOD_MS * 1000); // Convert ms to us

    // Initialize timing variables
    lastMeasurementTime = esp_timer_get_time();

    // Initialize I2C
    // Wire.begin(SDA_PIN, SCL_PIN);
}

float getRPM()
{
    return currentRPM;
}

void loop()
{
    // hopefully accurate rpm value???
    float rpm = getRPM();
    printf("RPM: %.2f\n", rpm);

    // Wire.beginTransmission(I2C_ADDRESS);
    // uint8_t* bytes = (uint8_t*)&rpm;
    // for (int i = 0; i < sizeof(float); i++) {
    //     Wire.write(bytes[i]);
    // }
    // Wire.endTransmission();

    // delay(100);
}