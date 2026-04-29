#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <vibration_sensor_inferencing.h>

// -------- I2C --------
#define SDA_PIN 8
#define SCL_PIN 9

// -------- MPU9250 --------
#define MPU9250_ADDR 0x68
#define ACCEL_SENSITIVITY 8192.0   // ±4g
#define GYRO_SENSITIVITY  65.5     // ±500°/s

// -------- OLED --------
#define SCREEN_WIDTH 128  
#define SCREEN_HEIGHT 32
#define OLED_ADDR 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// -------- Button --------
#define BUTTON_PIN 2
#define OUTPUT_PIN 4

bool state = false;
bool lastButtonState = HIGH;

// -------- Calibration --------
#define CALIBRATION_SAMPLES 500
float offsetAX = 0, offsetAY = 0, offsetAZ = 0;
float offsetGX = 0, offsetGY = 0, offsetGZ = 0;

// -------- Sampling at 100 Hz --------
#define FREQUENCY_HZ 100
#define INTERVAL_MS (1000 / (FREQUENCY_HZ + 1))

// -------- Inference --------
float features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];
size_t feature_ix = 0;
static unsigned long last_interval_ms = 0;

// -------- Display state --------
String healthStatus = "---";
float confidence = 0.0;
unsigned long lastDisplayTime = 0;
#define DISPLAY_INTERVAL_MS 300

// -------- Variables --------
int16_t ax, ay, az;
int16_t gx, gy, gz;

// -------- ei_printf helper --------
void ei_printf(const char *format, ...) {
  static char print_buf[1024] = { 0 };
  va_list args;
  va_start(args, format);
  int r = vsnprintf(print_buf, sizeof(print_buf), format, args);
  va_end(args);
  if (r > 0) {
    Serial.write(print_buf);
  }
}

void setup() {
  Serial.begin(115200);

  // I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  // OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("OLED failed");
    while (1);
  }
  display.setTextColor(SSD1306_WHITE);

  // MPU9250 wakeup
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);
  delay(100);

  // Accel range ±4g
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(0x1C);
  Wire.write(0x08);
  Wire.endTransmission(true);

  // Gyro range ±500°/s
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission(true);

  // DLPF 44 Hz
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission(true);

  // Sample rate 100 Hz
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(0x19);
  Wire.write(0x09);
  Wire.endTransmission(true);

  // Button
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(OUTPUT_PIN, OUTPUT);
  digitalWrite(OUTPUT_PIN, LOW);

  // -------- Calibration --------
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 4);
  display.print("Calibrating...");
  display.setCursor(0, 18);
  display.print("Keep sensor still!");
  display.display();

  delay(500);

  long sumAX = 0, sumAY = 0, sumAZ = 0;
  long sumGX = 0, sumGY = 0, sumGZ = 0;

  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    Wire.beginTransmission(MPU9250_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU9250_ADDR, 14, true);

    if (Wire.available() == 14) {
      int16_t rax = (int16_t)(Wire.read() << 8 | Wire.read());
      int16_t ray = (int16_t)(Wire.read() << 8 | Wire.read());
      int16_t raz = (int16_t)(Wire.read() << 8 | Wire.read());
      Wire.read(); Wire.read();
      int16_t rgx = (int16_t)(Wire.read() << 8 | Wire.read());
      int16_t rgy = (int16_t)(Wire.read() << 8 | Wire.read());
      int16_t rgz = (int16_t)(Wire.read() << 8 | Wire.read());

      sumAX += rax; sumAY += ray; sumAZ += raz;
      sumGX += rgx; sumGY += rgy; sumGZ += rgz;
    }
    delay(10);
  }

  offsetAX = sumAX / (float)CALIBRATION_SAMPLES;
  offsetAY = sumAY / (float)CALIBRATION_SAMPLES;
  offsetAZ = sumAZ / (float)CALIBRATION_SAMPLES;
  offsetGX = sumGX / (float)CALIBRATION_SAMPLES;
  offsetGY = sumGY / (float)CALIBRATION_SAMPLES;
  offsetGZ = sumGZ / (float)CALIBRATION_SAMPLES;

  // Print model info
  Serial.println("--- Vibration Monitor Ready ---");
  Serial.print("Features per inference: ");
  Serial.println(EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
  Serial.print("Classes: ");
  Serial.println(EI_CLASSIFIER_LABEL_COUNT);
  Serial.print("Frequency: ");
  Serial.print(FREQUENCY_HZ);
  Serial.println(" Hz");

  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Ready!");
  display.setCursor(0, 12);
  display.print("Press btn to start");
  display.display();
  delay(1000);
}

void loop() {

  // -------- BUTTON TOGGLE --------
  bool currentButtonState = digitalRead(BUTTON_PIN);

  if (lastButtonState == HIGH && currentButtonState == LOW) {
    state = !state;
    digitalWrite(OUTPUT_PIN, state ? HIGH : LOW);

    // Reset inference buffer when toggling
    feature_ix = 0;
    if (!state) {
      healthStatus = "---";
      confidence = 0.0;
    }

    delay(200);
  }

  lastButtonState = currentButtonState;

  // -------- WHEN FAN IS ON: COLLECT DATA & RUN INFERENCE --------
  if (state) {

    if (millis() > last_interval_ms + INTERVAL_MS) {
      last_interval_ms = millis();

      // Read 6-axis
      Wire.beginTransmission(MPU9250_ADDR);
      Wire.write(0x3B);
      Wire.endTransmission(false);
      Wire.requestFrom(MPU9250_ADDR, 14, true);

      if (Wire.available() == 14) {
        ax = (int16_t)(Wire.read() << 8 | Wire.read());
        ay = (int16_t)(Wire.read() << 8 | Wire.read());
        az = (int16_t)(Wire.read() << 8 | Wire.read());
        Wire.read(); Wire.read();
        gx = (int16_t)(Wire.read() << 8 | Wire.read());
        gy = (int16_t)(Wire.read() << 8 | Wire.read());
        gz = (int16_t)(Wire.read() << 8 | Wire.read());
      } else {
        return;
      }

      // Convert to calibrated values (same as training data)
      float ax_g   = (ax - offsetAX) / ACCEL_SENSITIVITY;
      float ay_g   = (ay - offsetAY) / ACCEL_SENSITIVITY;
      float az_g   = (az - offsetAZ) / ACCEL_SENSITIVITY;
      float gx_dps = (gx - offsetGX) / GYRO_SENSITIVITY;
      float gy_dps = (gy - offsetGY) / GYRO_SENSITIVITY;
      float gz_dps = (gz - offsetGZ) / GYRO_SENSITIVITY;

      // Fill features buffer (6 axes interleaved, same order as training)
      features[feature_ix++] = ax_g;
      features[feature_ix++] = ay_g;
      features[feature_ix++] = az_g;
      features[feature_ix++] = gx_dps;
      features[feature_ix++] = gy_dps;
      features[feature_ix++] = gz_dps;

      // -------- RUN INFERENCE when buffer is full --------
      if (feature_ix >= EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {

        signal_t signal;
        ei_impulse_result_t result;

        int err = numpy::signal_from_buffer(features, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
        if (err != 0) {
          ei_printf("Signal error (%d)\n", err);
          feature_ix = 0;
          return;
        }

        EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);

        if (res != 0) {
          Serial.println("Classifier error");
          feature_ix = 0;
          return;
        }

        // Find the class with highest confidence
        float maxVal = 0;
        int maxIdx = 0;
        for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
          if (result.classification[ix].value > maxVal) {
            maxVal = result.classification[ix].value;
            maxIdx = ix;
          }
        }

        healthStatus = String(result.classification[maxIdx].label);
        confidence = maxVal * 100.0;

        // Serial output
        Serial.print("Prediction: ");
        Serial.print(healthStatus);
        Serial.print(" | Confidence: ");
        Serial.print(confidence, 1);
        Serial.print("% | DSP: ");
        Serial.print(result.timing.dsp);
        Serial.print("ms | Classify: ");
        Serial.print(result.timing.classification);
        Serial.println("ms");

        // Reset buffer for next inference window
        feature_ix = 0;
      }
    }
  }

  // -------- OLED DISPLAY --------
  unsigned long nowMs = millis();
  if (nowMs - lastDisplayTime >= DISPLAY_INTERVAL_MS) {
    lastDisplayTime = nowMs;

    display.clearDisplay();
    display.setTextSize(1);

    // Line 1: Fan state
    display.setCursor(0, 0);
    display.print("Fan: ");
    display.print(state ? "ON" : "OFF");

    // Line 2: Health status
    display.setCursor(0, 11);
    display.print("Health: ");
    if (!state) {
      display.print("No Detection");
    } else {
      display.print(healthStatus);
    }

    // Line 3: Confidence (only when fan is ON and inference has run)
    display.setCursor(0, 22);
    if (state && confidence > 0) {
      display.print("Conf: ");
      display.print(confidence, 1);
      display.print("%");

      // Show a mini progress bar
      int barWidth = map((int)confidence, 0, 100, 0, 48);
      display.drawRect(80, 22, 48, 8, SSD1306_WHITE);
      display.fillRect(80, 22, barWidth, 8, SSD1306_WHITE);
    } else if (!state) {
      display.print("Press btn to start");
    } else {
      display.print("Collecting data...");
    }

    display.display();
  }
}
