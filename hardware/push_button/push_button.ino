#define BUTTON_PIN 2
#define OUTPUT_PIN 4   // this will toggle

bool state = false;
bool lastButtonState = HIGH;

void setup() {
  pinMode(BUTTON_PIN, INPUT_PULLUP); //pin , mode 
  pinMode(OUTPUT_PIN, OUTPUT);

  digitalWrite(OUTPUT_PIN, LOW);
}

void loop() {
  bool currentButtonState = digitalRead(BUTTON_PIN); //when pressed curent -> low

  // Detect button press (HIGH → LOW)
  if (lastButtonState == HIGH && currentButtonState == LOW) {
    state = !state;  // toggle state  

    digitalWrite(OUTPUT_PIN, state ? HIGH : LOW);

    delay(200); // debounce
  }

  lastButtonState = currentButtonState;
}