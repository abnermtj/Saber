#define BUTTON_PIN D1
#define LED_PIN D8
#define BUTTON_WINDOW_SIZE 40
int avgButtonVal = 100;
void setup()
{
  Serial.begin(115200);
   analogReadResolution(10);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
   pinMode(D7, OUTPUT);
    pinMode(LED_PIN, OUTPUT);
}
void loop()
{
  byte buttonState = analogRead(BUTTON_PIN);
  avgButtonVal -= avgButtonVal / BUTTON_WINDOW_SIZE;
  avgButtonVal += (float)(2 * buttonState) / BUTTON_WINDOW_SIZE;
  // Serial.println(buttonState);

  if (avgButtonVal > 50) {
    // Serial.println("NOT PRESSED");
    
  } else {
    Serial.println(" PRESSED");
    Serial.println(buttonState);
  }
  delay(1);}
