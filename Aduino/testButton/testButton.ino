#define BUTTON_PIN A6
void setup()
{
  Serial.begin(115200);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
}
void loop()
{
  byte buttonState = analogRead(BUTTON_PIN);
  
  Serial.println(buttonState);

  if (buttonState > 0) {
    Serial.println("NOT PRESSED");
    
  } else {
    Serial.println(" PRESSED");
  }
  delay(100);}
