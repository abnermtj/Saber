#define BUTTON_PIN A0
#define LED_PIN D8
void setup()
{
  Serial.begin(115200);
  pinMode(BUTTON_PIN, INPUT);
   pinMode(D7, OUTPUT);
    pinMode(LED_PIN, OUTPUT);
}
void loop()
{
  byte buttonState = analogRead(BUTTON_PIN);
  
  Serial.println(buttonState);

  if (buttonState > 150) {
    Serial.println("NOT PRESSED");
    
  } else {
    Serial.println(" PRESSED");
  }
  delay(100);}
