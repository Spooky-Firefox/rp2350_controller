#include <Adafruit_INA260.h>

Adafruit_INA260 ina260 = Adafruit_INA260();

void setup() {
  // Due klarar hög hastighet, 115200 är standard
  Serial.begin(115200);
  while (!Serial) delay(10);

  if (!ina260.begin()) {
    Serial.println("Fel: Hittade inte INA260-sensorn. Kontrollera kablarna!");
    while (1);
  }

  // Skriv ut rubrikerna för CSV-filen
  // Detta gör att program som Excel fattar vad varje kolumn betyder direkt
  Serial.println("Timestamp_ms,Voltage_mV,Current_mA,Power_mW");
}

void loop() {
  // Läs in värden från sensorn
  float voltage = ina260.readBusVoltage(); // Spänning i mV
  float current = ina260.readCurrent();    // Ström i mA
  float power = ina260.readPower();        // Effekt i mW
  unsigned long timeStamp = millis();      // Tid sedan start i ms

  // Skriv ut data i CSV-format: Tid, Volt, Ampere, Watt
  Serial.print(timeStamp);
  Serial.print(",");
  Serial.print(voltage);
  Serial.print(",");
  Serial.print(current);
  Serial.print(",");
  Serial.println(power);

  // Vänta 100ms mellan varje mätning (10 mätningar i sekunden)
  // Justera efter hur tät data du behöver
  delay(10);
}
