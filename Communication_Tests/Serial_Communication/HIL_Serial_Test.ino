// Serial Communication Test
// MOURA. Waldenê de Melo

// union to facilitate the manipulation of bytes in variables. 
typedef union {
  float number;
  uint8_t bytes[4];
} var_union;

// Variaveis de controle
var_union temp;

// ESP32 - Define LED_BUILTIN
// #define LED_BUILTIN 2

void setup() {
  // Initialize serial port
  Serial.begin(115200);

  // Initialize built-in LED 
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}

void loop() {

  // Check if serial port is available
  if (Serial.available() >= 5) {

    if (Serial.read() == 'W') {

      // Receive input data
      float val_rec = fncTraz();

      // Blink Led 
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

      // Send header
      Serial.write('A');

      // Send Information
      temp.number = val_rec;
      fncEnvia(temp);

      // Send terminator
      Serial.write(10);

      // Synchronizes
      delay(80);
    }
  }

  // Synchronizes
  delay(80);
}

// Gets data from the serial port
float fncTraz() {
  var_union temp;
  
  for (int i = 0; i < 4; i++) {
    temp.bytes[i] = Serial.read();
  }

  return temp.number;
}

// Send data to serial port. 
void fncEnvia(var_union x) {
  for (int i = 0; i < 4; i++) {
    Serial.write(x.bytes[i]);
  }
}
