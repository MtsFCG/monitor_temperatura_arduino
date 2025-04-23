#include <DHT.h>
#include <PID_v1.h>

// Configuración del DHT11
#define DHTPIN 2       // Pin donde está conectado el DHT11
#define DHTTYPE DHT11  // Tipo de sensor
DHT dht(DHTPIN, DHTTYPE);

// Configuración del relé
#define RELAY_PIN 8    // Pin donde está conectado el relé

// Variables para el PID
double Setpoint = 22.0; // Temperatura deseada (en grados Celsius)
double Input;           // Temperatura actual leída del DHT11
double Output;          // Salida del controlador PID (0-100%)

// Parámetros PID (ajusta estos valores según sea necesario)
double Kp = 2.0, Ki = 0.5, Kd = 1.0;

// Crear el objeto PID
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Variables para el temporizador no bloqueante
unsigned long previousMillis = 0; // Tiempo anterior en milisegundos
const long interval = 2000;        // Intervalo entre lecturas (2 segundos)
unsigned long startTime = 0;

void setup() {
  // Inicializar comunicación serial
  Serial.begin(9600);
  
  // Inicializar el sensor DHT11
  dht.begin();
  
  while (Serial.available()) {
    Serial.read(); // Descartar cualquier dato residual
  }

  startTime = millis();
  // Configurar el pin del relé como salida
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW); // Apagar el relé inicialmente
  
  // Configurar el controlador PID
  myPID.SetMode(AUTOMATIC); // Modo automático
  myPID.SetOutputLimits(0, 100); // Límites de salida (0-100%)
}

void loop() {

  if (millis() - startTime < 2000) {
    return; // Salir del loop hasta que pasen 2 segundos
  }
  // Obtener el tiempo actual
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    // Guardar el tiempo actual como referencia
    previousMillis = currentMillis;


    // Leer la temperatura del DHT11
    float temperature = dht.readTemperature();
    float humidity = dht.readHumidity();
    
    if (isnan(temperature) || isnan(humidity)) {
      Serial.println("Error al leer el sensor DHT11");
      return;
    }
    
    // Actualizar la entrada del PID
    Input = temperature;
    
    // Calcular la salida del PID
    myPID.Compute();
    
    // Controlar el relé según la salida del PID
    if (Output > 0) {
      digitalWrite(RELAY_PIN, HIGH); // Encender el calentador
    } else {
      digitalWrite(RELAY_PIN, LOW);  // Apagar el calentador
    }
    
    // Mostrar información en el monitor serial
    // Mostrar información en el monitor serial
    String json = "{\"temperature\":";
    json += String(temperature, 2);
    json += ",\"humidity\":";
    json += String(humidity, 2);
    json += ",\"pid_output\":";
    json += String(Output, 2);
    json += "}";
    Serial.println(json);
    
  }
}