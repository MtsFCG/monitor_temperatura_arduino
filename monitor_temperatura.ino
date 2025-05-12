#include <DHT.h>
#include <PID_v1.h>

// Configuración del DHT11
#define DHTPIN 4       // Pin donde está conectado el DHT11
#define DHTTYPE DHT11  // Tipo de sensor
DHT dht(DHTPIN, DHTTYPE);

// Configuración del relé
#define RELAY_PIN_PAR_RESISTENCIAS_CENTRALES_VENTILADOR 3   // Pin donde está conectado el relé 1 , primer par de resistencias que se deben prender 
#define RELAY_PIN_B_PAR_RESISTENCIAS_EXTERIORES_HUMIFICADOR 2   // Pin donde está conectado el relé 1 , primer par de resistencias que se deben prender 
// Variables para el PID
double Setpoint = 22.0; // Temperatura deseada (en grados Celsius)
double Input;           // Temperatura actual leída del DHT11
double Output;          // Salida del controlador PID (0-100%)

// Parámetros PID (ajusta estos valores según sea necesario)
double Kp = 2.0, Ki = 0.5, Kd = 1.0;

// Crear el objeto PID
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  // Inicializar comunicación serial
  Serial.begin(9600);
  
  // Inicializar el sensor DHT11
  dht.begin();
  
  // Configurar el pin del relé como salida
  pinMode(RELAY_PIN_PAR_RESISTENCIAS_CENTRALES_VENTILADOR, OUTPUT);
  pinMode(RELAY_PIN_B_PAR_RESISTENCIAS_EXTERIORES_HUMIFICADOR, OUTPUT);
  digitalWrite(RELAY_PIN_PAR_RESISTENCIAS_CENTRALES_VENTILADOR, LOW); // Apagar el relé inicialmente
  digitalWrite(RELAY_PIN_B_PAR_RESISTENCIAS_EXTERIORES_HUMIFICADOR, LOW);

  // Configurar el controlador PID
  myPID.SetMode(AUTOMATIC); // Modo automático
  myPID.SetOutputLimits(0, 100); // Límites de salida (0-100%)
}

void loop() {
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
  Serial.print("Temperatura: ");
  Serial.print(temperature);
  Serial.print(" °C, Humidity: ");
  Serial.print(humidity);
  Serial.print(" °C, Salida PID: ");
  Serial.print(Output);
  Serial.println("%");
  
  // Esperar antes de la siguiente lectura
  delay(2000); // 2 segundos
}