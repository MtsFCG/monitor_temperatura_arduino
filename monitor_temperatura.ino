#include <DHT.h>
#include <PID_v1.h>

// Configuración del DHT11
#define DHTPIN 4       // Pin donde está conectado el DHT11
#define DHTTYPE DHT11  // Tipo de sensor
DHT dht(DHTPIN, DHTTYPE);

// Configuración del relé
#define RELAY_PIN_PAR_RESISTENCIAS_CENTRALES_VENTILADOR 2   // Pin donde está conectado el relé 1 , primer par de resistencias que se deben prender RELE QLAO INVERTIDO
#define RELAY_PIN_B_PAR_RESISTENCIAS_EXTERIORES_HUMIFICADOR 3   // Pin donde está conectado el relé 2 , segundo par de resistencias que se deben prender 

// Variables de estado relay
bool relay1_estado = HIGH; // RELAY_PIN_PAR_RESISTENCIAS_CENTRALES_VENTILADOR (pin 2) parte en high por que este rele es a reves de los cristianos
bool relay2_estado = LOW; // RELAY_PIN_B_PAR_RESISTENCIAS_EXTERIORES_HUMIFICADOR (pin 3)

// Definición de zonas
#define ZONA_1_MIN 10.0   // Inicia zona 1
#define ZONA_2_MIN 50.0   // Inicia zona 2
#define ZONA_3_MIN 80.0   // Inicia zona 3 (advertencia)

// Variables para el PID
double Setpoint = 25.0; // Temperatura deseada (en grados Celsius)
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
  digitalWrite(RELAY_PIN_PAR_RESISTENCIAS_CENTRALES_VENTILADOR, HIGH); // Apagar el relé inicialmente
  digitalWrite(RELAY_PIN_B_PAR_RESISTENCIAS_EXTERIORES_HUMIFICADOR, LOW);

  // Configurar el controlador PID
  myPID.SetMode(AUTOMATIC); // Modo automático
  myPID.SetOutputLimits(0, 100); // Límites de salida (0-100%)

}

void loop() {

  // verifica el funcionamiento del sensor
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();
  if (isnan(temperature)) {
      Serial.println("Error al leer el sensor DHT11");
      return;
  }

  Input = temperature;
  myPID.Compute();

  Serial.print("Salida PID ajustada: ");
  Serial.println(Output);
  
  // Determinar la zona según la salida del PID
  if (Output <= ZONA_1_MIN) {
      // Zona 0: apagar todo
      if (relay1_estado == LOW || relay2_estado == HIGH) {
          digitalWrite(RELAY_PIN_PAR_RESISTENCIAS_CENTRALES_VENTILADOR, HIGH);
          digitalWrite(RELAY_PIN_B_PAR_RESISTENCIAS_EXTERIORES_HUMIFICADOR, LOW);
          relay1_estado = HIGH;
          relay2_estado = LOW;
          Serial.println("Zona 0: Sin calefacción necesaria");
      }
  } else if (Output > ZONA_1_MIN && Output <= ZONA_2_MIN) {
      // Zona 1: solo nivel 1
      if (relay1_estado == HIGH) {
          digitalWrite(RELAY_PIN_PAR_RESISTENCIAS_CENTRALES_VENTILADOR, LOW);
          relay1_estado = LOW;
          Serial.println("Zona 1: Encendido solo primer nivel");
      }
      if (relay2_estado == HIGH) {
          digitalWrite(RELAY_PIN_B_PAR_RESISTENCIAS_EXTERIORES_HUMIFICADOR, LOW);
          relay2_estado = LOW;
          Serial.println("Zona 1: Encendido solo primer nivel, nivel 2 apagado");
      }
  } else if (Output > ZONA_2_MIN && Output <= ZONA_3_MIN) {
      // Zona 2: ambos niveles
      if (relay1_estado == HIGH) {
          digitalWrite(RELAY_PIN_PAR_RESISTENCIAS_CENTRALES_VENTILADOR, LOW);
          relay1_estado = LOW;
          Serial.println("Zona 2: Encendido primer nivel en seguida debe prender el segundo nivel");
      }
      if (relay2_estado == LOW) {
          digitalWrite(RELAY_PIN_B_PAR_RESISTENCIAS_EXTERIORES_HUMIFICADOR, HIGH);
          relay2_estado = HIGH;
          Serial.println("Zona 2: Ambos niveles encendidos");
      }
  } else {
      // Zona 3: advertir que el sistema no puede seguir
      if (relay1_estado == HIGH) {
          digitalWrite(RELAY_PIN_PAR_RESISTENCIAS_CENTRALES_VENTILADOR, LOW);
          relay1_estado = LOW;
      }
      if (relay2_estado == LOW) {
          digitalWrite(RELAY_PIN_B_PAR_RESISTENCIAS_EXTERIORES_HUMIFICADOR, HIGH);
          relay2_estado = HIGH;
      }
      Serial.println("Zona 3: Máxima potencia. ¡Posible fallo o ambiente muy frío!");
  }
  // Mostrar información en el monitor serial
  Serial.print("Temperatura: ");
  Serial.print(temperature);
  Serial.print(" °C, Humidity: ");
  Serial.print(humidity);
  Serial.print(" °C, Salida PID: ");
  Serial.print(Output);
  Serial.print("% Relay 1: ");
  Serial.print(relay1_estado);
  Serial.print(" Relay 2: ");
  Serial.println(relay2_estado);
  // Esperar antes de la siguiente lectura
  delay(2000); // 2 segundos
}