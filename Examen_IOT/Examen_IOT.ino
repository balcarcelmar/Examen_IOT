#include <Arduino.h>
#include <ClosedCube_HDC1080.h> 
#include <WiFi.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <vector>
#include <TinyGPSPlus.h>

// ======== CONFIGURACIÓN WiFi y Servidor ========
const char* ssid = "UPBWiFi";
const char* password = "";
const char* server = "98.85.127.18";    // Dirección del servidor
const char* postEndpoint = "/data";     // Endpoint para el POST

// ======== Objetos de sensores ========
ClosedCube_HDC1080 sensor;
TinyGPSPlus gps;

// ======== Variables para la máquina de estados ========
int estado = 0;                       
std::vector<double> datos;            
double promedioTemp = 0.0;           

// ======== Declaración de funciones =========
std::vector<double> chirp();
double mean(const std::vector<double>& datos);
String bundling(double temperatura, double lat, double lng);
void sendDataToServer(double temperatura);
void sleepArduino(unsigned long seconds);
static void smartDelay(unsigned long ms);

// ======== smartDelay ========
static void smartDelay(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (Serial1.available()) {
      gps.encode(Serial1.read());
    }
  } while (millis() - start < ms);
}

void setup() {

  // Inicializar la comunicación serie para debug
  Serial.begin(115200);
  delay(1000);
  
  
  
  // Inicializar I2C en pines SDA=0 y SCL=4
  Wire.begin(0, 4);

  // Inicializar el sensor de temperatura
  sensor.begin(0x40);
  
  // Inicializar el GPS en Serial1 (RX=34, TX=12)
  Serial1.begin(9600, SERIAL_8N1, 34, 12);
  
  // Conectar a WiFi
  Serial.print("Conectando a WiFi ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi conectado!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
  
  // Estado inicial
  estado = 1;
  Serial.println("Setup completado. Estado inicial = 1");
}

void loop() {
  switch (estado) {
    case 1: {
      Serial.println("=== Estado 1: Captura de temperatura (Chirp) ===");
      datos = chirp();  // Captura al menos 10 mediciones
      estado = 2;
      break;
    }
    case 2: {
      Serial.println("=== Estado 2: Cálculo del promedio de temperatura ===");
      promedioTemp = mean(datos);
      estado = 3;
      break;
    }
    case 3: {
      Serial.println("=== Estado 3: Bundling y Envío de datos ===");
      // Procesa datos del GPS durante 1 segundo sin bloquear totalmente
      smartDelay(1000);
      
      sendDataToServer(promedioTemp);
      
      // Enviar paquete cada 10 segundos
      delay(10000);
      estado = 1;
      break;
    }
    default:
      estado = 1;
      break;
  }
}

// ======== Función chirp() ========
// Captura al menos 10 mediciones del sensor de temperatura
std::vector<double> chirp() {
  std::vector<double> tempDatos;
  tempDatos.reserve(10);
  for (int i = 0; i < 10; i++) {
    double lectura = sensor.readTemperature();
    tempDatos.push_back(lectura);
    Serial.print("Medida ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(lectura);
    smartDelay(10);
  }
  return tempDatos;
}

// ======== Función mean() ========
// Calcula el promedio de las mediciones de temperatura
double mean(const std::vector<double>& datos) {
  if (datos.empty()) return 0.0;
  double suma = 0.0;
  for (double valor : datos) {
    suma += valor;
  }
  double prom = suma / datos.size();
  Serial.print("Promedio de temperatura: ");
  Serial.println(prom);
  return prom;
}

// ======== Función bundling() ========
// Construye el payload JSON con el formato requerido:
// {"CODIGO":"1011","TEMPERATURA":<valor>,"LONGITUD":<valor>,"LATITUD":<valor>}
String bundling(double temperatura, double lat, double lng) {
  String payload = "{";
  payload += "\"CODIGO\":\"1002\",";
  payload += "\"TEMPERATURA\":" + String(temperatura, 2) + ",";
  payload += "\"LONGITUD\":" + String(lng, 6) + ",";
  payload += "\"LATITUD\":" + String(lat, 6);
  payload += "}";
  return payload;
}

// ======== Función sendDataToServer() ========
// Envía los datos al servidor vía HTTP POST
void sendDataToServer(double temperatura) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    WiFiClient client;
    
    // Construir la URL: http://98.85.127.18:80/data
    String url = String("http://") + server + postEndpoint;
    http.begin(client, url);
    http.addHeader("Content-Type", "application/json");
    
    double lat = 0.0, lng = 0.0;
    if (gps.location.isValid()) {
      lat = gps.location.lat();
      lng = gps.location.lng();
    } else {
      Serial.println("GPS no es válido o sin señal.");
    }
    
    // Usar bundling() para formar el payload JSON
    String payload = bundling(temperatura, lat, lng);
    
    // Imprimir el payload y su longitud en la consola local
    Serial.println("Enviando Payload:");
    Serial.println(payload);
    Serial.print("Longitud del payload: ");
    Serial.println(payload.length());
    
    // Enviar el POST
    int httpResponseCode = http.POST(payload);
    if (httpResponseCode > 0) {
      Serial.print("HTTP Response code: ");
      Serial.println(httpResponseCode);
    } else {
      Serial.print("Error en la petición HTTP: ");
      Serial.println(httpResponseCode);
    }
    http.end();
  } else {
    Serial.println("WiFi no conectado. No se envían datos.");
  }
}

// ======== Función sleepArduino() ========
// Espera un número de segundos (usando delay en este ejemplo)
void sleepArduino(unsigned long seconds) {
  Serial.print("Esperando ");
  Serial.print(seconds);
  Serial.println(" segundo(s)...");
  delay(seconds * 1000UL);
}
