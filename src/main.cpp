#include <Arduino.h>
#include <vector>
#include "enum_events.h"
#include "Utils.h"

#define REGULATOR_PIN 38  // Control del regulador
#define PWRKEY_PIN 41     // Control del PWRKEY
#define DEFAULT_SVR "34.196.135.179"
#define DEFAULT_PORT 5200
#define PDPTYPE "IP"
String inputCommand = "";
// UART del SIM7600
HardwareSerial sim7600(1); // UART1

String rawdata;             // Variable para almacenar los datos del GPS
unsigned long lastPrintTime = 0; // Tiempo del último envío
 unsigned long interval = 15000;
float lastValidCourse = 0.0;
String message="";
float previousCourse = -1.0;
int trackingCourse = 0;
bool LaststateIgnition = HIGH;
String state = "IP";
String apn = "internet.itelcel.com";
int c_id = 1;
std::vector<String> storedMessages;

struct GPSData {
  int mode;
  int gps_svs;
  int glonass_svs;
  int beidou_svs;
  String latitude = "0.0";
  String longitude = "0.0";
  char ns_indicator;
  char ew_indicator;
  String date;
  String utc_time;
  float altitude;
  float speed;
  float course;
  float pdop;
  float hdop;
  float vdop;
};
GPSData currentGNSSData;
String GNSSData;
int eventState;
bool ignState;
bool fix = 1;
String datetime;

void activeSIM();
void configSIM();
void activeGPS();
void activeReport();
String trimResponse(const String& response);
String sendCommandWithResponse( const char* command, int timeout );
String processResponse(const String& command,  const String& fcommand, const String& response);
String readSerialGNSSData();
void handleSerialInput();
String formatCoordinates(const String &coord, char direction);
String formatDate(const String &date);
String formatTime(const String &utcTime);
GPSData processGNSS(const String& response);
String cleanGNSS(const String& data);
bool sendData(const String& message);
int eventCourse(GPSData data);
void ignition_event(GPSData gpsData);
bool readInput();
void event_generated(GPSData gpsData, int event);
bool initializeModule();
String getDateTime();
String readData(String data, int timeout);
String sendReadDataToServer(const String& fcommand, const String& message, int timeout);
int commandType(const String& command);
bool validTcpNet();
void activeTCP();
bool configureTCP(const String& server, int port);
bool getPositionServer();
void confiGpsReports();
bool checkSignificantCourseChange(float currentCourse);

void setup() {
  // Configurar pines
  pinMode(REGULATOR_PIN, OUTPUT);
  pinMode(PWRKEY_PIN, OUTPUT);

  // Configurar UART
  sim7600.begin(115200, SERIAL_8N1, 5, 4); // RX=IO5, TX=IO4
  Serial.begin(115200); // UART para monitoreo en PC
  pinMode(10, INPUT);
  // Iniciar el módulo SIM7600
  activeSIM();
  do{Serial.println("Inicializando Modulo...");}while(!initializeModule());
  configSIM();
  do{Serial.println("Activando servicio TCP...");}while(!validTcpNet());
  do{Serial.println("Conectando al servidor...");}while(!configureTCP(DEFAULT_SVR, DEFAULT_PORT));
  confiGpsReports();
}

void loop() {
   handleSerialInput();
  GNSSData = readSerialGNSSData();                 
  currentGNSSData = processGNSS(GNSSData);
  readInput() ? ignState = 0 : ignState = 1;
  ignition_event(currentGNSSData);
  datetime = currentGNSSData.date+";"+currentGNSSData.utc_time;

  message = "STT;2049830928;3FFFFF;32;1.0.0;1;"+datetime+";103682809;334;020;40C6;20;"+currentGNSSData.latitude+";"+currentGNSSData.longitude+";" +String(currentGNSSData.speed) + ";" +
            String(currentGNSSData.course) + ";" +String(currentGNSSData.gps_svs)+";"+fix+";"+trackingCourse+"000000"+ignState+";00000000;1;1;0929;4.1;14.19";
  
  if (checkSignificantCourseChange(currentGNSSData.course)) {
    storedMessages.push_back(message);
    return; 
  }
   // Si no hay cambio significativo, pero hay mensajes almacenados
  if (!storedMessages.empty()) {
    // Enviar todos los mensajes almacenados
    for (const auto& msg : storedMessages) {
      sendData(msg);
    }
    storedMessages.clear(); // Limpiar el vector después de enviar los mensajes
  }
  unsigned long currentTime = millis();
  if (currentTime - lastPrintTime >= interval && ignState == 1) {
    lastPrintTime = currentTime;
    Serial.println("DATA =>"+message);
    Serial.println("RAWDATA =>"+GNSSData);
    sendData(message);
      /*if (!gpsData.isEmpty()) {
        Serial.print("GNSS DATA => ");
        Serial.println(gpsData); // Imprimir datos almacenados
      } else {
        Serial.println("No se han recibido datos del GPS.");
      }*/
  }
   
}
bool checkSignificantCourseChange(float currentCourse) {
  if (isnan(currentCourse)) {
    Serial.println("Advertencia: el valor del curso no es válido.");
    return false;
  }

  float difference = abs(currentCourse - previousCourse);
  if (difference >= 10) {
    Serial.print("Cambio significativo detectado en course: ");
    Serial.println(difference);
    trackingCourse = 1;
    previousCourse = currentCourse;  // Actualizar el valor anterior
    return true;
  }
  trackingCourse = 0;
  //previousCourse = currentCourse;  // Actualizar de todos modos para la próxima comparación
  return false;
}
String readSerialGNSSData() {
  String clean = "";
  while (sim7600.available()) {
    rawdata = sim7600.readStringUntil('\n'); // Leer una línea completa 
    //clean = cleanGNSS(rawdata);    
  }
  return rawdata;
}
void handleSerialInput() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    String response = sendCommandWithResponse(command.c_str(), 4000);
    Serial.print("Respuesta limpia: ");
    Serial.println(response);
  }
}

void activeSIM() {
  Serial.println("Iniciando módulo SIM7600...");

  // Paso 1: Activar regulador
  digitalWrite(REGULATOR_PIN, HIGH);
  Serial.println("Regulador activado.");
  delay(2000); // Esperar 2 segundos para estabilizar

  // Paso 2: Activar PWRKEY
  digitalWrite(PWRKEY_PIN, LOW); // Simula presionar PWRKEY
  Serial.println("PWRKEY presionado...");
  delay(1000); // Mantener bajo por 1 segundo

  digitalWrite(PWRKEY_PIN, HIGH); // Liberar PWRKEY
  Serial.println("PWRKEY liberado.");

  // Esperar unos segundos adicionales para el arranque
  delay(3000);
  Serial.println("Módulo SIM7600 iniciado.");
}
void configSIM() {
  String cfun = sendCommandWithResponse("AT+CFUN=1", 4000);
  Serial.println("+CFUN => "+ cfun);
  String cgdcont_cmd = "AT+CGDCONT=\""+String(c_id)+"\",\""+PDPTYPE+"\",\""+apn+"\"";
  Serial.println("Comando APN Conf => "+ cgdcont_cmd);
  String cgdcont = sendCommandWithResponse(cgdcont_cmd.c_str(), 4000);
  Serial.println("+CGDCONT => "+ cgdcont);
  String cgact = sendCommandWithResponse("AT+CGACT=1,1", 4000);
  Serial.println("+CGACT => "+ cgact);
  String cgps = sendCommandWithResponse("AT+CGPS=1", 4000);  
  Serial.println("+CGPS => "+ cgps);
}
String sendCommandWithResponse( const char* command, int timeout ) {
  
    int type = commandType(command);

  String formattedCommand = String(command).substring(3); // Quitar "AT+"
  if(type == READ){
    //quitar cualquier sufijo "?"
   // Serial.println("Quitando sufijo... ");
    if (formattedCommand.endsWith("?") ) {
      formattedCommand.remove(formattedCommand.length() - 1); // Remover sufijo
    }
  }else if(type == WRITE){
    //Serial.println("quitando signo '='");
    // Buscar si hay un '=' en el comando, y eliminar todo lo que esté después
    int posEqual = formattedCommand.indexOf("=");
    if (posEqual != -1) {
      formattedCommand = formattedCommand.substring(0, posEqual);  // Mantener solo hasta antes del '='
    }
  }else if(type == TEST){
    if (formattedCommand.endsWith("=?") ) {
      formattedCommand.remove(formattedCommand.length() - 2); // Remover sufijo
    }
  }
  if(formattedCommand == "AT") {
      sim7600.println(formattedCommand);  // Enviar comando
  }else{
    sim7600.println(command);  // Enviar comando    
  }
  String response = "";
  long startTime = millis();
  while ((millis() - startTime) < timeout) {
    if (sim7600.available()) {
      char c = sim7600.read();
      response += c;
    }
  }
    return processResponse(command, formattedCommand, response);
}
String cleanGNSS(const String& cleanData) {
  String processData = cleanData;
  processData.replace("+CGNSSINFO: ", "");
  return processData;
}
GPSData processGNSS(const String& trashData) {
    String data = trashData;
  GPSData gpsData;
  int index = 0;

  data.replace("+CGNSSINFO: ", "");
  String tokens[16]; // Array para almacenar las partes de la cadena

    // Dividir la cadena en partes usando ',' como delimitador
    for (int i = 0; i < data.length(); i++) {
        if (data[i] == ',' || i == data.length() - 1) {
            if (i == data.length() - 1) tokens[index] += data[i];
            index++;
        } else {
            tokens[index] += data[i];
        }
    }
    // Asignar los valores a la estructura
    gpsData.mode = tokens[0].toInt();
    gpsData.gps_svs = tokens[1].toInt();
    gpsData.glonass_svs = tokens[2].toInt();
    gpsData.beidou_svs = tokens[3].toInt();
    gpsData.latitude = !tokens[4].isEmpty() ? formatCoordinates(tokens[4], tokens[5][0]) : "0.0";
    gpsData.ns_indicator = tokens[5][0];
    gpsData.longitude = !tokens[6].isEmpty() ? formatCoordinates(tokens[6], tokens[7][0]) : "0.0";
    gpsData.ew_indicator = tokens[7][0];
    gpsData.date = formatDate(tokens[8]);      // Formatear la fecha
    gpsData.utc_time = formatTime(tokens[9]);  // Formatear la hora
    gpsData.altitude = tokens[10].toFloat();
    //gpsData.speed = tokens[11].toFloat();
    if (!tokens[11].isEmpty()) {
        gpsData.speed = tokens[11].toFloat() * 1.85;
    }
    //gpsData.course = tokens[12].toFloat();
    if (!tokens[12].isEmpty()) {
        float parsedCourse = tokens[12].toFloat();
        gpsData.course = (parsedCourse > 0) ? lastValidCourse = parsedCourse : lastValidCourse;
    } else {
        gpsData.course = lastValidCourse; // Usa el último valor válido si el campo está vacío
    }

    gpsData.pdop = tokens[13].toFloat();
    gpsData.hdop = tokens[14].toFloat();
    gpsData.vdop = tokens[15].toFloat();
    return gpsData;
}
String processResponse(const String& command,  const String& fcommand, const String& response) {
  String state_command = "";
  String processedResponse = response;
  processedResponse.replace(String(command), "");
  //processedResponse.replace("AT+" + fcommand, "");
  processedResponse.replace("+" + fcommand + ": ", "");
  /*Serial.print("Procesando respuesta... ");
  Serial.println(processedResponse);*/

  processedResponse = trimResponse(processedResponse);
  
  if (processedResponse.endsWith("OK")) {
    processedResponse.remove(processedResponse.length() - 2);
    state_command = "OK";
    //Serial.println("Estado del comando: "+state_command);
  } else if (processedResponse.endsWith("ERROR")) {
    processedResponse.remove(processedResponse.length() - 5);
    state_command = "ERROR";
    //Serial.println("Estado del comando: "+state_command);
    return "ERROR COMMAND";
  }

  /*Serial.print("Respuesta procesada: ");
  Serial.println(processedResponse);*/
  if(processedResponse.length() == 0){
    return state_command;
  }
  return processedResponse;
}

String trimResponse(const String& response) {
  String result = response;      // Hacemos una copia modificable de response
  result.replace("\n", "");      // Elimina saltos de línea
  result.replace("\r", "");      // Elimina retornos de carro
  result.trim();                 // Elimina espacios en blanco al inicio y al final
  return result;
}

String formatCoordinates(const String &coord, char direction) {
    int degreesLength = (coord.indexOf('.') > 4) ? 3 : 2; // Verificar si son 2 o 3 dígitos para grados
    int degrees = coord.substring(0, degreesLength).toInt();
    float minutes = coord.substring(degreesLength).toFloat();
    float decimalDegrees = degrees + (minutes / 60.0);

    // Aplicar signo dependiendo de la dirección
    if (direction == 'S' || direction == 'W') {
        decimalDegrees = -decimalDegrees;
    }

    // Formatear el resultado como string con signo explícito
    char buffer[12]; // Tamaño suficiente para contener coordenadas con precisión
    snprintf(buffer, sizeof(buffer), "%+.6f", decimalDegrees); // "%+" añade el signo explícito
    return String(buffer);
}
String formatDate(const String &date) {
    // Formato de entrada: DDMMYY (e.g., 141124)
    // Formato de salida: YYYYMMDD (e.g., 20241121)
    String year = "20" + date.substring(4, 6); // Asume que el año es 20XX
    String month = date.substring(2, 4);
    String day = date.substring(0, 2);
    return year + month + day;
}

String formatTime(const String &utcTime) {
    // Formato de entrada: HHMMSS.s (e.g., 040641.0)
    // Formato de salida: HH:MM:SS (e.g., 04:06:41)
    String hours = utcTime.substring(0, 2);
    String minutes = utcTime.substring(2, 4);
    String seconds = utcTime.substring(4, 6);
    return hours + ":" + minutes + ":" + seconds;
}

bool sendData(const String& message) {
  String cmd = "AT+CIPSEND=1," + String(message.length());
  String response = sendCommandWithResponse(cmd.c_str(), 1000);
  if (response.indexOf(">") != -1) {
    String respServer = sendReadDataToServer("CIPSEND", message, 1000); // Envía el mensaje   
    Serial.println("Respuesta procesada => " + respServer);
    String respCMD = readData(respServer, 1000);
    Serial.println("Comando recibido =>"+respCMD);
  }else {
    Serial.println("Error al enviar mensaje TCP. Intentando reconexión...");
    return false;
  }
    return true;
}

void ignition_event(GPSData gpsData) {
  int StateIgnition = readInput();
  if (StateIgnition == LOW && LaststateIgnition == HIGH) {
    Serial.println("*** ¡ignition ON! **** ");

    event_generated(gpsData, IGN_ON);
    
  }else if(StateIgnition == HIGH && LaststateIgnition == LOW) {
    Serial.println("**** ¡ignition OFF! ***** ");

    event_generated(gpsData, IGN_OFF);
  }
  LaststateIgnition = StateIgnition;
}
void event_generated(GPSData gpsData, int event){

  String data_event = "";
    data_event = "ALT;2049830928;3FFFFF;32;1.0.0;1;"+datetime+";103682809;334;020;40C6;20;"+currentGNSSData.latitude+";"+currentGNSSData.longitude+";"+String(currentGNSSData.speed) + ";" +
            String(currentGNSSData.course) + ";" +String(currentGNSSData.gps_svs)+";"+fix+";"+trackingCourse+"000000"+ignState+";00000000;"+event+";;";
    Serial.println("Event => "+ data_event);
    sendData(data_event);
}
bool readInput() {
    return digitalRead(10);
}
bool initializeModule(){
  String at_cmd = "AT+AT";
  String at = sendCommandWithResponse(at_cmd.c_str(), 4000);
  if(at == "AT"){
    return true;
  }
  Serial.println("Error initializing module!");
  return false;
}
String getDateTime(){
    String dt_cmd = "AT+CCLK?";
    String dt = sendCommandWithResponse(dt_cmd.c_str(), 1000);
    /*Serial.print("getDateTime MS: ");
    Serial.println(dt);*/
    
    return  getFormatUTC(dt);
}
String sendReadDataToServer(const String& fcommand, const String& message, int timeout) {

  sim7600.println(message);  // Enviar comando  
  String response = "";
  long startTime = millis();
  while ((millis() - startTime) < timeout) {
    if (sim7600.available()) {
      char c = sim7600.read();
      response += c;
    }
  }  
  response.replace(message, "");
  response = trimResponse(response);
  
  return response;
}
String readData(String data, int timeout) {
  int startIndex = data.indexOf("CMD");
  if (startIndex == -1) {
    // Si no encuentra "CMD", devuelve una cadena vacía
    return "";
  }
  if(data.substring(startIndex) == "CMD;02049830910;04;01"){
    Serial.println("Activa salida ===========>");
    
  }else if(data.substring(startIndex) == "CMD;02049830910;04;02"){
  
    Serial.println("Desactiva salida ===========>");
  }

  // Extrae desde "CMD" hasta el final de la cadena
  return data.substring(startIndex);  
}
int commandType(const String& command) {
  if (command.endsWith("=?")){
    //Serial.println("Es un comando de prueba (TEST).");
    return TEST;
  }else if (command.endsWith("?") && command.indexOf('=') == -1){
    //Serial.println("Es un comando de lectura (READ).");
    return READ;
  }else if (command.indexOf('=') != -1 && !command.endsWith("?")){ 
    //Serial.println("Es un comando de escritura (WRITE).");
    return WRITE;
  }else if (command.startsWith("AT+") && command.indexOf('=') == -1){
   //Serial.println("Es un comando de ejecución (EXECUTE).");
   return EXECUTE;
  }else if(command.startsWith("STT") ){  
    Serial.println("ES una Cadena de texto (SEND).");
    return SEND;
  }else{ 
    //Serial.println("Tipo de comando desconocido.");
    return UNKNOWN;
  }
}

bool validTcpNet() {
  String netopen_cmd = "AT+NETOPEN?";
  String netopen = sendCommandWithResponse(netopen_cmd.c_str(), 1000);
  Serial.println("Valid TCP => " + netopen);
  if(netopen == "0OK1" || netopen == "1") {
    Serial.println("Servicio TCP Inicializado!");
    return true;
  }else if(netopen == "0"){
    activeTCP();
  }
  return false;
}
void activeTCP() {
    String netactive_cmd = "AT+NETOPEN";
    String netactive = sendCommandWithResponse(netactive_cmd.c_str(), 4000);
    Serial.println("Active TCP => " + netactive);
}
bool getPositionServer() {
  //"0,\"TCP","34.196.135.179\",5200,-1123456789"; // sin configuración: 0123456789
  String cip_st_cmd = "AT+CIPOPEN?";
  String cip_state = sendCommandWithResponse(cip_st_cmd.c_str(), 4000);
  Serial.println("rsp cip_state => "+ cip_state);
  if(cip_state == "0123456789"){
    return false;
  }else if(cip_state.indexOf(",-1") != -1){
    return true;
  }
  return false;
}
bool configureTCP(const String& server, int port) {
  bool stNetOpen;
  String cip_cmd = "AT+CIPOPEN=1,\"TCP\",\"" + server + "\"," +port;
  Serial.println("cmd CIP => " + cip_cmd);
  if(!getPositionServer() ){
    String cip = sendCommandWithResponse(cip_cmd.c_str(), 1000);
    Serial.println("Response CIP => " + cip);
    if(cip != "OK0,0") {// el ,0 indica el indice del servidor
      Serial.println("## NO Conectado al servidor TCP!");
      return false;
    }
  }
  //calbiar la funcion a boil segun la respuesta de "cip" para validar que se configuró y se conecto al servidor tcp
  Serial.println("server tcp connected => ");
  return true;
}

void confiGpsReports(){
  String cgnss = sendCommandWithResponse("AT+CGNSSINFO=1", 4000);  
  Serial.println("+CGNSSINFO => "+ cgnss);
}