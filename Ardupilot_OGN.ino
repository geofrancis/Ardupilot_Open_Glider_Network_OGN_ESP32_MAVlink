// Force the MAVLink library to build using MAVLink 2.0 packet formats
#define MAVLINK_VERSION 2

#include <WiFi.h>
#include <HardwareSerial.h>
#include "mavlink/common/mavlink.h"

// --- ADD THESE PRAGMAS ---
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#include "mavlink/common/mavlink.h"
#pragma GCC diagnostic pop
// -------------------------

// WiFi Credentials
const char* ssid = "2.4";
const char* password = "password";

// OGN Server
const char* host = "aprs.glidernet.org";
const uint16_t port = 14580;

WiFiClient client;
HardwareSerial MAVSerial(2); // UART2 (Default: Pin 16 is RX, Pin 17 is TX)

// Onboard LED Pin for Visual Heartbeat (Most ESP32 boards use Pin 2)
const int LED_PIN = 22;

// MAVLink Network Identity Settings
const uint8_t sys_id = 1;       
const uint8_t comp_id = MAV_COMP_ID_ADSB;      

// Timing Trackers
unsigned long last_heartbeat = 0; 
unsigned long last_ogn_heartbeat = 0;
unsigned long last_led_toggle = 0;
unsigned long last_gps_wait_print = 0;
uint32_t led_blink_interval = 500; // Start with a slow blink (500ms)

// GPS State Variables
bool has_valid_position = false;
float current_lat = 0.0;
float current_lon = 0.0;

void setup() {
  Serial.begin(115200);
  
  // Initialize onboard LED
  pinMode(LED_PIN, OUTPUT);
  
  // Initialize MAVLink Serial telemetry stream
  MAVSerial.begin(57600, SERIAL_8N1, 16, 17);
  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Fast flicker during WiFi connect
  }
  Serial.println("\nConnected to WiFi");
}

void connectToOGN() {
  if (client.connect(host, port)) {
    String loginStr = "user ardupilot pass -1 vers ESP32-OGN 1.0 filter r/";
    loginStr += String(current_lat, 4) + "/" + String(current_lon, 4) + "/100";
    
    client.println(loginStr);
    
    Serial.println("\n[OGN] Connected to server!");
    Serial.print("[OGN] Active Filter: ");
    Serial.println(loginStr);
    
    // Switch LED to a rapid heartbeat (100ms) to signal full operational status
    led_blink_interval = 100; 
  }
}

// Generates and transmits a native MAVLink 2.0 Heartbeat frame
void send_mavlink_heartbeat() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  mavlink_msg_heartbeat_pack(sys_id, comp_id, &msg, 
                             MAV_COMP_ID_ADSB, 
                             MAV_AUTOPILOT_INVALID, 
                             0, 0, 0);
                             
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  MAVSerial.write(buf, len);
}

// Listen for ArduPilot's GPS position
void receive_mavlink_data() {
  mavlink_message_t msg;
  mavlink_status_t status;

  while (MAVSerial.available() > 0) {
    uint8_t c = MAVSerial.read();

    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      if (msg.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
        
        mavlink_global_position_int_t pos;
        mavlink_msg_global_position_int_decode(&msg, &pos);

        float lat = pos.lat / 10000000.0f;
        float lon = pos.lon / 10000000.0f;
        
        if (lat != 0.0 && lon != 0.0) {
          current_lat = lat;
          current_lon = lon;
          
          if (!has_valid_position) {
            has_valid_position = true;
            Serial.println("\n[SUCCESS] GPS Fix Received from ArduPilot!");
            Serial.print("Lat: "); Serial.print(current_lat, 6);
            Serial.print(" | Lon: "); Serial.println(current_lon, 6);
          }
        }
      }
    }
  }
}

uint32_t callsignToICAO(String callsign) {
  uint32_t hash = 0;
  for (unsigned int i = 0; i < callsign.length(); i++) {
    hash = (hash * 31) + callsign[i];
  }
  return hash;
}
void send_mavlink_adsb(uint32_t icao, float lat, float lon, int32_t alt_ft, String callsignStr, uint8_t emitter_type) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  int32_t lat_e7 = (int32_t)(lat * 10000000);
  int32_t lon_e7 = (int32_t)(lon * 10000000);
  int32_t alt_mm = (int32_t)(alt_ft * 304.8);

  char callsign[9] = {0};
  strncpy(callsign, callsignStr.c_str(), 8);

  mavlink_msg_adsb_vehicle_pack(
      sys_id, comp_id, &msg, 
      icao, lat_e7, lon_e7, ADSB_ALTITUDE_TYPE_GEOMETRIC, alt_mm, 
      0, 0, 0, callsign, 
      emitter_type,                  // Dynamically maps aircraft category (e.g., Glider, Rotorcraft)
      1, 0, 0
  );
  
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  MAVSerial.write(buf, len);
}
void loop() {
  // 1. VISUAL HEARTBEAT: Toggle the onboard LED on a timer
  if (millis() - last_led_toggle >= led_blink_interval) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    last_led_toggle = millis();
  }

  // 2. MAVLINK HEARTBEAT: Maintain a steady 1Hz stream to ArduPilot
  if (millis() - last_heartbeat >= 1000) {
    send_mavlink_heartbeat();
    last_heartbeat = millis();
  }

  // 3. TELEMETRY: Process incoming MAVLink to catch the GPS packet
  receive_mavlink_data();

  // 4. BLOCKING GATE: Stop here until ArduPilot sends a valid GPS location
  if (!has_valid_position) {
    led_blink_interval = 500; // Slow blink means "Waiting for GPS"
    if (millis() - last_gps_wait_print >= 5000) {
      Serial.println("[WAITING] Waiting for ArduPilot GPS lock...");
      last_gps_wait_print = millis();
    }
    return; 
  }

  // 5. OGN CONNECTION: Connect/Reconnect if dropped
  if (!client.connected()) {
    connectToOGN();
    delay(2000);
  }

  // 6. OGN SERVER HEARTBEAT: Keep the raw socket alive every 30 seconds
  if (millis() - last_ogn_heartbeat >= 30000) {
    if (client.connected()) {
      client.println("# keep alive"); 
      last_ogn_heartbeat = millis();
    }
  }

  // 7. STREAM PARSING: Process incoming data from the OGN raw stream
  if (client.available()) {
    String line = client.readStringUntil('\n');
    
    if (line.indexOf("/A=") != -1 && line.indexOf(">") != -1) {
      String callsign = line.substring(0, line.indexOf(">"));
      callsign.trim(); 

      // Filter ground stations
      if (callsign.startsWith("PW")) {
        return; 
      }
      
      // Extract Altitude
      int altIndex = line.indexOf("A=");
      String altitude = line.substring(altIndex + 2, altIndex + 8);
      int32_t alt_ft = altitude.toInt();

      // Find the 'h' timestamp marker to isolate coordinates
      int hIndex = line.indexOf('h');
      if (hIndex != -1 && line.length() > hIndex + 19) {
        
        // Isolate coordinate substrings
        String latStr = line.substring(hIndex + 1, hIndex + 9);   
        String lonStr = line.substring(hIndex + 10, hIndex + 19); 
        
        // Parse Aircraft Latitude
        float target_lat = latStr.substring(0, 2).toFloat();
        target_lat += (latStr.substring(2, 7).toFloat() / 60.0f);
        if (latStr.charAt(7) == 'S') target_lat = -target_lat;
        
        // Parse Aircraft Longitude
        float target_lon = lonStr.substring(0, 3).toFloat();
        target_lon += (lonStr.substring(3, 8).toFloat() / 60.0f);
        if (lonStr.charAt(8) == 'W') target_lon = -target_lon;

        // --- EXTRACT & DECODE AIRCRAFT TYPE ---
        String acType = "Unknown";
        uint8_t emitterType = 0; // Default: MAVLink Unknown Type Code
        
        int idIndex = line.indexOf("id");
        if (idIndex != -1 && line.length() > idIndex + 4) {
          String xxStr = line.substring(idIndex + 2, idIndex + 4);
          uint8_t xx = (uint8_t)strtoul(xxStr.c_str(), NULL, 16);
          uint8_t typeNum = (xx >> 2) & 0x0F; // Extract bits 2,3,4,5
          
          switch(typeNum) {
            case 1:  acType = "Glider";          emitterType = 9;  break; // ADSB_EMITTER_TYPE_GLIDER
            case 2:  acType = "Tow Plane";       emitterType = 1;  break; // ADSB_EMITTER_TYPE_LIGHT
            case 3:  acType = "Helicopter";      emitterType = 7;  break; // ADSB_EMITTER_TYPE_ROTORCRAFT
            case 4:  acType = "Parachutist";     emitterType = 11; break; // ADSB_EMITTER_TYPE_PARACHUTIST
            case 5:  acType = "Drop Plane";      emitterType = 1;  break; 
            case 6:  acType = "Hang Glider";     emitterType = 12; break; // ADSB_EMITTER_TYPE_ULTRA_LIGHT
            case 7:  acType = "Paraglider";      emitterType = 12; break; 
            case 8:  acType = "Powered GA";      emitterType = 1;  break; 
            case 9:  acType = "Jet/Turboprop";   emitterType = 6;  break; // ADSB_EMITTER_TYPE_HIGH_PERFORMANCE
            case 11: acType = "Balloon";         emitterType = 10; break; // ADSB_EMITTER_TYPE_LIGHTER_THAN_AIR
            case 12: acType = "Airship";         emitterType = 10; break; 
            case 13: acType = "UAV/Drone";       emitterType = 14; break; // ADSB_EMITTER_TYPE_UAV
            case 15: acType = "Obstacle";        emitterType = 0;  break;
            default: acType = "Unknown";         emitterType = 0;  break;
          }
        }

        // Output matching metadata seamlessly to the terminal
        Serial.print("Mavlink Send -> Type: [");
        Serial.print(acType);
        Serial.print("] Callsign: ");
        Serial.print(callsign);
        Serial.print(" | Pos: "); Serial.print(target_lat, 5);
        Serial.print(", "); Serial.print(target_lon, 5);
        Serial.print(" | Alt: "); Serial.print(alt_ft);
        Serial.println(" ft");

        // Stream parsed attributes to the flight controller
        uint32_t numeric_icao = callsignToICAO(callsign);
        send_mavlink_adsb(numeric_icao, target_lat, target_lon, alt_ft, callsign, emitterType);
      }
    }
  }
}
