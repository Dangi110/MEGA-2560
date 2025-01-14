#include <Arduino.h>

struct pms5003data {
    uint16_t framelen;
    uint16_t pm10_standard, pm25_standard, pm100_standard;
    uint16_t pm10_env, pm25_env, pm100_env;
    uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;
    uint16_t unused;
    uint16_t checksum;
};

pms5003data data1;  // Data for first sensor
pms5003data data2;  // Data for second sensor

// Updated empirical mass factors for smoke particles (\u03bcg/particle)
const float massFactor1um = 0.000000785;   // 1 \u03bcm particles
const float massFactor3um = 5.65;    // 3 \u03bcm particles
const float massFactor5um = 7.85;    // 5 \u03bcm particles
const float massFactor10um = 62.9;   // 10 \u03bcm particles

void setup() {
    Serial.begin(115200);
    Serial1.begin(9600);  // First sensor connected to Serial1
    Serial2.begin(9600);  // Second sensor connected to Serial2
    Serial.println("PMS5003 Smoke Particle Test Started");
}

void loop() {
    if (readPMSdata(&Serial1, &data1)) {
        Serial.println("---- PMS5003 Sensor 1 Data ----");
        printSensorData(data1);
        printMassConcentration(data1);  // Print mass concentration for Sensor 1
    }

    if (readPMSdata(&Serial2, &data2)) {
        Serial.println("---- PMS5003 Sensor 2 Data ----");
        printSensorData(data2);
        printMassConcentration(data2);  // Print mass concentration for Sensor 2
    }

    delay(500);  // Wait for the next data packet
}

boolean readPMSdata(Stream *s, pms5003data *sensorData) {
    if (s->available() < 32) {
        return false;
    }

    if (s->peek() != 0x42) {
        s->read();
        return false;
    }

    uint8_t buffer[32];
    s->readBytes(buffer, 32);

    if (buffer[0] != 0x42 || buffer[1] != 0x4D) {
        Serial.println("Invalid start bytes");
        return false;
    }

    uint16_t sum = 0;
    for (uint8_t i = 0; i < 30; i++) {
        sum += buffer[i];
    }

    uint16_t received_checksum = (buffer[30] << 8) | buffer[31];

    if (sum != received_checksum) {
        Serial.print("Checksum error! Calculated: ");
        Serial.print(sum, HEX);
        Serial.print(" Received: ");
        Serial.println(received_checksum, HEX);
        return false;
    }

    memcpy((void *)sensorData, buffer + 2, sizeof(pms5003data));
    return true;
}

void printSensorData(pms5003data &data) {
    Serial.print("Particles > 0.3\u03bcm: "); Serial.println(data.particles_03um);
    Serial.print("Particles > 0.5\u03bcm: "); Serial.println(data.particles_05um);
    Serial.print("Particles > 1.0\u03bcm: "); Serial.println(data.particles_10um);
    Serial.print("Particles > 2.5\u03bcm: "); Serial.println(data.particles_25um);
    Serial.print("Particles > 5.0\u03bcm: "); Serial.println(data.particles_50um);
    Serial.print("Particles > 10.0\u03bcm: "); Serial.println(data.particles_100um);
    Serial.println("----------------------");
}

void printMassConcentration(pms5003data &data) {
    // Estimate mass concentration for each fraction (\u03bcg/m\u00b3)
    float mass1um = data.particles_10um * massFactor1um;  // 1 \u03bcm particles
    float mass3um = data.particles_25um * massFactor3um;  // 3 \u03bcm particles
    float mass5um = data.particles_50um * massFactor5um;  // 5 \u03bcm particles
    float mass10um = data.particles_100um * massFactor10um; // 10 \u03bcm particles

    // Print estimated mass concentrations
    Serial.print("Mass Concentration (PM1.0): "); Serial.print(mass1um); Serial.println(" \u03bcg/m\u00b3");
    Serial.print("Mass Concentration (PM3.0): "); Serial.print(mass3um); Serial.println(" \u03bcg/m\u00b3");
    Serial.print("Mass Concentration (PM5.0): "); Serial.print(mass5um); Serial.println(" \u03bcg/m\u00b3");
    Serial.print("Mass Concentration (PM10): "); Serial.print(mass10um); Serial.println(" \u03bcg/m\u00b3");
    Serial.println("____________________________________________________________________________");
}
