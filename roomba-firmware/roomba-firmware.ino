#include <Arduino.h>
#include <WiFi.h>
#include <esp_task_wdt.h>
#include <netdb.h>
#include <netinet/in.h>
#include <string.h>
#include <sys/socket.h>
// For v_task_delay
#include <freertos/task.h>

#define LEFT_MOTOR_A 18
#define LEFT_MOTOR_B 19

#define RIGHT_MOTOR_A 5
#define RIGHT_MOTOR_B 23

#define LEFT_MOTOR_ENCODER_A 16
#define LEFT_MOTOR_ENCODER_B 17

#define RIGHT_MOTOR_ENCODER_A 14
#define RIGHT_MOTOR_ENCODER_B 27

#define RADIUS_WHEEL 3       // cm
#define DISTANCE_WHEELS 11.0 // cm
#define TICKS_PER_REV 695.1

#define TRIG 13
#define CENTER_ECHO 12
#define LEFT_ECHO 26
#define RIGHT_ECHO 25

volatile int left_encoder_count = 0;
volatile int right_encoder_count = 0;

int last_left_encoder_count = 0;
int last_right_encoder_count = 0;

int last_read_time = 0;

double x, y, theta;

int udp_sock;

// Task handles
TaskHandle_t encoderTaskHandle;
TaskHandle_t udpTaskHandle;

// Task functions
void encoderTask(void *parameter);
void udpTask(void *parameter);

// Interrupt handlers
void IRAM_ATTR left_encoder_interrupt() {
  if (digitalRead(LEFT_MOTOR_ENCODER_B)) {
    left_encoder_count++;
  } else {
    left_encoder_count--;
  }
}

void IRAM_ATTR right_encoder_interrupt() {
  if (digitalRead(RIGHT_MOTOR_ENCODER_B)) {
    right_encoder_count++;
  } else {
    right_encoder_count--;
  }
}

void setup() {
  Serial.begin(115200);
  // Connect to WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin("home", "parola@home34");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  // Create UDP socket
  udp_sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (udp_sock < 0) {
    return;
  }

  // Bind to port 5000
  struct sockaddr_in server_addr;
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(5001);
  server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  int err =
      bind(udp_sock, (struct sockaddr *)&server_addr, sizeof(server_addr));
  if (err < 0) {
    return;
  }

  // Set up the motor pins
  pinMode(LEFT_MOTOR_A, OUTPUT);
  pinMode(LEFT_MOTOR_B, OUTPUT);
  pinMode(RIGHT_MOTOR_A, OUTPUT);
  pinMode(RIGHT_MOTOR_B, OUTPUT);

  // Set up the encoder pins
  pinMode(LEFT_MOTOR_ENCODER_A, INPUT);
  pinMode(LEFT_MOTOR_ENCODER_B, INPUT);
  pinMode(RIGHT_MOTOR_ENCODER_A, INPUT);
  pinMode(RIGHT_MOTOR_ENCODER_B, INPUT);

  // Set up the ultrasonic sensor pins
  pinMode(TRIG, OUTPUT);
  pinMode(CENTER_ECHO, INPUT);
  pinMode(LEFT_ECHO, INPUT);
  pinMode(RIGHT_ECHO, INPUT);

  attachInterrupt(digitalPinToInterrupt(LEFT_MOTOR_ENCODER_A),
                  left_encoder_interrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_MOTOR_ENCODER_A),
                  right_encoder_interrupt, RISING);

  // Create encoder task on core 0
  xTaskCreatePinnedToCore(encoderTask, "EncoderTask", 10000, NULL, 1,
                          &encoderTaskHandle, 0);

  // Create UDP task on core 1
  xTaskCreatePinnedToCore(udpTask, "UDPTask", 10000, NULL, 1, &udpTaskHandle,
                          1);
}

void loop() {
  // Do nothing in the main loop
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}

// Encoder task: Handles encoder reading and location updating
void encoderTask(void *parameter) {
  while (true) {
    int left_encoder_diff = left_encoder_count - last_left_encoder_count;
    int right_encoder_diff = right_encoder_count - last_right_encoder_count;

    last_left_encoder_count = left_encoder_count;
    last_right_encoder_count = right_encoder_count;

    double left_distance =
        (left_encoder_diff / TICKS_PER_REV) * 2 * M_PI * RADIUS_WHEEL;

    double right_distance =
        (right_encoder_diff / TICKS_PER_REV) * 2 * M_PI * RADIUS_WHEEL;

    double distance = (left_distance + right_distance) / 2.0;

    double delta_theta = (right_distance - left_distance) / DISTANCE_WHEELS;

    x += distance * cos(theta + delta_theta / 2.0);
    y += distance * sin(theta + delta_theta / 2.0);
    theta += delta_theta;

    vTaskDelay(10);
  }
}

// Each UDP package contains the information:
// x y theta distance_to_obstacle sensor_identifier (0 = center, 1 = left, 2 =
// right)

// UDP task: Handles UDP communication
void udpTask(void *parameter) {
  char buffer[1024];

  // Broadcast a packet saying "Hello from Roomba"
  struct sockaddr_in broadcast_addr;
  broadcast_addr.sin_family = AF_INET;
  broadcast_addr.sin_port = htons(5000);
  broadcast_addr.sin_addr.s_addr = htonl(INADDR_BROADCAST);
  sendto(udp_sock, "Hello from Roomba", 17, 0,
         (struct sockaddr *)&broadcast_addr, sizeof(broadcast_addr));

  struct sockaddr_in client_addr;
  socklen_t client_addr_len = sizeof(client_addr);

  // Set socket to non-blocking mode
  int flags = fcntl(udp_sock, F_GETFL, 0);
  fcntl(udp_sock, F_SETFL, flags | O_NONBLOCK);

  int read_sensor = 0;

  while (true) {
    // Receive a UDP packet
    int ret = recvfrom(udp_sock, buffer, 1024, 0,
                       (struct sockaddr *)&client_addr, &client_addr_len);

    while (ret > 0) {
      buffer[ret] = '\0';

      // Parse the speed of the left and right motors
      int left_speed, right_speed;
      sscanf(buffer, "%d %d", &left_speed, &right_speed);

      if (left_speed > 0) {
        analogWrite(LEFT_MOTOR_A, left_speed);
        analogWrite(LEFT_MOTOR_B, 0);
      } else {
        analogWrite(LEFT_MOTOR_A, 0);
        analogWrite(LEFT_MOTOR_B, -left_speed);
      }

      if (right_speed > 0) {
        analogWrite(RIGHT_MOTOR_A, right_speed);
        analogWrite(RIGHT_MOTOR_B, 0);
      } else {
        analogWrite(RIGHT_MOTOR_A, 0);
        analogWrite(RIGHT_MOTOR_B, -right_speed);
      }

      ret = recvfrom(udp_sock, buffer, 1024, 0, (struct sockaddr *)&client_addr,
                     &client_addr_len);
    }

    // Send a package every 0.1 seconds

    if (millis() - last_read_time > 100) {
      // Alternate which sensor is read each time
      read_sensor = (read_sensor + 1) % 3;
      int distance = 0;

      // Send the pulse
      digitalWrite(TRIG, LOW);
      delayMicroseconds(2);
      digitalWrite(TRIG, HIGH);
      delayMicroseconds(10);
      digitalWrite(TRIG, LOW);

      // Read the echo
      if (read_sensor == 0) {
        distance = pulseIn(CENTER_ECHO, HIGH) / 58;
      } else if (read_sensor == 1) {
        distance = pulseIn(LEFT_ECHO, HIGH) / 58;
      } else {
        distance = pulseIn(RIGHT_ECHO, HIGH) / 58;
      }

      // Send the UDP packet
      sprintf(buffer, "%f %f %f %d %d", x, y, theta, distance, read_sensor);
      sendto(udp_sock, buffer, strlen(buffer), 0,
             (struct sockaddr *)&broadcast_addr, sizeof(broadcast_addr));

      last_read_time = millis();
    }
    vTaskDelay(10);
  }
}
