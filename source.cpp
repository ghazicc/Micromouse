#include <Wire.h>
#include <MPU6050_light.h>
#include "Adafruit_VL53L0X.h"
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <stdlib.h>
#include <math.h>

// Constants based on your hardware
const float TICKS_PER_REV = 70.0;
const float WHEEL_RADIUS_CM = 2.5;
const float WHEEL_CIRCUMFERENCE = 14;                           // ≈18.85 cm
const float CM_PER_TICK = WHEEL_CIRCUMFERENCE / TICKS_PER_REV;  // ~0.27 cm/tick


int prevMoveError = 0;
float targetAngle = 0;
int currentStepTargetDistance = 0;

#define MOTOR_LEFT_ENC_IN_1 17
#define MOTOR_LEFT_ENC_IN_2 18
#define MAZE_SIZE 8
#define MOTOR_RIGHT_ENC_IN_1 41
#define MOTOR_RIGHT_ENC_IN_2 40

// Define the XSHUT pins for each VL53L0X sensor
#define SHT_LEFT 11   // XSHUT pin for left sensor
#define SHT_FRONT 12  // XSHUT pin for front sensor
#define SHT_RIGHT 13  // XSHUT pin for right sensor

// Define unique I2C addresses for each VL53L0X sensor
#define FRONT_ADDRESS 0x30
#define LEFT_ADDRESS 0x31
#define RIGHT_ADDRESS 0x32

// Define Directions
#define LEFT_DIR 1
#define RIGHT_DIR -1

#define MPU_UPDATE_TIME 5

// #define SPEED_DIFFERENCE 12
#define SPEED_RATIO 170.0/155.0

// Define MPU6050 object
MPU6050 mpu(Wire);

// Create VL53L0X sensor objects
Adafruit_VL53L0X loxFront = Adafruit_VL53L0X();
Adafruit_VL53L0X loxLeft = Adafruit_VL53L0X();
Adafruit_VL53L0X loxRight = Adafruit_VL53L0X();

// Variables to hold VL53L0X measurements
VL53L0X_RangingMeasurementData_t measureFront;
VL53L0X_RangingMeasurementData_t measureLeft;
VL53L0X_RangingMeasurementData_t measureRight;

int leftMotorM1 = 5;
int leftMotorM2 = 4;
int leftMotorSpeedPin = 6;


int rightMotorM1 = 1;
int rightMotorM2 = 2;
int rightMotorSpeedPin = 42;

volatile long leftMotorEncoderCount = 0;
volatile long rightMotorEncoderCount = 0;

volatile double yawAngle;

// Set these to your desired credentials.
const char* ssid = "ESP32BWALL";
const char* password = "11112222";

// Web server and WebSocket server
WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);

bool robotStarted = false;
bool moveForwardState = false;
unsigned long lastTurnTime = 0;
int turnDirection = 1;  // 1 for left, -1 for right

float currentAngle;
float angleControl;

float currentDistance;
float distanceControl;

bool isLastMoveTurn = 1;



double initYawAngle = 0;

typedef struct PID {
  float Kp;           // Proportional gain
  float Ki;           // Integral gain
  float Kd;           // Derivative gain
  int previousValue;  // Previous error for derivative calculation
  float integral;     // Accumulated error for integral term
  float maxIntegral;  // Maximum value for integral term (anti-windup)
} PID;

typedef struct {
  short int x;
  short int y;
  short int heading;
} Mouse;

typedef struct {
  short int x;
  short int y;
} Cell;

typedef enum {
  FRONT = 0,
  LEFT = 1,
  BEHIND = 2,
  RIGHT = 3
} Heading;

typedef struct Node {
  Cell cell;
  struct Node* next;
} Node;

typedef struct {
  Node* front;
  Node* rear;
} Queue;

byte dp_matrix[MAZE_SIZE][MAZE_SIZE];
byte vertical_barrier[MAZE_SIZE - 1][MAZE_SIZE] = { 0 };
byte horizontal_barrier[MAZE_SIZE][MAZE_SIZE - 1] = { 0 };

Mouse mouse = { 0, 0, FRONT };


void fill_dp_matrix();
void print_final_dp_matrix();
void navigate();
Cell get_min_cell(Cell adj_cells[], int size);
void get_neighbors(short int x, short int y, Cell* cells, int* size);
void change_direction(Cell min_cell);
void update_walls();
void flood_fill();
void enqueue(Queue* queue, Cell cell);
Cell dequeue(Queue* queue);
int is_queue_empty(Queue* queue);

int targetDir;

int rightMotorSpeed;
int leftMotorSpeed;

int rightMotorPrevSpeed;
int leftMotorPrevSpeed;

// Initialize PID controllers
PID distanceController = { 1.0, 0.01, 1, 0, 0, 1000 };  // Kp, Ki, Kd, previousValue, integral, maxIntegral
PID angleController = { 10.0, 0.01, 1, 0, 0, 1000 };     // Kp, Ki, Kd, previousValue, integral, maxIntegral
PID turnController = {1.5, 0.01, 1, 0, 0, 1000};

const int TARGET_ENCODER_COUNTS = 97;  //20.0/14 * 70;


// Fuzzy PID Structure
typedef struct {
  // Input membership parameters
  float error_terms[3];    // [Negative, Zero, Positive]
  float delta_error_terms[3];
  
  // Output PID adjustments
  float Kp_adjustments[3][3];
  float Ki_adjustments[3][3];
  float Kd_adjustments[3][3];
} FuzzyPID;

// Updated Fuzzy PID parameters for distance control
FuzzyPID distanceFuzzyPID = {
  // Error terms (encoder ticks) - based on target of 97 ticks
  {0, 50, 90},        
  // Delta error terms (ticks/cycle) - 20ms update rate
  {0, 30, 60},        
  // Kp adjustments (±0.1 around base Kp=0.2)
  {{0.6,  0.5,  0.4},   // Larger Kd for fast changes
   {0.5,  0.4,  0.3},    // Moderate Kd for medium changes
   {0.4,  0.3,  0.2}},    // Smaller Kd for slow changes
  // Ki adjustments (±0.002 around base Ki=0.01)
  {{0.003, 0.002, 0.001}, // Larger Ki for steady-state errors
   {0.002, 0.001, 0.000}, // Moderate Ki for medium errors
   {0.001, 0.000, -0.001}}, // Smaller Ki for small errors
  // Kd adjustments (±0.5 around base Kd=1)
  {{0.6,  0.5,  0.4},   // Larger Kd for fast changes
   {0.5,  0.4,  0.3},    // Moderate Kd for medium changes
   {0.4,  0.3,  0.2}}    // Smaller Kd for slow changes
};

FuzzyPID angleFuzzyPID = {
  // Error terms (degrees) - typical heading errors during movement
  {-30, 0, 30},     
  // Delta error terms (degrees/s) - based on gyro measurements
  {-150, 0, 150},   
  // Kp adjustments (±5 around base Kp=15)
  {{5.0,  3.0,  1.0},   // Larger Kp for large errors
   {3.0,  1.0, -1.0},   // Moderate Kp for medium errors
   {1.0, -1.0, -3.0}},  // Smaller Kp for small errors
  // Ki adjustments (±0.005 around base Ki=0.01)
  {{0.006, 0.004, 0.002}, // Larger Ki for steady-state errors
   {0.004, 0.002, 0.000}, // Moderate Ki for medium errors
   {0.002, 0.000, -0.002}}, // Smaller Ki for small errors
  // Kd adjustments (±0.5 around base Kd=1)
  {{0.6,  0.5,  0.4},   // Larger Kd for fast changes
   {0.5,  0.4,  0.3},   // Moderate Kd for medium changes
   {0.4,  0.3,  0.2}}   // Smaller Kd for slow changes
};


FuzzyPID turnFuzzyPID = {
  // Error terms (degrees) - typical heading errors during movement
  {0, 30, 45},     
  // Delta error terms (degrees/s) - based on gyro measurements
  {-200, 0, 200},   
  // Kp adjustments (±5 around base Kp=15)
  {{5.0,  3.0,  1.0},   // Larger Kp for large errors
   {3.0,  1.0, -1.0},   // Moderate Kp for medium errors
   {1.0, -1.0, -3.0}},  // Smaller Kp for small errors
  // Ki adjustments (±0.005 around base Ki=0.01)
  {{0.006, 0.004, 0.002}, // Larger Ki for steady-state errors
   {0.004, 0.002, 0.000}, // Moderate Ki for medium errors
   {0.002, 0.000, -0.002}}, // Smaller Ki for small errors
  // Kd adjustments (±0.5 around base Kd=1)
  {{0.6,  0.5,  0.4},   // Larger Kd for fast changes
   {0.5,  0.4,  0.3},   // Moderate Kd for medium changes
   {0.4,  0.3,  0.2}}   // Smaller Kd for slow changes
};


// Add near other global variables
bool floodFillButtons = false;
bool byHand = false;

void calculateMembership(float value, float terms[3], float membership[3]) {
  if (value <= terms[0]) {
    membership[0] = 1; // Fully in Negative region
  } else if (value < terms[1]) {
    membership[0] = (terms[1] - value) / (terms[1] - terms[0]);
    membership[1] = 1 - membership[0]; // Partially in Zero region
  } else if (value == terms[1]) {
    membership[1] = 1; // Fully in Zero region
  } else if (value < terms[2]) {
    membership[1] = (terms[2] - value) / (terms[2] - terms[1]);
    membership[2] = 1 - membership[1]; // Partially in Positive region
  } else {
    membership[2] = 1; // Fully in Positive region
  }
}

void fuzzyAdjustPID(float error, float delta_error, FuzzyPID *fuzzy, PID *pid) {
  // Calculate membership degrees
  float e_membership[3] = {0};
  float de_membership[3] = {0};
  calculateMembership(error, fuzzy->error_terms, e_membership);
  calculateMembership(delta_error, fuzzy->delta_error_terms, de_membership);

  // Rule evaluation and defuzzification
  float kp_adj = 0, ki_adj = 0, kd_adj = 0;
  float total_strength = 0.001; // Avoid division by zero

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      float strength = fmin(e_membership[i], de_membership[j]);
      kp_adj += fuzzy->Kp_adjustments[i][j] * strength;
      ki_adj += fuzzy->Ki_adjustments[i][j] * strength;
      kd_adj += fuzzy->Kd_adjustments[i][j] * strength;
      total_strength += strength;
    }
  }

    if (pid == &angleController) {
      pid->Kp = constrain(pid->Kp + (kp_adj / total_strength), 7.0, 13.0);     // Base Kp=15
      pid->Ki = constrain(pid->Ki + (ki_adj / total_strength), 0.005, 0.015); // Base Ki=0.01
      pid->Kd = constrain(pid->Kd + (kd_adj / total_strength), 0.5, 1.5);   // Base Kd=1
  } else if (pid == &distanceController) {
      pid->Kp = constrain(pid->Kp + (kp_adj / total_strength), 0.5, 2.0);   // Base Kp=0.2
      pid->Ki = constrain(pid->Ki + (ki_adj / total_strength), 0.008, 0.012); // Base Ki=0.01
      pid->Kd = constrain(pid->Kd + (kd_adj / total_strength), 0.5, 1.5);   // Base Kd=1
  } else if (pid == &turnController) {
      pid->Kp = constrain(pid->Kp + (kp_adj / total_strength), 0.5, 2.0);   // Base Kp=0.2
      pid->Ki = constrain(pid->Ki + (ki_adj / total_strength), 0.008, 0.012); // Base Ki=0.01
      pid->Kd = constrain(pid->Kd + (kd_adj / total_strength), 0.5, 1.5);   // Base Kd=1
  }
}

void setup() {
  // Setup Serial Monitor
  Serial.begin(115200);


  rightMotorEncoderCount = 0;
  leftMotorEncoderCount = 0;
  ledcAttachChannel(rightMotorSpeedPin, freq, resolution, pwmChannelA);
  ledcAttachChannel(leftMotorSpeedPin, freq, resolution, pwmChannelB);
  // Start Wi-Fi AP
  if (!WiFi.softAP(ssid, password)) {
    log_e("Soft AP creation failed.");
    while (1)
      ;
  }
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);

  // Start Web server
  server.on("/", handleRoot);
  server.on("/turn", handleTurn);  // Add handler for turn commands

  // Route for the "Move Forward" button
  server.on("/moveForward", []() {
    moveForwardState = true;  // Call the moveForward() function
    Serial.print("SETTING MOVE FORWARD TRUE");
    server.send(200, "text/plain", "Moving forward");
  });

  server.on("/turnRight", []() {
    turn(-1);  // Call the moveForward() function
    server.send(200, "text/plain", "Turning Right");
  });

  server.on("/turnLeft", []() {
    turn(1);  // Call the moveForward() function
    server.send(200, "text/plain", "Turning Left");
  });

  // Route for updating PID parameters
  server.on("/updatePIDs", []() {
    if (server.hasArg("kp_distance") && server.hasArg("ki_distance") && server.hasArg("kd_distance") && server.hasArg("kp_angle") && server.hasArg("ki_angle") && server.hasArg("kd_angle")) {

      float kp_distance = server.arg("kp_distance").toFloat();
      float ki_distance = server.arg("ki_distance").toFloat();
      float kd_distance = server.arg("kd_distance").toFloat();

      float kp_angle = server.arg("kp_angle").toFloat();
      float ki_angle = server.arg("ki_angle").toFloat();
      float kd_angle = server.arg("kd_angle").toFloat();

      distanceController.Kp = kp_distance;
      distanceController.Ki = ki_distance;
      distanceController.Kd = kd_distance;

      angleController.Kp = kp_angle;
      angleController.Ki = ki_angle;
      angleController.Kd = kd_angle;

      server.send(200, "text/plain", "Distance PID updated: Kp=" + String(kp_distance) + ", Ki=" + String(ki_distance) + ", Kd=" + String(kd_distance) + "\nAngle PID updated: Kp=" + String(kp_angle) + ", Ki=" + String(ki_angle) + ", Kd=" + String(kd_angle));
    } else {
      server.send(400, "text/plain", "Invalid parameters");
    }
  });

  // Add in setup() with other server.on() calls
  server.on("/toggleFloodFillButtons", []() {
    floodFillButtons = !floodFillButtons;
    server.send(200, "text/plain", floodFillButtons ? "Flood Fill Buttons: ON" : "Flood Fill Buttons: OFF");
  });

  server.on("/toggleByHand", []() {
    byHand = !byHand;
    server.send(200, "text/plain", byHand ? "By Hand: ON" : "By Hand: OFF");
  });

  server.begin();

  // Start WebSocket server
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  // Set encoder as input with internal pullup
  pinMode(MOTOR_LEFT_ENC_IN_1, INPUT_PULLUP);
  pinMode(MOTOR_LEFT_ENC_IN_2, INPUT_PULLUP);
  pinMode(MOTOR_RIGHT_ENC_IN_1, INPUT_PULLUP);
  pinMode(MOTOR_RIGHT_ENC_IN_2, INPUT_PULLUP);

  // Set PWM and DIR connections as outputs
  pinMode(leftMotorM1, OUTPUT);
  pinMode(leftMotorM2, OUTPUT);
  pinMode(rightMotorM1, OUTPUT);
  pinMode(rightMotorM2, OUTPUT);
  pinMode(leftMotorSpeedPin, OUTPUT);
  pinMode(rightMotorSpeedPin, OUTPUT);

  ledcWrite(rightMotorSpeedPin, 0);
  ledcWrite(leftMotorSpeedPin, 0);

  digitalWrite(rightMotorM1, LOW);
  digitalWrite(rightMotorM2, HIGH);
  digitalWrite(leftMotorM1, LOW);
  digitalWrite(leftMotorM2, HIGH);

  // Attach interrupt
  attachInterrupt(digitalPinToInterrupt(MOTOR_LEFT_ENC_IN_1), updateLeftMotorEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(MOTOR_RIGHT_ENC_IN_1), updateRightMotorEncoder, RISING);

  // Configure XSHUT pins as outputs
  pinMode(SHT_FRONT, OUTPUT);
  pinMode(SHT_LEFT, OUTPUT);
  pinMode(SHT_RIGHT, OUTPUT);

  fill_dp_matrix();


  // Initialize VL53L0X sensors with unique addresses
  setVL53L0XIDs();
  initializeMPU6050();
  Serial.println(TARGET_ENCODER_COUNTS);
}

void sendDataToConnectedClients() {
  server.handleClient();  // Handle HTTP requests
  webSocket.loop();       // Handle WebSocket events

  // Send other data as before
  String data = "Left motor encoder: " + String(leftMotorEncoderCount) + "\n";
  data += "Right motor encoder: " + String(rightMotorEncoderCount) + "\n";
  data += "Right motor speed: " + String(rightMotorSpeed) + "\n";
  data += "Left motor speed: " + String(leftMotorSpeed) + "\n";
  data += "Front (mm): " + String(readFrontDistance()) + "\n";
  data += "Left (mm): " + String(readLeftDistance()) + "\n";
  data += "Right (mm): " + String(readRightDistance()) + "\n";
  data += "tX : " + String(mpu.getAngleX()) + "\n";
  data += "tY : " + String(mpu.getAngleY()) + "\n";
  data += "tZ : " + String(mpu.getAngleZ()) + "\n";
  data += "Init Yaw Angle: " + String(initYawAngle) + "\n";

  webSocket.broadcastTXT(data);
}



int cnt = 2;
int currentTime = 0;
int prev = 0;
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;
void loop() {


  // Send data to all connected WebSocket clients
  sendDataToConnectedClients();

  mpu.update();

  if (moveForwardState){
    moveForward();
    Serial.print("moveFrowardtrue\n");
  }

  if (robotStarted) {
    while (millis() - currentTime < 1000);
    if (dp_matrix[mouse.x][mouse.y] != 0) {
      update_walls();
      Cell current_cell = { mouse.x, mouse.y };
      Cell accessible_cells[4];
      int size = 0;
      get_neighbors(mouse.x, mouse.y, accessible_cells, &size);
      Cell min_accessible_cell = get_min_cell(accessible_cells, size);

      if (dp_matrix[current_cell.x][current_cell.y] > dp_matrix[min_accessible_cell.x][min_accessible_cell.y]) {
        change_direction(min_accessible_cell);
        while (millis() - currentTime < 1000);
        moveForward();
        mouse.x = min_accessible_cell.x;
        mouse.y = min_accessible_cell.y;
      } else {
        flood_fill();
      }
      if (floodFillButtons) {
        robotStarted = false;
    // Perform flood fill button actions
    }
    }
  }
}





void fill_dp_matrix() {
  int center = (MAZE_SIZE - 1) / 2;
  char text[20];
  for (int i = 0; i < MAZE_SIZE; i++) {
    for (int j = 0; j < MAZE_SIZE; j++) {
      int dist = abs(i - center) + abs(j - center);
      if (i >= center + 1) dist--;
      if (j >= center + 1) dist--;
      dp_matrix[i][j] = dist;
    }
  }
}

void flood_fill() {
  Queue queue = { NULL, NULL };
  Cell current_cell = { mouse.x, mouse.y };

  if (dp_matrix[mouse.x][mouse.y] == 0)
    return;

  enqueue(&queue, current_cell);
  while (!is_queue_empty(&queue)) {
    Cell considered_cell = dequeue(&queue);
    Cell accessible_cells[4];
    int size = 0;
    get_neighbors(considered_cell.x, considered_cell.y, accessible_cells, &size);
    Cell min_element = get_min_cell(accessible_cells, size);

    if (dp_matrix[considered_cell.x][considered_cell.y] - 1 != dp_matrix[min_element.x][min_element.y]) {
      int min_neighbour = dp_matrix[min_element.x][min_element.y];
      dp_matrix[considered_cell.x][considered_cell.y] = min_neighbour + 1;
      for (int i = 0; i < size; i++) {
        enqueue(&queue, accessible_cells[i]);
      }
    }
  }
}



void update_walls() {
  while (millis() - currentTime < 1000)
    ;
  bool frontWall = wallFront();
  bool leftWall = wallLeft();
  bool rightWall = wallRight();


  Serial.print("frontWall = ");
  Serial.print(frontWall);
  Serial.print("  ||  leftWall = ");
  Serial.print(leftWall);
  Serial.print("  ||  rightWall = ");
  Serial.println(rightWall);

  if (mouse.heading == FRONT) {
    if (mouse.x > 0 && leftWall)
      vertical_barrier[mouse.x - 1][mouse.y] = 1;
    if (mouse.x < MAZE_SIZE - 1 && rightWall)
      vertical_barrier[mouse.x][mouse.y] = 1;
    if (mouse.y < MAZE_SIZE - 1 && frontWall)
      horizontal_barrier[mouse.x][mouse.y] = 1;
  } else if (mouse.heading == LEFT) {
    if (mouse.x < MAZE_SIZE - 1 && frontWall)
      vertical_barrier[mouse.x][mouse.y] = 1;
    if (mouse.y > 0 && rightWall)
      horizontal_barrier[mouse.x][mouse.y - 1] = 1;
    if (mouse.y < MAZE_SIZE - 1 && leftWall)
      horizontal_barrier[mouse.x][mouse.y] = 1;
  } else if (mouse.heading == BEHIND) {
    if (mouse.x > 0 && rightWall)
      vertical_barrier[mouse.x - 1][mouse.y] = 1;
    if (mouse.x < MAZE_SIZE - 1 && leftWall)
      vertical_barrier[mouse.x][mouse.y] = 1;
    if (mouse.y > 0 && frontWall)
      horizontal_barrier[mouse.x][mouse.y - 1] = 1;
  } else if (mouse.heading == RIGHT) {
    if (mouse.x > 0 && frontWall)
      vertical_barrier[mouse.x - 1][mouse.y] = 1;
    if (mouse.y > 0 && leftWall)
      horizontal_barrier[mouse.x][mouse.y - 1] = 1;
    if (mouse.y < MAZE_SIZE - 1 && rightWall)
      horizontal_barrier[mouse.x][mouse.y] = 1;
  }

  while (millis() - currentTime < 1000)
    ;
}

void get_neighbors(short int x, short int y, Cell* cells, int* size) {
  if (x > 0 && !vertical_barrier[x - 1][y]) {
    cells[*size].x = x - 1;
    cells[*size].y = y;
    (*size)++;
  }
  if (x < MAZE_SIZE - 1 && !vertical_barrier[x][y]) {
    cells[*size].x = x + 1;
    cells[*size].y = y;
    (*size)++;
  }
  if (y > 0 && !horizontal_barrier[x][y - 1]) {
    cells[*size].x = x;
    cells[*size].y = y - 1;
    (*size)++;
  }
  if (y < MAZE_SIZE - 1 && !horizontal_barrier[x][y]) {
    cells[*size].x = x;
    cells[*size].y = y + 1;
    (*size)++;
  }
}

Cell get_min_cell(Cell cells[], int size) {
  Cell min_cell = cells[0];
  for (int i = 1; i < size; i++) {
    if (dp_matrix[cells[i].x][cells[i].y] < dp_matrix[min_cell.x][min_cell.y]) {
      min_cell = cells[i];
    }
  }
  return min_cell;
}

void change_direction(Cell min_cell) {
  if (mouse.x == min_cell.x) {
    if (mouse.y < min_cell.y) {
      if (mouse.heading == LEFT) {
        turn(1);
      } else if (mouse.heading == BEHIND) {
        turn(1);
        turn(1);
      } else if (mouse.heading == RIGHT) {
        turn(-1);
      }
      mouse.heading = FRONT;
    } else if (mouse.y > min_cell.y) {
      if (mouse.heading == FRONT) {
        turn(1);
        turn(1);
      } else if (mouse.heading == LEFT) {
        turn(-1);
      } else if (mouse.heading == RIGHT) {
        turn(1);
      }
      mouse.heading = BEHIND;
    }
  } else if (mouse.y == min_cell.y) {
    if (mouse.x < min_cell.x) {
      if (mouse.heading == FRONT) {
        turn(-1);
      } else if (mouse.heading == BEHIND) {
        turn(1);
      } else if (mouse.heading == RIGHT) {
        turn(1);
        turn(1);
      }
      mouse.heading = LEFT;
    } else if (mouse.x > min_cell.x) {
      if (mouse.heading == FRONT) {
        turn(1);
      } else if (mouse.heading == BEHIND) {
        turn(-1);
      } else if (mouse.heading == LEFT) {
        turn(1);
        turn(1);
      }
      mouse.heading = RIGHT;
    }
  }
}


void enqueue(Queue* queue, Cell cell) {
  Node* new_node = (Node*)malloc(sizeof(Node));
  new_node->cell = cell;
  new_node->next = NULL;

  if (queue->rear == NULL) {
    queue->front = queue->rear = new_node;
  } else {
    queue->rear->next = new_node;
    queue->rear = new_node;
  }
}

Cell dequeue(Queue* queue) {
  Cell cell = { 0, 0 };
  if (queue->front == NULL) return cell;  // Queue is empty

  Node* temp = queue->front;
  cell = queue->front->cell;
  queue->front = queue->front->next;

  if (queue->front == NULL) queue->rear = NULL;

  free(temp);
  return cell;
}

int is_queue_empty(Queue* queue) {
  return queue->front == NULL;
}


void handleTurn() {
  if (server.hasArg("dir")) {
    int direction = server.arg("dir").toInt();
    turn(direction);
  }
  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
}


void handleRoot() {
  // Start of HTML
  String html = "<!DOCTYPE html><html><head>";
  html += "<title>ESP32 WebSocket Client</title>";
  html += "<style>";
  html += "body { font-family: Arial, sans-serif; margin: 20px; }";
  html += ".data-field { margin-bottom: 10px; }";
  html += ".data-field label { font-weight: bold; display: inline-block; width: 200px; }";
  html += "button { padding: 10px 20px; font-size: 16px; margin-top: 20px; }";
  html += "input { padding: 5px; font-size: 16px; width: 100px; }";
  html += "#csv-data { white-space: pre; font-family: monospace; margin-top: 20px; }";
  html += "</style>";
  html += "<script>";
  html += "var ws = new WebSocket('ws://' + window.location.hostname + ':81/');";
  html += "function updateField(id, value) { document.getElementById(id).innerText = value; }";
  html += "function startRobot() { ws.send('start'); }";
  html += "function moveForward() { fetch('/moveForward').then(response => response.text()).then(data => console.log(data)); }";
  html += "function turnRight() { fetch('/turnRight').then(response => response.text()).then(data => console.log(data)); }";
  html += "function turnLeft() { fetch('/turnLeft').then(response => response.text()).then(data => console.log(data)); }";
  html += "function toggleFloodFillButtons() { fetch('/toggleFloodFillButtons').then(response => response.text()).then(data => console.log(data)); }";
  html += "function toggleByHand() { fetch('/toggleByHand').then(response => response.text()).then(data => console.log(data)); }";

  // PID Update Function
  html += "function updatePIDs() {";
  html += "  const kp_distance = document.getElementById('kp_distance').value;";
  html += "  const ki_distance = document.getElementById('ki_distance').value;";
  html += "  const kd_distance = document.getElementById('kd_distance').value;";
  html += "  const kp_angle = document.getElementById('kp_angle').value;";
  html += "  const ki_angle = document.getElementById('ki_angle').value;";
  html += "  const kd_angle = document.getElementById('kd_angle').value;";
  html += "  fetch(`/updatePIDs?kp_distance=${kp_distance}&ki_distance=${ki_distance}&kd_distance=${kd_distance}&kp_angle=${kp_angle}&ki_angle=${ki_angle}&kd_angle=${kd_angle}`)";
  html += "    .then(response => response.text())";
  html += "    .then(data => console.log(data));";
  html += "}";  // Closing brace for updatePIDs

  // WebSocket Handlers
  html += "ws.onmessage = function(event) {";
  html += "  var lines = event.data.split('\\n');";
  html += "  lines.forEach(function(line) {";
  html += "    if (line.startsWith('Left motor encoder:')) updateField('left-encoder', line.split(': ')[1]);";
  html += "    else if (line.startsWith('Right motor encoder:')) updateField('right-encoder', line.split(': ')[1]);";
  html += "    else if (line.startsWith('Right motor speed:')) updateField('right-encoder-speed', line.split(': ')[1]);";
  html += "    else if (line.startsWith('Left motor speed:')) updateField('left-encoder-speed', line.split(': ')[1]);";
  html += "    else if (line.startsWith('Front (mm):')) updateField('front-sensor', line.split(': ')[1]);";
  html += "    else if (line.startsWith('Left (mm):')) updateField('left-sensor', line.split(': ')[1]);";
  html += "    else if (line.startsWith('Right (mm):')) updateField('right-sensor', line.split(': ')[1]);";
  html += "    else if (line.startsWith('tX :')) updateField('angle-x', line.split(': ')[1]);";
  html += "    else if (line.startsWith('tY :')) updateField('angle-y', line.split(': ')[1]);";
  html += "    else if (line.startsWith('tZ :')) updateField('angle-z', line.split(': ')[1]);";
  html += "    else if (line.startsWith('Init Yaw Angle:')) updateField('init-Angle', line.split(': ')[1]);";
  html += "    else if (line.startsWith('CSV:')) document.getElementById('csv-data').innerText = line.split(':')[1];";
  html += "  });";
  html += "};";

  html += "ws.onopen = function() { console.log('WebSocket connection established'); };";
  html += "ws.onclose = function() { console.log('WebSocket connection closed'); };";
  html += "</script>";
  html += "</head><body>";
  html += "<h1>ESP32 Real-Time Data</h1>";

  // Data Fields
  html += "<div class='data-field'><label>Left Motor Encoder:</label><span id='left-encoder'>0</span></div>";
  html += "<div class='data-field'><label>Right Motor Encoder:</label><span id='right-encoder'>0</span></div>";
  html += "<div class='data-field'><label>Right Motor Speed:</label><span id='right-encoder-speed'>0</span></div>";
  html += "<div class='data-field'><label>Left Motor Speed:</label><span id='left-encoder-speed'>0</span></div>";
  html += "<div class='data-field'><label>Front Sensor (mm):</label><span id='front-sensor'>0</span></div>";
  html += "<div class='data-field'><label>Left Sensor (mm):</label><span id='left-sensor'>0</span></div>";
  html += "<div class='data-field'><label>Right Sensor (mm):</label><span id='right-sensor'>0</span></div>";
  html += "<div class='data-field'><label>Angle X:</label><span id='angle-x'>0</span></div>";
  html += "<div class='data-field'><label>Angle Y:</label><span id='angle-y'>0</span></div>";
  html += "<div class='data-field'><label>Angle Z:</label><span id='angle-z'>0</span></div>";
  html += "<div class='data-field'><label>Init Angle Z:</label><span id='init-Angle'>0</span></div>";

  // Control Buttons
  html += "<button onclick='startRobot()'>Start Robot</button>";
  html += "<button onclick='moveForward()'>Move Forward</button>";
  html += "<button onclick='turnLeft()'>Turn Left</button>";
  html += "<button onclick='turnRight()'>Turn Right</button>";
  html += "<button onclick='toggleFloodFillButtons()'>Toggle Flood Fill Buttons</button>";
  html += "<button onclick='toggleByHand()'>Toggle By Hand</button>";

  // PID Controls
  html += "<h2>Distance PID Control</h2>";
  html += "<div class='data-field'><label>Kp (Distance):</label><input type='number' id='kp_distance' value='1.0'></div>";
  html += "<div class='data-field'><label>Ki (Distance):</label><input type='number' id='ki_distance' value='0.0'></div>";
  html += "<div class='data-field'><label>Kd (Distance):</label><input type='number' id='kd_distance' value='0.0'></div>";

  html += "<h2>Angle PID Control</h2>";
  html += "<div class='data-field'><label>Kp (Angle):</label><input type='number' id='kp_angle' value='1.0'></div>";
  html += "<div class='data-field'><label>Ki (Angle):</label><input type='number' id='ki_angle' value='0.0'></div>";
  html += "<div class='data-field'><label>Kd (Angle):</label><input type='number' id='kd_angle' value='0.0'></div>";
  html += "<button onclick='updatePIDs()'>Update PIDs</button>";

  // Send the HTML to the client
  server.send(200, "text/html", html);
}



// WebSocket event handler
void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Disconnected!\n", num);
      break;
    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("[%u] Connected from %d.%d.%d.%d\n", num, ip[0], ip[1], ip[2], ip[3]);
      }
      break;
    case WStype_TEXT:
      Serial.printf("[%u] Received: %s\n", num, payload);
      if (strcmp((char*)payload, "start") == 0) {
        robotStarted = true;      // Start the robot
        lastTurnTime = millis();  // Reset the turn timer
      }
      break;
  }
}

void updateLeftMotorEncoder() {
  // Read the state of the second encoder pin
  int pinBState = digitalRead(MOTOR_LEFT_ENC_IN_2);

  // Determine direction based on the state of the second pin
  if (pinBState == LOW) {
    leftMotorEncoderCount++;  // Forward motion
  } else {
    leftMotorEncoderCount--;  // Reverse motion
  }
}

void updateRightMotorEncoder() {
  // Read the state of the second encoder pin
  int pinBState = digitalRead(MOTOR_RIGHT_ENC_IN_2);

  // Determine direction based on the state of the second pin
  if (pinBState == HIGH) {
    rightMotorEncoderCount++;  // Forward motion
  } else {
    rightMotorEncoderCount--;  // Reverse motion
  }
}

bool initializeVL53L0X(Adafruit_VL53L0X &sensor, int xshutPin, uint8_t address) {
  digitalWrite(xshutPin, HIGH);
  delay(10);

  if (!sensor.begin(address)) {
    Serial.print("Failed to boot VL53L0X at address 0x");
    Serial.println(address, HEX);
    return false;
  }

  Serial.print("VL53L0X initialized at address 0x");
  Serial.println(address, HEX);

  sensor.startRangeContinuous();

  return true;
}


void setVL53L0XIDs() {
  digitalWrite(SHT_FRONT, LOW);
  digitalWrite(SHT_LEFT, LOW);
  digitalWrite(SHT_RIGHT, LOW);
  delay(10);

  digitalWrite(SHT_FRONT, HIGH);
  digitalWrite(SHT_LEFT, HIGH);
  digitalWrite(SHT_RIGHT, HIGH);
  delay(10);

  digitalWrite(SHT_LEFT, LOW);
  digitalWrite(SHT_RIGHT, LOW);
  if (!initializeVL53L0X(loxFront, SHT_FRONT, FRONT_ADDRESS)) {
    while (1)
      ;
  }

  digitalWrite(SHT_LEFT, HIGH);
  digitalWrite(SHT_RIGHT, LOW);
  if (!initializeVL53L0X(loxLeft, SHT_LEFT, LEFT_ADDRESS)) {
    while (1)
      ;
  }

  digitalWrite(SHT_RIGHT, HIGH);
  if (!initializeVL53L0X(loxRight, SHT_RIGHT, RIGHT_ADDRESS)) {
    while (1)
      ;
  }
}


int readFrontDistance() {
  if (loxFront.isRangeComplete()) {                        // Check if a new measurement is available
    loxFront.getRangingMeasurement(&measureFront, false);  // Read the latest measurement

    if (measureFront.RangeStatus != 4) {    // Check if the measurement is valid
      return measureFront.RangeMilliMeter;  // Return the distance in millimeters
    }
  }
  return -1;  // Return -1 if out of range or no measurement is available
}

int readRightDistance() {
  if (loxRight.isRangeComplete()) {                        // Check if a new measurement is available
    loxRight.getRangingMeasurement(&measureRight, false);  // Read the latest measurement

    if (measureRight.RangeStatus != 4) {    // Check if the measurement is valid
      return measureRight.RangeMilliMeter;  // Return the distance in millimeters
    }
  }
  return -1;  // Return -1 if out of range or no measurement is available
}


int readLeftDistance() {
  if (loxLeft.isRangeComplete()) {                        // Check if a new measurement is available
    loxLeft.getRangingMeasurement(&measureLeft, false);  // Read the latest measurement

    if (measureLeft.RangeStatus != 4) {    // Check if the measurement is valid
      return measureLeft.RangeMilliMeter;  // Return the distance in millimeters
    }
  }
  return -1;  // Return -1 if out of range or no measurement is available
}



void initializeMPU6050() {
  Wire.begin();
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0) {}
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(true, true);
  Serial.println("Done!n");
}

void readMPU6050Data() {
  mpu.update();
  Serial.println("---------------MPU6050 Data--------------");
  Serial.print("tX : ");
  Serial.println(mpu.getAngleX());
  Serial.print("tY : ");
  Serial.println(mpu.getAngleY());
  Serial.print("tZ : ");
  yawAngle = mpu.getAngleZ();
  Serial.println(mpu.getAngleZ());
  Serial.println("-----------------------------");
}



void motor_A_set(int speed) {
  if (speed == 0) {
    if (rightMotorPrevSpeed > 0) {
      digitalWrite(rightMotorM1, HIGH);
      digitalWrite(rightMotorM2, LOW);
    } else {
      digitalWrite(rightMotorM1, LOW);
      digitalWrite(rightMotorM2, HIGH);
    }

    analogWrite(rightMotorSpeedPin, rightMotorPrevSpeed);
    Serial.println("STOP A COMPLETED");

    delay(50);

    Serial.println("STOP A - AFTER DELAY");

    digitalWrite(rightMotorM1, LOW);
    digitalWrite(rightMotorM2, LOW);
    digitalWrite(rightMotorSpeedPin, LOW);  // Ensure motor is completely stopped

  } else if (speed > 0) {

    digitalWrite(rightMotorM1, LOW);
    digitalWrite(rightMotorM2, HIGH);
    analogWrite(rightMotorSpeedPin, speed);

  } else {

    digitalWrite(rightMotorM1, HIGH);
    digitalWrite(rightMotorM2, LOW);
    analogWrite(rightMotorSpeedPin, -speed);
  }

  rightMotorPrevSpeed = speed;
}

void motor_B_set(int speed) {
  if (speed == 0) {
    if (leftMotorPrevSpeed > 0) {
      digitalWrite(leftMotorM1, HIGH);
      digitalWrite(leftMotorM2, LOW);
    } else {
      digitalWrite(leftMotorM1, LOW);
      digitalWrite(leftMotorM2, HIGH);
    }

    analogWrite(leftMotorSpeedPin, leftMotorPrevSpeed);
    Serial.println("STOP A COMPLETED");

    delay(50);

    Serial.println("STOP B - AFTER DELAY");

    digitalWrite(leftMotorM1, LOW);
    digitalWrite(leftMotorM2, LOW);
    digitalWrite(leftMotorSpeedPin, LOW);  // Ensure motor is completely stopped

  } else if (speed > 0) {

    digitalWrite(leftMotorM1, LOW);
    digitalWrite(leftMotorM2, HIGH);
    analogWrite(leftMotorSpeedPin, speed);

  } else {

    digitalWrite(leftMotorM1, HIGH);
    digitalWrite(leftMotorM2, LOW);
    analogWrite(leftMotorSpeedPin, -speed);
  }

  leftMotorPrevSpeed = speed;
}


float PID_step(PID* controller, float error, float deltaTime) {
  // Proportional term
  float proportional = controller->Kp * error;

  // Integral term: Accumulate error over time
  controller->integral += error * deltaTime;

  // Anti-windup: Limit the integral term
  if (controller->maxIntegral > 0) {
    if (controller->integral > controller->maxIntegral) {
      controller->integral = controller->maxIntegral;
    } else if (controller->integral < -controller->maxIntegral) {
      controller->integral = -controller->maxIntegral;
    }
  }

  float integral = controller->Ki * controller->integral;

  // Derivative term: Rate of change of error
  float derivative = controller->Kd * (error - controller->previousValue) / deltaTime;

  // Update previous error
  controller->previousValue = error;

  Serial.println(proportional);
  Serial.println(integral);
  Serial.println(derivative);

  // Calculate PID output
  float u = proportional + integral + derivative;

  return u;
}







void turn(int direction) {
  // init_ADRC();

  if (!isLastMoveTurn)
    targetAngle = mpu.getAngleZ();

  long enterLoop = millis();
  float prevError = 0;
  int stallCounter = 0;

  int prevRightEncoderCounts = 0;
  int prevLeftEncoderCounts = 0;

  int rightMotorSpeed = 0;
  int leftMotorSpeed = 0;

  unsigned long mpuTime = millis();

  float startAngle = mpu.getAngleZ();
  float residue = 0;

  int state = 0;

  while (true) {
    if(millis() - mpuTime > MPU_UPDATE_TIME) {
      mpu.update();
      mpuTime = millis();
    }

    if (millis() - enterLoop > 20) {  // ~66Hz update
      enterLoop = millis();
      float currentYAW = mpu.getAngleZ();

      float measurement = currentYAW - startAngle;

      float angleError = 45 * direction - (measurement + residue);

      float angularVelocity = mpu.getGyroZ();  // deg/s

      // prevError = angleError;

      // ADRC
      float previous_angle_error = 0;
      float delta_angle_error = (angleError - previous_angle_error)/0.02;
      fuzzyAdjustPID(angleError, delta_angle_error, &turnFuzzyPID, &turnController);
      float control = PID_step(&turnController, fabs(angleError), 0.02);
      previous_angle_error = angleError;

      control = constrain(control, 80, 120);

      // compute encoder differences for each motor

      int rightEncDif = rightMotorEncoderCount - prevRightEncoderCounts;
      int leftEncDif = leftMotorEncoderCount - prevLeftEncoderCounts;


      // Only apply compensation when significant difference exists
      if (abs(rightEncDif - leftEncDif) > 5) {
        float encoderRatio = (float)rightEncDif / (leftEncDif + 1);  // +1 to avoid div/0
        float compensation = constrain((1.0 - encoderRatio) * 10, -3, 3);

        if (rightEncDif < leftEncDif) {
          rightMotorSpeed = control + compensation;
          leftMotorSpeed = control - compensation;
        } else {
          rightMotorSpeed = control - compensation;
          leftMotorSpeed = control + compensation;
        }
      } else {
        // Default to ADRC control when encoders are balanced
        rightMotorSpeed = leftMotorSpeed = control;
      }

      rightMotorSpeed = control;
      leftMotorSpeed = control;

      Serial.print("Control:");
      Serial.println(control);

      // rightMotorSpeed = constrain(rightMotorSpeed, 80, 100);
      // leftMotorSpeed = constrain(leftMotorSpeed, 80, 100);

      // Anti-stall mechanism with increased torque
      if (fabs(angularVelocity) < 0.5) {  // when the robot is stuck, the angular speed doesn't exceed 0.5 degrees
        leftMotorSpeed = 120;
        rightMotorSpeed = 120;  // push it to break stall
      } else {
        // Two-phase braking system
        if (fabs(angleError) < 30) {
          rightMotorSpeed = constrain(control, 80, 100);
          leftMotorSpeed = constrain(control, 80, 100);  // Phase 1 braking
        }
        if (fabs(angleError) < 15) {
          rightMotorSpeed = constrain(control, 80, 90);
          leftMotorSpeed = constrain(control, 80, 90);  // Phase 2 braking
        }
      }



      setMotorSpeeds(-SPEED_RATIO * leftMotorSpeed * direction, rightMotorSpeed * direction);

      prevRightEncoderCounts = rightMotorEncoderCount;
      prevLeftEncoderCounts = leftMotorEncoderCount;

      // Termination conditions (both angle and velocity)
      if (fabs(angleError) < 5) {
        setMotorSpeeds(0, 0);

        startAngle = currentYAW;
        residue = angleError;

        int waitTillNext45 = millis();
        int mpu_update_time_inside_loop = millis();

        // wait 1s for the next 45 turn

        while((millis() - waitTillNext45) < 1000) {
          
          if((millis() - mpu_update_time_inside_loop) > 5) {
            mpu_update_time_inside_loop = millis();
            mpu.update();
          }

        }

        if(state)
          break;

        state++;
        // Verify final position
        
      }
    }
  }
  if (direction == 1)  // left
    targetAngle += 90;
  else
    targetAngle -= 90;


  setMotorSpeeds(0, 0);
  isLastMoveTurn = true;
}



// Modified moveForward function with ADRC
void moveForward() {
  // init_ADRC();  // Initialize parameters
  if (!isLastMoveTurn) {
    currentDistance = (rightMotorEncoderCount + leftMotorEncoderCount) / 2.0;
    Serial.println(currentDistance);
    prevMoveError = currentStepTargetDistance - currentDistance;
    Serial.println(prevMoveError);
  } else {
    prevMoveError = 0;
  }

  rightMotorEncoderCount = 0;
  leftMotorEncoderCount = 0;
  float currentDistance;
  unsigned long lastTime = millis();
  unsigned long lastTimeLidar = millis();

  currentStepTargetDistance = TARGET_ENCODER_COUNTS + prevMoveError;
  Serial.println(currentStepTargetDistance);

  float initialFront_cm = readFrontDistance();

  unsigned long mpuTime = millis();

  distanceController.previousValue = 0;
  distanceController.integral = 0;
  angleController.previousValue = 0;
  angleController.previousValue = 0;  
               
  if (byHand) {
    prevMoveError = 0;
    targetAngle = mpu.getAngleZ();
  }

  while (true) {
    if(millis() - mpuTime > MPU_UPDATE_TIME) {
      mpu.update();
      mpuTime = millis();
    }

    
    if (millis() - lastTime > 20) {  // 50Hz update
      lastTime = millis();
      // mpu.update();
      double currentAngle = mpu.getAngleZ();

      Read current front distance
      float lidar_front_mm = readFrontDistance();
      if(lidar_front_mm != -1 ) {
        float currentFront_cm = lidar_front_mm / 10.0;
        float delta_cm = initialFront_cm - currentFront_cm;
        currentDistance = delta_cm * 4.5;
      } else {
        currentDistance = (rightMotorEncoderCount + leftMotorEncoderCount) / 2.0;
      }


      // ADRC for distance control
      float previous_distance_error = 0;
      float distance_error = currentStepTargetDistance - currentDistance;
      float delta_distance_error = (distance_error - previous_distance_error) / 0.02;
      // fuzzyAdjustPID(distance_error, delta_distance_error, &distanceFuzzyPID, &distanceController);
      double distanceControl = PID_step(&distanceController, distance_error, 0.02);
      previous_distance_error = distance_error;

      Serial.println("distanceControl");
      Serial.println(distanceControl);
      

      // ADRC for angle control

      double centerMeasurement;



      int leftDist = readLeftDistance();
      int rightDist = readRightDistance();
      Serial.println("Left Distance");
      Serial.println(leftDist);
      Serial.println("Right Distance");
      Serial.println(rightDist);


      if (isWall(leftDist) && isWall(rightDist)) {

        int dif = rightDist - leftDist;
        centerMeasurement = dif / 10.0;
        Serial.println("centerMeasurement");
        Serial.println(centerMeasurement);
        
      } else
        centerMeasurement = currentAngle - targetAngle;
      
     

      float previous_angle_error = 0;
      float angle_error = centerMeasurement;
      float delta_angle_error = (angle_error - previous_angle_error) / 0.02;
      // fuzzyAdjustPID(angle_error, delta_angle_error, &angleFuzzyPID, &angleController);
      double angleControl = PID_step(&angleController, fabs(angle_error), 0.02);
      previous_angle_error = angle_error;

      if(angle_error < 0)
        angleControl *= -1;


      Serial.println("Angle Control");
      Serial.println(angleControl);

      int leftSpeed = 80 + distanceControl + angleControl;
      int rightSpeed = 80 + distanceControl - angleControl;


     
      Serial.println("leftSpeed");
      Serial.println(leftSpeed);
      Serial.println("rightSpeed");
      Serial.println(rightSpeed);

      leftSpeed = constrain(leftSpeed, 80, 150);
      rightSpeed = constrain(rightSpeed, 80, 150);


      if (distance_error < 40) {
        leftSpeed = constrain(leftSpeed, 70, 90);
        rightSpeed = constrain(rightSpeed, 70, 90);
      }

      if (distance_error > 15) {
        setMotorSpeeds(SPEED_RATIO * leftSpeed, rightSpeed);
      } else {
        // Final position lock
        moveForwardState = false;
        setMotorSpeeds(0, 0);
        break;
      }
    }
  }
  isLastMoveTurn = false;
}

// set the motor speeds
void setMotorSpeeds(int left, int right) {
  motor_A_set(right);
  motor_B_set(left);
}

int saturate(int x) {
  if (0 <= x && x <= 255)
    return x;
  else if (x < 100)
    return 100;
  else
    return 255;
}

//****** Algorthim ************


// left Hand Rule

void leftHandRule() {
  bool frontWall = wallFront();
  bool rightWall = wallRight();
  bool leftWall = wallLeft();
  static bool hasMoved = false;
  long currentTime = millis();


  Serial.println("******************");

  Serial.print("front wall = ");
  Serial.println(frontWall);

  Serial.print("Right wall = ");
  Serial.println(rightWall);

  Serial.print("Left wall = ");
  Serial.println(leftWall);

  Serial.println("******************");

  // Perform one decision based on the left-hand rule
  if (!leftWall) {
    turn(LEFT_DIR);  // Turn left if no wall on the left
    currentTime = millis();
    while (millis() - currentTime < 1000)
      ;
    moveForward();
  } else if (!frontWall) {
    moveForward();  // Move forward if no wall in front
    currentTime = millis();
    while (millis() - currentTime < 1000);
  } else if (!rightWall) {
    turn(RIGHT_DIR);  // Turn right if walls are on the left and front
    currentTime = millis();
    while (millis() - currentTime < 1000);
    moveForward();
  } else {
    turn(LEFT_DIR);
    currentTime = millis();
    while (millis() - currentTime < 1000);
  }
  setMotorSpeeds(0, 0);
  robotStarted = false;
}

bool wallFront() {
  int distance = readFrontDistance();
  return (distance > 10 && distance < 200);
}

bool wallLeft() {
  int distance = readLeftDistance();
  return (distance > 10 && distance < 200);
}

bool wallRight() {
  int distance = readRightDistance();
  return (distance > 10 && distance < 200);
}

bool isWall(int distance) {
  return (distance > 10 && distance < 200);
}
