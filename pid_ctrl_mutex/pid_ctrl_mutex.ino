// --- ピン定義 ---
// const int sensorPins[8] = {32, 35, 34, 39, 14, 27, 13, 4};
const int sensorPins[8] = {26, 25, 33, 32, 35, 34, 39, 36};
const int sensorWeights[8] = {-4, -3, -2, -1, 1, 2, 3, 4};

// たぶん左
const int ENA = 17;
const int IN1 = 16;
const int IN2 = 4;

// たぶん右
const int ENB = 15;
const int IN3 = 0;
const int IN4 = 2;

const int pwmChannelA = 0;
const int pwmChannelB = 1;
const int pwmFreq = 5000;
const int pwmResolution = 8;

// --- 制御パラメータ ---
const int baseSpeed = 100;
const float Kp = 130.0;
const float Ki = 10.0;
const float Kd = 80.0;

volatile float position = 0.0;
volatile int total = 0;
volatile int lastKnownDirection = 0;
volatile float integral = 0.0;
volatile float previousPosition = 0.0;

// --- タスクハンドル ---
TaskHandle_t sensorTaskHandle;
TaskHandle_t motorTaskHandle;

//ミューテックス宣言
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void setup() {
  Serial.begin(115200);

  // PWM設定
  /*
  ledcSetup(pwmChannelA, pwmFreq, pwmResolution);
  ledcAttachPin(ENA, pwmChannelA);
  ledcSetup(pwmChannelB, pwmFreq, pwmResolution);
  ledcAttachPin(ENB, pwmChannelB);
  */
  ledcAttach(ENA, pwmFreq, pwmResolution);
  ledcAttach(ENB, pwmFreq, pwmResolution);

  // モーター方向ピン
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);

  // タスク作成
  xTaskCreatePinnedToCore(sensorTask, "SensorTask", 2048, NULL, 1, &sensorTaskHandle, 0); // Core 0
  xTaskCreatePinnedToCore(motorTask, "MotorTask", 2048, NULL, 1, &motorTaskHandle, 1);     // Core 1
}

void sensorTask(void *pvParameters) {
  while (true) {
    int sensorValues[8];
    int weightedSum = 0;
    int sum = 0;

    for (int i = 0; i < 8; i++) {
      sensorValues[i] = analogRead(sensorPins[i]);
      weightedSum += sensorValues[i] * sensorWeights[i];
      sum += sensorValues[i];
    }

    // Serial.print("sensorValues:");
    Serial.printf("%6d %6d %6d %6d %6d %6d %6d %6d \n",
                  sensorValues[0], sensorValues[1], sensorValues[2], sensorValues[3],
                  sensorValues[4], sensorValues[5], sensorValues[6], sensorValues[7]);

    //ミューテックス処理
    portENTER_CRITICAL(&mux);
      if (sum != 0) {
        position = (float)weightedSum / sum;
      } else {
        position = 0.0;
      }
      total = sum;
    portEXIT_CRITICAL(&mux);

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void motorTask(void *pvParameters) {
  while (true) {

    //ミューテックス処理
    portENTER_CRITICAL(&mux);
      int sum = total;
      float error = position;
    portEXIT_CRITICAL(&mux);

    if (sum < 1500) {
      // Serial.println("Line lost");
      if (lastKnownDirection > 0) {
        ledcWrite(ENA, 255);
        ledcWrite(ENB, 0);
      } else if (lastKnownDirection < 0) {
        ledcWrite(ENA, 0);
        ledcWrite(ENB, 255);
      } else {
        ledcWrite(ENA, 0);
        ledcWrite(ENB, 0);
      }

      integral = 0;
      previousPosition = 0;

      vTaskDelay(100 / portTICK_PERIOD_MS);
      continue;
    }

    integral += error;

    integral = constrain(integral, -100.0, 100.0); // アンチワインドアップ
    float derivative = error - previousPosition;
    previousPosition = error;

    float turn = Kp * error + Ki * integral + Kd * derivative;

    // Serial.print("turn: "); // Serial.print(turn);
    int leftSpeed = constrain(baseSpeed + turn, 0, 255);
    int rightSpeed = constrain(baseSpeed - turn, 0, 255);

    ledcWrite(ENA, leftSpeed);
    ledcWrite(ENB, rightSpeed);

    if (error > 0.5) {
      lastKnownDirection = 1;
    } else if (error < -0.5) {
      lastKnownDirection = -1;
    }

    // デバッグ表示
    // Serial.print("Pos: "); // Serial.print(position);
    // Serial.print(" L: ");  // Serial.print(leftSpeed);
    // Serial.print(" R: ");  // Serial.println(rightSpeed);
    // Serial.print(" total: "); // Serial.println(total);

    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

void loop() {
  delay(1000);
}
