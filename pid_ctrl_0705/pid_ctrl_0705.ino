//2025/07/05 14:40分　一応は走ったバージョン
//2025/07/05 14:59分　I制御追加、検討の末係数Kiを0.0に補正
//2025/07/05 18:02分　D制御追加、重みづけパラメータ制御、baseSpeed = 200, Kp = 170.0, Ki = 0.01, Kd = 1000.0に変更
// --- ピン定義 ---
const int sensorPins[8] = {26, 25, 33, 32, 35, 34, 39, 36};
//重みを変えてパワーを均等に制御
const float sensorWeights[8] = {-6, -3, -3, -2, 2.25, 3.25, 3.25, 4};
//const float sensorWeights[8] = {-6, -4, -4, -4, 4, 4, 4, 4};

// たぶん左
const int ENA = 5;
const int IN1 = 19;
const int IN2 = 18;

// たぶん右
const int ENB = 15;
const int IN3 = 0;
const int IN4 = 2;

const int pwmChannelA = 0;
const int pwmChannelB = 1;
const int pwmFreq = 5000;
const int pwmResolution = 8;

// --- 制御パラメータ ---
const int baseSpeed = 200;
const float Kp = 170.0;
const float Ki = 0.01;
//const float Kd = 450.0;
const float Kd = 1000.0;

volatile float position = 0.0;
volatile int total = 0;
volatile int lastKnownDirection = 0;
volatile float integral = 0.0;
volatile float previousPosition = 0.0;

// --- タスクハンドル ---
TaskHandle_t sensorTaskHandle;
TaskHandle_t motorTaskHandle;

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
    //Serial.printf("%8d \n", weightedSum);


    // Serial.print("sensorValues:");
    //Serial.printf("%6d %6d %6d %6d %6d %6d %6d %6d \n",
    //              sensorValues[0], sensorValues[1], sensorValues[2], sensorValues[3],
    //              sensorValues[4], sensorValues[5], sensorValues[6], sensorValues[7]);

    if (sum != 0) {
      position = (float)weightedSum / sum;
    } else {
      position = 0.0;
    }

    total = sum;

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void motorTask(void *pvParameters) {
  while (true) {
    if (total < 2300) {
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

    float error = position;
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

    if (error > 0.7) {
      lastKnownDirection = 1;
    } else if (error < -0.7) {
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