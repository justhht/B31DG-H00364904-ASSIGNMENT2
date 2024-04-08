#include <Arduino.h>

// --------------------------------------------------------Definition--------------------------------------------------------------------
// 定义I/O引脚
#define t1out 1   // T1 digital out pin
#define t2in 2    // T2 digital in pin
#define t3in 3    // T3 digital in pin
#define t4in 4    // T4 analogue in pin
#define t4out 10  // T4 LED pin out
#define BUTTON_PIN 5  // 将按钮连接到GPIO2
#define LED_PIN 6    // 将LED连接到GPIO10

// --------------------------------------------------------Definition--------------------------------------------------------------------

QueueHandle_t buttonQueue; // Queue for button events 按钮事件队列
int analogValues[10]; // Array to store analog values 存储模拟值的数组
SemaphoreHandle_t freqSemaphore; // Semaphore for frequency measurement 频率测量的信号量 (此行未使用，可能需要移除或用于其他目的)
volatile int freqValueTask2 = 0; // Variable to store frequency value from Task 2 存储Task 2频率值的变量
volatile int freqValueTask3 = 0; // Variable to store frequency value from Task 3 存储Task 3频率值的变量
extern volatile int freqValueTask2; // Declaration for external linkage 外部链接声明
extern volatile int freqValueTask3; // Declaration for external linkage 外部链接声明
SemaphoreHandle_t freqMutex = xSemaphoreCreateMutex(); // Mutex for protecting frequency variables 保护频率变量的互斥量

// ------------------------------------------------------Task Prototypes-----------------------------------------------------------------

void Task1(void *pvParameters);
void Task2(void *pvParameters);
void Task3(void *pvParameters);
void Task4(void *pvParameters);
void Task5(void *pvParameters);
void Task6(void *pvParameters);
void monitorButtonTask(void *pvParameters);
void controlLedTask(void *pvParameters);
void TaskCPUWork(void *pvParameters);


// --------------------------------------------------------setup------------------------------------------------------------------------

void setup() {
    Serial.begin(9600);

    // Initialize pins

    pinMode(t1out, OUTPUT);
    pinMode(t4out, OUTPUT);
    pinMode(t2in, INPUT);
    pinMode(t3in, INPUT);
    pinMode(t4in, INPUT);

    freqMutex = xSemaphoreCreateMutex(); // 创建互斥量Create mutex
    buttonQueue = xQueueCreate(10, sizeof(int));// 创建一个长度为10的队列来存储按钮事件Create a queue with a length of 10 to store button events

    // Create tasks

    xTaskCreate(Task1, "Task 1", 636, NULL, 1, NULL); // 低优先级Low priority 488
    xTaskCreate(Task2, "Task 2", 656, NULL, 1, NULL); // 低优先级Low priority 504
    xTaskCreate(Task3, "Task 3", 656, NULL, 1, NULL); // 低优先级Low priority 504
    xTaskCreate(Task4, "Task 4", 740, NULL, 1, NULL); // 低优先级Low priority 568
    xTaskCreate(Task5, "Task 5", 700, NULL, 1, NULL); // 低优先级Low priority 536
    xTaskCreate(monitorButtonTask, "Monitor Button", 636, NULL, 4, NULL); // 高优先级High priority 488
    xTaskCreate(controlLedTask, "Control LED", 636, NULL, 4, NULL); // 高优先级High priority 488
    xTaskCreate(TaskCPUWork, "CPU Work Task", 676, NULL, 1, NULL); // 低优先级Low priority 520
}


void loop() {
    // Empty. Control is transferred to FreeRTOS tasks.
}

// --------------------------------------------------------Task1----------------------------------------------------------------------

void Task1(void *pvParameters) {
    const int pin = t1out; // 任务关联的输出针脚Output pins associated with tasks
    pinMode(pin, OUTPUT); // 设置针脚为输出模式Set pins to output mode

    TickType_t xLastWakeTime = xTaskGetTickCount(); // 获取当前tick计数Get the current ticket count
    const TickType_t xFrequency = pdMS_TO_TICKS(4); // 设置任务周期为4msSet the task cycle to 4ms

    while (1) {
        // 输出高电平
        digitalWrite(pin, HIGH);
        delayMicroseconds(180);

        // 输出低电平40μs
        digitalWrite(pin, LOW);
        delayMicroseconds(40);

        // 输出高电平530μs
        digitalWrite(pin, HIGH);
        delayMicroseconds(530);

        // 输出低电平3.25ms
        digitalWrite(pin, LOW);
        delayMicroseconds(3250);

        // 等待下一个周期
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        // UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
        // Serial.print("Minimum stack space left: ");
        // Serial.println(uxHighWaterMark);
    }
}


// --------------------------------------------------------Task2----------------------------------------------------------------------

void Task2(void *pvParameters) {
    const uint8_t signalPin = t2in; // 信号引脚 Signal pin
    pinMode(signalPin, INPUT); // 设置信号引脚为输入模式 Set the signal pin to input mode

    const TickType_t xDelay = 20 / portTICK_PERIOD_MS; // 定义测量间隔为20ms Define the measurement interval as 20ms
    TickType_t xLastWakeTime = xTaskGetTickCount(); // 获取当前的tick计数 Get the current tick count
    
    while (1) {
        // 使用pulseIn测量高电平脉冲的持续时间，最大等待时间设置为3000微秒
        // Use pulseIn to measure the duration of the high pulse, with a maximum waiting time of 3000 microseconds
        unsigned long freqHigh = pulseIn(signalPin, HIGH, 3000);

        if (freqHigh > 0) {
            // 计算频率，使用测得的高电平持续时间，注意乘以2计算完整周期
            // Calculate the frequency using the measured duration of the high pulse, note to multiply by 2 for the complete cycle
            int tempFreq = 1000000 / (freqHigh * 2);

            // 在修改共享资源前获取信号量
            // Obtain the semaphore before modifying the shared resource
            if (xSemaphoreTake(freqMutex, portMAX_DELAY) == pdTRUE) {
                freqValueTask2 = tempFreq; // 更新频率值 Update the frequency value
                xSemaphoreGive(freqMutex); // 修改完成，释放信号量 Release the semaphore after modification
            }

            // 打印计算出的频率值
            // Print the calculated frequency value
            // Serial.print("Measured Frequency: ");
            // Serial.println(freqValueTask2); // 显示计算的频率值 Display the calculated frequency value
        }

        // 使用vTaskDelayUntil确保恒定的测量周期
        // Use vTaskDelayUntil to ensure a constant measurement cycle
        vTaskDelayUntil(&xLastWakeTime, xDelay);
    }
}


// --------------------------------------------------------Task3--------------------------------------------------------------------

void Task3(void *pvParameters) {
    const uint8_t signalPin = t3in; // 请将t3in替换为实际的第二个信号引脚编号 Please replace t3in with the actual second signal pin number
    pinMode(signalPin, INPUT); // 设置信号引脚为输入模式 Set the signal pin to input mode

    const TickType_t xDelay = 8 / portTICK_PERIOD_MS; // 定义测量间隔为8ms Define the measurement interval as 8ms
    TickType_t xLastWakeTime = xTaskGetTickCount(); // 获取当前的tick计数 Get the current tick count
    extern SemaphoreHandle_t freqMutex; // 假设已经在其他地方声明和创建了freqMutex Assume that freqMutex has been declared and created elsewhere
    extern volatile int freqValueTask3; // 用于存储Task3测量的频率值 Used to store the frequency value measured by Task3

    while (1) {
        // 使用pulseIn测量高电平脉冲的持续时间，最大等待时间设置为2000微秒
        // Use pulseIn to measure the duration of the high pulse, with a maximum waiting time of 2000 microseconds
        // 考虑到测量的频率范围，2000微秒的超时足以捕捉到500Hz到1000Hz的信号
        // Given the frequency range being measured, a 2000 microseconds timeout is sufficient to capture signals between 500Hz to 1000Hz
        unsigned long freqHigh = pulseIn(signalPin, HIGH, 2000);

        if (freqHigh > 0) {
            // 计算频率，使用测得的高电平持续时间，注意乘以2计算完整周期
            // Calculate the frequency using the measured high pulse duration, note to multiply by 2 for the complete cycle
            int tempFreq = 1000000 / (freqHigh * 2);

            // 在修改共享资源前获取信号量
            // Obtain the semaphore before modifying the shared resource
            if (xSemaphoreTake(freqMutex, portMAX_DELAY) == pdTRUE) {
                freqValueTask3 = tempFreq; // 更新频率值 Update the frequency value
                xSemaphoreGive(freqMutex); // 修改完成，释放信号量 Release the semaphore after modification
            }

            // 打印计算出的频率值
            // Print the calculated frequency value
            Serial.print("Task3 Measured Frequency: ");
            Serial.println(freqValueTask3); // 显示计算的频率值 Display the calculated frequency value
        }

        // 使用vTaskDelayUntil确保恒定的测量周期
        // Use vTaskDelayUntil to ensure a constant measurement cycle
        vTaskDelayUntil(&xLastWakeTime, xDelay);
    }
}


// --------------------------------------------------------Task4--------------------------------------------------------------------

void Task4(void *pvParameters) {
    const int numReadings = 10;
    int readings[numReadings]; // 存储读数的数组 Array to store readings
    int readIndex = 0;         // 下一次读数存储的位置 The position to store the next reading
    long total = 0;            // 读数总和 Total of readings
    long average = 0;          // 平均读数 Average reading

    for (int i = 0; i < numReadings; i++) {
        readings[i] = 0; // 初始化所有读数为0 Initialize all readings to 0
    }

    while (1) {
        int newReading = analogRead(t4in); // 从模拟输入引脚读取新的值 Read a new value from the analog input pin
        total = total - readings[readIndex]; // 从总和中减去最早的读数 Subtract the oldest reading from the total
        readings[readIndex] = newReading; // 将新的读数添加到数组中 Add the new reading to the array
        total = total + readings[readIndex]; // 将新的读数添加到总和中 Add the new reading to the total
        readIndex = (readIndex + 1) % numReadings; // 更新读数索引 Update the reading index

        average = total / numReadings; // 计算平均值 Calculate the average

        // 打印当前读数和平均值到串口监视器
        // Print the current reading and the average to the serial monitor
        // Serial.print("Current reading: ");
        // Serial.print(newReading);
        // Serial.print(" Average: ");
        // Serial.println(average);

        if (average > 2047) {
            digitalWrite(t4out, HIGH); // 如果平均值大于最大范围的一半，点亮LED Light up the LED if the average is more than half of the maximum range
        } else {
            digitalWrite(t4out, LOW); // 否则熄灭LED Turn off the LED otherwise
        }

        vTaskDelay(pdMS_TO_TICKS(20)); // 每20ms运行一次 Run every 20ms
    }
}


// --------------------------------------------------------Task5--------------------------------------------------------------------

void Task5(void *pvParameters) {
    static unsigned long lastLogTime = 0; // 上次记录日志的时间 The last time a log was recorded
    const unsigned long logInterval = 200; // 日志记录间隔，以毫秒为单位 The log recording interval, in milliseconds
    
    while (1) {
        unsigned long currentMillis = millis();
        // 如果当前时间超过了记录间隔 If the current time exceeds the logging interval
        if (currentMillis - lastLogTime >= logInterval) {
            int scaledFreqTask2, scaledFreqTask3;

            // 尝试获取互斥量以安全访问 freqValueTask2 和 freqValueTask3
            // Attempt to acquire the mutex to safely access freqValueTask2 and freqValueTask3
            if(xSemaphoreTake(freqMutex, portMAX_DELAY) == pdTRUE) {
                // 成功获取互斥量后，安全读取共享变量的值
                // Upon successfully acquiring the mutex, safely read the values of the shared variables
                scaledFreqTask2 = map(freqValueTask2, 333, 1000, 0, 99);
                scaledFreqTask3 = map(freqValueTask3, 500, 1000, 0, 99);

                // 释放互斥量 Release the mutex
                xSemaphoreGive(freqMutex);
            } else {
                // 如果没有成功获取互斥量，可以设置一个错误值或者进行错误处理
                // If the mutex is not successfully acquired, set an error value or handle the error
                scaledFreqTask2 = -1; // 示例错误值 Example error value
                scaledFreqTask3 = -1; // 示例错误值 Example error value
            }

            // 限制值的范围在0到99之间 Limit the values to a range between 0 and 99
            scaledFreqTask2 = constrain(scaledFreqTask2, 0, 99);
            scaledFreqTask3 = constrain(scaledFreqTask3, 0, 99);

            // 输出到串口 Print to the serial port
            Serial.print(scaledFreqTask2);
            Serial.print(",");
            Serial.println(scaledFreqTask3);

            // 更新上次记录日志的时间 Update the time the last log was recorded
            lastLogTime = currentMillis;
        }
        // 短暂延时以减轻CPU负担 Briefly delay to reduce CPU load
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}


// --------------------------------------------------------Task7-1--------------------------------------------------------------------

void monitorButtonTask(void *pvParameters) { // Task7.1 任务7.1
    pinMode(BUTTON_PIN, INPUT_PULLUP); // 按钮使用上拉电阻 Use internal pull-up resistor for the button
    static int lastButtonState = HIGH; // 初始状态设为未按下 Initialize the last button state as not pressed
    int buttonState;
    int debounceDelay = 50; // 按钮去抖动延迟50ms Button debounce delay of 50ms

    while (1) {
        buttonState = digitalRead(BUTTON_PIN); // 读取按钮状态 Read the button state
        // 如果按钮状态从未按下变为按下 If the button state changes from not pressed to pressed
        if (buttonState == LOW && lastButtonState == HIGH) {
            // 去抖动: 等待一段时间再次检查按钮状态 Debouncing: wait for a period then check the button state again
            vTaskDelay(pdMS_TO_TICKS(debounceDelay));
            buttonState = digitalRead(BUTTON_PIN); // 再次检查按钮状态 Re-check the button state
            if (buttonState == LOW) { // 确认按钮确实被按下 Confirm the button is indeed pressed
                int event = 1; // 定义一个简单的事件标志 Define a simple event flag
                xQueueSend(buttonQueue, &event, portMAX_DELAY); // 将事件发送到队列 Send the event to the queue
            }
        }
        lastButtonState = buttonState; // 更新按钮状态 Update the button state
        vTaskDelay(pdMS_TO_TICKS(10)); // 短暂延时减少CPU使用 Briefly delay to reduce CPU usage
    }
}


// --------------------------------------------------------Task7-2--------------------------------------------------------------------

void controlLedTask(void *pvParameters) { // Task7.2 任务7.2
    int event; // 事件变量 Event variable
    int ledState = LOW; // 初始LED状态 Initialize LED state as OFF
    pinMode(LED_PIN, OUTPUT); // 将LED引脚设置为输出模式 Set the LED pin to output mode

    while(1) {
        // 尝试从队列中接收事件，如果成功，切换LED状态
        // Attempt to receive an event from the queue, if successful, toggle the LED state
        if(xQueueReceive(buttonQueue, &event, portMAX_DELAY)) {
            // 从队列接收到事件，切换LED状态 Toggle the LED state upon receiving an event from the queue
            ledState = !ledState; // 切换LED状态 Toggle the LED state
            digitalWrite(LED_PIN, ledState); // 根据ledState的值设置LED引脚 Set the LED pin according to the value of ledState
        }  
    }
}

// --------------------------------------------------------Task8-1--------------------------------------------------------------------

void CPU_work(int time) {
    unsigned long endTime = millis() + time; // 计算结束时间 Calculate the end time
    while (millis() < endTime) {
        // 空循环用于消耗时间 Empty loop to consume time
        volatile int dummy = 0; // 使用volatile关键字防止编译器优化 Prevent compiler optimization with the volatile keyword
        for (int i = 0; i < 1000; i++) {
            dummy += i; // 执行简单计算进行时间消耗 Perform simple calculation to consume time
        }    
    }    
}


// --------------------------------------------------------Task8-2--------------------------------------------------------------------

void TaskCPUWork(void *pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // 设置任务周期为20ms Set the task period to 20ms
    TickType_t xLastWakeTime = xTaskGetTickCount(); // 获取当前tick计数 Get the current tick count

    for (;;) {
        CPU_work(2); // 调用CPU_work模拟CPU工作2ms Call CPU_work to simulate 2ms of CPU work
        vTaskDelayUntil(&xLastWakeTime, xFrequency); // 延时直到下一个周期 Delay until the next period
    }
}


