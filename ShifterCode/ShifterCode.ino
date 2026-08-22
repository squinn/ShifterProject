#define LGFX_USE_V1
#include <lvgl.h>               // For graphical interface
#include <LovyanGFX.hpp>        // For display driver

#include <Adafruit_NeoPixel.h>  // For controlling LED
#include <Adafruit_BNO08x.h>    // For reading the gryometer
#include <WiFi.h>
#include <WiFiUdp.h>

#include <Adafruit_SSD1306.h>

#include "CST816D.h"      // Touch driver file
#include "ui.h"           // All LVGL interfaces designed, indexed by this

// I2C pins for the touch screen
#define TP_I2C_SDA_PIN 6
#define TP_I2C_SCL_PIN 7

// I2C pins for connecting to the BNO89x
#define I2C_SDA_PIN 38
#define I2C_SCL_PIN 39


String wifiId = "quinnhome";            // WiFi SSID to connect to
String wifiPwd = "quinn3485";          // PASSWORD of the WiFi SSID to connect to

/*
  Five LEDs below the 1.28-inch display
*/
#define POWER_LIGHT_PIN 40            // Power light pin                               
#define LED_PIN 48                    // LED pin
#define LED_NUM 5                     // Total of 5 rendering LEDs

Adafruit_NeoPixel led = Adafruit_NeoPixel(LED_NUM, LED_PIN, NEO_GRB + NEO_KHZ800);  
bool isLed = true;

#define ENCODER_A_PIN 45              // Rotary display control pin 1, CLK pin
#define ENCODER_B_PIN 42              // Rotary display control pin 2, DT pin
#define SWITCH_PIN 41                 // Interrupt pin for rotary knob press

enum ScreenType {
    LOADING,
    GEAR,
    SETTINGS,
    CALIBRATE
};

volatile int8_t position_tmp = -1;    // Temporary position of the rotary knob (0: clockwise, 1: counterclockwise)
volatile int8_t currentA = 0;         // For rotary encoder state detection, current state
volatile int8_t lastA = 0;            // For rotary encoder state detection, previous pin state

volatile unsigned long lastPressTime = 0;     // Record the last button press time (for debounce and double-click detection)
volatile bool pressFlag = false;      // Button press flag
volatile int clickCount = 0;          // Number of clicks (for detecting double-click)
const unsigned long debounceTime = 20;        // Debounce time (milliseconds)
const unsigned long doubleClickTime = 300;    // Double-click interval (milliseconds)

void IRAM_ATTR buttonISR() {
  static unsigned long lastInterruptTime = 0;
  unsigned long interruptTime = millis();
  if (interruptTime - lastInterruptTime > debounceTime) {
    if (digitalRead(SWITCH_PIN)) {
      pressFlag = false;
    } else {
      pressFlag = true;
      lastPressTime = interruptTime;
      clickCount++;
    }
  }
  lastInterruptTime = interruptTime;
}

TaskHandle_t ledTestTaskHandle = NULL;
TaskHandle_t encTaskHandle = NULL;
TaskHandle_t gyroTaskHandle = NULL;

#define TP_INT 5                  // Touch interrupt pin
#define TP_RST 13                 // Touch reset pin
#define SCREEN_BACKLIGHT_PIN 46   // Screen backlight pin

const int pwmFreq = 5000;
const int pwmChannel = 0;
const int pwmResolution = 8;


class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_GC9A01 _panel_instance;
  lgfx::Bus_SPI _bus_instance;
public:
  LGFX(void) {
    {
      auto cfg = _bus_instance.config();
      cfg.spi_host = SPI2_HOST; 
      cfg.spi_mode = 0;      
      cfg.freq_write = 80000000;    
      cfg.freq_read = 20000000;         
      cfg.spi_3wire = true;      
      cfg.use_lock = true;       
      cfg.dma_channel = SPI_DMA_CH_AUTO;  
      cfg.pin_sclk = 10; 
      cfg.pin_mosi = 11;  
      cfg.pin_miso = -1; 
      cfg.pin_dc = 3;  
      _bus_instance.config(cfg);          
      _panel_instance.setBus(&_bus_instance); 
    }
    {                            
      auto cfg = _panel_instance.config();
      cfg.pin_cs = 9;                 
      cfg.pin_rst = 14;       
      cfg.pin_busy = -1;         
      cfg.memory_width = 240; 
      cfg.memory_height = 240;  
      cfg.panel_width = 240;
      cfg.panel_height = 240; 
      cfg.offset_x = 0;   
      cfg.offset_y = 0;   
      cfg.offset_rotation = 0;  
      cfg.dummy_read_pixel = 8;
      cfg.dummy_read_bits = 1;  
      cfg.readable = false;  
      cfg.invert = true;    
      cfg.rgb_order = false;   
      cfg.dlen_16bit = false;   
      cfg.bus_shared = false;   
      _panel_instance.config(cfg);
    }
    setPanel(&_panel_instance);  
  }
};

LGFX gfx;                                                  
CST816D touch(TP_I2C_SDA_PIN, TP_I2C_SCL_PIN, TP_RST, TP_INT); 
Adafruit_BNO08x bno08x;

static const uint32_t screenWidth = 240;    
static const uint32_t screenHeight = 240;

static lv_disp_draw_buf_t draw_buf;     // Drawing buffer
static lv_color_t *buf = NULL;          // Buffer required by LVGL
static lv_color_t *buf1 = NULL;         // Buffer required by LVGL

lv_obj_t *current_screen = NULL;        // Pointer to the current screen   // Currently active screen object

// Variables related to UDP communication
WiFiUDP udp;

//****************************************************SETUP****************************************************
void setup() {
  Serial.begin(115200);
  
  pinMode(POWER_LIGHT_PIN, OUTPUT);   
  digitalWrite(POWER_LIGHT_PIN, LOW); 

  pinMode(1, OUTPUT);         // Power pin 1, output current
  digitalWrite(1, HIGH);  
  pinMode(2, OUTPUT);         // Power pin 2, output current
  digitalWrite(2, HIGH);
  
  // Give the BNO08x internal sensor hub time to stabilize post-brownout
  // delay(100);
  // recoverI2CSlow(I2C_SDA_PIN, I2C_SCL_PIN);

  // Recover the I2C bus before starting Hardware Wire (so that we can communicate with the BNO08X is reset after a hard restart)
  // recoverI2C(I2C_SDA_PIN, I2C_SCL_PIN);

  // Make sure the I2C bus for communicating with the BNO08X is reset after a hard restart
  // clearI2CBus();

  // Initialize the primary Wire bus on GPIO 38 and 39 (External Connector)
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);  // Initialize I2C bus
  Wire.setClock(100000);

  // Start the touch screen
  touch.begin();              

  // Initialize display and set background color
  gfx.init();               
  gfx.initDMA();
  gfx.startWrite();
  gfx.setColor(0, 0, 0);
  gfx.setTextSize(2);
  gfx.fillScreen(TFT_BLACK);

  pinMode(ENCODER_A_PIN, INPUT);        // Set rotary encoder pins
  pinMode(ENCODER_B_PIN, INPUT);
  pinMode(SWITCH_PIN, INPUT_PULLUP);    // Set switch (press) pin -- (with interrupt) buttonISR
  attachInterrupt(digitalPinToInterrupt(SWITCH_PIN), buttonISR, CHANGE);

  lv_init();        // Initialize LVGL

  size_t buffer_size = sizeof(lv_color_t) * screenWidth * screenHeight;
  buf = (lv_color_t *)heap_caps_malloc(buffer_size, MALLOC_CAP_SPIRAM);  //MALLOC_CAP_DEFAULT,MALLOC_CAP_DMA,MALLOC_CAP_INTERNAL
  buf1 = (lv_color_t *)heap_caps_malloc(buffer_size, MALLOC_CAP_SPIRAM);
  if (!buf)
    Serial.println("Failed to allocate for LVGL buf!");
  if (!buf1)
    Serial.println("Failed to allocate for LVGL buf1!");
  lv_disp_draw_buf_init(&draw_buf, buf, buf1, screenWidth * screenHeight);

  /*Initialize the display*/
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = screenWidth;
  disp_drv.ver_res = screenHeight;
  disp_drv.flush_cb = my_disp_flush;  //Used to refresh the display
  disp_drv.draw_buf = &draw_buf;
  // disp_drv.full_refresh = 1;  //0
  lv_disp_drv_register(&disp_drv);

  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touchpad_read; // Used to read touch events
  lv_indev_drv_register(&indev_drv);
  delay(100);

  ui_init();

  // Initialize LED strip (Adafruit_NeoPixel)
  // Five LEDs at the bottom of the device (for ambiance)
  led.begin();            
  led.setBrightness(25);  
  led.clear();
  led.show();

  delay(500);
  initBacklight(); // Initialize backlight (controlled via PWM)

  // 3. Initialize the BNO08x on the primary Wire bus (Address 0x4A)
  if (!bno08x.begin_I2C(0x4A, &Wire)) {
    Serial.println("Failed to find BNO08x chip on GPIO 38/39!");
    while (1) { delay(10); } // Stop execution if sensor fails to connect
  }
  
  Serial.println("BNO08x Found!");

  enableReports();

  xTaskCreatePinnedToCore(ledTestTask, "LED Test", 2048, NULL, 1, &ledTestTaskHandle, 0);
  xTaskCreatePinnedToCore(encTask, "ENC", 2048, NULL, 1, &encTaskHandle, 0);
  xTaskCreatePinnedToCore(gyroTask, "GYRO", 2048, NULL, 1, &gyroTaskHandle, 0);

  Serial.println("Switching to GearScreen...");
  // _ui_screen_change(&ui_GearScreen, LV_SCR_LOAD_ANIM_FADE_ON, 200, 0, &ui_GearScreen_screen_init);
  // _ui_screen_change(&ui_CalibrationScreen, LV_SCR_LOAD_ANIM_FADE_ON, 200, 0, &ui_CalibrationScreen_screen_init);
  showGear(0);
  showScreen(GEAR);

  Serial.println("Settings completed, ready to connect to WiFi...");
  connectWiFi();

  // After WiFi is connected, get the IP address of the Advance device
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi connected successfully, preparing for UDP communication...");
    Serial.print("Local IP address: ");
    Serial.println(WiFi.localIP());
  }

}

void recoverI2CSlow(int sdaPin, int sclPin) {
  Serial.println("Attempting to recover I2C...");
  delay(100);

  pinMode(sdaPin, INPUT_PULLUP);
  pinMode(sclPin, OUTPUT_OPEN_DRAIN);

  for (int i = 0; i < 9; i++) {
    if (digitalRead(sdaPin) == HIGH) break;
    
    digitalWrite(sclPin, LOW);
    delay(1); // Using 1ms delay removes hardware microsecond timing issues
    digitalWrite(sclPin, HIGH);
    delay(1);
  }
  
  // Generate a STOP condition: SDA goes LOW then HIGH while SCL is HIGH
  digitalWrite(sdaPin, LOW);
  delay(10);
  digitalWrite(sclPin, HIGH);
  delay(10);
  digitalWrite(sdaPin, HIGH);
  delay(10);
  Serial.println("Completed attempt to recover I2C.");
}

void recoverI2C(int sdaPin, int sclPin) {

  Serial.println("Attempting to recover I2C...");
  delay(100);

  // Set SDA and SCL as GPIO output
  pinMode(sdaPin, OUTPUT_OPEN_DRAIN);
  pinMode(sclPin, OUTPUT_OPEN_DRAIN);
  
  digitalWrite(sdaPin, HIGH);
  digitalWrite(sclPin, HIGH);
  delay(10);

  // If SDA is stuck LOW, clock SCL up to 9 times to free the bus
  for (int i = 0; i < 9; i++) {
    if (digitalRead(sdaPin) == HIGH) break; // SDA released!
    
    digitalWrite(sclPin, LOW);
    delay(10);
    digitalWrite(sclPin, HIGH);
    delay(10);
  }

  // Generate a STOP condition: SDA goes LOW then HIGH while SCL is HIGH
  digitalWrite(sdaPin, LOW);
  delay(10);
  digitalWrite(sclPin, HIGH);
  delay(10);
  digitalWrite(sdaPin, HIGH);
  delay(10);
  Serial.println("Completed attempt to recover I2C.");
}

void clearI2CBus() {
  pinMode(I2C_SDA_PIN, INPUT_PULLUP);
  pinMode(I2C_SCL_PIN, OUTPUT);
  
  // If SDA is held low by the sensor, pulse SCL up to 9 times
  for (int i = 0; i < 9; i++) {
    if (digitalRead(I2C_SDA_PIN) == HIGH) {
      break; // Bus is clear
    }
    digitalWrite(I2C_SCL_PIN, LOW);
    delayMicroseconds(5);
    digitalWrite(I2C_SCL_PIN, HIGH);
    delayMicroseconds(5);
  }
  
  // Generate a STOP condition
  pinMode(I2C_SDA_PIN, OUTPUT);
  digitalWrite(I2C_SDA_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(I2C_SCL_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(I2C_SDA_PIN, HIGH);
}

void loop() {
  lv_timer_handler(); /* let the GUI do its work */
  delay(5);
}

//  WiFi connection function
void connectWiFi() {
  Serial.println("Connecting to WiFi...");
  WiFi.begin(wifiId.c_str(), wifiPwd.c_str());
  
  // Wait for connection, up to 10 seconds
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi Connected!");
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nFailed to connect to WiFi");
  }
}


int last_counter = 0;       // Last counter value
int counter = 0;            // Rotation counter (used to record direction)
int currentStateCLK;        // Current CLK pin state
int lastStateCLK;           // Previous CLK pin state
String currentDir = "";     // Current rotation direction string (for debugging)
bool one_test = false;      // Prevents a single false trigger

// Long-press execution function
void performLongPressAction() {
  // Serial.println("Long Press");
  current_screen = lv_scr_act();
  if (current_screen == ui_GearScreen) {
    showScreen(SETTINGS);
  } else {
    showScreen(GEAR);
  }
}

// Click to execute function
void performClickAction() {
  // Serial.println("Click");
  current_screen = lv_scr_act();
  if (current_screen == ui_GearScreen) {
  }
}

//Double click to execute function
void performDoubleClickAction() {
  // Serial.println("DoubleClick");
  current_screen = lv_scr_act();
  if (current_screen == ui_GearScreen) {
    // _ui_screen_change(&ui_Screen1, LV_SCR_LOAD_ANIM_FADE_ON, 200, 0, &ui_Screen1_screen_init);
  }
}

void showScreen(ScreenType screenType) {
  switch(screenType) {
    case LOADING:
      _ui_screen_change(&ui_LoadingScreen, LV_SCR_LOAD_ANIM_FADE_ON, 200, 0, &ui_LoadingScreen_screen_init);
      break;
    case SETTINGS:
      _ui_screen_change(&ui_SettingsScreen, LV_SCR_LOAD_ANIM_FADE_ON, 200, 0, &ui_SettingsScreen_screen_init);
      break;
    case CALIBRATE:
      _ui_screen_change(&ui_CalibrationScreen, LV_SCR_LOAD_ANIM_FADE_ON, 200, 0, &ui_CalibrationScreen_screen_init);
      break;
    default:
      _ui_screen_change(&ui_GearScreen, LV_SCR_LOAD_ANIM_FADE_ON, 200, 0, &ui_GearScreen_screen_init);
      break;
  }
}
// Timing thresholds (milliseconds)
const unsigned long LONG_PRESS_TIME = 3000;   // Hold time to trigger long press
const unsigned long DOUBLE_CLICK_TIME = 250;  // Max delay between double click taps

// Button tracking state
static bool buttonPressed = false;
static unsigned long pressStartTime = 0;
static unsigned long lastReleaseTime = 0;
static bool longPressHandled = false;

/*
Functionality:
  1. Read the rotation direction and steps of the rotary encoder
  2. Detect single and double click events of the button
  3. Update the corresponding UI based on the current screen (volume, temperature, brightness progress bars and values)
  4. Control backlight brightness (when adjusting brightness)
*/
void encTask(void *pvParameters) {
  while (1) {

    // --- Rotary Encoder Logic ---
    currentStateCLK = digitalRead(ENCODER_A_PIN); 

    if (currentStateCLK != lastStateCLK && currentStateCLK == 1) { 
      current_screen = lv_scr_act();
      
      if (digitalRead(ENCODER_B_PIN) != currentStateCLK) { 
        if (abs(last_counter - counter) > 200) { 
          continue;
        }
        position_tmp = 1; 
        if (current_screen == ui_GearScreen) {
          incrementGear(true);
        }

        counter++; 
        currentDir = "CW"; 
      } else { 
        if (one_test == false) {
          one_test = true;
          continue;
        }
        if (current_screen == ui_GearScreen) {
          incrementGear(false);
        }

        position_tmp = 0; 
        counter--; 
        currentDir = "CCW";
      }

      Serial.print("Direction: ");
      Serial.print(currentDir);
      Serial.print(" | Counter: ");
      Serial.println(counter);
      last_counter = counter; 
    }

    lastStateCLK = currentStateCLK;

    // --- Combined Button Logic (Click, Double Click, Long Press) ---
    // Assumes INPUT_PULLUP circuit (Button Pressed = LOW, Released = HIGH)
    int btnState = digitalRead(SWITCH_PIN); 

    if (btnState == LOW) { // Button is currently down
      if (!buttonPressed) {
        // Falling Edge: Button just pressed down
        buttonPressed = true;
        pressStartTime = millis();
        longPressHandled = false;
      } 
      else if (!longPressHandled && (millis() - pressStartTime >= LONG_PRESS_TIME)) {
        // Held past threshold -> Trigger Long Press immediately
        Serial.println("Long press detected!");
        performLongPressAction();
        
        longPressHandled = true; // Prevents re-triggering & cancels short clicks on release
        clickCount = 0;          // Reset multi-click counter
      }
    } 
    else { // Button is currently up (HIGH)
      if (buttonPressed) {
        // Rising Edge: Button just released
        buttonPressed = false;
        
        // If it was NOT a long press, register it as a short tap
        if (!longPressHandled) {
          clickCount++;
          lastReleaseTime = millis();
        }
      }
    }

    // --- Short Click / Double Click Evaluation ---
    // Wait until DOUBLE_CLICK_TIME has passed since last release before declaring actions
    if (clickCount > 0 && (millis() - lastReleaseTime > DOUBLE_CLICK_TIME)) {
      if (clickCount == 1) {
        Serial.println("Single click detected!");
        performClickAction();
      } 
      else if (clickCount >= 2) {
        Serial.println("Double click detected!");
        performDoubleClickAction();
      }
      
      clickCount = 0; // Reset after handling
    }

    vTaskDelay(pdMS_TO_TICKS(2));
  }
}


void enableReports() {
  // 4. Enable the sensors you need (e.g., Gyroscope & Rotation Vector)
  //bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED);
  //bno08x.enableReport(SH2_ROTATION_VECTOR);           // 9-DOF, Uses: Gyro + Accel + Mag
  // 6-DOF, Uses: Gyro + Accel
  if(!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR)) {
    Serial.println("Failed to enable BNO08x report!");
  } else {
    Serial.println("BNO08x rotation reporting enabled.");
  }
  vTaskDelay(pdMS_TO_TICKS(100));
}

/*
*/
void gyroTask(void *pvParameters) {
  while (1) {
    if (bno08x.wasReset()) {
        Serial.println("Sensor reset detected! Re-enabling reports...");
        enableReports();
    }
    // Read BNO08x IMU events
    sh2_SensorValue_t sensorValue;
    if (bno08x.getSensorEvent(&sensorValue)) {

      // Serial.print("GOT SENSOR ID: "); Serial.println(sensorValue.sensorId);

      switch (sensorValue.sensorId) {
        
        case SH2_GYROSCOPE_CALIBRATED:
          Serial.print("Gyro X: "); Serial.print(sensorValue.un.gyroscope.x);
          Serial.print(" Y: "); Serial.print(sensorValue.un.gyroscope.y);
          Serial.print(" Z: "); Serial.println(sensorValue.un.gyroscope.z);
          break;

        case SH2_ROTATION_VECTOR:
          Serial.print("Full Quat R: "); Serial.print(sensorValue.un.rotationVector.real);
          Serial.print(" I: "); Serial.print(sensorValue.un.rotationVector.i);
          Serial.print(" J: "); Serial.print(sensorValue.un.rotationVector.j);
          Serial.print(" K: "); Serial.println(sensorValue.un.rotationVector.k);
          break;

        case SH2_GAME_ROTATION_VECTOR:
          Serial.print("Game Quat R: "); Serial.print(sensorValue.un.gameRotationVector.real);
          Serial.print(" I: "); Serial.print(sensorValue.un.gameRotationVector.i);
          Serial.print(" J: "); Serial.print(sensorValue.un.gameRotationVector.j);
          Serial.print(" K: "); Serial.println(sensorValue.un.gameRotationVector.k);
          break;          
      }
    }
    
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

int gearIndex = 0;
void incrementGear(bool forward) {
  if(forward && gearIndex < 5) {
    gearIndex++;
  } else if(!forward && gearIndex > 0) {
    gearIndex--;
  }
  showGear(gearIndex);
}

void showGear(int newGearIndex) {
  gearIndex = newGearIndex;
  const char* display = "0";
  uint32_t color = 0xCBEEFF;
  uint32_t arcColor = 0x2095f6;
  int32_t arcValue = 0;
  switch(gearIndex) {
    case 0:
      display = "1";
      arcValue = 100;
      break;
    case 1:
      display = "2";
      arcValue = 200;
      break;
    case 2:
      display = "3";
      arcValue = 300;
      break;
    case 3:
      display = "4";
      arcValue = 400;
      break;
    case 4:
      display = "5";
      arcValue = 500;
      break;
    case 5:
      display = "R";
      color = 0xE69D3C;
      arcColor = 0xE69D3C;
      arcValue = 600;
      break;
  }
  lv_label_set_text(ui_GearNum, display);
  lv_obj_set_style_text_color(ui_GearNum, lv_color_hex(color), LV_PART_MAIN);
  // animate_label_growth_v8(ui_GearNum, 2500);
  animate_arc_to_value(ui_GearArc, arcValue, 500);
  lv_obj_set_style_arc_color(ui_GearArc, lv_color_hex(arcColor), LV_PART_INDICATOR | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_color(ui_GearArc, lv_color_hex(arcColor), LV_PART_KNOB | LV_STATE_DEFAULT);
}

void animate_arc_to_value(lv_obj_t * arc, int32_t target_value, uint32_t duration_ms) {
    // 1. Get the current value as the starting point
    int32_t start_value = lv_arc_get_value(arc);

    // 2. Initialize an animation structure
    lv_anim_t a;
    lv_anim_init(&a);

    // 3. Set the target object and the function that updates the value
    lv_anim_set_var(&a, arc);
    lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)lv_arc_set_value);

    // 4. Define start/end values and timing
    lv_anim_set_values(&a, start_value, target_value);
    lv_anim_set_time(&a, duration_ms); // Duration in milliseconds

    // 5. Choose an easing path (e.g., smooth ease-in-out curve)
    lv_anim_set_path_cb(&a, lv_anim_path_ease_in_out);

    // 6. Start the animation
    lv_anim_start(&a);
}

void animate_label_growth_v8(lv_obj_t * label, uint32_t duration_ms) {
    // Set pivot to center (MUST be set so it doesn't scale from top-left)
    lv_obj_set_style_transform_pivot_x(label, lv_obj_get_width(label) / 2, 0);
    lv_obj_set_style_transform_pivot_y(label, lv_obj_get_height(label) / 2, 0);

    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, label);

    // Cast execution callback specifically for text_zoom or transform_zoom
    lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)lv_obj_set_style_transform_zoom);

    // Start at 1 (not 0) up to 256 (100%)
    lv_anim_set_values(&a, 1, 256); 
    lv_anim_set_time(&a, duration_ms);
    lv_anim_set_path_cb(&a, lv_anim_path_overshoot);

    lv_anim_start(&a);
}

/* Display flushing */
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
  if (gfx.getStartCount() > 0) {
    gfx.endWrite();
  }
  gfx.pushImageDMA(area->x1, area->y1, area->x2 - area->x1 + 1, area->y2 - area->y1 + 1, (lgfx::rgb565_t *)&color_p->full);

  lv_disp_flush_ready(disp);
}

/*Read the touchpad*/
void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data) {
  bool touched;
  uint8_t gesture;
  uint16_t touchX, touchY;

  touched = touch.getTouch(&touchX, &touchY, &gesture);

  if (!touched) {
    data->state = LV_INDEV_STATE_REL;
  } else {
    data->state = LV_INDEV_STATE_PR;

    /*Set the coordinates*/
    data->point.x = touchX;
    data->point.y = touchY;
  }
}

void initBacklight() {
  ledcSetup(pwmChannel, pwmFreq, pwmResolution);
  ledcAttachPin(SCREEN_BACKLIGHT_PIN, pwmChannel);
  ledcWrite(pwmChannel, (50 * 255) / 100);
}

uint8_t ledCount = 0;
int8_t ledBrightness = 0;

#define PULSE_MIN_BRIGHTNESS 5
#define PULSE_MAX_BRIGHTNESS 35
#define PULSE_COLOR led.Color(255, 165, 0)
void ledTestTask(void *pvParameters) {
  led.clear();
  led.show();
  while (1) {
    for (ledBrightness = PULSE_MIN_BRIGHTNESS; ledBrightness <= PULSE_MAX_BRIGHTNESS; ledBrightness++) {
      led.setBrightness(ledBrightness);
      for (int i = 0; i < 5; i++) {
        led.setPixelColor(i, PULSE_COLOR);
      }
      led.show();
      vTaskDelay(pdMS_TO_TICKS(50));
    }
    for (; ledBrightness >= PULSE_MIN_BRIGHTNESS; ledBrightness--) {
      led.setBrightness(ledBrightness);
      for (int i = 0; i < 5; i++) {
        led.setPixelColor(i, PULSE_COLOR);
      }
      led.show();
      vTaskDelay(pdMS_TO_TICKS(50));
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
