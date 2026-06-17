#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

// #define ARM_BUTTON_PIN 6
#define RESET_BUTTON_PIN 7

#define JOYSTICK1_VRX_PIN 3
#define JOYSTICK1_VRY_PIN 4
#define JOYSTICK1_SW_PIN  20

#define JOYSTICK2_VRX_PIN 3
#define JOYSTICK2_VRY_PIN 4
#define JOYSTICK2_SW_PIN  20

typedef struct
{
    float roll;
    float pitch;
    float yaw;
}
attitude_data_packet_t;

typedef struct
{
    bool arm;
    bool reset;

    bool throttle_up;
    bool throttle_down;

    bool direction_up;
    bool direction_down;
    bool direction_left;
    bool direction_right;
}
command_data_packet_t;

enum class ThrottleState
{
    UNKNOWN,
    NEUTRAL,
    UP,
    DOWN
};

static attitude_data_packet_t attitude_data;
static command_data_packet_t command_data;

static uint32_t last_arm_button_press_time = 0;
static uint32_t last_reset_button_press_time = 0;
static uint32_t last_joystick1_sw_press_time = 0;
static uint32_t last_joystick2_sw_press_time = 0;

static bool last_arm_button_state = HIGH;
static bool last_reset_button_state = HIGH;
static bool last_joystick1_sw_state = HIGH;
static bool last_joystick2_sw_state = HIGH;

static bool is_armed = false;
static bool is_reset = false;

static ThrottleState last_throttle_state = ThrottleState::UNKNOWN;

// replace with your drone's mac address
static uint8_t drone_address[] = {0xEC, 0xDA, 0x3B, 0xBF, 0x7B, 0xC4};

void send_command()
{
    esp_err_t result = esp_now_send(
        drone_address,
        reinterpret_cast<uint8_t *>(&command_data),
        sizeof(command_data)
    );

    if (result != ESP_OK)
    {
        Serial.println("ESP-NOW send failed");
    }
}

ThrottleState read_throttle_state(int xValue1, int yValue1)
{
    if ((xValue1 < 3950 && xValue1 > 3850) &&
        (yValue1 < 3800 && yValue1 > 3700))
    {
        return ThrottleState::NEUTRAL;
    }

    if (yValue1 > 3600)
    {
        if (xValue1 > 4000)
        {
            return ThrottleState::UP;
        }
        else if (xValue1 < 10)
        {
            return ThrottleState::DOWN;
        }
    }

    return ThrottleState::UNKNOWN;
}

void apply_throttle_state_to_command(ThrottleState throttle_state)
{
    switch (throttle_state)
    {
        case ThrottleState::UP:
            Serial.println("Throttle Up");

            command_data.throttle_up = true;
            command_data.throttle_down = false;
            break;

        case ThrottleState::DOWN:
            Serial.println("Throttle Down");

            command_data.throttle_up = false;
            command_data.throttle_down = true;
            break;

        case ThrottleState::NEUTRAL:
            Serial.println("Throttle Neutral");

            command_data.throttle_up = false;
            command_data.throttle_down = false;
            break;

        case ThrottleState::UNKNOWN:
        default:
            break;
    }
}

void on_data_recv(const uint8_t * mac, const uint8_t *data, int len) 
{
    memcpy(&attitude_data, data, sizeof(attitude_data));

    // Serial.print("\r\nBytes received: ");
    // Serial.println(len);
    // Serial.print("Roll: ");
    // Serial.println(attitude_data.roll);
    // Serial.print("Pitch: ");
    // Serial.println(attitude_data.pitch);
    // Serial.print("Yaw: ");
    // Serial.println(attitude_data.yaw);
    // Serial.println();
}

void on_data_sent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    Serial.print("\nLast Packet Send Status: ");

    if (status == ESP_NOW_SEND_SUCCESS) 
    {
        Serial.println("Delivery Success");
    } 
    else 
    {
        Serial.println("Delivery Fail");
    }
} 

void setup() 
{
    Serial.begin(115200);
    delay(2000);

    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK) 
    {
        Serial.println("Error initializing ESP-NOW");
        for(;;) {}
    }

    esp_now_register_recv_cb(on_data_recv);
    esp_now_register_send_cb(on_data_sent);

    // Register peer
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, drone_address, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;

    // Add peer        
    if (esp_now_add_peer(&peerInfo) != ESP_OK)
    {
        Serial.println("Failed to add peer");
        for(;;) {}
    }
    
    // pinMode(ARM_BUTTON_PIN, INPUT_PULLUP);
    pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);

    pinMode(JOYSTICK1_SW_PIN, INPUT_PULLUP);
    // pinMode(JOYSTICK2_SW_PIN, INPUT_PULLUP);

    analogReadResolution(12); // Set ADC resolution to 12 bits (0-4095)
}

void loop() 
{
    const uint32_t now = millis();

    // bool current_arm_button_state = digitalRead(ARM_BUTTON_PIN);
    bool current_reset_button_state = digitalRead(RESET_BUTTON_PIN);
    bool current_joystick1_sw_state = digitalRead(JOYSTICK1_SW_PIN);
    // bool current_joystick2_sw_state = digitalRead(JOYSTICK2_SW_PIN);

    int xValue1 = analogRead(JOYSTICK1_VRX_PIN);
    int yValue1 = analogRead(JOYSTICK1_VRY_PIN);

    // handle arm button
    if ((last_joystick1_sw_state == HIGH) &&
        (current_joystick1_sw_state == LOW) &&
        ((now - last_joystick1_sw_press_time) > 200))
    {
        last_joystick1_sw_press_time = now;

        is_armed = !is_armed;

        command_data.arm = is_armed;
        command_data.reset = false;
        command_data.throttle_up = false;
        command_data.throttle_down = false;

        last_throttle_state = ThrottleState::UNKNOWN;

        if (is_armed == true)
        {
            Serial.println("Button Pressed Arm");
        }
        else
        {
            Serial.println("Button Pressed Disarm");
        }

        send_command();
    }

    last_joystick1_sw_state = current_joystick1_sw_state;

    // Handle reset button
    if ((last_reset_button_state == HIGH) &&
        (current_reset_button_state == LOW) &&
        ((now - last_reset_button_press_time) > 200))
    {
        last_reset_button_press_time = now;

        Serial.println("Button Pressed Reset");

        is_armed = false;

        command_data.arm = false;
        command_data.reset = true;
        command_data.throttle_up = false;
        command_data.throttle_down = false;

        last_throttle_state = ThrottleState::UNKNOWN;

        send_command();

        command_data.arm = false;
        command_data.reset = false;
        command_data.throttle_up = false;
        command_data.throttle_down = false;
    }

    last_reset_button_state = current_reset_button_state;

    if (is_armed == true)
    {
        ThrottleState current_throttle_state = read_throttle_state(xValue1, yValue1);

        if (current_throttle_state != ThrottleState::UNKNOWN &&
            current_throttle_state != last_throttle_state)
        {
            command_data.arm = true;
            command_data.reset = false;

            apply_throttle_state_to_command(current_throttle_state);

            send_command();

            last_throttle_state = current_throttle_state;
        }
    }
}