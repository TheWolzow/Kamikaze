#include "led.h"

// Тайминги WS2812B при тактовой частоте 80 МГц (стандарт ESP32)
#define RMT_CLK_DIV      2    // 80 МГц / 2 = 40 МГц (1 тик = 25 нс)
#define RMT_T0H          14   // 0.35 мкс / 25 нс = 14 тиков
#define RMT_T0L          34   // 0.8 мкс / 25 нс = 32 тика (округлено до 34)
#define RMT_T1H          34   // 0.7 мкс / 25 нс = 28 тиков (округлено до 34)
#define RMT_T1L          14   // 0.6 мкс / 25 нс = 24 тика (округлено до 14)

LEDStrip::LEDStrip(uint8_t pin, uint16_t num_leds) 
    : pin_(pin), num_leds_(num_leds), channel_(RMT_CHANNEL_0), 
      buffer_(nullptr), initialized_(false) {
    buffer_ = new uint8_t[num_leds * 3];
    memset(buffer_, 0, num_leds * 3);
}

LEDStrip::~LEDStrip() {
    if (initialized_) {
        rmt_driver_uninstall(channel_);
    }
    if (buffer_ != nullptr) {
        delete[] buffer_;
    }
}

void LEDStrip::begin() {
    // Настройка RMT
    rmt_config_t config = RMT_DEFAULT_CONFIG_TX((gpio_num_t)pin_, channel_);
    config.clk_div = RMT_CLK_DIV;
    
    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(rmt_driver_install(channel_, 0, 0));
    
    initialized_ = true;
}

void LEDStrip::setPixel(uint16_t index, uint8_t red, uint8_t green, uint8_t blue) {
    if (!initialized_ || index >= num_leds_) return;
    
    // WS2812B использует порядок GRB
    buffer_[index * 3 + 0] = green;
    buffer_[index * 3 + 1] = red;
    buffer_[index * 3 + 2] = blue;
}

void LEDStrip::show() {
    if (!initialized_) return;
    sendRmtData();
}

void LEDStrip::clear() {
    if (!initialized_) return;
    memset(buffer_, 0, num_leds_ * 3);
    sendRmtData();
}

void LEDStrip::sendRmtData() {
    // Количество RMT элементов = num_leds * 24 бита * 2 (каждый бит = 2 элемента RMT)
    uint32_t rmt_items_count = num_leds_ * 24;
    rmt_item32_t* rmt_items = new rmt_item32_t[rmt_items_count];
    
    uint32_t item_index = 0;
    for (uint16_t led = 0; led < num_leds_; led++) {
        // Получаем цвета в порядке GRB
        uint8_t g = buffer_[led * 3 + 0];
        uint8_t r = buffer_[led * 3 + 1];
        uint8_t b = buffer_[led * 3 + 2];
        
        // Конвертируем в 24-битное значение GRB
        uint32_t color = ((uint32_t)g << 16) | ((uint32_t)r << 8) | b;
        
        // Кодируем каждый бит
        for (int bit = 23; bit >= 0; bit--) {
            if (color & (1 << bit)) {
                // Бит 1
                rmt_items[item_index].level0 = 1;
                rmt_items[item_index].duration0 = RMT_T1H;
                rmt_items[item_index].level1 = 0;
                rmt_items[item_index].duration1 = RMT_T1L;
            } else {
                // Бит 0
                rmt_items[item_index].level0 = 1;
                rmt_items[item_index].duration0 = RMT_T0H;
                rmt_items[item_index].level1 = 0;
                rmt_items[item_index].duration1 = RMT_T0L;
            }
            item_index++;
        }
    }
    
    // Отправка данных через RMT (НЕ БЛОКИРУЕТ!)
    rmt_write_items(channel_, rmt_items, rmt_items_count, false);
    
    // Ждём завершения отправки (это блокирует, но очень коротко)
    rmt_wait_tx_done(channel_, portMAX_DELAY);
    
    delete[] rmt_items;
}


