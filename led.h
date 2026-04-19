#ifndef LED_STRIP_H
#define LED_STRIP_H

#include <Arduino.h>
#include <driver/rmt.h>

class LEDStrip {
public:
    /**
     * @brief Конструктор
     * @param pin Пин GPIO (например, 13)
     * @param num_leds Количество светодиодов в ленте
     */
    LEDStrip(uint8_t pin, uint16_t num_leds);
    
    /**
     * @brief Деструктор
     */
    ~LEDStrip();
    
    /**
     * @brief Инициализация RMT канала
     */
    void begin();
    
    /**
     * @brief Установка цвета светодиода
     * @param index Индекс (0..num_leds-1)
     * @param red Красный (0-255)
     * @param green Зелёный (0-255)
     * @param blue Синий (0-255)
     */
    void setPixel(uint16_t index, uint8_t red, uint8_t green, uint8_t blue);
    
    /**
     * @brief Отправка данных на ленту (НЕ БЛОКИРУЕТ CPU!)
     */
    void show();
    
    /**
     * @brief Выключить все светодиоды
     */
    void clear();
    
    /**
     * @brief Получить количество светодиодов
     */
    uint16_t numLeds() const { return num_leds_; }

private:
    uint8_t pin_;
    uint16_t num_leds_;
    rmt_channel_t channel_;
    uint8_t* buffer_;
    bool initialized_;
    
    // Конвертация RGB в формат WS2812B (GRB)
    void fillRmtItem(uint32_t index, uint8_t r, uint8_t g, uint8_t b);
    void sendRmtData();
};

#endif