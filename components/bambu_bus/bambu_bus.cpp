#include "bambu_bus.h"
#include "esphome/core/preferences.h"
#include "esphome/core/log.h"
namespace bambu_bus {

void BambuBus::setup() {
    // ESP_LOGI(BambuBus::TAG, "Setup started");
    // 确保全局偏好已初始化
    if (!esphome::global_preferences) {
        esphome::global_preferences = esphome::global_preferences;
    }
    BambuBus_init();
}

void BambuBus::loop() {
    static uint8_t buf[1000];
    static size_t pos = 0;
    
    // Read incoming data
    while (available()) {
        uint8_t c;
        if (read_byte(&c)) {
            RX_IRQ(c);
        }
    }
    
    // Process received data
    // BambuBus_run();
}

void BambuBus::BambuBUS_UART_Init() {
    // UART is initialized by ESPHome
}

void BambuBus::BambuBus_init() {
    bool _init_ready = Bambubus_read();
    
    if (!_init_ready) {
        // Initialize default filament colors
        data_save.filament[0][0].color_R = 0xFF;
        data_save.filament[0][0].color_G = 0x00;
        data_save.filament[0][0].color_B = 0x00;
        
        data_save.filament[0][1].color_R = 0x00;
        data_save.filament[0][1].color_G = 0xFF;
        data_save.filament[0][1].color_B = 0x00;
        
        data_save.filament[0][2].color_R = 0x00;
        data_save.filament[0][2].color_G = 0x00;
        data_save.filament[0][2].color_B = 0xFF;
        
        data_save.filament[0][3].color_R = 0x88;
        data_save.filament[0][3].color_G = 0x88;
        data_save.filament[0][3].color_B = 0x88;
        
        // Initialize other AMS slots
        for (int i = 1; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                data_save.filament[i][j].color_R = 0xFF >> (i + 1);
                data_save.filament[i][j].color_G = 0xFF >> (i + 1);
                data_save.filament[i][j].color_B = 0xFF >> (i + 1);
            }
        }
    }
    
    // Set initial status
    for (auto &i : data_save.filament) {
        for (auto &j : i) {
            j.statu = offline;
            j.motion_set = idle;
        }
    }
}

bool BambuBus::Bambubus_read() {
    // 使用 ESPHome 的偏好存储系统
    pref_ = esphome::global_preferences->make_preference<flash_save_struct>(0, false);
    
    if (pref_.load(&data_save)) {
        return (data_save.check == 0x40614061) && (data_save.version == 1);
    }
    return false;
}

void BambuBus::Bambubus_save() {
    pref_ = esphome::global_preferences->make_preference<flash_save_struct>(0, false);
    pref_.save(&data_save);
}

void BambuBus::RX_IRQ(uint8_t data) {
    static int _index = 0;
    static int length = 500;
    static uint8_t data_length_index;
    static uint8_t data_CRC8_index;
    
    if (_index == 0) {
        if (data == 0x3D) {
            BambuBus_data_buf[0] = 0x3D;
            _RX_IRQ_crcx.restart();
            _RX_IRQ_crcx.add(0x3D);
            data_length_index = 4;
            length = data_CRC8_index = 6;
            _index = 1;
        }
        return;
    } else {
        BambuBus_data_buf[_index] = data;
        if (_index == 1) {
            if (data & 0x80) {
                data_length_index = 2;
                data_CRC8_index = 3;
            } else {
                data_length_index = 4;
                data_CRC8_index = 6;
            }
        }
        if (_index == data_length_index) {
            length = data;
        }
        if (_index < data_CRC8_index) {
            _RX_IRQ_crcx.add(data);
        } else if (_index == data_CRC8_index) {
            if (data != _RX_IRQ_crcx.calc()) {
                _index = 0;
                return;
            }
        }
        ++_index;
        if (_index >= length) {
            _index = 0;
            memcpy(buf_X, BambuBus_data_buf, length);
            BambuBus_have_data = length;
        }
        if (_index >= 999) {
            _index = 0;
        }
    }
}

void BambuBus::send_uart(const uint8_t *data, uint16_t length) {
    write_array(data, length);
}

bool BambuBus::package_check_crc16(uint8_t *data, int data_length) {
    crc_16.restart();
    data_length -= 2;
    for (auto i = 0; i < data_length; i++) {
        crc_16.add(data[i]);
    }
    uint16_t num = crc_16.calc();
    return (data[data_length] == (num & 0xFF)) && (data[data_length + 1] == ((num >> 8) & 0xFF));
}

void BambuBus::package_send_with_crc(uint8_t *data, int data_length) {
    crc_8.restart();
    if (data[1] & 0x80) {
        for (auto i = 0; i < 3; i++) {
            crc_8.add(data[i]);
        }
        data[3] = crc_8.calc();
    } else {
        for (auto i = 0; i < 6; i++) {
            crc_8.add(data[i]);
        }
        data[6] = crc_8.calc();
    }
    
    crc_16.restart();
    data_length -= 2;
    for (auto i = 0; i < data_length; i++) {
        crc_16.add(data[i]);
    }
    uint16_t num = crc_16.calc();
    data[data_length] = num & 0xFF;
    data[data_length + 1] = num >> 8;
    data_length += 2;
    
    send_uart(data, data_length);
    
    if (need_debug) {
        // ESP_LOGD("BambuBus", "Sent package:");
        for (int i = 0; i < data_length; i++) {
            // ESP_LOGD("BambuBus", "%02X ", data[i]);
        }
        need_debug = false;
    }
}

package_type BambuBus::get_packge_type(uint8_t *buf, int length) {
    if (!package_check_crc16(buf, length)) {
        return BambuBus_package_NONE;
    }
    
    if (buf[1] == 0xC5) {
        switch (buf[4]) {
            case 0x03: return BambuBus_package_filament_motion_short;
            case 0x04: return BambuBus_package_filament_motion_long;
            case 0x05: return BambuBus_package_online_detect;
            case 0x06: return BambuBus_package_REQx6;
            case 0x07: return BambuBus_package_NFC_detect;
            case 0x08: return BambuBus_package_set_filament;
            case 0x20: return BambuBus_package_heartbeat;
            default: return BambuBus_package_ETC;
        }
    } else if (buf[1] == 0x05) {
        // Long package handling would go here
        return BambuBus_package_ETC;
    }
    
    return BambuBus_package_NONE;
}

}
// Implement all the remaining methods following the same pattern...
// [Rest of the implementation would follow the same structure as above]