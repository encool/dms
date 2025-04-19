#include "esphome/core/log.h"
#include "bambu_bus.h"
#include "esphome/core/preferences.h"
#include "esphome/core/hal.h" 
namespace bambu_bus {


void BambuBus::setup() {
    ESP_LOGCONFIG(BambuBus::TAG, "Setup started");
    // 确保全局偏好已初始化
    if (!esphome::global_preferences) {
        esphome::global_preferences = esphome::global_preferences;
    }
    
   // 设置 DE 引脚 (如果已配置)
   if (this->de_pin_ != nullptr) {
        // GPIOBinaryOutput* 的 setup 通常由框架自动调用
        // this->de_pin_->setup(); // 可能不需要
        this->de_pin_->digital_write(false); // <<<--- 使用 turn_off() 设置初始状态 (接收)
        // vvv--- 获取引脚号需要通过 get_pin() 方法 ---vvv
        ESP_LOGCONFIG(TAG, "DE Pin (GPIOBinaryOutput) configured on GPIO%d. Initial state: OFF (Receive)", this->de_pin_->get_pin());
        // ^^^--- 注意是 de_pin_->get_pin()->get_pin() ---^^^
    } else {
        ESP_LOGCONFIG(TAG, "DE Pin not configured.");
    }

    BambuBus_init();
}

void BambuBus::loop() {
    static uint8_t buf[1000];
    static size_t pos = 0;

    // Read incoming data
    while (available()) {
        // ESP_LOGI(BambuBus::TAG, "data available");
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
    ESP_LOGI(BambuBus::TAG, "Setup started BambuBus_init");
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
    
    // 打印每次进入函数时的基本信息
    // ESP_LOGI(TAG, "RX_IRQ: data=0x%02X, _index=%d, length=%d", data, _index, length);

    if (_index == 0) {
        if (data == 0x3D) {
            ESP_LOGD(TAG, "Start of frame detected (0x3D)");
            BambuBus_data_buf[0] = 0x3D;
            _RX_IRQ_crcx.restart();
            _RX_IRQ_crcx.add(0x3D);
            data_length_index = 4;
            length = data_CRC8_index = 6;
            _index = 1;
        } else {
            ESP_LOGVV(TAG, "Waiting for start byte (0x3D), got 0x%02X", data);
        }
        return;
    } else {
        BambuBus_data_buf[_index] = data;
        
        // 打印每个字节的存储位置
        // ESP_LOGD(TAG, "Stored data 0x%02X at index %d", data, _index);

        if (_index == 1) {
            if (data & 0x80) {
                data_length_index = 2;
                data_CRC8_index = 3;
                ESP_LOGD(TAG, "Short frame format detected (bit7 set)");
            } else {
                data_length_index = 4;
                data_CRC8_index = 6;
                ESP_LOGD(TAG, "Long frame format detected");
            }
        }

        if (_index == data_length_index) {
            length = data;
            ESP_LOGD(TAG, "Frame length set to %d bytes", length);
        }

        if (_index < data_CRC8_index) {
            _RX_IRQ_crcx.add(data);
            ESP_LOGVV(TAG, "Added 0x%02X to CRC (index %d)", data, _index);
        } else if (_index == data_CRC8_index) {
            uint8_t crc = _RX_IRQ_crcx.calc();
            ESP_LOGD(TAG, "CRC check: received=0x%02X, calculated=0x%02X", data, crc);
            if (data != crc) {
                ESP_LOGW(TAG, "CRC mismatch! Resetting frame parser");
                _index = 0;
                return;
            }
        }

        ++_index;
        if (_index >= length) {
            ESP_LOGI(TAG, "Complete frame received (%d bytes)", length);
            // 使用 ESPHome 的 format_hex_pretty
            std::string hexdump = esphome::format_hex_pretty(BambuBus_data_buf, length);
            // 注意：可能需要分行打印，如果 hexdump 太长
            ESP_LOGD(TAG, "Received Data:\n%s", hexdump.c_str());            
            _index = 0;
            memcpy(buf_X, BambuBus_data_buf, length);
            BambuBus_have_data = length;
        }

        if (_index >= 999) {
            ESP_LOGW(TAG, "Index overflow detected, resetting parser");
            _index = 0;
        }
    }
}

void BambuBus::send_uart(const uint8_t *data, uint16_t length) {
    write_array(data, length);
}

// 用于带 DE 控制发送的新函数
void BambuBus::send_uart_with_de(const uint8_t *data, uint16_t length) {
    if (this->de_pin_ != nullptr) {
        this->de_pin_->digital_write(true); // 激活发送 (高电平)
        // 根据收发器的需要，可能需要短暂延迟
        esphome::delayMicroseconds(10); // 示例：10 微秒
    }

    // 写入数据
    ESP_LOGD(TAG, "Sending %d bytes with DE control (if enabled)...", length);
    this->write_array(data, length);

    // 等待发送完成 - 非常重要!
    this->flush();
    ESP_LOGV(TAG, "UART flush complete.");


    if (this->de_pin_ != nullptr) {
         // 在禁用 DE 之前可能需要短暂延迟
        esphome::delayMicroseconds(10); // 示例：10 微秒
        this->de_pin_->digital_write(false); // 禁用发送 (低电平)
        ESP_LOGV(TAG, "DE pin set LOW.");
    }
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
    
    if (this->need_debug) {
        ESP_LOGD(TAG, "Prepared package to send (%d bytes):", data_length);
        // 使用 format_hex_pretty 打印准备发送的数据（需要包含 esphome/core/helpers.h）
        ESP_LOGD(TAG, "  %s", esphome::format_hex_pretty(data, data_length).c_str());
     }
    // send_uart(data, data_length);
    // 调用新函数以使用 DE 控制进行发送
    send_uart_with_de(data, data_length);
    
}

// --- 实现所有其他方法 (BambuBus_run, send_for_..., helpers 等) ---
// ... (将原始代码的逻辑迁移过来，确保使用 this-> 访问成员变量和方法) ...
// 例如 BambuBus_run:
package_type BambuBus::BambuBus_run() {
    package_type stu = BambuBus_package_NONE;
    // 假设 time_set_ 和 time_motion_ 是 uint32_t 类型的成员变量
    static uint32_t time_set_ = 0;
    static uint32_t time_motion_ = 0;

    uint32_t now = esphome::millis(); // 使用 ESPHome 的时间函数

    if (this->BambuBus_have_data > 0) {
        int data_length = this->BambuBus_have_data;
        this->BambuBus_have_data = 0; // 清除标志位
        // need_debug = false; // Reset debug flag if needed

        stu = this->get_packge_type(this->buf_X, data_length);
        ESP_LOGD(TAG, "Processing package type: %d", (int)stu);

        switch (stu) {
            case BambuBus_package_heartbeat:
                time_set_ = now + 1000; // 更新心跳时间戳
                ESP_LOGD(TAG, "Heartbeat received. Timeout extended.");
                break;
            case BambuBus_package_filament_motion_short:
                this->send_for_Cxx(this->buf_X, data_length);
                break;
            case BambuBus_package_filament_motion_long:
                this->send_for_Dxx(this->buf_X, data_length);
                time_motion_ = now + 1000; // 更新运动状态时间戳
                 ESP_LOGD(TAG, "Motion long received. Timeout extended.");
                break;
            case BambuBus_package_online_detect:
                this->send_for_Fxx(this->buf_X, data_length);
                break;
            case BambuBus_package_REQx6:
                 ESP_LOGW(TAG, "REQx6 received, but handler is not fully implemented.");
                // this->send_for_REQx6(this->buf_X, data_length); // 如果要实现的话
                break;
            case BambuBus_long_package_MC_online:
                 ESP_LOGW(TAG, "Long MC Online received, but handler might need review.");
                // this->send_for_long_packge_MC_online(this->buf_X, data_length);
                break;
            // ... (处理其他 case) ...
             case BambuBus_longe_package_filament:
                 ESP_LOGW(TAG, "Long filament info received, but handler might need review.");
                // this->send_for_long_packge_filament(this->buf_X, data_length);
                 break;
             case BambuBus_long_package_version:
                  ESP_LOGW(TAG, "Long version info received, but handler might need review.");
                // this->send_for_long_packge_version(this->buf_X, data_length);
                 break;
             case BambuBus_package_NFC_detect:
                  ESP_LOGW(TAG, "NFC detect received, but handler might need review.");
                 // this->send_for_NFC_detect(this->buf_X, data_length);
                 break;
             case BambuBus_package_set_filament:
                  ESP_LOGI(TAG, "Set filament received.");
                 this->send_for_Set_filament(this->buf_X, data_length);
                 break;
            case BambuBus_package_NONE:
                 ESP_LOGW(TAG, "Invalid package received (CRC mismatch or unknown type).");
                 break;
            default:
                 ESP_LOGW(TAG, "Received unhandled package type: %d", (int)stu);
                break;
        }
    }

    // 处理超时
    if (time_set_ != 0 && now > time_set_) { // 检查非零避免首次误判
        ESP_LOGE(TAG, "Heartbeat timeout! Assuming offline.");
        stu = BambuBus_package_ERROR; // 标记错误状态
        time_set_ = 0; // 防止重复触发错误，直到下次心跳
        // 可能需要在这里执行一些离线处理逻辑
    }
    if (time_motion_ != 0 && now > time_motion_) {
         ESP_LOGD(TAG, "Motion timeout. Setting filaments to idle.");
         for(int ams = 0; ams < 4; ++ams) {
             for(int slot = 0; slot < 4; ++slot) {
                 // Check if this filament was actually in motion before setting to idle
                 if (data_save.filament[ams][slot].motion_set != idle) {
                    data_save.filament[ams][slot].motion_set = idle;
                 }
             }
         }
        time_motion_ = 0; // 防止重复触发
    }

    // 处理保存
    if (this->Bambubus_need_to_save) {
        ESP_LOGI(TAG, "Saving data to preferences...");
        this->Bambubus_save();
        this->Bambubus_need_to_save = false;
         if (stu != BambuBus_package_ERROR) { // 如果没超时，重置超时计时器
              time_set_ = now + 1000;
         }
    }

    // NFC_detect_run(); // 如果需要实现这个逻辑

    return stu;
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