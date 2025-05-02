#include "esphome/core/log.h"
#include "bambu_bus.h"
#include "esphome/core/preferences.h"
#include "esphome/core/hal.h"
namespace bambu_bus
{

    void BambuBus::setup()
    {
        ESP_LOGCONFIG(BambuBus::TAG, "Setup started");
        // 确保全局偏好已初始化
        if (!esphome::global_preferences)
        {
            esphome::global_preferences = esphome::global_preferences;
        }

        // 设置 DE 引脚 (如果已配置)
        if (this->de_pin_ != nullptr)
        {
            // GPIOBinaryOutput* 的 setup 通常由框架自动调用
            // this->de_pin_->setup(); // 可能不需要
            this->de_pin_->digital_write(false); // <<<--- 使用 turn_off() 设置初始状态 (接收)
            // vvv--- 获取引脚号需要通过 get_pin() 方法 ---vvv
            ESP_LOGCONFIG(TAG, "DE Pin (GPIOBinaryOutput) configured on GPIO%d. Initial state: OFF (Receive)", this->de_pin_->dump_summary());
            // ^^^--- 注意是 de_pin_->get_pin()->get_pin() ---^^^
        }
        else
        {
            ESP_LOGCONFIG(TAG, "DE Pin not configured.");
        }

        BambuBus_init();
    }

    void BambuBus::loop()
    {
        static uint8_t buf[1000];
        static size_t pos = 0;

        // Read incoming data
        while (available())
        {
            // ESP_LOGI(BambuBus::TAG, "data available");
            uint8_t c;
            if (read_byte(&c))
            {
                RX_IRQ(c);
            }
        }

        // Process received data
        BambuBus_run();
    }

    void BambuBus::BambuBUS_UART_Init()
    {
        // UART is initialized by ESPHome
    }

    void BambuBus::BambuBus_init()
    {
        ESP_LOGCONFIG(BambuBus::TAG, "Setup started BambuBus_init");
        bool _init_ready = Bambubus_read();

        if (!_init_ready)
        {
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
            for (int i = 1; i < 4; i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    data_save.filament[i][j].color_R = 0xFF >> (i + 1);
                    data_save.filament[i][j].color_G = 0xFF >> (i + 1);
                    data_save.filament[i][j].color_B = 0xFF >> (i + 1);
                }
            }
        }
        else
        {
            ESP_LOGCONFIG(TAG, "成功从 Preferences 加载数据。");
        }
        // Set initial status
        for (auto &i : data_save.filament)
        {
            for (auto &j : i)
            {
                // j.statu = NFC_waiting;
                j.statu = online;
                j.motion_set = idle;
                // j.motion_set = need_send_out;
            }
        }
    }

    bool BambuBus::Bambubus_read()
    {
        // 使用 ESPHome 的偏好存储系统
        pref_ = esphome::global_preferences->make_preference<flash_save_struct>(0, false);

        if (pref_.load(&data_save))
        {
            return (data_save.check == 0x40614061) && (data_save.version == 1);
        }
        return false;
    }

    void BambuBus::Bambubus_save()
    {
        pref_ = esphome::global_preferences->make_preference<flash_save_struct>(0, false);
        pref_.save(&data_save);
    }

    void BambuBus::RX_IRQ(uint8_t data)
    {
        static int _index = 0;
        static int length = 500;
        static uint8_t data_length_index;
        static uint8_t data_CRC8_index;

        // 打印每次进入函数时的基本信息
        // ESP_LOGI(TAG, "RX_IRQ: data=0x%02X, _index=%d, length=%d", data, _index, length);

        if (_index == 0)
        {
            if (data == 0x3D)
            {
                ESP_LOGV(TAG, "Start of frame detected (0x3D)");
                BambuBus_data_buf[0] = 0x3D;
                _RX_IRQ_crcx.restart();
                _RX_IRQ_crcx.add(0x3D);
                data_length_index = 4;
                length = data_CRC8_index = 6;
                _index = 1;
            }
            else
            {
                ESP_LOGVV(TAG, "Waiting for start byte (0x3D), got 0x%02X", data);
            }
            return;
        }
        else
        {
            BambuBus_data_buf[_index] = data;

            // 打印每个字节的存储位置
            // ESP_LOGD(TAG, "Stored data 0x%02X at index %d", data, _index);

            if (_index == 1)
            {
                if (data & 0x80)
                {
                    data_length_index = 2;
                    data_CRC8_index = 3;
                    ESP_LOGV(TAG, "Short frame format detected (bit7 set)");
                }
                else
                {
                    data_length_index = 4;
                    data_CRC8_index = 6;
                    ESP_LOGV(TAG, "Long frame format detected");
                }
            }

            if (_index == data_length_index)
            {
                length = data;
                ESP_LOGV(TAG, "Frame length set to %d bytes", length);
            }

            if (_index < data_CRC8_index)
            {
                _RX_IRQ_crcx.add(data);
                ESP_LOGVV(TAG, "Added 0x%02X to CRC (index %d)", data, _index);
            }
            else if (_index == data_CRC8_index)
            {
                uint8_t crc = _RX_IRQ_crcx.calc();
                ESP_LOGV(TAG, "CRC check: received=0x%02X, calculated=0x%02X", data, crc);
                if (data != crc)
                {
                    ESP_LOGW(TAG, "CRC mismatch! Resetting frame parser");
                    _index = 0;
                    return;
                }
            }

            ++_index;
            if (_index >= length)
            {
                // 使用 ESPHome 的 format_hex_pretty
                std::string hexdump = esphome::format_hex_pretty(BambuBus_data_buf, length);
                // 注意：可能需要分行打印，如果 hexdump 太长
                ESP_LOGD(TAG, "Received Data:\n%s length (%d bytes)", hexdump.c_str(), length);
                _index = 0;
                memcpy(buf_X, BambuBus_data_buf, length);
                this->BambuBus_have_data = length;
            }

            if (_index >= 999)
            {
                ESP_LOGW(TAG, "Index overflow detected, resetting parser");
                _index = 0;
            }
        }
    }

    void BambuBus::send_uart(const uint8_t *data, uint16_t length)
    {
        write_array(data, length);
    }

    // 用于带 DE 控制发送的新函数
    void BambuBus::send_uart_with_de(const uint8_t *data, uint16_t length)
    {

        if (this->de_pin_ != nullptr)
        {
            this->de_pin_->digital_write(true); // 激活发送 (高电平)
            // 可能需要极短的延迟确保收发器状态切换 (通常非常快)
            esphome::delayMicroseconds(5); // 示例: 5 微秒，根据硬件调整
            ESP_LOGV(TAG, "DE pin set HIGH.");
        }
        else
        {
            ESP_LOGV(TAG, "DE pin not configured, sending without DE control.");
        }

        ESP_LOGD(TAG, "Sending %d bytes...", length);
        // 使用 format_hex_pretty 打印准备发送的数据
        if (this->need_debug)
        { // 或者使用更精细的日志级别控制
            ESP_LOGD(TAG, "  %s", esphome::format_hex_pretty(data, length).c_str());
        }
        this->write_array(data, length);

        // 等待发送完成 - 非常重要!
        this->flush();
        ESP_LOGV(TAG, "UART flush complete.");

        // 在 flush() 之后再禁用 DE
        if (this->de_pin_ != nullptr)
        {
            // 在禁用 DE 之前可能需要短暂延迟，确保最后一个停止位完全发出
            esphome::delayMicroseconds(5);       // 示例: 5 微秒，根据硬件调整
            this->de_pin_->digital_write(false); // 禁用发送 (低电平)
            ESP_LOGV(TAG, "DE pin set LOW.");
        }
    }

    bool BambuBus::package_check_crc16(uint8_t *data, int data_length)
    {
        crc_16.restart();
        data_length -= 2;
        for (auto i = 0; i < data_length; i++)
        {
            crc_16.add(data[i]);
        }
        uint16_t num = crc_16.calc();
        return (data[data_length] == (num & 0xFF)) && (data[data_length + 1] == ((num >> 8) & 0xFF));
    }

    void BambuBus::package_send_with_crc(uint8_t *data, int data_length)
    {
        crc_8.restart();
        if (data[1] & 0x80)
        {
            for (auto i = 0; i < 3; i++)
            {
                crc_8.add(data[i]);
            }
            data[3] = crc_8.calc();
        }
        else
        {
            for (auto i = 0; i < 6; i++)
            {
                crc_8.add(data[i]);
            }
            data[6] = crc_8.calc();
        }

        crc_16.restart();
        data_length -= 2;
        for (auto i = 0; i < data_length; i++)
        {
            crc_16.add(data[i]);
        }
        uint16_t num = crc_16.calc();
        data[data_length] = num & 0xFF;
        data[data_length + 1] = num >> 8;
        data_length += 2;

        if (this->need_debug)
        {
            // ESP_LOGD(TAG, "Prepared package to send (%d bytes):", data_length);
            // 使用 format_hex_pretty 打印准备发送的数据（需要包含 esphome/core/helpers.h）
            // ESP_LOGD(TAG, "  %s", esphome::format_hex_pretty(data, data_length).c_str());
        }
        // send_uart(data, data_length);
        // 调用新函数以使用 DE 控制进行发送
        send_uart_with_de(data, data_length);
    }

    // --- 实现所有其他方法 (BambuBus_run, send_for_..., helpers 等) ---
    // ... (将原始代码的逻辑迁移过来，确保使用 this-> 访问成员变量和方法) ...
    // 例如 BambuBus_run:
    package_type BambuBus::BambuBus_run()
    {
        package_type stu = BambuBus_package_NONE;
        // 假设 time_set_ 和 time_motion_ 是 uint32_t 类型的成员变量
        static uint32_t time_set_ = 0;
        static uint32_t time_motion_ = 0;

        uint32_t now = esphome::millis(); // 使用 ESPHome 的时间函数

        if (this->BambuBus_have_data > 0)
        {
            int data_length = this->BambuBus_have_data;
            this->BambuBus_have_data = 0; // 清除标志位
            // need_debug = false; // Reset debug flag if needed

            stu = this->get_packge_type(this->buf_X, data_length);
            ESP_LOGD(TAG, "Processing package type: %d", (int)stu);

            switch (stu)
            {
            case BambuBus_package_heartbeat:
                time_set_ = now + 1000; // 更新心跳时间戳
                ESP_LOGD(TAG, "Processing package (Type: BambuBus_package_heartbeat)...");
                // 使用 INFO 级别记录心跳，因为它比较重要且不应过于频繁（除非有问题）
                // ESP_LOGD(TAG, "Processed package (Type: BambuBus_package_heartbeat). Timeout extended.");
                break;

            case BambuBus_package_filament_motion_short:
                ESP_LOGD(TAG, "Processing package (Type: BambuBus_package_filament_motion_short)...");
                this->send_for_Cxx(this->buf_X, data_length);
                // ESP_LOGD(TAG, "Finished processing package (Type: BambuBus_package_filament_motion_short).");
                break;

            case BambuBus_package_filament_motion_long:
                ESP_LOGD(TAG, "Processing package (Type: BambuBus_package_filament_motion_long)...");
                this->send_for_Dxx(this->buf_X, data_length);
                // time_motion_ = now + 1000; // 更新运动状态时间戳
                // ESP_LOGD(TAG, "Finished processing package (Type: BambuBus_package_filament_motion_long). Motion timeout extended.");
                break;

            case BambuBus_package_online_detect:
                ESP_LOGD(TAG, "Processing package (Type: BambuBus_package_online_detect)...");
                this->send_for_Fxx(this->buf_X, data_length);
                // ESP_LOGD(TAG, "Finished processing package (Type: BambuBus_package_online_detect).");
                break;

            case BambuBus_package_REQx6:
                ESP_LOGW(TAG, "Processing package (Type: BambuBus_package_REQx6), but handler is not fully implemented.");
                // this->send_for_REQx6(this->buf_X, data_length); // 如果要实现的话
                // ESP_LOGW(TAG, "Finished processing package (Type: BambuBus_package_REQx6)."); // 记录完成，即使未完全处理
                break;

            case BambuBus_long_package_MC_online:
                ESP_LOGD(TAG, "Processing package (Type: BambuBus_long_package_MC_online)...");
                this->send_for_long_packge_MC_online(this->buf_X, data_length);
                // ESP_LOGD(TAG, "Finished processing package (Type: BambuBus_long_package_MC_online).");
                break;

            case BambuBus_longe_package_filament:
                ESP_LOGD(TAG, "Processing package (Type: BambuBus_longe_package_filament)...");
                this->send_for_long_packge_filament(this->buf_X, data_length);
                // ESP_LOGD(TAG, "Finished processing package (Type: BambuBus_longe_package_filament).");
                break;

            case BambuBus_long_package_version:
                ESP_LOGD(TAG, "Processing package (Type: BambuBus_long_package_version)...");
                this->send_for_long_packge_version(this->buf_X, data_length);
                // ESP_LOGD(TAG, "Finished processing package (Type: BambuBus_long_package_version).");
                break;

            case BambuBus_package_NFC_detect:
                ESP_LOGD(TAG, "Processing package (Type: BambuBus_package_NFC_detect)...");
                this->send_for_NFC_detect(this->buf_X, data_length); // 确保这个函数被调用
                // ESP_LOGD(TAG, "Finished processing package (Type: BambuBus_package_NFC_detect).");
                break;

            case BambuBus_package_set_filament:
                ESP_LOGD(TAG, "Processing package (Type: BambuBus_package_set_filament)...");
                this->send_for_Set_filament(this->buf_X, data_length);
                // ESP_LOGD(TAG, "Finished processing package (Type: BambuBus_package_set_filament).");
                break;

            case BambuBus_package_ETC: // 未知但有效的短包类型
                // ESP_LOGW(TAG, "Received package (Type: BambuBus_package_ETC) - Unknown short command 0x%02X at buf[4]. No specific handler.", this->buf_X[4]);
                // 这里可能不需要做任何事，或者发送一个通用的否定应答（如果协议需要）
                break;

            case BambuBus_package_NONE: // 无效包 (CRC错误或get_packge_type无法识别)
                ESP_LOGW(TAG, "Invalid package received (CRC mismatch or get_packge_type returned NONE).");
                // 可以在这里增加错误计数器或执行其他错误处理
                break;

            case BambuBus_package_ERROR: // 这个枚举值通常不由 get_packge_type 返回，而是由超时逻辑设置
                // 这个 case 理论上在 switch 中不会被命中，因为 stu 是由 get_packge_type 设置的
                ESP_LOGE(TAG, "Internal error: BambuBus_run attempting to process ERROR state in switch.");
                break;

            default: // 捕获所有未明确处理的枚举值或意外值
                ESP_LOGE(TAG, "Received UNHANDLED package type value: %d. This indicates a potential issue in get_packge_type or package_type enum.", (int)stu);
                break;
            }
        }

        // 处理超时
        if (time_set_ != 0 && now > time_set_)
        { // 检查非零避免首次误判
            ESP_LOGE(TAG, "Heartbeat timeout! Assuming offline.");
            stu = BambuBus_package_ERROR; // 标记错误状态
            time_set_ = 0;                // 防止重复触发错误，直到下次心跳
            // 可能需要在这里执行一些离线处理逻辑
        }
        if (time_motion_ != 0 && now > time_motion_)
        {
            ESP_LOGD(TAG, "Motion timeout. Setting filaments to idle.");
            for (int ams = 0; ams < 4; ++ams)
            {
                for (int slot = 0; slot < 4; ++slot)
                {
                    // Check if this filament was actually in motion before setting to idle
                    if (data_save.filament[ams][slot].motion_set != idle)
                    {
                        data_save.filament[ams][slot].motion_set = idle;
                    }
                }
            }
            time_motion_ = 0; // 防止重复触发
        }

        // 处理保存
        if (this->Bambubus_need_to_save)
        {
            ESP_LOGI(TAG, "Saving data to preferences...");
            this->Bambubus_save();
            this->Bambubus_need_to_save = false;
            if (stu != BambuBus_package_ERROR)
            { // 如果没超时，重置超时计时器
                time_set_ = now + 1000;
            }
        }

        // NFC_detect_run(); // 如果需要实现这个逻辑

        return stu;
    }

    package_type BambuBus::get_packge_type(uint8_t *buf, int length)
    {
        if (!package_check_crc16(buf, length))
        {
            return BambuBus_package_NONE;
        }

        if (buf[1] == 0xC5)
        {
            switch (buf[4])
            {
            case 0x03:
                return BambuBus_package_filament_motion_short;
            case 0x04:
                return BambuBus_package_filament_motion_long;
            case 0x05:
                return BambuBus_package_online_detect;
            case 0x06:
                return BambuBus_package_REQx6;
            case 0x07:
                return BambuBus_package_NFC_detect;
            case 0x08:
                return BambuBus_package_set_filament;
            case 0x20:
                return BambuBus_package_heartbeat;
            default:
                return BambuBus_package_ETC;
            }
        }
        else if (buf[1] == 0x05)
        {
            // 解析长包头信息 (填充到成员变量 parsed_long_package 中)
            // 注意：这里调用分析函数来填充 this->parsed_long_package
            this->Bambubus_long_package_analysis(buf, length, &this->parsed_long_package);

            // 更新设备地址 (如果需要，根据原始逻辑)
            if (this->parsed_long_package.target_address == 0x0700 || this->parsed_long_package.target_address == 0x1200)
            {
                if (this->BambuBus_address != this->parsed_long_package.target_address)
                {
                    ESP_LOGI(TAG, "更新 BambuBus 地址为 0x%04X (来自长包目标地址)", this->parsed_long_package.target_address);
                    this->BambuBus_address = this->parsed_long_package.target_address;
                }
            }

            // 根据解析出的长包类型 (type 字段) 返回正确的枚举值
            switch (this->parsed_long_package.type)
            {
            case 0x21A: // MC Online?
                ESP_LOGI(TAG, "识别到长包类型: MC Online (0x21A)");
                return BambuBus_long_package_MC_online;
            case 0x211: // 请求耗材信息
                ESP_LOGI(TAG, "识别到长包类型: 请求耗材信息 (0x211)");
                return BambuBus_longe_package_filament; // <--- 返回这个，以便调用 send_for_long_packge_filament
            case 0x103:                                 // 请求版本/名称?
            case 0x402:                                 // 请求序列号?
                ESP_LOGI(TAG, "识别到长包类型: 版本/序列号 (0x%X)", this->parsed_long_package.type);
                return BambuBus_long_package_version;
            default:
                ESP_LOGI(TAG, "收到未知的长包类型: 0x%03X", this->parsed_long_package.type);
                return BambuBus_package_ETC; // 其他未识别长包
            }
        }

        return BambuBus_package_NONE;
    }

    void BambuBus::send_for_Fxx(uint8_t *buf, int length)
    {
        ESP_LOGD(TAG, "Processing Fxx (Online Detect) request. buf[5]=0x%02X", buf[5]);
        uint8_t F00_res[4 * sizeof(F01_res)];
        if ((buf[5] == 0x00))
        {
            // if((buf[8]==0x30)&&(buf[9]==0x31));
            for (auto i = 0; i < 4; i++)
            {
                memcpy(F00_res + i * sizeof(F01_res), F01_res, sizeof(F01_res));
                F00_res[i * sizeof(F01_res) + 5] = 0;
                F00_res[i * sizeof(F01_res) + 6] = i;
                F00_res[i * sizeof(F01_res) + 7] = i;
            }
            package_send_with_crc(F00_res, sizeof(F00_res));
        }

        if ((buf[5] == 0x01) && (buf[6] < 4))
        {
            memcpy(F01_res + 4, buf + 4, 3);
            // memcpy(F00_res + 8, online_detect_num, sizeof(online_detect_num));
            /*
            if (buf[6])
                memcpy(F00_res + 8, online_detect_num, sizeof(online_detect_num));
            else
                memcpy(F00_res + 8, online_detect_num2, sizeof(online_detect_num2));*/
            // memcpy(online_detect_num, buf + 8, sizeof(online_detect_num));
            //  F00_res[5] = buf[5];
            //  F00_res[6] = buf[6];
            package_send_with_crc(F01_res, sizeof(F01_res));
        }
    }

    // In bambu_bus.cpp:
    void BambuBus::Bambubus_long_package_analysis(uint8_t *buf, int data_length, long_packge_data *data)
    {
        // Original logic: copies 11 bytes starting from buf + 2 into the data struct.
        // This assumes the layout of long_packge_data matches these 11 bytes.
        // Let's verify the original struct alignment and size.
        // #pragma pack(push, 1) was used. C++ struct alignment might differ if not forced.
        // However, direct memcpy of the first few fields *might* work if layout is simple.
        // Fields: package_number (2), package_length (2), crc8 (1), target_address (2), source_address (2), type (2) = 11 bytes.
        memcpy(data, buf + 2, 11); // Be cautious about potential alignment issues.

        data->datas = buf + 13;               // Pointer to data payload within the original buffer
        data->data_length = data_length - 15; // Total length minus header (13) and CRC16 (2) = payload length
        ESP_LOGD(TAG, "Long package analysis: type=0x%X, src=0x%X, tgt=0x%X, data_len=%d",
                 data->type, data->source_address, data->target_address, data->data_length);
    }

    void BambuBus::Bambubus_long_package_send(long_packge_data *data)
    {
        // Need a member buffer for sending, packge_send_buf[1000];
        this->packge_send_buf[0] = 0x3D;
        this->packge_send_buf[1] = 0x00; // Assuming long package identifier byte 1 is 0x00

        // Original code sets package_length = data_length + 15 BEFORE memcpy.
        // This seems correct: total length = payload + header(13) + crc16(2)
        data->package_length = data->data_length + 15;

        // Copy the first 11 bytes of the struct (header fields) into the send buffer
        // Again, relies on struct layout matching the first 11 bytes.
        memcpy(this->packge_send_buf + 2, data, 11);

        // Copy the actual payload data
        memcpy(this->packge_send_buf + 13, data->datas, data->data_length);

        // Calculate CRCs and send using the existing method
        ESP_LOGD(TAG, "Sending long package: type=0x%X, src=0x%X, tgt=0x%X, total_len=%d",
                 data->type, data->source_address, data->target_address, data->package_length);
        this->package_send_with_crc(this->packge_send_buf, data->package_length); // Send total length (header + payload + CRC16 space)
    }

    // Helper function from original code, make it a private member method
    uint8_t BambuBus::get_filament_left_char(uint8_t ams_id)
    {
        uint8_t data = 0;
        for (int i = 0; i < 4; i++)
        {
            // Check if the filament slot exists and is online
            if (this->data_save.filament[ams_id][i].statu == online)
            {
                // Bit pattern: 01 (online, idle), 11 (online, moving) per slot
                data |= (0x1 << (i * 2)); // Set bit 2*i to 1 (indicates online)
                if (this->data_save.filament[ams_id][i].motion_set != idle)
                {
                    data |= (0x2 << (i * 2)); // Set bit 2*i + 1 to 1 (indicates moving)
                }
            }
            // Offline slots have 00 implicitly
        }
        return data;
    }

    // Helper function from original code, make it a private member method
    void BambuBus::set_motion_res_datas(uint8_t *set_buf, uint8_t ams_id, uint8_t read_num)
    {
        // unsigned char statu_flags = buf[6];

        // unsigned char fliment_motion_flag = buf[8];
        float meters = 0;
        uint8_t flagx = 0x02;
        if (read_num != 0xFF)
        {
            if (BambuBus_address == 0x700) // AMS08
            {
                meters = -data_save.filament[AMS_num][read_num].meters;
            }
            else if (BambuBus_address == 0x1200) // AMS lite
            {
                meters = data_save.filament[AMS_num][read_num].meters;
            }
        }
        set_buf[0] = AMS_num;
        set_buf[2] = flagx;
        set_buf[3] = read_num; // maybe using number
        memcpy(set_buf + 4, &meters, sizeof(meters));
        set_buf[13] = 0;
        set_buf[24] = get_filament_left_char(AMS_num);
    }

    // Helper function from original code, make it a private member method
    bool BambuBus::set_motion(uint8_t ams_id, uint8_t read_num, uint8_t statu_flags, uint8_t fliment_motion_flag)
    {
        // Ensure ams_id is valid (assuming max 4 AMS units, 0-3)
        if (ams_id >= 4)
        {
            ESP_LOGW(TAG, "set_motion called with invalid AMS ID: %d", ams_id);
            return false;
        }

        int filament_global_index = ams_id * 4 + read_num;

        if (this->BambuBus_address == 0x700)
        { // AMS08 (Hub)
            if ((read_num != 0xFF) && (read_num < 4))
            {
                if ((statu_flags == 0x03) && (fliment_motion_flag == 0x00))
                { // 03 00 -> Load filament
                    ESP_LOGD(TAG, "Set Motion (AMS Hub): Slot %d -> need_send_out", filament_global_index);
                    this->data_save.BambuBus_now_filament_num = filament_global_index;
                    this->data_save.filament[ams_id][read_num].motion_set = need_send_out;
                    this->data_save.filament[ams_id][read_num].pressure = 0x3600; // Reset pressure?
                }
                else if (((statu_flags == 0x09) && (fliment_motion_flag == 0xA5)) || // 09 A5 -> Active/Printing?
                         ((statu_flags == 0x07) && (fliment_motion_flag == 0x7F)))
                { // 07 7F -> Active/Printing?
                    ESP_LOGD(TAG, "Set Motion (AMS Hub): Slot %d -> on_use", filament_global_index);
                    this->data_save.BambuBus_now_filament_num = filament_global_index;
                    this->data_save.filament[ams_id][read_num].motion_set = on_use;
                }
                else if ((statu_flags == 0x07) && (fliment_motion_flag == 0x00))
                { // 07 00 -> Unload filament
                    ESP_LOGD(TAG, "Set Motion (AMS Hub): Slot %d -> need_pull_back", filament_global_index);
                    this->data_save.BambuBus_now_filament_num = filament_global_index;
                    this->data_save.filament[ams_id][read_num].motion_set = need_pull_back;
                }
                else
                {
                    ESP_LOGW(TAG, "Set Motion (AMS Hub): Slot %d - Unhandled flags: statu=0x%02X, motion=0x%02X", filament_global_index, statu_flags, fliment_motion_flag);
                }
            }
            else if (read_num == 0xFF)
            { // Command for all slots in this AMS unit
                if ((statu_flags == 0x01) || (statu_flags == 0x03))
                { // Reset all slots in this AMS to idle
                    ESP_LOGD(TAG, "Set Motion (AMS Hub): Resetting all slots in AMS %d to idle", ams_id);
                    for (auto i = 0; i < 4; i++)
                    {
                        this->data_save.filament[ams_id][i].motion_set = idle;
                        this->data_save.filament[ams_id][i].pressure = 0x3600; // Reset pressure?
                    }
                }
                else
                {
                    ESP_LOGW(TAG, "Set Motion (AMS Hub): All Slots - Unhandled flags: statu=0x%02X", statu_flags);
                }
            }
        }
        else if (this->BambuBus_address == 0x1200)
        { // AMS lite
            if (read_num < 4)
            {
                if ((statu_flags == 0x03) && (fliment_motion_flag == 0x3F))
                { // 03 3F -> Unload?
                    ESP_LOGD(TAG, "Set Motion (AMS Lite): Slot %d -> need_pull_back", filament_global_index);
                    this->data_save.BambuBus_now_filament_num = filament_global_index;
                    this->data_save.filament[ams_id][read_num].motion_set = need_pull_back;
                }
                else if ((statu_flags == 0x03) && (fliment_motion_flag == 0xBF))
                { // 03 BF -> Load?
                    ESP_LOGD(TAG, "Set Motion (AMS Lite): Slot %d -> need_send_out", filament_global_index);
                    this->data_save.BambuBus_now_filament_num = filament_global_index;
                    this->data_save.filament[ams_id][read_num].motion_set = need_send_out;
                }
                else
                { // Other flags likely mean transition to idle or on_use
                    _filament_motion_state_set current_state = this->data_save.filament[ams_id][read_num].motion_set;
                    if (current_state == need_pull_back)
                    {
                        ESP_LOGD(TAG, "Set Motion (AMS Lite): Slot %d -> idle (from pull_back)", filament_global_index);
                        this->data_save.filament[ams_id][read_num].motion_set = idle;
                    }
                    else if (current_state == need_send_out)
                    {
                        ESP_LOGD(TAG, "Set Motion (AMS Lite): Slot %d -> on_use (from send_out)", filament_global_index);
                        this->data_save.filament[ams_id][read_num].motion_set = on_use;
                    }
                    else
                    {
                        // If already idle or on_use, maybe stay there? Or log warning.
                        ESP_LOGD(TAG, "Set Motion (AMS Lite): Slot %d - No state change for flags: statu=0x%02X, motion=0x%02X", filament_global_index, statu_flags, fliment_motion_flag);
                    }
                }
            }
            else if (read_num == 0xFF)
            { // Reset all slots for AMS Lite
                ESP_LOGD(TAG, "Set Motion (AMS Lite): Resetting all slots in AMS %d to idle", ams_id);
                for (int i = 0; i < 4; i++)
                {
                    this->data_save.filament[ams_id][i].motion_set = idle;
                }
            }
        }
        else if (this->BambuBus_address == 0x00)
        { // No AMS / Spool holder?
            if ((read_num != 0xFF) && (read_num < 4))
            { // Assuming external spool maps to AMS 0, slot 0-3? Or just slot 0? Let's assume 0-3 for now.
                ESP_LOGD(TAG, "Set Motion (No AMS): Slot %d -> on_use", filament_global_index);
                this->data_save.BambuBus_now_filament_num = filament_global_index; // Update current filament
                this->data_save.filament[ams_id][read_num].motion_set = on_use;
            }
            // Reset (read_num == 0xFF) is not explicitly handled for address 0x00 in original
        }
        else
        {
            ESP_LOGE(TAG, "Set Motion: Unknown BambuBus address 0x%04X", this->BambuBus_address);
            return false; // Unknown address, cannot determine behavior
        }
        return true;
    }

    void BambuBus::send_for_Cxx(uint8_t *buf, int length)
    {
        ESP_LOGD(TAG, "Processing Cxx (Short Motion) request");

        // Extract necessary data from the request buffer (buf)
        uint8_t request_ams_num = buf[5];     // AMS index from request
        uint8_t request_statu_flags = buf[6]; // Status flags from request
        uint8_t request_read_num = buf[7];    // Filament slot index from request (0-3 or 0xFF)
        uint8_t request_motion_flag = buf[8]; // Motion flag from request

        // Update filament motion state based on the request
        // It's crucial this happens *before* generating the response if the response depends on the new state.
        if (!this->set_motion(request_ams_num, request_read_num, request_statu_flags, request_motion_flag))
        {
            ESP_LOGE(TAG, "Failed to set motion state based on Cxx request. No response sent.");
            return; // Don't send a response if the state update failed (e.g., invalid address)
        }

        // Prepare the response using the Cxx_res template
        // Create a temporary buffer to avoid modifying the class member template
        uint8_t temp_Cxx_res[sizeof(this->Cxx_res)];
        memcpy(temp_Cxx_res, this->Cxx_res, sizeof(this->Cxx_res));

        // Modify the header byte 1: Set sequence number (package_num)
        // Original: Cxx_res[1] = 0xC0 | (package_num << 3);
        temp_Cxx_res[1] = 0xC0 | (this->package_num << 3);

        // Populate the data section of the response buffer (starting at temp_Cxx_res + 5)
        // Use the helper function to fill in dynamic data like meters, flags, etc.
        this->set_motion_res_datas(temp_Cxx_res + 5, request_ams_num, request_read_num);

        // Send the prepared response package (CRC calculation is handled inside)
        ESP_LOGD(TAG, "Sending Cxx response (%d bytes)", (int)sizeof(temp_Cxx_res));
        this->package_send_with_crc(temp_Cxx_res, sizeof(temp_Cxx_res));

        // Increment package number for the next response
        if (this->package_num < 7)
        {
            this->package_num++;
        }
        else
        {
            this->package_num = 0;
        }
    }

    // In bambu_bus.cpp

    void BambuBus::send_for_Dxx(uint8_t *buf, int length)
    {
        unsigned char filament_flag_on = 0x00;
        unsigned char filament_flag_NFC = 0x00;
        unsigned char AMS_num = buf[5];
        unsigned char statu_flags = buf[6];
        unsigned char fliment_motion_flag = buf[7];
        unsigned char read_num = buf[9];

        for (auto i = 0; i < 4; i++)
        {
            // filament[i].meters;
            if (data_save.filament[AMS_num][i].statu == online)
            {
                filament_flag_on |= 1 << i;
            }
            else if (data_save.filament[AMS_num][i].statu == NFC_waiting)
            {
                filament_flag_on |= 1 << i;
                filament_flag_NFC |= 1 << i;
            }
        }
        ESP_LOGD(TAG, "Filament presence flags for AMS %d: on=0x%02X, nfc_wait=0x%02X", AMS_num, filament_flag_on, filament_flag_NFC);
        ESP_LOGD(TAG, "set_motion from Dxx - request_ams_num=%d, request_read_num=%d, request_statu_flags=0x%02X, request_motion_flag=0x%02X",
                 AMS_num, read_num, statu_flags, fliment_motion_flag);

        if (!set_motion(AMS_num, read_num, statu_flags, fliment_motion_flag))
        {
            ESP_LOGE(TAG, "Failed to set motion state based on Dxx request. No response sent.");
            return;
        }

        /*if (need_res_for_06)
        {
            Dxx_res2[1] = 0xC0 | (package_num << 3);
            Dxx_res2[9] = filament_flag_on;
            Dxx_res2[10] = filament_flag_on - filament_flag_NFC;
            Dxx_res2[11] = filament_flag_on - filament_flag_NFC;
            Dxx_res[19] = flagx;
            Dxx_res[20] = Dxx_res2[12] = res_for_06_num;
            Dxx_res2[13] = filament_flag_NFC;
            Dxx_res2[41] = get_filament_left_char();
            package_send_with_crc(Dxx_res2, sizeof(Dxx_res2));
            need_res_for_06 = false;
        }
        else*/

        {
            Dxx_res[1] = 0xC0 | (package_num << 3);
            Dxx_res[5] = AMS_num;
            Dxx_res[9] = filament_flag_on;
            Dxx_res[10] = filament_flag_on - filament_flag_NFC;
            Dxx_res[11] = filament_flag_on - filament_flag_NFC;
            Dxx_res[12] = read_num;
            Dxx_res[13] = filament_flag_NFC;

            set_motion_res_datas(Dxx_res + 17, AMS_num, read_num);
        }
        if (last_detect != 0)
        {
            if (last_detect > 10)
            {
                Dxx_res[19] = 0x01;
            }
            else
            {
                Dxx_res[12] = filament_flag_detected;
                Dxx_res[19] = 0x01;
                Dxx_res[20] = filament_flag_detected;
            }
            last_detect--;
        }
        package_send_with_crc(Dxx_res, sizeof(Dxx_res));
        if (package_num < 7)
            package_num++;
        else
            package_num = 0;
    }

    // In bambu_bus.cpp

    void BambuBus::send_for_Set_filament(uint8_t *buf, int length)
    {
        ESP_LOGI(TAG, "Processing Set Filament request");

        if (length < 43)
        { // Check minimum length for safety (based on offsets accessed)
            ESP_LOGE(TAG, "Set Filament request too short (%d bytes). Expected at least 43.", length);
            return; // Don't process if too short
        }

        // Extract data from the request buffer (buf)
        uint8_t combined_num = buf[5];                // Contains AMS (high nibble) and Slot (low nibble)
        uint8_t ams_num = (combined_num >> 4) & 0x0F; // Extract AMS index (0-3)
        uint8_t read_num = combined_num & 0x0F;       // Extract Slot index (0-3)

        // Validate indices
        if (ams_num >= 4 || read_num >= 4)
        {
            ESP_LOGE(TAG, "Set Filament request has invalid AMS/Slot index: AMS=%d, Slot=%d", ams_num, read_num);
            // Send NACK? Original just sends ACK. Let's stick to original for now.
        }
        else
        {
            ESP_LOGD(TAG, "Updating filament data for AMS %d, Slot %d", ams_num, read_num);

            // Get pointer to the filament data structure to modify
            _filament &target_filament = this->data_save.filament[ams_num][read_num];

            // Copy filament ID (8 bytes, offset 7 in request)
            memcpy(target_filament.ID, buf + 7, sizeof(target_filament.ID));
            // Ensure null termination if ID is treated as a string later
            // target_filament.ID[sizeof(target_filament.ID) - 1] = '\0'; // Optional safety

            // Copy filament color RGBA (4 bytes, offset 15)
            target_filament.color_R = buf[15];
            target_filament.color_G = buf[16];
            target_filament.color_B = buf[17];
            target_filament.color_A = buf[18]; // Alpha or other property

            // Copy temperature min/max (2 bytes each, offset 19 and 21)
            memcpy(&target_filament.temperature_min, buf + 19, sizeof(target_filament.temperature_min));
            memcpy(&target_filament.temperature_max, buf + 21, sizeof(target_filament.temperature_max));

            // Copy filament name (20 bytes, offset 23)
            memcpy(target_filament.name, buf + 23, sizeof(target_filament.name));
            // Ensure null termination
            target_filament.name[sizeof(target_filament.name) - 1] = '\0';

            // Log the received data
            ESP_LOGI(TAG, "  ID: %.8s", target_filament.ID); // Print first 8 bytes
            ESP_LOGI(TAG, "  Color: R=0x%02X G=0x%02X B=0x%02X A=0x%02X", target_filament.color_R, target_filament.color_G, target_filament.color_B, target_filament.color_A);
            ESP_LOGI(TAG, "  Temp: Min=%d Max=%d", target_filament.temperature_min, target_filament.temperature_max);
            ESP_LOGI(TAG, "  Name: %s", target_filament.name);

            // Mark data as needing to be saved
            this->set_need_to_save(); // Call the existing method to set the flag
        }

        // Send the acknowledgement response
        // The response (Set_filament_res) seems fixed in the original code.
        ESP_LOGD(TAG, "Sending Set Filament ACK response (%d bytes)", (int)sizeof(this->Set_filament_res));
        // Create a temporary buffer just in case we need to modify seq num later, though original doesn't seem to.
        uint8_t temp_Set_filament_res[sizeof(this->Set_filament_res)];
        memcpy(temp_Set_filament_res, this->Set_filament_res, sizeof(this->Set_filament_res));
        // Modify sequence number if needed (byte 1) - original seems to use fixed 0xC0? Let's assume fixed.
        // temp_Set_filament_res[1] = 0xC0 | (this->package_num << 3); // Uncomment if seq num needed

        this->package_send_with_crc(temp_Set_filament_res, sizeof(temp_Set_filament_res));

        // Increment package number? Original doesn't seem to for this response. Let's skip.
        /*
        if (this->package_num < 7) {
            this->package_num++;
        } else {
            this->package_num = 0;
        }
        */
    }

    // In bambu_bus.cpp

    void BambuBus::send_for_REQx6(uint8_t *buf, int length)
    {
        ESP_LOGW(TAG, "Function send_for_REQx6 received request but is not implemented.");
        // Original code was mostly commented out.
        // Need to decide if/how to respond. Maybe send nothing or a default ACK/NACK if protocol defines one.
        this->need_res_for_06 = true;  // Original sets this flag
        this->res_for_06_num = buf[7]; // Original stores requested number
    }

    void BambuBus::send_for_NFC_detect(uint8_t *buf, int length)
    {
        ESP_LOGD(TAG, "Processing NFC Detect request for slot %d", buf[6]);

        this->last_detect = 20;                       // Set NFC activity counter (used in Dxx response)
        this->filament_flag_detected = (1 << buf[6]); // Store which slot triggered detection

        // Prepare response using NFC_detect_res template
        uint8_t temp_NFC_detect_res[sizeof(this->NFC_detect_res)];
        memcpy(temp_NFC_detect_res, this->NFC_detect_res, sizeof(this->NFC_detect_res));

        // Modify sequence number (byte 1)
        temp_NFC_detect_res[1] = 0xC0 | (this->package_num << 3); // Assuming seq num needed

        // Populate dynamic fields (bytes 6 and 7)
        temp_NFC_detect_res[6] = buf[6]; // Slot index
        temp_NFC_detect_res[7] = buf[7]; // Unknown data from request byte 7

        // Send response
        ESP_LOGD(TAG, "Sending NFC Detect response (%d bytes)", (int)sizeof(temp_NFC_detect_res));
        this->package_send_with_crc(temp_NFC_detect_res, sizeof(temp_NFC_detect_res));

        // Increment package number? Assume yes for consistency.
        if (this->package_num < 7)
        {
            this->package_num++;
        }
        else
        {
            this->package_num = 0;
        }
    }

    void BambuBus::send_for_long_packge_MC_online(uint8_t *buf, int length)
    {
        ESP_LOGD(TAG, "Processing Long Package MC Online request");

        // Parse the incoming long package (header info is now in this->parsed_long_package)
        // The actual parsing happened in get_packge_type before calling this handler.
        // We need the source/target addresses and package number from the parsed data.

        if (this->parsed_long_package.target_address != 0x0700 && this->parsed_long_package.target_address != 0x1200)
        {
            ESP_LOGW(TAG, "MC Online request for unknown target address 0x%04X. Ignoring.", this->parsed_long_package.target_address);
            return;
        }

        uint8_t request_ams_num = this->parsed_long_package.datas[0]; // Get AMS num from payload

        // Prepare the response data structure
        long_packge_data response_data;
        response_data.package_number = this->parsed_long_package.package_number; // Echo package number
        response_data.type = this->parsed_long_package.type;                     // Echo type
        response_data.source_address = this->parsed_long_package.target_address; // Swap source and target
        response_data.target_address = this->parsed_long_package.source_address;

        // Prepare payload
        // uint8_t long_packge_MC_online[6] = {0x00, ...}; // This is a member template
        uint8_t response_payload[sizeof(this->long_packge_MC_online)];
        memcpy(response_payload, this->long_packge_MC_online, sizeof(response_payload)); // Copy template
        response_payload[0] = request_ams_num;                                           // Set AMS num in payload

        response_data.datas = response_payload; // Point to the payload buffer
        response_data.data_length = sizeof(response_payload);

        // Send the long package response
        this->Bambubus_long_package_send(&response_data);
    }

    void BambuBus::send_for_long_packge_filament(uint8_t *buf, int length)
    {
        ESP_LOGD(TAG, "Processing Long Package Filament Info request");

        // Parsing already done in get_packge_type. Use this->parsed_long_package.
        uint8_t request_ams_num = this->parsed_long_package.datas[0];
        uint8_t request_filament_num = this->parsed_long_package.datas[1];

        // Validate indices
        if (request_ams_num >= 4 || request_filament_num >= 4)
        {
            ESP_LOGE(TAG, "Long Filament request has invalid AMS/Slot index: AMS=%d, Slot=%d", request_ams_num, request_filament_num);
            // Send NACK or ignore? Original sends response.
            return; // Let's ignore invalid requests for now.
        }

        ESP_LOGD(TAG, "Responding with filament data for AMS %d, Slot %d", request_ams_num, request_filament_num);
        _filament &source_filament = this->data_save.filament[request_ams_num][request_filament_num];

        // Prepare the response structure
        long_packge_data response_data;
        response_data.package_number = this->parsed_long_package.package_number;
        response_data.type = this->parsed_long_package.type;
        response_data.source_address = this->parsed_long_package.target_address;
        response_data.target_address = this->parsed_long_package.source_address;

        // Prepare payload using the template long_packge_filament
        uint8_t response_payload[sizeof(this->long_packge_filament)];
        memcpy(response_payload, this->long_packge_filament, sizeof(response_payload)); // Copy template

        // --- Populate dynamic fields in the payload ---
        response_payload[0] = request_ams_num;
        response_payload[1] = request_filament_num;
        // Offset 19: Filament ID (8 bytes)
        memcpy(response_payload + 19, source_filament.ID, sizeof(source_filament.ID));
        // Offset 27: Filament Name (20 bytes)
        memcpy(response_payload + 27, source_filament.name, sizeof(source_filament.name));
        // Ensure null termination for safety if name buffer wasn't full
        response_payload[27 + sizeof(source_filament.name) - 1] = '\0';
        // Offset 59: Color R, G, B, A (4 bytes)
        response_payload[59] = source_filament.color_R;
        response_payload[60] = source_filament.color_G;
        response_payload[61] = source_filament.color_B;
        response_payload[62] = source_filament.color_A;
        // Offset 79: Temp Max (2 bytes)
        memcpy(response_payload + 79, &source_filament.temperature_max, sizeof(source_filament.temperature_max));
        // Offset 81: Temp Min (2 bytes)
        memcpy(response_payload + 81, &source_filament.temperature_min, sizeof(source_filament.temperature_min));
        // Other fields seem fixed in the template.

        response_data.datas = response_payload;
        response_data.data_length = sizeof(response_payload);

        // Send the long package response
        this->Bambubus_long_package_send(&response_data);
    }

    void BambuBus::send_for_long_packge_version(uint8_t *buf, int length)
    {
        ESP_LOGD(TAG, "Processing Long Package Version request");

        // Parsing done in get_packge_type. Use this->parsed_long_package.
        uint8_t request_ams_num = 0; // Default
        unsigned char *payload_template = nullptr;
        size_t payload_size = 0;

        // Determine which version info is requested (type 0x402 or 0x103)
        switch (this->parsed_long_package.type)
        {
        case 0x402: // Serial number request
            ESP_LOGD(TAG, "  Type 0x402 (Serial Number)");
            request_ams_num = this->parsed_long_package.datas[33]; // AMS num seems to be at offset 33 in request payload? Verify this.
            payload_template = this->long_packge_version_serial_number;
            payload_size = sizeof(this->long_packge_version_serial_number);
            break;
        case 0x103: // Version string request
            ESP_LOGD(TAG, "  Type 0x103 (Version String)");
            request_ams_num = this->parsed_long_package.datas[0]; // AMS num at offset 0
            // Choose template based on target address (AMS Hub or Lite)
            if (this->parsed_long_package.target_address == 0x0700)
            {
                ESP_LOGD(TAG, "  Target is AMS Hub (0x0700)");
                payload_template = this->long_packge_version_version_and_name_AMS08;
                payload_size = sizeof(this->long_packge_version_version_and_name_AMS08);
            }
            else if (this->parsed_long_package.target_address == 0x1200)
            {
                ESP_LOGD(TAG, "  Target is AMS Lite (0x1200)");
                payload_template = this->long_packge_version_version_and_name_AMS_lite;
                payload_size = sizeof(this->long_packge_version_version_and_name_AMS_lite);
            }
            else
            {
                ESP_LOGW(TAG, "  Version request for unknown target address 0x%04X", this->parsed_long_package.target_address);
                return; // Ignore
            }
            break;
        default:
            ESP_LOGW(TAG, "  Unknown version request type 0x%03X", this->parsed_long_package.type);
            return; // Ignore
        }

        // Prepare the response structure
        long_packge_data response_data;
        response_data.package_number = this->parsed_long_package.package_number;
        response_data.type = this->parsed_long_package.type;
        response_data.source_address = this->parsed_long_package.target_address;
        response_data.target_address = this->parsed_long_package.source_address;

        // Prepare payload
        uint8_t response_payload[payload_size];
        memcpy(response_payload, payload_template, payload_size); // Copy appropriate template

        // Modify payload based on type
        if (this->parsed_long_package.type == 0x402)
        {
            // Set serial number length and data (using fixed "STUDY0ONLY")
            // char serial_number[] = "STUDY0ONLY"; // Defined elsewhere? Make it a const member?
            const char serial_number[] = "STUDY0ONLY";       // Use local const for clarity
            response_payload[0] = sizeof(serial_number) - 1; // Length (excluding null terminator)
            memcpy(response_payload + 1, serial_number, sizeof(serial_number) - 1);
            // Set AMS num at offset 65
            response_payload[65] = request_ams_num;
        }
        else if (this->parsed_long_package.type == 0x103)
        {
            // Set AMS num at offset 20
            response_payload[20] = request_ams_num;
        }

        response_data.datas = response_payload;
        response_data.data_length = payload_size;

        // Send the long package response
        this->Bambubus_long_package_send(&response_data);
    }

    // Implement the getter/setter helper methods requested by the user
    void BambuBus::set_need_to_save()
    {
        ESP_LOGD(TAG, "Setting flag: need to save data.");
        this->Bambubus_need_to_save = true;
    }

    int BambuBus::get_now_filament_num()
    {
        return this->data_save.BambuBus_now_filament_num;
    }

    void BambuBus::reset_filament_meters(int num)
    {
        int ams_id = num / 4;
        int slot_id = num % 4;
        if (ams_id >= 4 || slot_id >= 4)
            return; // Basic validation
        ESP_LOGD(TAG, "Resetting meters for filament %d (AMS %d, Slot %d)", num, ams_id, slot_id);
        this->data_save.filament[ams_id][slot_id].meters = 0.0f;
        this->set_need_to_save(); // Changing meters requires saving
    }

    void BambuBus::add_filament_meters(int num, float meters)
    {
        int ams_id = num / 4;
        int slot_id = num % 4;
        if (ams_id >= 4 || slot_id >= 4)
            return; // Basic validation
        ESP_LOGV(TAG, "Adding %.2f meters for filament %d (AMS %d, Slot %d)", meters, num, ams_id, slot_id);
        this->data_save.filament[ams_id][slot_id].meters += meters;
        // Decide if adding meters requires immediate saving, or if it's saved periodically/on shutdown
        // this->set_need_to_save(); // Uncomment if save is needed after adding meters
    }

    float BambuBus::get_filament_meters(int num)
    {
        int ams_id = num / 4;
        int slot_id = num % 4;
        if (ams_id >= 4 || slot_id >= 4)
            return 0.0f; // Basic validation
        return this->data_save.filament[ams_id][slot_id].meters;
    }

    void BambuBus::set_filament_online(int num, bool if_online)
    {
        int ams_id = num / 4;
        int slot_id = num % 4;
        if (ams_id >= 4 || slot_id >= 4)
            return; // Basic validation
        _filament_status new_status = if_online ? online : offline;
        ESP_LOGD(TAG, "Setting filament %d (AMS %d, Slot %d) status to %s", num, ams_id, slot_id, if_online ? "online" : "offline");
        this->data_save.filament[ams_id][slot_id].statu = new_status;
        // Setting online/offline might require saving state
        this->set_need_to_save();
    }

    bool BambuBus::get_filament_online(int num)
    {
        int ams_id = num / 4;
        int slot_id = num % 4;
        if (ams_id >= 4 || slot_id >= 4)
            return false;                                                    // Basic validation
        return (this->data_save.filament[ams_id][slot_id].statu != offline); // online or NFC_waiting counts as "online" for presence
    }

    void BambuBus::set_filament_motion(int num, _filament_motion_state_set motion)
    {
        int ams_id = num / 4;
        int slot_id = num % 4;
        if (ams_id >= 4 || slot_id >= 4)
            return; // Basic validation
        ESP_LOGD(TAG, "Setting filament %d (AMS %d, Slot %d) motion to %d", num, ams_id, slot_id, (int)motion);
        this->data_save.filament[ams_id][slot_id].motion_set = motion;
        // Changing motion state might not require saving immediately, depends on how state is recovered.
        // Let's assume it doesn't need saving unless other filament data changes.
    }

    _filament_motion_state_set BambuBus::get_filament_motion(int num)
    {
        int ams_id = num / 4;
        int slot_id = num % 4;
        if (ams_id >= 4 || slot_id >= 4)
            return idle; // Basic validation, return idle default
        return this->data_save.filament[ams_id][slot_id].motion_set;
    }

}
// Implement all the remaining methods following the same pattern...
// [Rest of the implementation would follow the same structure as above]