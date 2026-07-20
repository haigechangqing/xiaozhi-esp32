#ifndef LOCATION_CONTROLLER_H
#define LOCATION_CONTROLLER_H

#include "mcp_server.h"
#include "board.h"
#include "audio_codec.h"
#include "application.h"
#include <at_modem.h>
#include <at_uart.h>
#include <string>
#include <variant>
#include <vector>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#define TAG "LocationController"

class LocationController {
public:
    LocationController() {
        auto& mcp_server = McpServer::GetInstance();

        mcp_server.AddTool("self.location.get_gnss",
            "Get current location using GNSS (GPS/BeiDou/GLONASS) satellite positioning.\n"
            "Requires GNSS antenna connected and open sky view. Cold start takes 30-60 seconds.\n"
            "Returns JSON with type, latitude, longitude, altitude.",
            PropertyList(),
            [this](const PropertyList& properties) -> ReturnValue {
                return HandleGetGnss();
            });

        mcp_server.AddTool("self.location.get_lbs",
            "Get current location using cellular base station positioning (LBS).\n"
            "Works without GNSS antenna, only needs SIM card and cellular network.\n"
            "Accuracy typically 50-500 meters. Fast response (3-5 seconds).\n"
            "Returns JSON with type, latitude, longitude, accuracy.",
            PropertyList(),
            [this](const PropertyList& properties) -> ReturnValue {
                return HandleGetLbs();
            });

        mcp_server.AddTool("self.location.get_current",
            "Get current location using the best available method.\n"
            "Tries GNSS satellite positioning first; if it fails (no fix within timeout),\n"
            "automatically falls back to cellular LBS positioning.\n"
            "Returns JSON with type, latitude, longitude, and optional altitude/accuracy.",
            PropertyList(),
            [this](const PropertyList& properties) -> ReturnValue {
                return HandleGetCurrent();
            });
    }

    ~LocationController() {
        PowerOffGnss();
    }

private:
    bool gnss_powered_on_ = false;
    bool lbs_configured_ = false;

    AtModem* GetModem() {
        auto* network = Board::GetInstance().GetNetwork();
        return dynamic_cast<AtModem*>(network);
    }

    // ==================== 音频状态管理 ====================

    void PauseAudioOutput() {
        auto& app = Application::GetInstance();
        auto state_before = app.GetDeviceState();
        ESP_LOGI(TAG, "PauseAudioOutput: state before=%d", (int)state_before);

        // 1. 发送中止指令到服务器，停止推送音频流
        ESP_LOGI(TAG, "Aborting speaking...");
        app.AbortSpeaking(kAbortReasonNone);

        // 2. 等待服务器停止推送 + 本地缓冲排空 + UHCI 释放
        // 从日志看 LBS 命令在 ~14s 后成功，这里用 10s 折中
        ESP_LOGI(TAG, "Waiting 10s for UHCI to become free...");
        vTaskDelay(pdMS_TO_TICKS(10000));

        auto state_after = app.GetDeviceState();
        ESP_LOGI(TAG, "PauseAudioOutput: state after wait=%d", (int)state_after);

        // 3. 禁用 I2S 音频输出
        auto* audio_codec = Board::GetInstance().GetAudioCodec();
        if (audio_codec) {
            ESP_LOGI(TAG, "Disabling I2S audio output...");
            audio_codec->EnableOutput(false);
            vTaskDelay(pdMS_TO_TICKS(200));
            ESP_LOGI(TAG, "I2S audio output disabled");
        }
    }

    void ResumeAudioOutput() {
        auto* audio_codec = Board::GetInstance().GetAudioCodec();
        if (audio_codec) {
            ESP_LOGI(TAG, "Enabling I2S audio output...");
            audio_codec->EnableOutput(true);
            ESP_LOGI(TAG, "I2S audio output enabled");
        }
    }

    // ==================== AT 命令发送 ====================

    bool SendAtCommand(const std::string& cmd, int timeout_ms = 3000, int max_retries = 5, int delay_ms = 1000) {
        auto* modem = GetModem();
        if (!modem) {
            ESP_LOGE(TAG, "SendAtCommand: modem not available");
            return false;
        }

        auto at_uart = modem->GetAtUart();
        ESP_LOGI(TAG, "SendAtCommand: [%s] (timeout=%d, retries=%d)", cmd.c_str(), timeout_ms, max_retries);

        for (int retry = 0; retry < max_retries; retry++) {
            bool ok = at_uart->SendCommand(cmd, timeout_ms);
            ESP_LOGI(TAG, "  Attempt %d/%d: %s", retry + 1, max_retries, ok ? "OK" : "FAIL");

            if (ok) return true;

            if (retry < max_retries - 1) {
                ESP_LOGW(TAG, "  Retry %d/%d for: %s (waiting %d ms)", retry + 1, max_retries, cmd.c_str(), delay_ms);
                vTaskDelay(pdMS_TO_TICKS(delay_ms));
            }
        }
        ESP_LOGE(TAG, "SendAtCommand FAILED after %d retries: %s", max_retries, cmd.c_str());
        return false;
    }

    // ==================== GNSS Power Management ====================

    bool PowerOnGnss() {
        if (gnss_powered_on_) {
            ESP_LOGI(TAG, "GNSS already powered on");
            return true;
        }

        ESP_LOGI(TAG, "Powering on GNSS (AT+CGNSSPWR=1)...");
        if (!SendAtCommand("AT+CGNSSPWR=1", 5000, 5, 2000)) {
            ESP_LOGE(TAG, "Failed to power on GNSS with AT+CGNSSPWR=1");
            return false;
        }

        gnss_powered_on_ = true;
        ESP_LOGI(TAG, "GNSS powered on, waiting for module to initialize (2s)...");
        vTaskDelay(pdMS_TO_TICKS(2000));
        return true;
    }

    bool PowerOffGnss() {
        if (!gnss_powered_on_) {
            ESP_LOGI(TAG, "GNSS already off");
            return true;
        }

        ESP_LOGI(TAG, "Powering off GNSS...");
        auto* modem = GetModem();
        if (modem) {
            auto at_uart = modem->GetAtUart();
            at_uart->SendCommand("AT+CGNSSPWR=0", 3000);
        }
        gnss_powered_on_ = false;
        ESP_LOGI(TAG, "GNSS powered off");
        return true;
    }

    // ==================== GNSS ====================

    ReturnValue HandleGetGnss() {
        ESP_LOGI(TAG, "=== HandleGetGnss START ===");

        auto* modem = GetModem();
        if (!modem) {
            return std::string("{\"error\":\"Modem not available. Please switch to 4G network mode.\"}");
        }

        auto at_uart = modem->GetAtUart();

        PauseAudioOutput();

        if (!PowerOnGnss()) {
            ResumeAudioOutput();
            return std::string("{\"error\":\"Failed to power on GNSS module. Check SIM card and antenna.\"}");
        }

        ESP_LOGI(TAG, "Polling GNSS info (max 60s, every 2s)...");
        std::string response;
        bool got_fix = false;
        const int max_attempts = 30;

        for (int i = 0; i < max_attempts; i++) {
            vTaskDelay(pdMS_TO_TICKS(2000));

            if (!SendAtCommand("AT+CGNSSINFO", 3000, 3)) {
                ESP_LOGW(TAG, "GNSS: query failed, attempt %d/%d", i + 1, max_attempts);
                continue;
            }

            response = at_uart->GetResponse();
            ESP_LOGI(TAG, "GNSS raw response [%d]: %s", i + 1, response.c_str());

            int state = ParseGnssState(response);
            ESP_LOGI(TAG, "GNSS state: %d", state);

            if (state > 0) {
                got_fix = true;
                ESP_LOGI(TAG, "GNSS fix obtained after %d seconds", (i + 1) * 2);
                break;
            }
        }

        PowerOffGnss();
        ResumeAudioOutput();

        if (!got_fix) {
            return std::string("{\"error\":\"GNSS fix timeout. Ensure antenna is connected and device has clear sky view.\"}");
        }

        std::string latitude, longitude, altitude;
        if (!ParseGnssResponse(response, latitude, longitude, altitude)) {
            return std::string("{\"error\":\"Failed to parse GNSS response.\"}");
        }

        std::string result = BuildGnssJson(latitude, longitude, altitude);
        ESP_LOGI(TAG, "GNSS result: %s", result.c_str());
        ESP_LOGI(TAG, "=== HandleGetGnss END ===");
        return result;
    }

    int ParseGnssState(const std::string& response) {
        size_t pos = response.find("+CGNSSINFO:");
        if (pos == std::string::npos) return -1;

        std::string data = response.substr(pos + 11);
        while (!data.empty() && data[0] == ' ') data = data.substr(1);

        size_t comma = data.find(',');
        if (comma == std::string::npos) return -1;

        try {
            return std::stoi(data.substr(0, comma));
        } catch (...) {
            return -1;
        }
    }

    bool ParseGnssResponse(const std::string& response,
                           std::string& latitude, std::string& longitude, std::string& altitude) {
        size_t pos = response.find("+CGNSSINFO:");
        if (pos == std::string::npos) return false;

        std::string data = response.substr(pos + 11);
        while (!data.empty() && data[0] == ' ') data = data.substr(1);

        auto parts = SplitByComma(data);
        if (parts.size() < 5) return false;

        try {
            double lat_raw = std::stod(parts[1]);
            int lat_deg = (int)(lat_raw / 100);
            double lat_min = lat_raw - lat_deg * 100;
            double lat_dec = lat_deg + lat_min / 60.0;
            if (parts[2] == "S") lat_dec = -lat_dec;
            latitude = std::to_string(lat_dec);

            double lon_raw = std::stod(parts[3]);
            int lon_deg = (int)(lon_raw / 100);
            double lon_min = lon_raw - lon_deg * 100;
            double lon_dec = lon_deg + lon_min / 60.0;
            if (parts[4] == "W") lon_dec = -lon_dec;
            longitude = std::to_string(lon_dec);

            if (parts.size() >= 6 && !parts[5].empty()) {
                altitude = parts[5];
            }
        } catch (...) {
            return false;
        }

        return true;
    }

    std::string BuildGnssJson(const std::string& lat, const std::string& lon, const std::string& alt) {
        std::string json = "{\"type\":\"gnss\",\"latitude\":";
        json += lat + ",\"longitude\":" + lon;
        if (!alt.empty()) json += ",\"altitude\":" + alt;
        json += "}";
        return json;
    }

    // ==================== LBS ====================

    bool ConfigureLbs() {
        if (lbs_configured_) {
            ESP_LOGI(TAG, "LBS already configured");
            return true;
        }

        ESP_LOGI(TAG, "Configuring LBS (OneOS, method=40)...");
        if (!SendAtCommand("AT+MLBSCFG=\"method\",40")) {
            ESP_LOGE(TAG, "Failed to set LBS method to OneOS");
            return false;
        }

        SendAtCommand("AT+MLBSCFG=\"nearbtsen\",1");

        lbs_configured_ = true;
        ESP_LOGI(TAG, "LBS configured (OneOS, no apikey needed)");
        return true;
    }

    ReturnValue HandleGetLbs() {
        ESP_LOGI(TAG, "=== HandleGetLbs START ===");

        auto* modem = GetModem();
        if (!modem) {
            return std::string("{\"error\":\"Modem not available. Please switch to 4G network mode.\"}");
        }

        auto at_uart = modem->GetAtUart();

        PauseAudioOutput();

        if (!ConfigureLbs()) {
            ResumeAudioOutput();
            return std::string("{\"error\":\"Failed to configure LBS service.\"}");
        }

        // +MLBSLOC 响应是 URC 格式（+MLBSLOC: ...），会被 SendCommand 内部消费
        // 必须用 URC 回调捕获，不能用 GetResponse()
        SemaphoreHandle_t semaphore = xSemaphoreCreateBinary();
        if (!semaphore) {
            ResumeAudioOutput();
            return std::string("{\"error\":\"Failed to create semaphore.\"}");
        }

        int status_code = 0;
        std::string longitude, latitude, accuracy;

        // 注册 URC 回调（AtUart::ParseResponse 会去掉 + 前缀，所以匹配 "MLBSLOC"）
        ESP_LOGI(TAG, "Registering URC callback for MLBSLOC...");
        auto callback_it = at_uart->RegisterUrcCallback(
            [&](const std::string& command, const std::vector<AtArgumentValue>& arguments) {
                ESP_LOGI(TAG, "URC callback fired: command=%s, args=%d", command.c_str(), (int)arguments.size());
                if (command == "MLBSLOC" && arguments.size() >= 3) {
                    status_code = arguments[0].int_value;
                    if (arguments[1].type == AtArgumentValue::Type::String) {
                        longitude = arguments[1].string_value;
                    } else {
                        longitude = std::to_string(arguments[1].double_value);
                    }
                    if (arguments[2].type == AtArgumentValue::Type::String) {
                        latitude = arguments[2].string_value;
                    } else {
                        latitude = std::to_string(arguments[2].double_value);
                    }
                    if (arguments.size() >= 4) {
                        if (arguments[3].type == AtArgumentValue::Type::Int) {
                            accuracy = std::to_string(arguments[3].int_value);
                        } else if (arguments[3].type == AtArgumentValue::Type::String) {
                            accuracy = arguments[3].string_value;
                        } else {
                            accuracy = std::to_string(arguments[3].double_value);
                        }
                    } else {
                        accuracy = "500";
                    }
                    ESP_LOGI(TAG, "URC MLBSLOC parsed: status=%d, lon=%s, lat=%s, acc=%s",
                             status_code, longitude.c_str(), latitude.c_str(), accuracy.c_str());
                    xSemaphoreGive(semaphore);
                }
            });

        // 发送 AT+MLBSLOC
        ESP_LOGI(TAG, "Sending AT+MLBSLOC...");
        if (!SendAtCommand("AT+MLBSLOC", 30000, 1)) {
            at_uart->UnregisterUrcCallback(callback_it);
            vSemaphoreDelete(semaphore);
            ResumeAudioOutput();
            return std::string("{\"error\":\"Failed to send LBS location request.\"}");
        }

        // 同时检查 GetResponse（可能是同步响应）
        std::string response = at_uart->GetResponse();
        ESP_LOGI(TAG, "LBS GetResponse: [%s]", response.c_str());

        // 如果 GetResponse 没有数据，尝试从 URC 回调获取
        if (response.empty() || response.find("+MLBSLOC:") == std::string::npos) {
            ESP_LOGI(TAG, "No +MLBSLOC: in GetResponse, waiting for URC callback (max 30s)...");
            if (xSemaphoreTake(semaphore, pdMS_TO_TICKS(30000)) != pdTRUE) {
                ESP_LOGW(TAG, "URC callback timeout (30s)");
                at_uart->UnregisterUrcCallback(callback_it);
                vSemaphoreDelete(semaphore);
                ResumeAudioOutput();
                return std::string("{\"error\":\"LBS location timeout. URC callback not triggered.\"}");
            }
        } else {
            // 从 GetResponse 解析
            ESP_LOGI(TAG, "Parsing +MLBSLOC: from GetResponse...");
            ParseLbsResponse(response, status_code, longitude, latitude, accuracy);
        }

        at_uart->UnregisterUrcCallback(callback_it);
        vSemaphoreDelete(semaphore);

        ResumeAudioOutput();

        ESP_LOGI(TAG, "LBS result: status=%d, lon=%s, lat=%s, acc=%s",
                 status_code, longitude.c_str(), latitude.c_str(), accuracy.c_str());

        if (status_code != 100) {
            return std::string("{\"error\":\"LBS failed with status code " + std::to_string(status_code) + ".\"}");
        }

        std::string result = BuildLbsJson(latitude, longitude, accuracy);
        ESP_LOGI(TAG, "=== HandleGetLbs END: %s", result.c_str());
        return result;
    }

    bool ParseLbsResponse(const std::string& response, int& status_code,
                          std::string& longitude, std::string& latitude, std::string& accuracy) {
        size_t pos = response.find("+MLBSLOC:");
        if (pos == std::string::npos) return false;

        std::string data = response.substr(pos + 9);
        while (!data.empty() && data[0] == ' ') data = data.substr(1);

        auto parts = SplitByComma(data);
        if (parts.size() < 3) return false;

        try {
            status_code = std::stoi(parts[0]);
            longitude = parts[1];
            latitude = parts[2];
            if (parts.size() >= 4) accuracy = parts[3]; else accuracy = "500";
        } catch (...) {
            return false;
        }

        return true;
    }

    std::string BuildLbsJson(const std::string& lat, const std::string& lon, const std::string& acc) {
        return "{\"type\":\"lbs\",\"latitude\":" + lat +
               ",\"longitude\":" + lon +
               ",\"accuracy\":" + acc + "}";
    }

    // ==================== Combined ====================

    ReturnValue HandleGetCurrent() {
        ESP_LOGI(TAG, "=== HandleGetCurrent START ===");

        auto gnss_result = HandleGetGnss();
        std::string result = std::get<std::string>(gnss_result);

        if (result.find("\"error\"") == std::string::npos) {
            return result;
        }

        ESP_LOGI(TAG, "GNSS failed, falling back to LBS...");
        return HandleGetLbs();
    }

    // ==================== Utility ====================

    static std::vector<std::string> SplitByComma(const std::string& str) {
        std::vector<std::string> parts;
        std::string part;
        for (char c : str) {
            if (c == ',') {
                parts.push_back(part);
                part.clear();
            } else {
                part += c;
            }
        }
        if (!part.empty()) parts.push_back(part);
        return parts;
    }
};

#undef TAG

#endif // LOCATION_CONTROLLER_H