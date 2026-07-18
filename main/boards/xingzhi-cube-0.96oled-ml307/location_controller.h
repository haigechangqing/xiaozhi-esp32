#ifndef LOCATION_CONTROLLER_H
#define LOCATION_CONTROLLER_H

#include "mcp_server.h"
#include "board.h"
#include <at_modem.h>
#include <at_uart.h>
#include <string>
#include <variant>
#include <vector>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define TAG "LocationController"

class LocationController {
public:
    LocationController() {
        auto& mcp_server = McpServer::GetInstance();

        // GNSS satellite positioning
        mcp_server.AddTool("self.location.get_gnss",
            "Get current location using GNSS (GPS/BeiDou/GLONASS) satellite positioning.\n"
            "Requires GNSS antenna connected and open sky view. Cold start takes 30-60 seconds.\n"
            "Returns JSON with type, latitude, longitude, altitude.",
            PropertyList(),
            [this](const PropertyList& properties) -> ReturnValue {
                return HandleGetGnss();
            });

        // LBS cellular positioning
        mcp_server.AddTool("self.location.get_lbs",
            "Get current location using cellular base station positioning (LBS).\n"
            "Works without GNSS antenna, only needs SIM card and cellular network.\n"
            "Accuracy typically 50-500 meters. Fast response (3-5 seconds).\n"
            "Returns JSON with type, latitude, longitude, accuracy.",
            PropertyList(),
            [this](const PropertyList& properties) -> ReturnValue {
                return HandleGetLbs();
            });

        // Combined positioning
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
        // Ensure GNSS is powered off on destruction
        PowerOffGnss();
    }

private:
    bool gnss_powered_on_ = false;
    bool lbs_configured_ = false;

    // Get the AtModem pointer from the current board
    AtModem* GetModem() {
        auto* network = Board::GetInstance().GetNetwork();
        return dynamic_cast<AtModem*>(network);
    }

    // 带重试的 AT 命令发送
    // 在音频传输期间 UART 可能被占用，需要重试
    bool SendAtCommand(const std::string& cmd, int timeout_ms = 3000, int max_retries = 5) {
        auto* modem = GetModem();
        if (!modem) return false;

        auto at_uart = modem->GetAtUart();

        for (int retry = 0; retry < max_retries; retry++) {
            if (at_uart->SendCommand(cmd, timeout_ms)) {
                return true;
            }
            if (retry < max_retries - 1) {
                ESP_LOGW(TAG, "Retry %d/%d for: %s", retry + 1, max_retries, cmd.c_str());
                vTaskDelay(pdMS_TO_TICKS(1000)); // 等待 UART 空闲
            }
        }
        ESP_LOGE(TAG, "Failed to send after %d retries: %s", max_retries, cmd.c_str());
        return false;
    }

    // ==================== GNSS Power Management ====================

    bool PowerOnGnss() {
        if (gnss_powered_on_) return true;

        auto* modem = GetModem();
        if (!modem) {
            ESP_LOGE(TAG, "Modem not available, cannot power on GNSS");
            return false;
        }

        if (!SendAtCommand("AT+CGNSSPWR=1")) {
            return false;
        }

        gnss_powered_on_ = true;
        ESP_LOGI(TAG, "GNSS powered on, waiting for module to initialize...");
        vTaskDelay(pdMS_TO_TICKS(2000)); // Give GNSS module time to initialize
        return true;
    }

    bool PowerOffGnss() {
        if (!gnss_powered_on_) return true;

        auto* modem = GetModem();
        if (!modem) {
            gnss_powered_on_ = false;
            return false;
        }

        auto at_uart = modem->GetAtUart();
        at_uart->SendCommand("AT+CGNSSPWR=0", 3000);
        gnss_powered_on_ = false;
        ESP_LOGI(TAG, "GNSS powered off");
        return true;
    }

    // ==================== GNSS ====================

    ReturnValue HandleGetGnss() {
        auto* modem = GetModem();
        if (!modem) {
            return std::string("{\"error\":\"Modem not available. Please switch to 4G network mode.\"}");
        }

        auto at_uart = modem->GetAtUart();

        // Power on GNSS
        if (!PowerOnGnss()) {
            return std::string("{\"error\":\"Failed to power on GNSS module.\"}");
        }

        // Poll for GNSS fix: every 2 seconds, max 60 seconds (30 attempts)
        ESP_LOGI(TAG, "Waiting for GNSS fix (max 60s)...");
        std::string response;
        bool got_fix = false;
        const int max_attempts = 30;

        for (int i = 0; i < max_attempts; i++) {
            vTaskDelay(pdMS_TO_TICKS(2000));

            if (!SendAtCommand("AT+CGNSSINFO", 3000, 3)) {
                ESP_LOGW(TAG, "Failed to query GNSS info, attempt %d/%d", i + 1, max_attempts);
                continue;
            }

            response = at_uart->GetResponse();
            ESP_LOGI(TAG, "GNSS raw response: %s", response.c_str());

            // Check if we have a valid fix
            int state = ParseGnssState(response);
            if (state > 0) {
                got_fix = true;
                ESP_LOGI(TAG, "GNSS fix obtained after %d seconds", (i + 1) * 2);
                break;
            }
        }

        // Power off GNSS to save power
        PowerOffGnss();

        if (!got_fix) {
            return std::string("{\"error\":\"GNSS fix timeout. Ensure antenna is connected and device has clear sky view.\"}");
        }

        // Parse the fix response
        std::string latitude, longitude, altitude;
        if (!ParseGnssResponse(response, latitude, longitude, altitude)) {
            return std::string("{\"error\":\"Failed to parse GNSS response.\"}");
        }

        return BuildGnssJson(latitude, longitude, altitude);
    }

    // Parse GNSS state from +CGNSSINFO response
    // Format: +CGNSSINFO: <state>,<lat>,<N/S>,<lon>,<E/W>,<alt>,...
    // Returns: 0=no fix, 1=fix, 2=2D fix, 3=3D fix, -1=parse error
    int ParseGnssState(const std::string& response) {
        size_t pos = response.find("+CGNSSINFO:");
        if (pos == std::string::npos) return -1;

        std::string data = response.substr(pos + 11);
        // Trim leading space
        while (!data.empty() && data[0] == ' ') data = data.substr(1);

        size_t comma = data.find(',');
        if (comma == std::string::npos) return -1;

        try {
            return std::stoi(data.substr(0, comma));
        } catch (...) {
            return -1;
        }
    }

    // Parse GNSS fix response into decimal degrees
    bool ParseGnssResponse(const std::string& response,
                           std::string& latitude, std::string& longitude, std::string& altitude) {
        size_t pos = response.find("+CGNSSINFO:");
        if (pos == std::string::npos) return false;

        std::string data = response.substr(pos + 11);
        while (!data.empty() && data[0] == ' ') data = data.substr(1);

        auto parts = SplitByComma(data);
        if (parts.size() < 5) return false;

        // state = parts[0]
        // lat = parts[1] (DDMM.MMMM format)
        // N/S = parts[2]
        // lon = parts[3] (DDDMM.MMMM format)
        // E/W = parts[4]
        // alt = parts[5] (optional)

        try {
            // Convert latitude from DDMM.MMMM to decimal degrees
            double lat_raw = std::stod(parts[1]);
            int lat_deg = (int)(lat_raw / 100);
            double lat_min = lat_raw - lat_deg * 100;
            double lat_dec = lat_deg + lat_min / 60.0;
            if (parts[2] == "S") lat_dec = -lat_dec;
            latitude = std::to_string(lat_dec);

            // Convert longitude from DDDMM.MMMM to decimal degrees
            double lon_raw = std::stod(parts[3]);
            int lon_deg = (int)(lon_raw / 100);
            double lon_min = lon_raw - lon_deg * 100;
            double lon_dec = lon_deg + lon_min / 60.0;
            if (parts[4] == "W") lon_dec = -lon_dec;
            longitude = std::to_string(lon_dec);

            // Altitude (optional)
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
        if (!alt.empty()) {
            json += ",\"altitude\":" + alt;
        }
        json += "}";
        return json;
    }

    // ==================== LBS ====================

    // 配置 LBS 定位服务（OneOS，模组内置鉴权，无需 apikey）
    bool ConfigureLbs() {
        if (lbs_configured_) return true;

        // 使用 OneOS 定位服务（method=40），模组内置鉴权参数，无需 apikey
        if (!SendAtCommand("AT+MLBSCFG=\"method\",40")) {
            ESP_LOGE(TAG, "Failed to set LBS method to OneOS");
            return false;
        }

        // 启用邻区信息参与定位，提升精度
        SendAtCommand("AT+MLBSCFG=\"nearbtsen\",1");

        lbs_configured_ = true;
        ESP_LOGI(TAG, "LBS configured (OneOS, no apikey needed)");
        return true;
    }

    ReturnValue HandleGetLbs() {
        auto* modem = GetModem();
        if (!modem) {
            return std::string("{\"error\":\"Modem not available. Please switch to 4G network mode.\"}");
        }

        auto at_uart = modem->GetAtUart();

        // 确保 LBS 服务已配置
        if (!ConfigureLbs()) {
            return std::string("{\"error\":\"Failed to configure LBS service.\"}");
        }

        // 发送 AT+MLBSLOC 请求定位
        // 模组会先返回 OK，然后异步返回 +MLBSLOC: <状态码>,<经度>,<纬度>,<精度>
        if (!SendAtCommand("AT+MLBSLOC")) {
            return std::string("{\"error\":\"Failed to request LBS location.\"}");
        }

        // 等待异步响应 +MLBSLOC:
        // AT+MLBSLOC 先返回 OK，+MLBSLOC: 结果在之后异步返回
        std::string response;
        bool got_location = false;
        const int max_wait = 15; // 最多等待 15 秒

        for (int i = 0; i < max_wait; i++) {
            vTaskDelay(pdMS_TO_TICKS(1000));

            // 发送 AT 命令触发串口读取，检查缓冲区中是否有 +MLBSLOC:
            SendAtCommand("AT", 2000, 3);
            response = at_uart->GetResponse();
            ESP_LOGI(TAG, "LBS wait %d/%d: %s", i + 1, max_wait, response.c_str());

            if (response.find("+MLBSLOC:") != std::string::npos) {
                got_location = true;
                break;
            }
        }

        if (!got_location) {
            return std::string("{\"error\":\"LBS location timeout. Check network and SIM card.\"}");
        }

        // 解析 +MLBSLOC: <状态码>,<经度>,<纬度>[,<精度>]
        int status_code = 0;
        std::string longitude, latitude, accuracy;
        if (!ParseLbsResponse(response, status_code, longitude, latitude, accuracy)) {
            return std::string("{\"error\":\"Failed to parse LBS response: " + response + "\"}");
        }

        // 状态码 100 = 定位成功
        if (status_code != 100) {
            ESP_LOGE(TAG, "LBS location failed, status code: %d", status_code);
            return std::string("{\"error\":\"LBS failed with status code " + std::to_string(status_code) + ".\"");
        }

        return BuildLbsJson(latitude, longitude, accuracy);
    }

    // 解析 +MLBSLOC: 响应
    // 格式: +MLBSLOC: <状态码>,<经度>,<纬度>[,<精度>]
    // 状态码: 100=成功, 126=失败需重启, 其他=错误
    bool ParseLbsResponse(const std::string& response,
                          int& status_code,
                          std::string& longitude, std::string& latitude, std::string& accuracy) {
        size_t pos = response.find("+MLBSLOC:");
        if (pos == std::string::npos) return false;

        std::string data = response.substr(pos + 9);
        while (!data.empty() && data[0] == ' ') data = data.substr(1);

        // 截取到行尾（+MLBSLOC: 后面同一行）
        size_t newline = data.find('\n');
        if (newline != std::string::npos) {
            data = data.substr(0, newline);
        }
        // 去掉尾部 \r
        if (!data.empty() && data.back() == '\r') {
            data.pop_back();
        }

        auto parts = SplitByComma(data);
        if (parts.size() < 3) return false;

        try {
            status_code = std::stoi(parts[0]);
        } catch (...) {
            return false;
        }

        // parts[1] = 经度, parts[2] = 纬度, parts[3] = 精度(可选)
        longitude = parts[1];
        latitude = parts[2];
        if (parts.size() >= 4) {
            accuracy = parts[3];
        } else {
            accuracy = "500"; // 默认精度
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
        // Try GNSS first
        auto gnss_result = HandleGetGnss();
        std::string result = std::get<std::string>(gnss_result);

        if (result.find("\"error\"") == std::string::npos) {
            return result;
        }

        // GNSS failed, fall back to LBS
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
        if (!part.empty()) {
            parts.push_back(part);
        }
        return parts;
    }
};

#undef TAG

#endif // LOCATION_CONTROLLER_H