# Module cảm biến (Sensor) — hộp đen xe

Module đo gia tốc/góc quay (MPU6050), tốc độ bánh xe (cảm biến Hall), tính tốc độ/quãng đường và
phát hiện **phanh gấp** / **va chạm**. Chạy trên ESP32 (ESP-IDF v5.2+, FreeRTOS).
Phần lõi (driver, thuật toán) viết bằng C thuần, không phụ thuộc SDK nên test được trên PC.

## 1. Kiến trúc

```
 GPIO ISR ──> Drivers/Hall (hall_on_pulse) ──snapshot──> Services/Speed ──speed_kmh──┐
 I2C ──────> Drivers/MPU6050 (mpu6050_read) ──accel/gyro─────────────────────────────┤
                                                                                     v
                                               Services/EventDetection (phanh/va chạm) ─> event_t
                                                                                     |
 Application/sensor_task  (task FreeRTOS 100 Hz, ghép tất cả)                        |
   |-- sensor_get_latest()      : bản ghi mới nhất (mutex)                           |
   |-- sensor_get_history()     : vòng đệm 200 mẫu = 2 s (dữ liệu trước sự kiện)     |
   '-- sensor_get_event_queue() : queue event_t <────────────────────────────────────'
```

| Thư mục | Component ESP-IDF | Vai trò |
|---|---|---|
| `Drivers/MPU6050` | `MPU6050` | Driver IMU (lõi + port `port/mpu6050_port_esp32.c`, I2C master mới) |
| `Drivers/Hall` | `Hall` | Bắt xung Hall trong ISR (debounce 2 ms), snapshot an toàn dual-core |
| `Services/Speed` | `Speed` | Tốc độ (EMA), gia tốc, quãng đường từ xung Hall |
| `Services/EventDetection` | `EventDetection` | Máy trạng thái phanh gấp + phát hiện va chạm |
| `Application` | `Application` | `sensor_task`: task 100 Hz, API cho module khác |

## 2. Đấu dây

| MPU6050 | ESP32 |
|---|---|
| VCC | 3V3 |
| GND | GND |
| SDA | GPIO 21 |
| SCL | GPIO 22 |
| AD0 | GND (địa chỉ I2C = 0x68) |
| INT | không dùng |

| Hall (A3144 / KY-003) | ESP32 |
|---|---|
| VCC | 3V3 (hoặc 5V theo module) |
| GND | GND |
| OUT | GPIO 27 (tích cực mức thấp, ngắt cạnh xuống) |

Lưu ý: GPIO 34–39 của ESP32 **không có** pull-up nội, nếu dùng chân này phải gắn điện trở kéo lên 10 kΩ ngoài.
GPIO 27 dùng pull-up nội được, nhưng với dây dài/nhiễu nên thêm 10 kΩ ngoài (OUT → 3V3).
Cảm biến A3144 là open-collector nên bắt buộc có pull-up.

## 3. Hệ trục và cách gắn MPU6050

```
 +X = phía trước xe      +Y = bên trái      +Z = hướng lên
 Khi xe đứng yên, nằm ngang:  ax ≈ 0, ay ≈ 0, az ≈ +1 g
```

- Gắn chắc chắn (băng keo 2 mặt / vít), mặt chip hướng lên, mũi tên trục X của module hướng về phía trước xe.
- Gia tốc dương theo X = tăng tốc, âm = phanh.
- Hiệu chuẩn (`calib_samples`, mặc định 200 mẫu) chạy lúc khởi động: **xe phải đứng yên và nằm ngang**. Giữa các mẫu driver chờ 2 ms, nhưng với `CONFIG_FREERTOS_HZ=100` (mặc định ESP-IDF) port làm tròn lên 1 tick = 10 ms nên 200 mẫu mất khoảng 2 s (khoảng 0.4 s nếu `CONFIG_FREERTOS_HZ=1000`). `sensor_task_start()` **chặn task gọi** trong suốt thời gian này.

## 4. Thêm vào dự án ESP-IDF

Trong `CMakeLists.txt` của project (trước `project(...)`):

```cmake
cmake_minimum_required(VERSION 3.16)
set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/Drivers/MPU6050"
    "${CMAKE_CURRENT_LIST_DIR}/Drivers/Hall"
    "${CMAKE_CURRENT_LIST_DIR}/Services/Speed"
    "${CMAKE_CURRENT_LIST_DIR}/Services/EventDetection"
    "${CMAKE_CURRENT_LIST_DIR}/Application")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(blackbox)
```

Component của `main` thêm `REQUIRES Application` rồi `#include "sensor_task.h"`.

## 5. API cho các nhóm khác

```c
#include "sensor_task.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

void app_main(void)
{
    sensor_task_config_t cfg;
    sensor_task_default_config(&cfg);       /* chỉnh cfg.* nếu cần (chân, ngưỡng, chu vi bánh...) */
    if (sensor_task_start(&cfg) != 0) {
        /* lỗi nghiêm trọng: queue/mutex/task hoặc GPIO Hall */
    }

    QueueHandle_t q = (QueueHandle_t)sensor_get_event_queue();
    for (;;) {
        event_t evt;
        if (xQueueReceive(q, &evt, pdMS_TO_TICKS(100)) == pdTRUE) {
            /* Logger: lưu 2 s dữ liệu trước sự kiện + sự kiện */
            static sensor_record_t hist[SENSOR_HISTORY_LEN];
            size_t n = sensor_get_history(hist, SENSOR_HISTORY_LEN);   /* cũ nhất trước */
            /* ghi hist[0..n-1], evt.type (EVT_HARD_BRAKE / EVT_CRASH), evt.peak_g, evt.speed_kmh */
        }
        sensor_record_t now;
        if (sensor_get_latest(&now)) {
            /* OLED: now.speed_kmh, now.distance_m, now.temp_c ... */
        }
    }
}
```

| Hàm | Mô tả |
|---|---|
| `sensor_task_default_config(cfg)` | Giá trị mặc định (I2C0, SDA 21, SCL 22, 400 kHz, Hall GPIO 27 active-low, task core 1, stack 4096, ưu tiên 5) |
| `sensor_task_start(cfg)` | Khởi tạo và chạy task. Trả `0` = OK, âm = lỗi nghiêm trọng. Gọi 2 lần trả lỗi. Không tìm thấy MPU6050 **không** phải lỗi: task vẫn chạy, thử lại mỗi 1 s |
| `sensor_get_latest(&rec)` | Bản ghi mới nhất (thread-safe). `false` nếu chưa có |
| `sensor_get_history(buf, max)` | Tối đa `max` bản ghi gần nhất, cũ nhất trước; trả số bản ghi. Mỗi mẫu 10 ms |
| `sensor_get_event_queue()` | `QueueHandle_t` chứa `event_t`, `NULL` nếu chưa start |
| `sensor_get_dropped_events()` | Số sự kiện bị mất vì queue đầy (độ dài 8) |

Các hàm `sensor_get_*` gọi an toàn từ bất kỳ task nào (không gọi từ ISR), và trước khi start sẽ trả `false/0/NULL`.
`sensor_record_t.status`: bit `SENSOR_STATUS_IMU_OK` (mẫu IMU hợp lệ chu kỳ này), `SENSOR_STATUS_IMU_CAL` (đã hiệu chuẩn).
Khi IMU lỗi, các trường gia tốc/gyro/nhiệt độ bằng 0 và không chạy phát hiện sự kiện; tốc độ Hall vẫn cập nhật.
`timestamp_ms` = ms kể từ boot (`esp_timer`).

## 6. Ngưỡng mặc định và cách chỉnh

Gán vào `cfg.event_cfg` / `cfg.speed_cfg` sau `sensor_task_default_config()` và trước `sensor_task_start()`.

| Tham số | Mặc định | Ý nghĩa |
|---|---|---|
| `brake_threshold_g` | 0.45 g | Gia tốc (đã lọc) phanh vượt ngưỡng này thì bắt đầu tính phanh gấp |
| `brake_min_duration_ms` | 150 ms | Phải duy trì liên tục thì mới báo |
| `brake_min_speed_kmh` | 5 km/h | Dưới tốc độ này không xét phanh |
| `brake_cooldown_ms` | 2000 ms | Giãn cách tối thiểu giữa 2 sự kiện phanh (và cần hồi về trên ngưỡng) |
| `crash_threshold_g` | 3.0 g | Ngưỡng độ lớn gia tốc tổng (dữ liệu thô) |
| `crash_min_samples` | 2 | Số mẫu liên tiếp (20 ms) vượt ngưỡng |
| `crash_cooldown_ms` | 5000 ms | Giãn cách giữa 2 sự kiện va chạm |
| `lpf_alpha` | 0.3 | Hệ số lọc thông thấp cho ax khi xét phanh |
| `wheel_circumference_m` | 0.21 m | Chu vi bánh (xe mô hình); xe thật ví dụ 1.9 m |
| `pulses_per_rev` | 1 | Số nam châm trên bánh |
| `stop_timeout_us` | 2 s | Không có xung quá thời gian này thì tốc độ = 0 |
| `ema_alpha` (speed) | 0.5 | Lọc tốc độ, 1 = không lọc |

Cách chỉnh:
- Phanh báo quá nhiều: tăng `brake_threshold_g` hoặc `brake_min_duration_ms`. Bỏ sót phanh: giảm.
- Va chạm báo nhầm khi xe xóc/đi qua ổ gà: tăng `crash_threshold_g` (ví dụ 4–5 g) hoặc `crash_min_samples`.
- Tốc độ sai: kiểm tra `wheel_circumference_m` và `pulses_per_rev` (tốc độ tỉ lệ thuận với chu vi/số nam châm).
- Tốc độ giật: giảm `ema_alpha`; muốn nhạy hơn: tăng.
- Task, tần số: `SENSOR_PERIOD_MS` (10 ms), `SENSOR_SPEED_DIV` (5 chu kỳ = 20 Hz), `SENSOR_HISTORY_LEN` trong `sensor_task.h`.

## 7. Chạy test trên PC (host)

Không cần ESP32. Cần trình biên dịch C99:

```bash
# Có gcc/clang:
CC=gcc bash Tests/run_tests.sh
# Windows không có gcc (dùng zig):
CC="python -m ziglang cc" bash Tests/run_tests.sh
```

Script biên dịch với `-std=c99 -Wall -Wextra -Werror`, chạy: `test_mpu6050`, `test_hall`, `test_speed`,
`test_event_detection`, `test_integration` (kịch bản lái xe giả lập: đứng yên, tăng tốc, đi đều, phanh gấp, va chạm, dừng).
Kết quả cuối phải là `ALL TESTS PASSED`. `Application/sensor_task.c` chỉ biên dịch trong ESP-IDF, test host mô phỏng lại
logic vòng lặp của nó.

## 8. Giới hạn đã biết

- Chỉ hỗ trợ **1 MPU6050 và 1 cảm biến Hall** (trạng thái tĩnh, port dùng context tĩnh).
- Hiệu chuẩn cần xe **đứng yên và nằm ngang** lúc khởi động; nếu gắn nghiêng, ax/ay bị lệch (chưa bù góc lắp đặt).
- Tính tốc độ hoàn toàn từ xung Hall: độ phân giải thấp khi chậm (1 xung/vòng) và trễ khoảng 1 chu kỳ xung.
- Ngưỡng va chạm 3 g là giá trị ban đầu, **cần tinh chỉnh trên xe thật**.
- Khi mất IMU, việc khởi tạo lại (`mpu6050_init`, ~110 ms) chặn vòng lặp 100 Hz ngắn trong lúc thử lại (tối đa 1 lần/giây).
- Gia tốc tính từ vi phân tốc độ (`speed_result_t.accel_mps2`) nhiễu, chỉ dùng để tham khảo; phát hiện phanh dùng IMU.
