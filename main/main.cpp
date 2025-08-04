#include <atomic>
#include <chrono>
#include <thread>

#include <driver/gpio.h>

#include <usb/usb_host.h>
#include <usb/hid_host.h>
#include <usb/hid_usage_keyboard.h>
#include <usb/hid_usage_mouse.h>

#include "logger.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

static espp::Logger logger({.tag = "esp-usb-latency-test", .level = espp::Logger::Verbosity::DEBUG});

enum class ControllerType {
  UNKNOWN,
  SONY,
  XBOXONE,
  XBOX360,
  SWITCH_PRO,
  BACKBONE,
  EIGHTBITDO, // NOTE: use 'D' compatibility setting
};

// array of report byte indexes for each controller type that should be checked
// for changes, or -1 if the controller only reports changes. Can be any number
// of bytes, but the first byte should be the first byte of the report that
// should be checked. The second byte should be the last byte of the report that
// should be checked. All bytes between the two will be checked for changes.
static const int report_bytes[][2] = {
  { -1, -1 }, // UNKNOWN
  { 8, 9 }, // SONY
  { -1, -1 }, // XBOXONE
  { -1, -1 }, // XBOX360
  // { 3, 5 }, // SWITCH_PRO
  { -1, -1 }, // SWITCH_PRO
  // { 12, 13 }, // BACKBONE
  { -1, -1 }, // BACKBONE
  { 8, 9 }, // EIGHTBITDO; NOTE: use 'D' compatibility setting
};

static const int input_report_id[] = {
  -1, // UNKNOWN - don't care
  0x01, // SONY
  0x01, // XBOXONE
  0x01, // XBOX360
  -1, // SWITCH_PRO
  0x01, // BACKBONE
  -1, // EIGHTBITDO; NOTE: use 'D' compatibility setting
};

// for libfmt printing of gpio_num_t
template <> struct fmt::formatter<gpio_num_t> : fmt::formatter<std::string> {
  template <typename FormatContext> auto format(gpio_num_t t, FormatContext &ctx) const {
    return fmt::format_to(ctx.out(), "GPIO_NUM_{}", (int)t);
  }
};

// for libfmt printing of ControllerType
template <> struct fmt::formatter<ControllerType> : fmt::formatter<std::string> {
  template <typename FormatContext> auto format(ControllerType t, FormatContext &ctx) const {
    switch (t) {
    case ControllerType::UNKNOWN:
      return fmt::format_to(ctx.out(), "UNKNOWN");
    case ControllerType::SONY:
      return fmt::format_to(ctx.out(), "Sony");
    case ControllerType::XBOXONE:
      return fmt::format_to(ctx.out(), "Xbox One");
    case ControllerType::XBOX360:
      return fmt::format_to(ctx.out(), "Xbox 360");
    case ControllerType::SWITCH_PRO:
      return fmt::format_to(ctx.out(), "Nintendo Switch Pro");
    case ControllerType::BACKBONE:
      return fmt::format_to(ctx.out(), "Backbone");
    case ControllerType::EIGHTBITDO:
      return fmt::format_to(ctx.out(), "8BitDo");
    default:
      return fmt::format_to(ctx.out(), "UNKNOWN");
    }
  }
};

// button pin configuration
static constexpr gpio_num_t button_pin = (gpio_num_t)CONFIG_BUTTON_GPIO;
static int BUTTON_PRESSED_LEVEL = CONFIG_BUTTON_PRESS_LEVEL;
static int BUTTON_RELEASED_LEVEL = !BUTTON_PRESSED_LEVEL;

// button press/release timing configuration
static uint64_t button_press_start = 0;
static uint64_t button_release_start = 0;
static uint64_t latency_us = 0;
static constexpr uint64_t IDLE_US = 50 * 1000; // time between button presses
static constexpr uint64_t HOLD_TIME_US = CONFIG_BUTTON_HOLD_TIME_MS * 1000;
static constexpr uint64_t MAX_SHIFT_MS = CONFIG_MAX_BUTTON_DELAY_MS;

// used for notifying the HID task to check the latency
static TaskHandle_t hid_task_handle_ = NULL;

// randomly shift the button press time within the 1s period
static int shift = 0;

// what device is connected
static std::atomic<bool> connected = false;
static std::string connected_manufacturer = "";
static std::string connected_product = "";
static std::atomic<ControllerType> connected_controller_type = ControllerType::UNKNOWN;

// HID signaling events for the callback functions
QueueHandle_t app_event_queue = NULL;
typedef enum {
  APP_EVENT_HID_HOST = 0
} app_event_group_t;
typedef struct {
  app_event_group_t event_group;
  /* HID Host - Device related info */
  struct {
    hid_host_device_handle_t handle;
    hid_host_driver_event_t event;
    void *arg;
  } hid_host_device;
} app_event_queue_t;

// HID Host Device callback functions
static std::vector<uint8_t> last_report_data;
static bool check_report_changed(const uint8_t *const data, const int length);
static void hid_host_generic_report_callback(const uint8_t *const data, const int length);
static void hid_host_interface_event_callback(hid_host_device_handle_t hid_device_handle,
                                              const hid_host_interface_event_t event,
                                              void *arg);
static void hid_host_driver_event_callback(hid_host_device_handle_t hid_device_handle,
                                           const hid_host_driver_event_t event,
                                           void *arg);
static void handle_hid_host_device_event(hid_host_device_handle_t hid_device_handle,
                                           const hid_host_driver_event_t event,
                                           void *arg);
static void usb_lib_task(void *arg);

enum class SwitchProState : uint8_t {
  DISCONNECTED = 0x00,
  CONNECTED,
  DEVICE_INFO,
  HANDSHAKE_1,
  SET_BAUDRATE,
  HANDSHAKE_2,
  HID_ONLY_MODE,
  READY,
};
static SwitchProState switch_pro_state = SwitchProState::DISCONNECTED;
static hid_host_device_handle_t switch_pro_handle = NULL;
static void configure_switch_pro();

// main code
extern "C" void app_main(void) {
  static auto start = std::chrono::high_resolution_clock::now();
  static auto elapsed = [&]() {
    auto now = std::chrono::high_resolution_clock::now();
    return std::chrono::duration<float>(now - start).count();
  };

  logger.info("Bootup");

  logger.info("Setting up button GPIO: {}", button_pin);
  gpio_set_direction(button_pin, GPIO_MODE_OUTPUT);
  gpio_set_level(button_pin, BUTTON_RELEASED_LEVEL);

  // set GPIO18 (USB_SEL) to high to enable USB Host (receptacle) mode. Default
  // is low (USB device)
  logger.info("Enabling USB Host mode");
  static constexpr auto GPIO_USB_SEL = GPIO_NUM_18;
  gpio_set_direction(GPIO_USB_SEL, GPIO_MODE_OUTPUT);
  gpio_set_level(GPIO_USB_SEL, 1);

  // set GPIO12 (dev_vbus_en) high to enable VBUS output to USB device. Default
  // is low (disabled)
  logger.info("Enabling VBUS output to USB device");
  static constexpr auto GPIO_DEV_VBUS_EN = GPIO_NUM_12;
  gpio_set_direction(GPIO_DEV_VBUS_EN, GPIO_MODE_OUTPUT);
  gpio_set_level(GPIO_DEV_VBUS_EN, 1);

  // Set GPIO17 (IDEV_LIMIT_EN) high to enable current limiting IC to output
  // voltage
  logger.info("Enabling current limiting IC to output voltage");
  static constexpr auto GPIO_IDEV_LIMIT_EN = GPIO_NUM_17;
  gpio_set_direction(GPIO_IDEV_LIMIT_EN, GPIO_MODE_OUTPUT);
  gpio_set_level(GPIO_IDEV_LIMIT_EN, 1);

  app_event_queue_t evt_queue;

  /*
   * Create usb_lib_task to:
   * - initialize USB Host library
   * - Handle USB Host events while APP pin in in HIGH state
   */
  auto task_created = xTaskCreatePinnedToCore(usb_lib_task,
                                         "usb_events",
                                         4096,
                                         xTaskGetCurrentTaskHandle(),
                                         20, NULL, 0);
  assert(task_created == pdTRUE);

  // Wait for notification from usb_lib_task to proceed
  ulTaskNotifyTake(false, 1000);

  /*
   * HID host driver configuration
   * - create background task for handling low level event inside the HID driver
   * - provide the device callback to get new HID Device connection event
   */
  static const hid_host_driver_config_t hid_host_driver_config = {
    .create_background_task = true,
    .task_priority = 10,
    .stack_size = 4096,
    .core_id = 1,
    .callback = hid_host_driver_event_callback,
    .callback_arg = NULL
  };

  ESP_ERROR_CHECK(hid_host_install(&hid_host_driver_config));

  // Create queue
  app_event_queue = xQueueCreate(10, sizeof(app_event_queue_t));

  // make a task to handle HID events
  espp::Task hid_task({
        .callback = [&]() -> bool {
          // set the task handle
          hid_task_handle_ = xTaskGetCurrentTaskHandle();

          // wait for a HID device to be connected
          logger.info("Waiting for HID Device to be connected");
          // Wait queue
          while (!connected) {
            std::this_thread::sleep_for(1s);
          }

          if (switch_pro_handle != NULL) {
            logger.info("Configuring Switch Pro Controller...");
            while (switch_pro_state != SwitchProState::READY) {
              configure_switch_pro();
              std::this_thread::sleep_for(1s);
            }
          }

          bool button_level = BUTTON_RELEASED_LEVEL;

          // wait until we can successfully set the button and get a packet
          // (fully connected)
          while (connected) {
            // update the button level
            button_level = !button_level;
            // set the button
            gpio_set_level(button_pin, button_level);
            auto start = esp_timer_get_time();
            // wait to get notified by the HID task
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));
            auto end = esp_timer_get_time();
            auto elapsed_time = end - start;
            // if we got notified before the timeout, then we can break out of the loop
            if (elapsed_time < (1000-1)*1000) {
              logger.info("HID Device is ready, button press detected in {:.3f} ms", elapsed_time / 1000.0f);
              break;
            }
          }

          if (!connected) {
            logger.error("HID Device disconnected before button press detected");
            return false;
          }

          gpio_set_level(button_pin, BUTTON_RELEASED_LEVEL);
          ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(500));

          // device is connected, start the latency test
          logger.info("Starting latency test");
          fmt::print("% time (s), latency (ms), num missed inputs\n");

          uint32_t num_missed_inputs = 0;

          // loop until the device is disconnected
          while (connected) {
            // reset the state at the beginning of the loop
            shift = (rand() % MAX_SHIFT_MS) * 1000;
            button_press_start = 0;
            button_release_start = 0;
            static constexpr uint64_t MAX_LATENCY_MS = 200;

            logger.debug("Waiting for {} microseconds before button press", IDLE_US + shift);

            // wait for (IDLE_US + shift) microseconds
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS((IDLE_US + shift) / 1000));

            // trigger a button press
            button_press_start = esp_timer_get_time();
            gpio_set_level(button_pin, BUTTON_PRESSED_LEVEL);

            // wait for up to 500ms for the button press to be detected
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(MAX_LATENCY_MS));
            latency_us = esp_timer_get_time() - button_press_start;

            if (latency_us < (MAX_LATENCY_MS-1) * 1000) {
              // log the latency
              fmt::print("{:.3f}, {:.3f}, {}\n", elapsed(), latency_us / 1e3f, num_missed_inputs);
            } else {
              num_missed_inputs++;
            }

            logger.debug("Button pressed for {} microseconds", HOLD_TIME_US);

            // latency reached, release the button after hold time
            std::this_thread::sleep_for(std::chrono::microseconds(HOLD_TIME_US - latency_us));

            // release the button
            button_release_start = esp_timer_get_time();
            gpio_set_level(button_pin, BUTTON_RELEASED_LEVEL);

            // wait for up to 500ms for the button release to be detected
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(MAX_LATENCY_MS));
            latency_us = esp_timer_get_time() - button_release_start;

            if (latency_us < (MAX_LATENCY_MS-1) * 1000) {
              // log the latency
              fmt::print("{:.3f}, {:.3f}, {}\n", elapsed(), latency_us / 1e3f, num_missed_inputs);
            } else {
              num_missed_inputs++;
            }
          }
          logger.info("HID Device disconnected");

        // we don't want to stop the task, so return false
        return false;
        },
          .task_config = {
            .name = "HID Task",
            .stack_size_bytes = 8192,
            .priority = 10,
            .core_id = 1,
          },
        });
  hid_task.start();

  // set the task priority (for main) to high
  vTaskPrioritySet(nullptr, 20);

  while (true) {
    if (xQueueReceive(app_event_queue, &evt_queue, portMAX_DELAY)) {
      if (APP_EVENT_HID_HOST ==  evt_queue.event_group) {
        handle_hid_host_device_event(evt_queue.hid_host_device.handle,
                                     evt_queue.hid_host_device.event,
                                     evt_queue.hid_host_device.arg);
      }
    }
  }

  // we should never reach this point
  logger.info("HID Driver uninstall");
  ESP_ERROR_CHECK(hid_host_uninstall());
  xQueueReset(app_event_queue);
  vQueueDelete(app_event_queue);
}

/**
 * @brief Check if the report has changed
 *
 * @param[in] data    Pointer to input report data buffer
 * @param[in] length  Length of input report data buffer
 *
 * @return true if the report has changed, false otherwise
 *
 * @note This function takes as input the input report data buffer and its
 *       length and uses the connected_controller type to determine if we need
 *       to check specific bytes of the report to determine change (e.g. if the
 *       device is constantly sending data) or if simply receiving a report
 *       means a change was detected. It uses the controller type as an index
 *       into the report_bytes array to determine which bytes to check (if any).
 */
static bool IRAM_ATTR check_report_changed(const uint8_t *const data, const int length) {
  int raw_controller_type = static_cast<int>(connected_controller_type.load());
  int report_id_value = input_report_id[raw_controller_type];
  if (report_id_value != -1 && data[0] != report_id_value) {
    return false;
  }
  int start_byte = report_bytes[raw_controller_type][0];
  int end_byte = report_bytes[raw_controller_type][1];
  if (start_byte == -1 || end_byte == -1) {
    return true;
  }
  int num_bytes = end_byte - start_byte + 1;
  // use static so that the vector is not reallocated every time
  static std::vector<uint8_t> report;
  report.reserve(num_bytes);
  // copy the relevant bytes from the report into a vector
  report.assign(data + start_byte, data + end_byte + 1);
  // if the report size has changed, then the report has changed
  if (last_report_data.size() != report.size()) {
    last_report_data = report;
    return true;
  }
  // compare the bytes
  for (size_t i = 0; i < report.size(); i++) {
    if (report[i] != last_report_data[i]) {
      last_report_data = report;
      return true;
    }
  }
  // no change
  last_report_data = report;
  return false;
}

#include "switch_controller_protocol.hpp"

void configure_switch_pro() {
  if (!switch_pro_handle) {
    logger.error("Switch Pro Controller handle is NULL, cannot configure");
    return;
  }

  // originally found https://github.com/ttsuki/ProControllerHid/tree/develop

  static constexpr uint8_t init_device_info[] = {sp::HOST_INIT_REPORT, sp::INIT_COMMAND_DEVICE_INFO};
  static constexpr uint8_t init_handshake[] = {sp::HOST_INIT_REPORT, sp::INIT_COMMAND_HANDSHAKE};
  static constexpr uint8_t init_set_baudrate[] = {sp::HOST_INIT_REPORT, sp::INIT_COMMAND_SET_BAUD_RATE};
  static constexpr uint8_t init_enable_usb_hid[] = {sp::HOST_INIT_REPORT, sp::INIT_COMMAND_ENABLE_USB_HID};

  // send the packets over the hid interface to the controller
  static constexpr uint8_t report_type = HID_REPORT_TYPE_OUTPUT;
  static constexpr uint8_t report_id = 0x01; // Note: switch pro doesn't actually care about the report ID

  switch (switch_pro_state) {
    case SwitchProState::DISCONNECTED:
      // do nothing
      break;
    case SwitchProState::CONNECTED:
      // send the handshake
      ESP_ERROR_CHECK(hid_class_request_set_report(switch_pro_handle, report_type, report_id, const_cast<uint8_t*>(init_device_info), sizeof(init_device_info)));
      switch_pro_state = SwitchProState::HANDSHAKE_1;
      break;
    case SwitchProState::DEVICE_INFO:
      // send the handshake
      ESP_ERROR_CHECK(hid_class_request_set_report(switch_pro_handle, report_type, report_id, const_cast<uint8_t*>(init_handshake), sizeof(init_handshake)));
      switch_pro_state = SwitchProState::HANDSHAKE_1;
      break;
    case SwitchProState::HANDSHAKE_1:
      // send the set baudrate
      ESP_ERROR_CHECK(hid_class_request_set_report(switch_pro_handle, report_type, report_id, const_cast<uint8_t*>(init_set_baudrate), sizeof(init_set_baudrate)));
      switch_pro_state = SwitchProState::SET_BAUDRATE;
      break;
    case SwitchProState::SET_BAUDRATE:
      switch_pro_state = SwitchProState::HANDSHAKE_2;
      // send the handshake
      ESP_ERROR_CHECK(hid_class_request_set_report(switch_pro_handle, report_type, report_id, const_cast<uint8_t*>(init_handshake), sizeof(init_handshake)));
      break;
    case SwitchProState::HANDSHAKE_2:
      // send the hid only mode
      ESP_ERROR_CHECK(hid_class_request_set_report(switch_pro_handle, report_type, report_id, const_cast<uint8_t*>(init_enable_usb_hid), sizeof(init_enable_usb_hid)));
      switch_pro_state = SwitchProState::HID_ONLY_MODE;
      break;
    case SwitchProState::HID_ONLY_MODE:
      // now we're ready to use the controller
      switch_pro_state = SwitchProState::READY;
      logger.info("Switch Pro Controller configured");
      break;
    case SwitchProState::READY:
      // do nothing
      break;
  default:
    break;
  }
}

/**
 * @brief USB HID Host Generic Interface report callback handler
 *
 * 'generic' means anything else than mouse or keyboard
 *
 * @param[in] data    Pointer to input report data buffer
 * @param[in] length  Length of input report data buffer
 */
static void IRAM_ATTR hid_host_generic_report_callback(const uint8_t *const data, const int length)
{
  if (check_report_changed(data, length)) {
    if (hid_task_handle_ != nullptr) vTaskNotifyGiveFromISR(hid_task_handle_, NULL);
    logger.debug("Report changed[{}]: {::#02x}", length, std::vector<uint8_t>(data, data + length));
  } else {
    logger.debug("Report unchanged: {::#02x}", std::vector<uint8_t>(data, data + length));
  }
}

/**
 * @brief USB HID Host interface callback
 *
 * @param[in] hid_device_handle  HID Device handle
 * @param[in] event              HID Host interface event
 * @param[in] arg                Pointer to arguments, does not used
 */
static void IRAM_ATTR hid_host_interface_event_callback(hid_host_device_handle_t hid_device_handle,
                                                        const hid_host_interface_event_t event,
                                                        void *arg)
{
  uint8_t data[64] = { 0 };
  size_t data_length = 0;
  hid_host_dev_params_t dev_params;
  ESP_ERROR_CHECK(hid_host_device_get_params(hid_device_handle, &dev_params));

  if (connected_controller_type == ControllerType::SWITCH_PRO) {
    switch_pro_handle = hid_device_handle;
  }

  logger.debug("HID Device Interface event: {}",
              (int)(event));

  switch (event) {
  case HID_HOST_INTERFACE_EVENT_INPUT_REPORT:
    ESP_ERROR_CHECK(hid_host_device_get_raw_input_report_data(hid_device_handle,
                                                              data,
                                                              64,
                                                              &data_length));
    hid_host_generic_report_callback(data, data_length);
    break;
  case HID_HOST_INTERFACE_EVENT_DISCONNECTED:
    logger.info("HID Device DISCONNECTED");
    connected_manufacturer = "";
    connected_product = "";
    connected_controller_type = ControllerType::UNKNOWN;
    connected = false;
    switch_pro_state = SwitchProState::DISCONNECTED;
    switch_pro_handle = NULL;
    ESP_ERROR_CHECK(hid_host_device_close(hid_device_handle));
    break;
  case HID_HOST_INTERFACE_EVENT_TRANSFER_ERROR:
    logger.info("HID Device TRANSFER_ERROR");
    break;
  default:
    logger.error("HID Device Unhandled event");
    break;
  }
}

/**
 * @brief USB HID Host Device event
 *
 * @param[in] hid_device_handle  HID Device handle
 * @param[in] event              HID Host Device event
 * @param[in] arg                Pointer to arguments, does not used
 */
static void IRAM_ATTR handle_hid_host_device_event(hid_host_device_handle_t hid_device_handle,
                                                     const hid_host_driver_event_t event,
                                                     void *arg)
{
  hid_host_dev_params_t dev_params;
  ESP_ERROR_CHECK(hid_host_device_get_params(hid_device_handle, &dev_params));

  switch (event) {
  case HID_HOST_DRIVER_EVENT_CONNECTED: {
    logger.info("HID Device CONNECTED");

    connected = true;
    last_report_data.clear();
    // get the device info
    hid_host_dev_info_t dev_info;
    ESP_ERROR_CHECK(hid_host_get_device_info(hid_device_handle, &dev_info));
    // convert the wchar_t strings to utf-8
    char manufacturer[128] = { 0 };
    char product[128] = { 0 };
    char serial[128] = { 0 };
    wcstombs(manufacturer, dev_info.iManufacturer, sizeof(manufacturer));
    wcstombs(product, dev_info.iProduct, sizeof(product));
    wcstombs(serial, dev_info.iSerialNumber, sizeof(serial));
    logger.info("  - VID: 0x{:04X}, PID: 0x{:04X}, Manufacturer: '{}', Product: '{}', Serial: '{}'",
             dev_info.VID, dev_info.PID, manufacturer, product, serial);
    connected_manufacturer = manufacturer;
    connected_product = product;
    if (connected_manufacturer.contains("Sony")) {
      connected_controller_type = ControllerType::SONY;
    } else if (connected_manufacturer.contains("Microsoft")) {
      // TODO: properly handle 360 vs One controllers
      connected_controller_type = ControllerType::XBOXONE;
    } else if (connected_manufacturer.contains("Nintendo")) {
      if (connected_product == "Pro Controller") {
        connected_controller_type = ControllerType::SWITCH_PRO;
      }
    } else if (connected_manufacturer.contains("Backbone")) {
      connected_controller_type = ControllerType::BACKBONE;
    } else if (connected_manufacturer.contains("8BitDo")) {
      connected_controller_type = ControllerType::EIGHTBITDO;
    }
    logger.info("  - Controller Type: {}", connected_controller_type.load());

    if (connected_controller_type == ControllerType::SWITCH_PRO) {
      switch_pro_state = SwitchProState::CONNECTED;
      switch_pro_handle = hid_device_handle;
      // configure_switch_pro();
    }

    static const hid_host_device_config_t dev_config = {
      .callback = hid_host_interface_event_callback,
      .callback_arg = NULL
    };

    ESP_ERROR_CHECK(hid_host_device_open(hid_device_handle, &dev_config));
    if (HID_SUBCLASS_BOOT_INTERFACE == dev_params.sub_class) {
      ESP_ERROR_CHECK(hid_class_request_set_protocol(hid_device_handle, HID_REPORT_PROTOCOL_BOOT));
      if (HID_PROTOCOL_KEYBOARD == dev_params.proto) {
        ESP_ERROR_CHECK(hid_class_request_set_idle(hid_device_handle, 0, 0));
      }
    }
    ESP_ERROR_CHECK(hid_host_device_start(hid_device_handle));

    // get the report descriptor and print it out for debugging
    size_t report_descriptor_length = 0;
    uint8_t *desc = hid_host_get_report_descriptor(hid_device_handle,
                                                   &report_descriptor_length);
    std::vector<uint8_t> report_descriptor(desc, desc + report_descriptor_length);
    logger.info("Report Descriptor: {::#02x}", report_descriptor);
  } break;
  default:
    break;
  }
}

/**
 * @brief HID Host Device callback
 *
 * Puts new HID Device event to the queue
 *
 * @param[in] hid_device_handle HID Device handle
 * @param[in] event             HID Device event
 * @param[in] arg               Not used
 */
static void IRAM_ATTR hid_host_driver_event_callback(hid_host_device_handle_t hid_device_handle,
                                                     const hid_host_driver_event_t event,
                                                     void *arg)
{
  app_event_queue_t evt_queue;
  memset(&evt_queue, 0, sizeof(evt_queue));
  evt_queue.event_group = APP_EVENT_HID_HOST;
  // HID Host Device related info
  evt_queue.hid_host_device.handle = hid_device_handle;
  evt_queue.hid_host_device.event = event;
  evt_queue.hid_host_device.arg = arg;

  if (app_event_queue) {
    xQueueSend(app_event_queue, &evt_queue, 0);
  }
}

/**
 * @brief Start USB Host install and handle common USB host library events while app pin not low
 *
 * @param[in] arg  Not used
 */
static void IRAM_ATTR usb_lib_task(void *arg)
{
  usb_host_config_t host_config;
  memset(&host_config, 0, sizeof(host_config));
  host_config.skip_phy_setup = false;
  host_config.intr_flags = ESP_INTR_FLAG_LEVEL1;

  ESP_ERROR_CHECK(usb_host_install(&host_config));
  xTaskNotifyGive(static_cast<TaskHandle_t>(arg));

  while (true) {
    uint32_t event_flags;
    usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
    // In this example, there is only one client registered
    // So, once we deregister the client, this call must succeed with ESP_OK
    if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
      ESP_ERROR_CHECK(usb_host_device_free_all());
      break;
    }
  }

  logger.info("USB shutdown");
  // Clean up USB Host
  vTaskDelay(10); // Short delay to allow clients clean-up
  ESP_ERROR_CHECK(usb_host_uninstall());
  vTaskDelete(NULL);
}
