#include <chrono>
#include <thread>

#include <driver/gpio.h>

#include "usb/usb_host.h"
#include "usb/hid_host.h"
#include "usb/hid_usage_keyboard.h"
#include "usb/hid_usage_mouse.h"

#include "logger.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

static espp::Logger logger({.tag = "esp-usb-latency-test", .level = espp::Logger::Verbosity::DEBUG});


QueueHandle_t app_event_queue = NULL;

/**
 * @brief APP event group
 *
 * Application logic can be different. There is a one among other ways to distingiush the
 * event by application event group.
 * In this example we have two event groups:
 * APP_EVENT            - General event, which is APP_QUIT_PIN press event (Generally, it is IO0).
 * APP_EVENT_HID_HOST   - HID Host Driver event, such as device connection/disconnection or input report.
 */
typedef enum {
  APP_EVENT = 0,
  APP_EVENT_HID_HOST
} app_event_group_t;

/**
 * @brief APP event queue
 *
 * This event is used for delivering the HID Host event from callback to a task.
 */
typedef struct {
  app_event_group_t event_group;
  /* HID Host - Device related info */
  struct {
    hid_host_device_handle_t handle;
    hid_host_driver_event_t event;
    void *arg;
  } hid_host_device;
} app_event_queue_t;

/**
 * @brief HID Protocol string names
 */
static const char *hid_proto_name_str[] = {
  "NONE",
  "KEYBOARD",
  "MOUSE"
};

static void hid_host_generic_report_callback(const uint8_t *const data, const int length);
static void hid_host_interface_callback(hid_host_device_handle_t hid_device_handle,
                                        const hid_host_interface_event_t event,
                                        void *arg);
static void hid_host_device_event(hid_host_device_handle_t hid_device_handle,
                                  const hid_host_driver_event_t event,
                                  void *arg);
static void hid_host_device_callback(hid_host_device_handle_t hid_device_handle,
                                     const hid_host_driver_event_t event,
                                     void *arg);
static void usb_lib_task(void *arg);


extern "C" void app_main(void) {
  static auto start = std::chrono::high_resolution_clock::now();
  static auto elapsed = [&]() {
    auto now = std::chrono::high_resolution_clock::now();
    return std::chrono::duration<float>(now - start).count();
  };

  logger.info("Bootup");

  // set GPIO12 (dev_vbus_en) high to enable VBUS output to USB device
  static constexpr auto GPIO_DEV_VBUS_EN = GPIO_NUM_12;
  gpio_set_direction(GPIO_DEV_VBUS_EN, GPIO_MODE_OUTPUT);
  gpio_set_level(GPIO_DEV_VBUS_EN, 1);

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
                                         2, NULL, 0);
  assert(task_created == pdTRUE);

  // Wait for notification from usb_lib_task to proceed
  ulTaskNotifyTake(false, 1000);

  /*
   * HID host driver configuration
   * - create background task for handling low level event inside the HID driver
   * - provide the device callback to get new HID Device connection event
   */
  const hid_host_driver_config_t hid_host_driver_config = {
    .create_background_task = true,
    .task_priority = 5,
    .stack_size = 4096,
    .core_id = 0,
    .callback = hid_host_device_callback,
    .callback_arg = NULL
  };

  ESP_ERROR_CHECK(hid_host_install(&hid_host_driver_config));

  // Create queue
  app_event_queue = xQueueCreate(10, sizeof(app_event_queue_t));

  logger.info("Waiting for HID Device to be connected");

  while (1) {
    // Wait queue
    if (xQueueReceive(app_event_queue, &evt_queue, portMAX_DELAY)) {
      if (APP_EVENT == evt_queue.event_group) {
        // User pressed button
        usb_host_lib_info_t lib_info;
        ESP_ERROR_CHECK(usb_host_lib_info(&lib_info));
        if (lib_info.num_devices == 0) {
          // End while cycle
          break;
        } else {
          logger.warn("To shutdown example, remove all USB devices and press button again.");
          // Keep polling
        }
      }

      if (APP_EVENT_HID_HOST ==  evt_queue.event_group) {
        hid_host_device_event(evt_queue.hid_host_device.handle,
                              evt_queue.hid_host_device.event,
                              evt_queue.hid_host_device.arg);
      }
    }
  }

  logger.info("HID Driver uninstall");
  ESP_ERROR_CHECK(hid_host_uninstall());
  xQueueReset(app_event_queue);
  vQueueDelete(app_event_queue);


  // make a simple task that prints "Hello World!" every second
  espp::Task task({
      .name = "Hello World",
      .callback = [&](auto &m, auto &cv) -> bool {
        logger.debug("[{:.3f}] Hello from the task!", elapsed());
        std::unique_lock<std::mutex> lock(m);
        cv.wait_for(lock, 1s);
        // we don't want to stop the task, so return false
        return false;
      },
      .stack_size_bytes = 4096,
    });
  task.start();

  // also print in the main thread
  while (true) {
    logger.debug("[{:.3f}] Hello World!", elapsed());
    std::this_thread::sleep_for(1s);
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
static void hid_host_generic_report_callback(const uint8_t *const data, const int length)
{
  logger.debug("Generic report: ");
  for (int i = 0; i < length; i++) {
    printf("%02X", data[i]);
  }
  putchar('\r');
}

/**
 * @brief USB HID Host interface callback
 *
 * @param[in] hid_device_handle  HID Device handle
 * @param[in] event              HID Host interface event
 * @param[in] arg                Pointer to arguments, does not used
 */
static void hid_host_interface_callback(hid_host_device_handle_t hid_device_handle,
                                        const hid_host_interface_event_t event,
                                        void *arg)
{
  uint8_t data[64] = { 0 };
  size_t data_length = 0;
  hid_host_dev_params_t dev_params;
  ESP_ERROR_CHECK(hid_host_device_get_params(hid_device_handle, &dev_params));

  switch (event) {
  case HID_HOST_INTERFACE_EVENT_INPUT_REPORT:
    ESP_ERROR_CHECK(hid_host_device_get_raw_input_report_data(hid_device_handle,
                                                              data,
                                                              64,
                                                              &data_length));
    hid_host_generic_report_callback(data, data_length);
    break;
  case HID_HOST_INTERFACE_EVENT_DISCONNECTED:
    logger.info("HID Device, protocol '%s' DISCONNECTED",
             hid_proto_name_str[dev_params.proto]);
    ESP_ERROR_CHECK(hid_host_device_close(hid_device_handle));
    break;
  case HID_HOST_INTERFACE_EVENT_TRANSFER_ERROR:
    logger.info("HID Device, protocol '%s' TRANSFER_ERROR",
             hid_proto_name_str[dev_params.proto]);
    break;
  default:
    logger.error("HID Device, protocol '%s' Unhandled event",
             hid_proto_name_str[dev_params.proto]);
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
static void hid_host_device_event(hid_host_device_handle_t hid_device_handle,
                                  const hid_host_driver_event_t event,
                                  void *arg)
{
  hid_host_dev_params_t dev_params;
  ESP_ERROR_CHECK(hid_host_device_get_params(hid_device_handle, &dev_params));

  switch (event) {
  case HID_HOST_DRIVER_EVENT_CONNECTED: {
    logger.info("HID Device, protocol '%s' CONNECTED",
             hid_proto_name_str[dev_params.proto]);

    const hid_host_device_config_t dev_config = {
      .callback = hid_host_interface_callback,
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
static void hid_host_device_callback(hid_host_device_handle_t hid_device_handle,
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
static void usb_lib_task(void *arg)
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
