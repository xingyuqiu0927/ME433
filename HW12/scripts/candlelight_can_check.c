#include <errno.h>
#include <libusb.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define GS_USB_VID                 0x1d50
#define GS_USB_PID                 0x606f
#define CANDLELIGHT_VID            0x1209
#define CANDLELIGHT_PID            0x2323

#define GS_USB_BREQ_HOST_FORMAT    0
#define GS_USB_BREQ_BITTIMING      1
#define GS_USB_BREQ_MODE           2
#define GS_USB_BREQ_BT_CONST       4
#define GS_USB_BREQ_DEVICE_CONFIG  5
#define GS_USB_BREQ_SET_TERMINATION 12

#define GS_CAN_MODE_RESET          0
#define GS_CAN_MODE_START          1
#define GS_CAN_TERMINATION_ON      1

#define GS_HOST_FRAME_ECHO_ID_RX   0xffffffffU
#define HW12_ID                    0x433U
#define CANABLE_TEST_ID            0x321U
#define CAN_BAUD_RATE              250000U
#define CAN_EFF_FLAG               0x80000000U
#define CAN_RTR_FLAG               0x40000000U
#define CAN_ERR_FLAG               0x20000000U
#define CAN_ID_MASK                0x1fffffffU

struct gs_host_config {
  uint32_t byte_order;
} __attribute__((packed));

struct gs_device_config {
  uint8_t reserved1;
  uint8_t reserved2;
  uint8_t reserved3;
  uint8_t icount;
  uint32_t sw_version;
  uint32_t hw_version;
} __attribute__((packed));

struct gs_device_bittiming {
  uint32_t prop_seg;
  uint32_t phase_seg1;
  uint32_t phase_seg2;
  uint32_t sjw;
  uint32_t brp;
} __attribute__((packed));

struct gs_device_bt_const {
  uint32_t feature;
  uint32_t fclk_can;
  uint32_t tseg1_min;
  uint32_t tseg1_max;
  uint32_t tseg2_min;
  uint32_t tseg2_max;
  uint32_t sjw_max;
  uint32_t brp_min;
  uint32_t brp_max;
  uint32_t brp_inc;
} __attribute__((packed));

struct gs_device_mode {
  uint32_t mode;
  uint32_t flags;
} __attribute__((packed));

struct gs_device_termination_state {
  uint32_t state;
} __attribute__((packed));

struct gs_host_frame {
  uint32_t echo_id;
  uint32_t can_id;
  uint8_t can_dlc;
  uint8_t channel;
  uint8_t flags;
  uint8_t reserved;
  uint8_t data[8];
} __attribute__((packed));

static volatile sig_atomic_t stop_requested = 0;

static void handle_signal(int signal_number)
{
  (void)signal_number;
  stop_requested = 1;
}

static int control_out(libusb_device_handle *handle, uint8_t request,
                       uint16_t value, void *data, uint16_t length)
{
  return libusb_control_transfer(handle,
                                 LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,
                                 request, value, 0, data, length, 1000);
}

static int control_in(libusb_device_handle *handle, uint8_t request,
                      uint16_t value, void *data, uint16_t length)
{
  return libusb_control_transfer(handle,
                                 LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE,
                                 request, value, 0, data, length, 1000);
}

static int find_bulk_endpoints(libusb_device *device, uint8_t *ep_in, uint8_t *ep_out)
{
  struct libusb_config_descriptor *config = NULL;
  int result = libusb_get_active_config_descriptor(device, &config);

  if (result != 0) {
    result = libusb_get_config_descriptor(device, 0, &config);
  }
  if (result != 0) {
    return result;
  }

  for (int i = 0; i < config->bNumInterfaces; i++) {
    const struct libusb_interface *iface = &config->interface[i];
    for (int j = 0; j < iface->num_altsetting; j++) {
      const struct libusb_interface_descriptor *alt = &iface->altsetting[j];
      for (int k = 0; k < alt->bNumEndpoints; k++) {
        const struct libusb_endpoint_descriptor *ep = &alt->endpoint[k];
        if ((ep->bmAttributes & LIBUSB_TRANSFER_TYPE_MASK) != LIBUSB_TRANSFER_TYPE_BULK) {
          continue;
        }
        if ((ep->bEndpointAddress & LIBUSB_ENDPOINT_DIR_MASK) == LIBUSB_ENDPOINT_IN) {
          *ep_in = ep->bEndpointAddress;
        } else {
          *ep_out = ep->bEndpointAddress;
        }
      }
    }
  }

  libusb_free_config_descriptor(config);
  return (*ep_in != 0 && *ep_out != 0) ? 0 : LIBUSB_ERROR_NOT_FOUND;
}

static int choose_bittiming(const struct gs_device_bt_const *bt, uint32_t bitrate,
                            struct gs_device_bittiming *timing)
{
  uint32_t best_error = UINT32_MAX;
  uint32_t best_sample_error = UINT32_MAX;

  memset(timing, 0, sizeof(*timing));

  for (uint32_t brp = bt->brp_min; brp <= bt->brp_max; brp += bt->brp_inc) {
    for (uint32_t tseg1 = bt->tseg1_min; tseg1 <= bt->tseg1_max; tseg1++) {
      for (uint32_t tseg2 = bt->tseg2_min; tseg2 <= bt->tseg2_max; tseg2++) {
        uint32_t tq_total = 1U + tseg1 + tseg2;
        uint32_t actual = bt->fclk_can / (brp * tq_total);
        uint32_t error = (actual > bitrate) ? (actual - bitrate) : (bitrate - actual);
        uint32_t sample_per_mille = ((1U + tseg1) * 1000U) / tq_total;
        uint32_t sample_error = (sample_per_mille > 875U) ?
                                (sample_per_mille - 875U) : (875U - sample_per_mille);

        if (error < best_error || (error == best_error && sample_error < best_sample_error)) {
          best_error = error;
          best_sample_error = sample_error;
          timing->brp = brp;
          timing->phase_seg2 = tseg2;
          timing->sjw = (bt->sjw_max >= 1U) ? 1U : bt->sjw_max;
          timing->prop_seg = tseg1 / 2U;
          if (timing->prop_seg == 0U) {
            timing->prop_seg = 1U;
          }
          timing->phase_seg1 = tseg1 - timing->prop_seg;
          if (timing->phase_seg1 == 0U) {
            timing->phase_seg1 = 1U;
            timing->prop_seg = tseg1 - 1U;
          }
        }
      }
    }
  }

  return (best_error == 0U) ? 0 : -1;
}

static void print_frame(const char *prefix, const struct gs_host_frame *frame)
{
  const uint32_t id = frame->can_id & CAN_ID_MASK;

  if ((frame->can_id & CAN_ERR_FLAG) != 0U) {
    printf("%s CAN-ERROR raw_id=0x%08x class=0x%03x DLC=%u DATA=",
           prefix, frame->can_id, id, frame->can_dlc);
  } else {
    printf("%s ID=0x%03x%s%s DLC=%u DATA=", prefix, id,
           (frame->can_id & CAN_EFF_FLAG) ? " EXT" : "",
           (frame->can_id & CAN_RTR_FLAG) ? " RTR" : "",
           frame->can_dlc);
  }

  for (uint8_t i = 0; i < frame->can_dlc && i < 8U; i++) {
    printf("%02X", frame->data[i]);
    if (i + 1U < frame->can_dlc && i + 1U < 8U) {
      putchar(' ');
    }
  }
  putchar('\n');
  fflush(stdout);
}

static int send_test_frame(libusb_device_handle *handle, uint8_t ep_out, uint32_t echo_id)
{
  struct gs_host_frame frame = {0};
  int transferred = 0;

  frame.echo_id = echo_id;
  frame.can_id = CANABLE_TEST_ID;
  frame.can_dlc = 8;
  frame.channel = 0;
  memcpy(frame.data, "CANABLE!", 8);

  print_frame("CANABLE TX", &frame);
  return libusb_bulk_transfer(handle, ep_out, (unsigned char *)&frame, sizeof(frame),
                              &transferred, 1000);
}

int main(int argc, char **argv)
{
  const int seconds = (argc > 1) ? atoi(argv[1]) : 30;
  libusb_context *context = NULL;
  libusb_device_handle *handle = NULL;
  libusb_device *device = NULL;
  uint8_t ep_in = 0;
  uint8_t ep_out = 0;
  int result = 0;
  int saw_hw12_frame = 0;
  int sent_test_frame = 0;
  uint32_t next_echo_id = 0;

  signal(SIGINT, handle_signal);

  result = libusb_init(&context);
  if (result != 0) {
    fprintf(stderr, "libusb_init failed: %s\n", libusb_error_name(result));
    return 1;
  }

  handle = libusb_open_device_with_vid_pid(context, GS_USB_VID, GS_USB_PID);
  if (handle == NULL) {
    handle = libusb_open_device_with_vid_pid(context, CANDLELIGHT_VID, CANDLELIGHT_PID);
  }
  if (handle == NULL) {
    fprintf(stderr, "No candleLight/CANable USB device found.\n");
    libusb_exit(context);
    return 1;
  }

  device = libusb_get_device(handle);
  result = find_bulk_endpoints(device, &ep_in, &ep_out);
  if (result != 0) {
    fprintf(stderr, "Could not find CANable bulk endpoints: %s\n", libusb_error_name(result));
    goto out;
  }

  libusb_set_auto_detach_kernel_driver(handle, 1);
  result = libusb_claim_interface(handle, 0);
  if (result != 0) {
    fprintf(stderr, "Could not claim CANable USB interface: %s\n", libusb_error_name(result));
    fprintf(stderr, "Close any CAN tool or VS Code monitor that may be using the CANable.\n");
    goto out;
  }

  struct gs_host_config host_config = { .byte_order = 0x0000beefU };
  result = control_out(handle, GS_USB_BREQ_HOST_FORMAT, 0, &host_config, sizeof(host_config));
  if (result < 0) {
    fprintf(stderr, "HOST_FORMAT failed: %s\n", libusb_error_name(result));
    goto release;
  }

  struct gs_device_config device_config = {0};
  result = control_in(handle, GS_USB_BREQ_DEVICE_CONFIG, 0, &device_config, sizeof(device_config));
  if (result < 0) {
    fprintf(stderr, "DEVICE_CONFIG failed: %s\n", libusb_error_name(result));
    goto release;
  }

  struct gs_device_bt_const bt_const = {0};
  result = control_in(handle, GS_USB_BREQ_BT_CONST, 0, &bt_const, sizeof(bt_const));
  if (result < 0) {
    fprintf(stderr, "BT_CONST failed: %s\n", libusb_error_name(result));
    goto release;
  }

  printf("CANable opened: endpoints IN=0x%02x OUT=0x%02x, CAN clock=%u Hz\n",
         ep_in, ep_out, bt_const.fclk_can);

  struct gs_device_mode mode = { .mode = GS_CAN_MODE_RESET, .flags = 0 };
  control_out(handle, GS_USB_BREQ_MODE, 0, &mode, sizeof(mode));

  struct gs_device_termination_state term = { .state = GS_CAN_TERMINATION_ON };
  control_out(handle, GS_USB_BREQ_SET_TERMINATION, 0, &term, sizeof(term));

  struct gs_device_bittiming timing = {0};
  if (choose_bittiming(&bt_const, CAN_BAUD_RATE, &timing) != 0) {
    fprintf(stderr, "Could not find exact %u bit/s timing for this CANable.\n", CAN_BAUD_RATE);
    goto release;
  }

  printf("CANable timing: prop=%u phase1=%u phase2=%u sjw=%u brp=%u\n",
         timing.prop_seg, timing.phase_seg1, timing.phase_seg2, timing.sjw, timing.brp);
  result = control_out(handle, GS_USB_BREQ_BITTIMING, 0, &timing, sizeof(timing));
  if (result < 0) {
    fprintf(stderr, "BITTIMING failed: %s\n", libusb_error_name(result));
    goto release;
  }

  mode.mode = GS_CAN_MODE_START;
  mode.flags = 0;
  result = control_out(handle, GS_USB_BREQ_MODE, 0, &mode, sizeof(mode));
  if (result < 0) {
    fprintf(stderr, "MODE START failed: %s\n", libusb_error_name(result));
    goto release;
  }

  printf("CANable is active at %u bit/s. Press STM32 blue USER button now.\n", CAN_BAUD_RATE);
  printf("This program will also send ID 0x%03x once so STM32 should print RX if wiring works.\n", CANABLE_TEST_ID);
  fflush(stdout);

  const time_t deadline = time(NULL) + seconds;
  while (!stop_requested && time(NULL) < deadline) {
    struct gs_host_frame frame = {0};
    int transferred = 0;

    if (!sent_test_frame) {
      (void)send_test_frame(handle, ep_out, next_echo_id++);
      sent_test_frame = 1;
    }

    result = libusb_bulk_transfer(handle, ep_in, (unsigned char *)&frame, sizeof(frame),
                                  &transferred, 500);
    if (result == LIBUSB_ERROR_TIMEOUT) {
      continue;
    }
    if (result != 0) {
      fprintf(stderr, "bulk RX failed: %s\n", libusb_error_name(result));
      break;
    }
    if (transferred < 12) {
      continue;
    }

    if (frame.echo_id == GS_HOST_FRAME_ECHO_ID_RX) {
      print_frame("CANABLE RX", &frame);
      if ((frame.can_id & (CAN_ERR_FLAG | CAN_RTR_FLAG | CAN_ID_MASK)) == HW12_ID) {
        saw_hw12_frame = 1;
      }
    } else {
      print_frame("CANABLE TX-ECHO", &frame);
    }
  }

  puts("");
  if (saw_hw12_frame) {
    puts("PASS: CANable received STM32 ID 0x433. The STM32 CAN bus is communicating externally.");
    result = 0;
  } else {
    puts("FAIL: CANable did not receive STM32 ID 0x433 during this run.");
    puts("If STM32 still prints CAN TX timeout, the fault is physical bus wiring/termination/transceiver mode.");
    result = 1;
  }

  mode.mode = GS_CAN_MODE_RESET;
  mode.flags = 0;
  control_out(handle, GS_USB_BREQ_MODE, 0, &mode, sizeof(mode));

release:
  libusb_release_interface(handle, 0);
out:
  libusb_close(handle);
  libusb_exit(context);
  return result == 0 ? 0 : 1;
}
