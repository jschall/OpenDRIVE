#include <canard.h>

#define APP_VERSION_MAJOR                                           0
#define APP_VERSION_MINOR                                           1
#define APP_NODE_NAME                                               "org.jc.esc"

#define UAVCAN_NODE_STATUS_MESSAGE_SIZE                             7
#define UAVCAN_NODE_STATUS_DATA_TYPE_ID                             341
#define UAVCAN_NODE_STATUS_DATA_TYPE_SIGNATURE                      0x0f0868d0c1a7c6f1

#define UAVCAN_NODE_HEALTH_OK                                       0
#define UAVCAN_NODE_HEALTH_WARNING                                  1
#define UAVCAN_NODE_HEALTH_ERROR                                    2
#define UAVCAN_NODE_HEALTH_CRITICAL                                 3

#define UAVCAN_NODE_MODE_OPERATIONAL                                0
#define UAVCAN_NODE_MODE_INITIALIZATION                             1

static CanardInstance canard;
static uint8_t canard_memory_pool[1024];

static uint8_t node_health = UAVCAN_NODE_HEALTH_OK;
static uint8_t node_mode   = UAVCAN_NODE_MODE_INITIALIZATION;
static uint32_t started_at_sec;

static void makeNodeStatusMessage(uint8_t* buffer)
{
    memset(buffer, 0, UAVCAN_NODE_STATUS_MESSAGE_SIZE);

    if (started_at_sec == 0) {
        started_at_sec = millis()/1000U;
    }

    const uint32_t uptime_sec = millis()/1000U - started_at_sec;

    canardEncodeScalar(buffer,  0, 32, &uptime_sec);
    canardEncodeScalar(buffer, 32,  2, &node_health);
    canardEncodeScalar(buffer, 34,  3, &node_mode);
}

static bool shouldAcceptTransfer(const CanardInstance* ins, uint64_t* out_data_type_signature, uint16_t data_type_id, CanardTransferType transfer_type, uint8_t source_node_id) {
    return false;
}

static void process1HzTasks()
{
    canardCleanupStaleTransfers(&canard, micros());

    {
        uint8_t buffer[UAVCAN_NODE_STATUS_MESSAGE_SIZE];
        makeNodeStatusMessage(buffer);

        static uint8_t transfer_id;

        const int bc_res = canardBroadcast(&canard, UAVCAN_NODE_STATUS_DATA_TYPE_SIGNATURE,
                                           UAVCAN_NODE_STATUS_DATA_TYPE_ID, &transfer_id, CANARD_TRANSFER_PRIORITY_LOWEST,
                                           buffer, UAVCAN_NODE_STATUS_MESSAGE_SIZE);
        if (bc_res <= 0)
        {
            semihost_debug_printf("Could not broadcast node status; error %d\n", bc_res);
        }
    }

    node_mode = UAVCAN_NODE_MODE_OPERATIONAL;
}

static void onTransferReceived(CanardInstance* ins, CanardRxTransfer* transfer) {}

static void uavcan_init(void)
{
    canardInit(&canard, canard_memory_pool, sizeof(canard_memory_pool), onTransferReceived, shouldAcceptTransfer);
    canardSetLocalNodeID(&canard, 65);

}

static uint32_t last_1hz_ms;

static void uavcan_update(void)
{
    uint32_t tnow_ms = millis();
    if (tnow_ms-last_1hz_ms >= 1000) {
        process1HzTasks();
        last_1hz_ms = tnow_ms;
    }

    // transmit
    for (const CanardCANFrame* txf = NULL; (txf = canardPeekTxQueue(&canard)) != NULL;)
    {
        struct canbus_msg msg;
        msg.ide = true;
        msg.rtr = false;
        msg.id = txf->id;
        msg.dlc = txf->data_len;
        memcpy(msg.data, txf->data, 8);

        bool success = canbus_send_message(&msg);

        if (success) {
            canardPopTxQueue(&canard);
        } else {
            break;
        }
    }
}
