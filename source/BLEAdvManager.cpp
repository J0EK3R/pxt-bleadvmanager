#include "BLEAdvManager.h"


#include <stdbool.h>
#include <stdint.h>
#include "nordic_common.h"
// #include "bsp.h"
#include "nrf_soc.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "ble_advdata.h"
// #include "ble_gap.h"
#include "app_timer.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


#define APP_BLE_CONN_CFG_TAG            1                  /**< A tag identifying the SoftDevice BLE configuration. */
#define ADV_INTERVAL    MSEC_TO_UNITS(100, UNIT_0_625_MS)  /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */
//#define ADV_INTERVAL    MSEC_TO_UNITS(152.5, UNIT_0_625_MS)  /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */

#define ADV_DATA_MAX                      2

static bool m_bleStackInit = false;
static uint8_t m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;

static uint8_t m_adv_data_current = 0;
static uint8_t m_adv_data[31][ADV_DATA_MAX];

/*
 * Many of the interfaces we need to use only support callbacks to plain C functions, rather than C++ methods.
 * So, we maintain a pointer to the MicroBitBLEManager that's in use. Ths way, we can still access resources on the micro:bit
 * whilst keeping the code modular.
 */
BLEAdvManager *BLEAdvManager::manager = NULL; // Singleton reference to the BLE manager. many BLE API callbacks still do not support member functions. :-(


/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(uint8_t *p_adv_data) 
{
    // https://academy.nordicsemi.com/courses/bluetooth-low-energy-fundamentals/lessons/lesson-2-bluetooth-le-advertising/topic/advertising-process/
    
    ble_gap_adv_data_t  gap_adv_data;
    memset(&gap_adv_data, 0, sizeof(gap_adv_data));

    gap_adv_data.adv_data.p_data    = p_adv_data;     //m_adv_data[m_adv_data_current];
    gap_adv_data.adv_data.len       = 31;

    if (m_adv_handle == BLE_GAP_ADV_SET_HANDLE_NOT_SET) 
    {
        ble_gap_adv_params_t gap_adv_params;
        memset(&gap_adv_params, 0, sizeof(gap_adv_params));

        gap_adv_params.properties.type  = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
        gap_adv_params.p_peer_addr      = NULL;    // Undirected advertisement.
        gap_adv_params.filter_policy    = BLE_GAP_ADV_FP_ANY;

        gap_adv_params.interval         = ADV_INTERVAL;
        // limit interval to defined MIN/MAX
        if ( gap_adv_params.interval < BLE_GAP_ADV_INTERVAL_MIN) gap_adv_params.interval = BLE_GAP_ADV_INTERVAL_MIN;
        if ( gap_adv_params.interval > BLE_GAP_ADV_INTERVAL_MAX) gap_adv_params.interval = BLE_GAP_ADV_INTERVAL_MAX;

        gap_adv_params.duration         = 0;       // Never time out.
        gap_adv_params.primary_phy      = BLE_GAP_PHY_1MBPS; // BLE_GAP_PHY_CODED
        gap_adv_params.secondary_phy    = BLE_GAP_PHY_1MBPS; // BLE_GAP_PHY_CODED

        // https://docs.nordicsemi.com/bundle/s113_v7.3.0_api/page/group_b_l_e_g_a_p_f_u_n_c_t_i_o_n_s_4.html#ga9969047f4e7485c3f856c841978cc31a
        // Configure an advertising set. Set, clear or update advertising and scan response data.
        // Note
        //     The format of the advertising data will be checked by this call to ensure interoperability. 
        //     Limitations imposed by this API call to the data provided include having a flags data type in the 
        //     scan response data and duplicating the local name in the advertising data and scan response data.
        //     In order to update advertising data while advertising, new advertising buffers must be provided.
        //  [in,out]    p_adv_handle	Provide a pointer to a handle containing BLE_GAP_ADV_SET_HANDLE_NOT_SET to configure a new advertising set. 
        //                              On success, a new handle is then returned through the pointer. Provide a pointer to an existing advertising 
        //                              handle to configure an existing advertising set.
        //  [in]	    p_adv_data	    Advertising data. If set to NULL, no advertising data will be used. See ble_gap_adv_data_t.
        //  [in]	    p_adv_params	Advertising parameters. When this function is used to update advertising data while advertising, 
        //                              this parameter must be NULL. See ble_gap_adv_params_t.
        MICROBIT_BLE_ECHK(sd_ble_gap_adv_set_configure(&m_adv_handle, &gap_adv_data, &gap_adv_params));
    }
    else 
    {
        MICROBIT_BLE_ECHK(sd_ble_gap_adv_set_configure(&m_adv_handle, &gap_adv_data, NULL));
    }
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void) 
{
    // https://docs.nordicsemi.com/bundle/s113_v7.3.0_api/page/group_b_l_e_g_a_p_f_u_n_c_t_i_o_n_s_4.html#ga74c21287bd6cbcd5822bc73792f678d8
    // Start advertising (GAP Discoverable, Connectable modes, Broadcast Procedure). 
    // Note
    //     Only one advertiser may be active at any time.
    //     If privacy is enabled, the advertiser's private address will be refreshed when this function is called. See sd_ble_gap_privacy_set().
    // [in]	        adv_handle	    Advertising handle to advertise on, received from sd_ble_gap_adv_set_configure.
    // [in]	        conn_cfg_tag	Tag identifying a configuration set by sd_ble_cfg_set or BLE_CONN_CFG_TAG_DEFAULT to use the default connection configuration. 
    //                              For non-connectable advertising, this is ignored.    
    MICROBIT_BLE_ECHK(sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG));
}


/**
 * @brief Function for stop advertising.
 */
static void advertising_stop() 
{
    MICROBIT_DEBUG_DMESG("stopAdvertising");

    MICROBIT_BLE_ECHK(sd_ble_gap_adv_stop(m_adv_handle));
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    if (!m_bleStackInit) // prevent multiple init
    {
        MICROBIT_BLE_ECHK(nrf_sdh_enable_request());

        // Configure the BLE stack using the default settings.
        // Fetch the start address of the application RAM.
        uint32_t ram_start = 0;
        MICROBIT_BLE_ECHK(nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start));

        // Enable BLE stack.
        MICROBIT_BLE_ECHK(nrf_sdh_ble_enable(&ram_start));
        m_bleStackInit = true;
    }
}


static void advertising_set_data(uint8_t *p_payload) 
{
    uint8_t *p_adv_data = m_adv_data[m_adv_data_current];

    memcpy(p_adv_data, p_payload, 31);

    // choose next buffer
    m_adv_data_current++;
    if (m_adv_data_current >= ADV_DATA_MAX) {

        m_adv_data_current = 0;
    }

    advertising_stop();

    advertising_init(p_adv_data);

    advertising_start();
}


static bool check_client_handle(uint8_t clienthandle) 
{
    return clienthandle < MAX_CLIENTS_COUNT;
}


/**
 * Constructor.
 * Create a representation of the MK6HubService
 * @param _ble The instance of a BLE device that we're running on.
 */
BLEAdvManager::BLEAdvManager(BLEDevice &_ble) : ble(_ble) 
{
    this->internal_init();
    ble_stack_init();
}


/**
 * When called, the micro:bit will begin advertising for a predefined period,
 * MICROBIT_BLE_ADVERTISING_TIMEOUT seconds to bonded devices.
 */
BLEAdvManager *BLEAdvManager::getInstance()
{
    if (manager == 0)
    {
        manager = new BLEAdvManager(*uBit.ble);
    }
    return manager;
}


void BLEAdvManager::internal_init() 
{
    if (this->status & DEVICE_COMPONENT_RUNNING)
    {
      return;
    }

    MICROBIT_DEBUG_DMESG( "BLEAdvManager::internal_init");

    // initialize members
    for (uint8_t index = 0; index < MAX_CLIENTS_COUNT; index++) 
    {
        this->m_registeredClients[index] = NULL;
        this->m_payloads[index] = NULL;
        this->m_dropLoop[index] = 0x00;
    }

    // MICROBIT_BLE_ECHK( app_timer_init());

    // fiber_add_idle_component(this);

    // enable periodicCallback
    this->status |= DEVICE_COMPONENT_STATUS_SYSTEM_TICK;

    this->status |= DEVICE_COMPONENT_RUNNING;
}


/**
  * Periodic callback from Device system timer.
  *
  */
void BLEAdvManager::periodicCallback()
{
    if (this->m_loop_payloads)
    {
        this->loop_next();
    }
}


uint8_t BLEAdvManager::register_client(IBLEAdvClient* p_bleAdvClient) 
{
    // find clientHandle for bleAdvClient
    uint8_t clientHandle = this->find_client_handle(p_bleAdvClient);

    if (clientHandle != UNSET_HANDLE)   // bleAdvClient was registered
    {

        return clientHandle;
    }

    // find first unused entry and return index as clientHandle
    for (clientHandle = 0; clientHandle < MAX_CLIENTS_COUNT; clientHandle++) 
    {
        if(m_registeredClients[clientHandle] == NULL)
        {
            m_registeredClients[clientHandle] = p_bleAdvClient;

            return clientHandle;
        }
    }

    // max number of clients reached
    return UNSET_HANDLE;
}


void BLEAdvManager::unregister_client(IBLEAdvClient* p_bleAdvClient) 
{
    // find clientHandle
    uint8_t clientHandle = this->find_client_handle(p_bleAdvClient);

    if (clientHandle != UNSET_HANDLE) 
    {
        this->unregister_client(clientHandle);
    }
}


void BLEAdvManager::unregister_client(uint8_t clientHandle) 
{
    if (!check_client_handle(clientHandle)) 
    {
        return;
    }

    if (this->m_registeredClients[clientHandle] != NULL) 
    {
        this->m_registeredClients[clientHandle] = NULL; // reset
        this->m_payloads[clientHandle] = NULL;          // reset
    }
}


uint8_t BLEAdvManager::find_client_handle(IBLEAdvClient* p_bleAdvClient) 
{
    // find clientHandle
    for (uint8_t clientHandle = 0; clientHandle < MAX_CLIENTS_COUNT; clientHandle++) 
    {
        if (this->m_registeredClients[clientHandle] == p_bleAdvClient) 
        {
            return clientHandle;
        }
    }

    return UNSET_HANDLE;
}


void BLEAdvManager::loop_next() 
{
    for (uint8_t index = 0; index < MAX_CLIENTS_COUNT; index++) 
    {
        // select next registered client
        this->m_currentClientHandle++;                          // increment currentClient
        if (this->m_currentClientHandle >= MAX_CLIENTS_COUNT)   // if currentClient exceeds maximum then reset to 0
        {
            this->m_currentClientHandle = 0;
        }

        // currentClient is valid
        if (this->m_registeredClients[this->m_currentClientHandle] != NULL) // valid client
        {
            // droploop-value > 0 means that current data was advertised manually since last loop
            // -> no advertising of current data in this loop
            if (this->m_dropLoop[this->m_currentClientHandle] > 0) 
            {
                this->m_dropLoop[this->m_currentClientHandle] = 0; // reset
            }
            else 
            {
                uint8_t *p_payload = this->m_payloads[this->m_currentClientHandle];

                if (p_payload != NULL) 
                {
                    // advertising
                    advertising_set_data(p_payload);
                    return;
                }
            }
        }
    }
}


void BLEAdvManager::advertise(uint8_t clientHandle, uint8_t *p_payload) 
{
    if (p_payload == NULL)  // NULL was set -> it's like calling advertise_stop
    {
        this->advertise_stop(clientHandle);
        return;
    }

    if (!check_client_handle(clientHandle)) 
    {
        return;
    }

    bool isNew = (this->m_payloads[clientHandle] == NULL);

    // set pointer to payload
    this->m_payloads[clientHandle] = p_payload;

    if (isNew) 
    {
        this->m_dropLoop[clientHandle] = 1;

        // start advertising new entries immediatly
        advertising_set_data(p_payload);

        this->m_payloads_count++;

        // on second entry start looping: 
        // looping the entries to be advertised
        if (this->m_payloads_count >= 2 && //
            !this->m_loop_payloads) 
        {
            this->m_loop_payloads = true;
        }
    }
    else 
    {
        if (this->m_loop_payloads) // loop is running because more than 1 payload
        {
            this->m_dropLoop[clientHandle]++;

            // if doploop value is lower than MAX than immediatly advertise
            // -> fast reaction on changes
            if (this->m_dropLoop[clientHandle] <= MAX_ADV_DATA_CHANGES_IN_LOOP_COUNT) 
            {
                advertising_set_data(p_payload);
            }
        }
        else
        {
            advertising_set_data(p_payload);
        }
    }
}


void BLEAdvManager::advertise_stop(uint8_t clientHandle) 
{
    if (!check_client_handle(clientHandle)) 
    {
        return;
    }

    if (this->m_payloads[clientHandle] != NULL) 
    {
        this->m_payloads[clientHandle] = NULL;

        this->m_payloads_count--;

        // if there is just one entry left than the looping the entries to be advertised is unneccessary
        if (this->m_payloads_count <= 1 && 
            this->m_loop_payloads) 
        {
            this->m_loop_payloads = false;
        }

        if (this->m_payloads_count == 0) 
        {
            advertising_stop();
        }
    }
}
