#ifndef BLEADVMANAGER_H
#define BLEADVMANAGER_H

#include "MicroBitConfig.h"
#include "pxt.h"
#include "IBLEAdvClient.h"
#include "MicroBitBLEManager.h" 

#define MAX_CLIENTS_COUNT                   10
#define MAX_ADV_DATA_CHANGES_IN_LOOP_COUNT  2     // max number of advertising data changes instanly commited
#define UNSET_HANDLE                        0xFF

class BLEAdvManager : public CodalComponent
{
    public:
      static BLEAdvManager *manager;

      /**
      * Constructor
       *
      */
      BLEAdvManager(BLEDevice &_ble);

      /**
       * getInstance
       *
       * Allows other objects to easily obtain a pointer to the single instance of this object. By rights the constructor should be made
       * private to properly implement the singleton pattern.
       */
      static BLEAdvManager *getInstance();


      uint8_t register_client(IBLEAdvClient* p_bleAdvClient);
      void unregister_client(IBLEAdvClient* p_bleAdvClient);
      void unregister_client(uint8_t clientHandle);

      void advertise(uint8_t clientHandle, uint8_t *p_payload);
      void advertise_stop(uint8_t clientHandle);

      /**
      * Periodic callback from Device system timer.
       *
      */
      virtual void periodicCallback() override;

    private:

      // Bluetooth stack we're running on.
      BLEDevice &ble;

      void internal_init();
      uint8_t find_client_handle(IBLEAdvClient* p_bleAdvClient);
      void loop_next();


      bool m_loop_payloads = false;
      IBLEAdvClient* m_registeredClients[MAX_CLIENTS_COUNT];
    
      uint8_t* m_payloads[MAX_CLIENTS_COUNT];

      uint8_t m_dropLoop[MAX_CLIENTS_COUNT];

      uint8_t m_currentClientHandle = 0;
      uint8_t m_payloads_count = 0;
};

#endif