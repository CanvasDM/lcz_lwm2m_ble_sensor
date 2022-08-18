# LwM2M BLE Sensor Advertisement Handler

This modules scans for Laird Connectivity sensor advertisements.  It filters them based on the available LwM2M objects available. When new measurements are received the LwM2M objects are created (if needed) and the values are updated.

This module relies on the index/table provided by the LwM2M gateway object.  When enabled, the gateway object handles the allow list.
