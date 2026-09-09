/*
   Reports the lamp to Matter controllers as IKEA / SKAFTSÄRV.
*/

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/** Install the custom device instance info provider.
 *  Must be called before esp_matter::start(). */
void lamp_device_info_register(void);

#ifdef __cplusplus
}
#endif
