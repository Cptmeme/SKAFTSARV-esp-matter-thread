/*
   Custom DeviceInstanceInfoProvider so the lamp shows up with a sensible
   manufacturer and model instead of the example defaults.

   Only the names are ours. The vendor and product IDs deliberately stay on the
   test values the device attestation certificate was issued against: swapping
   in IKEA's real Matter vendor ID would fail attestation during commissioning
   and would claim a certification this lamp does not hold.
*/

#include "lamp_device_info.h"

#include <stdio.h>
#include <string.h>

#include <esp_log.h>
#include <esp_mac.h>
#include <esp_matter.h>
#include <esp_matter_providers.h>

#include <lib/support/CodeUtils.h>
#include <lib/support/Span.h>
#include <platform/CHIPDeviceConfig.h>
#include <platform/DeviceInstanceInfoProvider.h>

static const char *TAG = "lamp_info";

#define LAMP_VENDOR_NAME    "IKEA"
#define LAMP_PRODUCT_NAME   "SKAFTSARV"

namespace {

CHIP_ERROR copy_string(char *buf, size_t buf_size, const char *value)
{
    size_t len = strlen(value);
    VerifyOrReturnError(buf != nullptr, CHIP_ERROR_INVALID_ARGUMENT);
    VerifyOrReturnError(buf_size > len, CHIP_ERROR_BUFFER_TOO_SMALL);
    memcpy(buf, value, len + 1);
    return CHIP_NO_ERROR;
}

class LampDeviceInfoProvider : public chip::DeviceLayer::DeviceInstanceInfoProvider
{
public:
    CHIP_ERROR GetVendorName(char *buf, size_t buf_size) override
    {
        return copy_string(buf, buf_size, LAMP_VENDOR_NAME);
    }

    CHIP_ERROR GetProductName(char *buf, size_t buf_size) override
    {
        return copy_string(buf, buf_size, LAMP_PRODUCT_NAME);
    }

    CHIP_ERROR GetProductLabel(char *buf, size_t buf_size) override
    {
        return copy_string(buf, buf_size, LAMP_PRODUCT_NAME);
    }

    CHIP_ERROR GetVendorId(uint16_t &vendorId) override
    {
        vendorId = static_cast<uint16_t>(CHIP_DEVICE_CONFIG_DEVICE_VENDOR_ID);
        return CHIP_NO_ERROR;
    }

    CHIP_ERROR GetProductId(uint16_t &productId) override
    {
        productId = static_cast<uint16_t>(CHIP_DEVICE_CONFIG_DEVICE_PRODUCT_ID);
        return CHIP_NO_ERROR;
    }

    CHIP_ERROR GetHardwareVersion(uint16_t &hardwareVersion) override
    {
        hardwareVersion = static_cast<uint16_t>(CHIP_DEVICE_CONFIG_DEFAULT_DEVICE_HARDWARE_VERSION);
        return CHIP_NO_ERROR;
    }

    CHIP_ERROR GetHardwareVersionString(char *buf, size_t buf_size) override
    {
        return copy_string(buf, buf_size, CHIP_DEVICE_CONFIG_DEFAULT_DEVICE_HARDWARE_VERSION_STRING);
    }

    /* Optional Basic Information attributes this lamp simply does not publish. */
    CHIP_ERROR GetPartNumber(char *, size_t) override { return CHIP_ERROR_NOT_IMPLEMENTED; }
    CHIP_ERROR GetProductURL(char *, size_t) override { return CHIP_ERROR_NOT_IMPLEMENTED; }
    /* Kept as a fallback for the CHIP cluster code path. In practice esp-matter
       answers this one from its own data model, which is why the attribute also
       has to be created in app_main - see the note there. */
    CHIP_ERROR GetSerialNumber(char *buf, size_t buf_size) override
    {
        uint8_t mac[6];
        VerifyOrReturnError(esp_read_mac(mac, ESP_MAC_BASE) == ESP_OK, CHIP_ERROR_INTERNAL);

        char serial[13];
        snprintf(serial, sizeof(serial), "%02X%02X%02X%02X%02X%02X",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        return copy_string(buf, buf_size, serial);
    }
    CHIP_ERROR GetManufacturingDate(uint16_t &, uint8_t &, uint8_t &) override
    {
        return CHIP_ERROR_NOT_IMPLEMENTED;
    }
    CHIP_ERROR GetRotatingDeviceIdUniqueId(chip::MutableByteSpan &) override
    {
        return CHIP_ERROR_NOT_IMPLEMENTED;
    }
};

LampDeviceInfoProvider s_provider;

} // namespace

void lamp_device_info_register(void)
{
#if CONFIG_CUSTOM_DEVICE_INSTANCE_INFO_PROVIDER
    esp_matter::set_custom_device_instance_info_provider(&s_provider);
    ESP_LOGI(TAG, "Reporting as %s / %s", LAMP_VENDOR_NAME, LAMP_PRODUCT_NAME);
#else
    ESP_LOGW(TAG, "CONFIG_CUSTOM_DEVICE_INSTANCE_INFO_PROVIDER is off, so the vendor "
                  "and product names will stay on the build defaults");
#endif
}
