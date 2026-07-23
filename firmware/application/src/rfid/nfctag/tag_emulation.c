#include "tag_emulation.h"

#include "app_timer.h"
#include "crc_utils.h"
#include "fds_ids.h"
#include "fds_util.h"
#include "lf_tag_em.h"
#include "nfc_14a.h"
#include "rfid_main.h"
#include "nfc_mf0_ntag.h"
#include "nfc_mf1.h"
#include "nfc_14a_4.h"
#include "rgb_marquee.h"
#include "tag_persistence.h"

#define NRF_LOG_MODULE_NAME tag_emu
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
NRF_LOG_MODULE_REGISTER();

/*
 * A card slot can simulate up to two cards at the same time, one ID 125kHz EM410X, and one IC 13.56MHz 14A.(May be able to support more in the future)
 * When starting, you should start the startup listener on demand (there is no emulated card when there is no data, but you need to monitor the state on demand)
 * If the retrieved card slot configuration has a specified type of card, then loading the specified type of data should be carried out, and the necessary parameters of initialization should be performed.
 * When the on-field entry is detected, in addition to the relevant LED, you also need to start the emulation card according to whether the current data is loaded.
 * In the emulation card, all operations should be carried out based on the data loaded in RAM. After the analog card is over, the modified data should be preserved to Flash
 *
 * ......
 */

// Is the logo in the analog card?
bool g_is_tag_emulating = false;
// Sense of the field currently waking the device; see tag_emulation.h.
tag_sense_type_t g_active_sense = TAG_SENSE_HF;

// Smart-poll deferred persistence flag. Set inside the field-detect ISR,
// cleared by tag_emulation_smart_poll_process() running in the main loop so
// that no FDS/Flash write ever happens from interrupt context.
static volatile bool g_smart_poll_dirty = false;

// ---------------------------------------------------------------------------
// Auto-poll cycle session.
// When a reader field *passively* wakes the device with TIMER_ROTATE enabled we
// enter a rotation cycle that keeps advancing through the enabled slots
// (1,2,3,4,...) at the configured interval, protected by a time guard. The guard
// is refreshed on every field (re-)assert, so a field blip / re-detect does NOT
// reset us back to the first slot — the reader sees the cards in turn. The cycle
// ends only when the guard expires (no field for AUTO_POLL_CYCLE_GUARD_MS).
// ---------------------------------------------------------------------------
static bool     g_ap_cycling = false;
static uint32_t g_ap_cycle_end_ticks = 0;
#ifndef AUTO_POLL_CYCLE_GUARD_MS
#define AUTO_POLL_CYCLE_GUARD_MS 5000   // keep cycling this long after the last field
#endif

// Manual (key-wakeup) slot selection. When set, a following reader field
// emulates the button-chosen slot and never enters the auto-poll cycle.
// Cleared on FIELD_LOST so the next passive wake can auto-cycle again.
static bool g_manual_slot_select = false;

// When the auto-poll timer decides to rotate to a new LF slot, we must NOT
// rebuild the PWM sequence (m_pwm_seq) from interrupt context while the LF
// PWM is actively playing -- mutating the sequence under a running peripheral
// corrupts the replay state machine (EVT_STOPPED may stop firing) and hangs
// the device. Instead the timer only flips this flag + the active slot; the
// actual data/sequence reload is deferred to lf_tag_em.c's pwm_handler
// EVT_STOPPED, which is exactly when the PWM is fully stopped and safe to
// re-point at a new sequence.
static volatile bool g_lf_rotate_pending = false;

static tag_specific_type_t tag_specific_type_old2new_lf_values[][2] = {TAG_SPECIFIC_TYPE_OLD2NEW_LF_VALUES};
static tag_specific_type_t tag_specific_type_old2new_hf_values[][2] = {TAG_SPECIFIC_TYPE_OLD2NEW_HF_VALUES};
static tag_specific_type_t tag_specific_type_lf_values[] = {TAG_SPECIFIC_TYPE_LF_VALUES};
static tag_specific_type_t tag_specific_type_hf_values[] = {TAG_SPECIFIC_TYPE_HF_VALUES};

bool is_tag_specific_type_valid(tag_specific_type_t tag_type) {
    bool valid = false;
    for (uint16_t i = 0; i < ARRAYLEN(tag_specific_type_lf_values); i++) {
        valid |= (tag_type == tag_specific_type_lf_values[i]);
    }
    for (uint16_t i = 0; i < ARRAYLEN(tag_specific_type_hf_values); i++) {
        valid |= (tag_type == tag_specific_type_hf_values[i]);
    }
    return valid;
}
// **********************  Specific parameters start **********************

/**
 * Tag data stored in flash. Total length must be aligned by 4 bytes (whole words).
 */
static uint8_t m_tag_data_buffer_lf[20];  // LF card data buffer
static uint16_t m_tag_data_lf_crc;
static tag_data_buffer_t m_tag_data_lf = {sizeof(m_tag_data_buffer_lf), m_tag_data_buffer_lf, &m_tag_data_lf_crc};

static uint8_t m_tag_data_buffer_hf[4500];  // HF card data buffer
static uint16_t m_tag_data_hf_crc;
static tag_data_buffer_t m_tag_data_hf = {sizeof(m_tag_data_buffer_hf), m_tag_data_buffer_hf, &m_tag_data_hf_crc};

/**
 * Eight card slots, each card slot has its own unique configuration
 */
static tag_slot_config_t slotConfig ALIGN_U32 = {
    // Configure activated card slot, default activation of the 0th card slot (the first card)
    .version = TAG_SLOT_CONFIG_CURRENT_VERSION,
    .active_slot = 0,
    // Configuration card slots
    // See tag_emulation_factory_init for actual tag content
    .slots = {
        { .enabled_hf = true,  .enabled_lf = true,  .tag_hf = TAG_TYPE_MIFARE_1024, .tag_lf = TAG_TYPE_EM410X,    },  // 1
        { .enabled_hf = true,  .enabled_lf = false, .tag_hf = TAG_TYPE_MF0ICU1,     .tag_lf = TAG_TYPE_UNDEFINED, },  // 2
        { .enabled_hf = false, .enabled_lf = true,  .tag_hf = TAG_TYPE_UNDEFINED,   .tag_lf = TAG_TYPE_EM410X,    },  // 3
        { .enabled_hf = false, .enabled_lf = false, .tag_hf = TAG_TYPE_UNDEFINED,   .tag_lf = TAG_TYPE_UNDEFINED, },  // 4
        { .enabled_hf = false, .enabled_lf = false, .tag_hf = TAG_TYPE_UNDEFINED,   .tag_lf = TAG_TYPE_UNDEFINED, },  // 5
        { .enabled_hf = false, .enabled_lf = false, .tag_hf = TAG_TYPE_UNDEFINED,   .tag_lf = TAG_TYPE_UNDEFINED, },  // 6
        { .enabled_hf = false, .enabled_lf = false, .tag_hf = TAG_TYPE_UNDEFINED,   .tag_lf = TAG_TYPE_UNDEFINED, },  // 7
        { .enabled_hf = false, .enabled_lf = false, .tag_hf = TAG_TYPE_UNDEFINED,   .tag_lf = TAG_TYPE_UNDEFINED, },  // 8
    },
    // v9 additions: auto-poll. v10 default = full auto-poll (wake-on-field +
    // idle multi-slot rotation) so the feature works out of the box.
    .auto_poll_enable = AUTO_POLL_ALL,
    .last_auth_slot = 0,
    .auto_poll_interval_ms = AUTO_POLL_INTERVAL_DEFAULT_MS,
    .last_auth_hf = 0,
    .last_auth_lf = 0,
};
// The card slot configuration unique CRC, once the slot configuration changes, can be checked by CRC
static uint16_t m_slot_config_crc;

// ********************** Specific parameter ends **********************

/**
 * The data of the tag is loaded to the RAM and the mapping table of the operation of the regulating notification,
 * The mapping structure is:
 * Field -type detailed tag type Loading data The notification of the notification of the notification of the notification of the call recovery data before saving the data of the realization data of the function card data
 */
static tag_base_handler_map_t tag_base_map[] = {
    // LF tag emulation
    {TAG_SENSE_LF, TAG_TYPE_EM410X,      lf_tag_data_loadcb,           lf_tag_em410x_data_savecb,    lf_tag_em410x_data_factory,    &m_tag_data_lf},
    {TAG_SENSE_LF, TAG_TYPE_EM410X_ELECTRA, lf_tag_data_loadcb,        lf_tag_em410x_data_savecb,    lf_tag_em410x_data_factory,    &m_tag_data_lf},
    {TAG_SENSE_LF, TAG_TYPE_HID_PROX,    lf_tag_data_loadcb,           lf_tag_hidprox_data_savecb,   lf_tag_hidprox_data_factory,   &m_tag_data_lf},
    {TAG_SENSE_LF, TAG_TYPE_IOPROX,      lf_tag_data_loadcb,           lf_tag_ioprox_data_savecb,    lf_tag_ioprox_data_factory,    &m_tag_data_lf},
    {TAG_SENSE_LF, TAG_TYPE_VIKING,      lf_tag_data_loadcb,           lf_tag_viking_data_savecb,    lf_tag_viking_data_factory,    &m_tag_data_lf},
    {TAG_SENSE_LF, TAG_TYPE_PAC,         lf_tag_data_loadcb,           lf_tag_pac_data_savecb,       lf_tag_pac_data_factory,       &m_tag_data_lf},
    {TAG_SENSE_LF, TAG_TYPE_JABLOTRON,   lf_tag_data_loadcb,           lf_tag_jablotron_data_savecb, lf_tag_jablotron_data_factory, &m_tag_data_lf},
    {TAG_SENSE_LF, TAG_TYPE_IDTECK,      lf_tag_data_loadcb,           lf_tag_idteck_data_savecb,    lf_tag_idteck_data_factory,    &m_tag_data_lf},
    // MF1 tag emulation
    {TAG_SENSE_HF, TAG_TYPE_MIFARE_Mini, nfc_tag_mf1_data_loadcb,      nfc_tag_mf1_data_savecb,      nfc_tag_mf1_data_factory,      &m_tag_data_hf},
    {TAG_SENSE_HF, TAG_TYPE_MIFARE_1024, nfc_tag_mf1_data_loadcb,      nfc_tag_mf1_data_savecb,      nfc_tag_mf1_data_factory,      &m_tag_data_hf},
    {TAG_SENSE_HF, TAG_TYPE_MIFARE_2048, nfc_tag_mf1_data_loadcb,      nfc_tag_mf1_data_savecb,      nfc_tag_mf1_data_factory,      &m_tag_data_hf},
    {TAG_SENSE_HF, TAG_TYPE_MIFARE_4096, nfc_tag_mf1_data_loadcb,      nfc_tag_mf1_data_savecb,      nfc_tag_mf1_data_factory,      &m_tag_data_hf},
    // NTAG tag emulation
    {TAG_SENSE_HF, TAG_TYPE_NTAG_210,    nfc_tag_mf0_ntag_data_loadcb, nfc_tag_mf0_ntag_data_savecb, nfc_tag_mf0_ntag_data_factory, &m_tag_data_hf},
    {TAG_SENSE_HF, TAG_TYPE_NTAG_212,    nfc_tag_mf0_ntag_data_loadcb, nfc_tag_mf0_ntag_data_savecb, nfc_tag_mf0_ntag_data_factory, &m_tag_data_hf},
    {TAG_SENSE_HF, TAG_TYPE_NTAG_213,    nfc_tag_mf0_ntag_data_loadcb, nfc_tag_mf0_ntag_data_savecb, nfc_tag_mf0_ntag_data_factory, &m_tag_data_hf},
    {TAG_SENSE_HF, TAG_TYPE_NTAG_215,    nfc_tag_mf0_ntag_data_loadcb, nfc_tag_mf0_ntag_data_savecb, nfc_tag_mf0_ntag_data_factory, &m_tag_data_hf},
    {TAG_SENSE_HF, TAG_TYPE_NTAG_216,    nfc_tag_mf0_ntag_data_loadcb, nfc_tag_mf0_ntag_data_savecb, nfc_tag_mf0_ntag_data_factory, &m_tag_data_hf},
    // MF0 tag emulation
    {TAG_SENSE_HF, TAG_TYPE_MF0ICU1,     nfc_tag_mf0_ntag_data_loadcb, nfc_tag_mf0_ntag_data_savecb, nfc_tag_mf0_ntag_data_factory, &m_tag_data_hf},
    {TAG_SENSE_HF, TAG_TYPE_MF0ICU2,     nfc_tag_mf0_ntag_data_loadcb, nfc_tag_mf0_ntag_data_savecb, nfc_tag_mf0_ntag_data_factory, &m_tag_data_hf},
    {TAG_SENSE_HF, TAG_TYPE_MF0UL11,     nfc_tag_mf0_ntag_data_loadcb, nfc_tag_mf0_ntag_data_savecb, nfc_tag_mf0_ntag_data_factory, &m_tag_data_hf},
    {TAG_SENSE_HF, TAG_TYPE_MF0UL21,     nfc_tag_mf0_ntag_data_loadcb, nfc_tag_mf0_ntag_data_savecb, nfc_tag_mf0_ntag_data_factory, &m_tag_data_hf},
    // ISO14443-4 T=CL emulation
    {TAG_SENSE_HF, TAG_TYPE_HF14A_4,     nfc_tag_14a_4_data_loadcb,    nfc_tag_14a_4_data_savecb,    nfc_tag_14a_4_data_factory,    &m_tag_data_hf},
};

static void tag_emulation_load_config(void);
static void tag_emulation_save_config(void);

/**
 * get the data loader for the specific type of tag
 */
static tag_datas_loadcb_t get_data_loadcb_from_tag_type(tag_specific_type_t type) {
    for (int i = 0; i < ARRAY_SIZE(tag_base_map); i++) {
        if (tag_base_map[i].tag_type == type) {
            return tag_base_map[i].data_on_load;
        }
    }
    return NULL;
}

/**
 * accordingToTheSpecifiedDetailedLabelType,ObtainTheOperationFunctionBeforeTheDataPreservationOfTheData
 */
static tag_datas_savecb_t get_data_savecb_from_tag_type(tag_specific_type_t type) {
    for (int i = 0; i < ARRAY_SIZE(tag_base_map); i++) {
        if (tag_base_map[i].tag_type == type) {
            return tag_base_map[i].data_on_save;
        }
    }
    return NULL;
}

/**
 * get factory data for specific tag type
 */
static tag_datas_factory_t get_data_factory_from_tag_type(tag_specific_type_t type) {
    for (int i = 0; i < ARRAY_SIZE(tag_base_map); i++) {
        if (tag_base_map[i].tag_type == type) {
            return tag_base_map[i].data_factory;
        }
    }
    return NULL;
}

/**
 * accordingToTheSpecifiedDetailedLabelType,ObtainItsBasicFieldInductionType
 */
tag_sense_type_t get_sense_type_from_tag_type(tag_specific_type_t type) {
    for (int i = 0; i < ARRAY_SIZE(tag_base_map); i++) {
        if (tag_base_map[i].tag_type == type) {
            return tag_base_map[i].sense_type;
        }
    }
    return TAG_SENSE_NO;
}

/**
 * Get buffer data according to tag type.
 */
tag_data_buffer_t *get_buffer_by_tag_type(tag_specific_type_t type) {
    if (type == TAG_TYPE_UNDEFINED) {
        return NULL;
    }
    for (int i = 0; i < ARRAY_SIZE(tag_base_map); i++) {
        if (tag_base_map[i].tag_type == type) {
            return tag_base_map[i].data_buffer;
        }
    }
    NRF_LOG_ERROR("no buffer valid for tag type %d.", type);
    return NULL;
}

/**
 * Load data from memory to the emulated card data.
 */
bool tag_emulation_load_by_buffer(tag_specific_type_t tag_type, bool update_crc) {
    // data has been read to buffer,
    // here we load buffer to the emulator to config pwm seq for the activated card slot.
    tag_datas_loadcb_t loader = get_data_loadcb_from_tag_type(tag_type);
    if (loader == NULL) {
        NRF_LOG_INFO("no data loader exists for the tag type.");
        return false;
    }

    tag_data_buffer_t *buffer = get_buffer_by_tag_type(tag_type);
    if (buffer == NULL) {
        return false;
    }

    int length = loader(tag_type, buffer);
    if (length > 0 && update_crc) {
        // afterReadingIsCompleted,WeCanSaveACrcOfTheCurrentDataWhenItIsStoredLater,ItCanBeUsedAsAReferenceForChangesComparison
        calc_14a_crc_lut(buffer->buffer, length, (uint8_t *)buffer->crc);
        return true;
    }
    return false;
}

/**
 * Load card data based on tag type.
 */
static void load_data_by_tag_type(uint8_t slot, tag_specific_type_t tag_type) {
    // maybeTheCardSlotIsNotEnabledToUseTheemulationOfThisTypeOfLabel,AndSkipTheDataDirectlyToLoadThisData
    if (tag_type == TAG_TYPE_UNDEFINED) {
        return;
    }

    tag_data_buffer_t *buffer = get_buffer_by_tag_type(tag_type);
    if (buffer == NULL) {
        return;
    }

    tag_sense_type_t sense_type = get_sense_type_from_tag_type(tag_type);

    // get fds record for the card slot
    fds_slot_record_map_t map_info;
    get_fds_map_by_slot_sense_type_for_dump(slot, sense_type, &map_info);

    // load data to the buffer according to the card slot currently activated.
    // If the length of data does not match the length of the buffer,
    // it may be caused by the firmware update at this time, the data must be deleted and rebuilt.
    uint16_t length = buffer->length;
    bool ret = fds_read_sync(map_info.id, map_info.key, &length, buffer->buffer);
    if (false == ret) {
        NRF_LOG_INFO("tag slot data no exists.");
        return;
    }
    ret = tag_emulation_load_by_buffer(tag_type, true);
    if (ret) {
        NRF_LOG_INFO("load tag data in slot %d, type %d done.", slot, tag_type);
    }
}

/**
 * Save data according to the type
 */
static void save_data_by_tag_type(uint8_t slot, tag_specific_type_t tag_type) {
    // Maybe the card slot is not enabled to use the emulation of this type of label, and skip it directly to save this data
    if (tag_type == TAG_TYPE_UNDEFINED) {
        return;
    }

    tag_data_buffer_t *buffer = get_buffer_by_tag_type(tag_type);
    if (buffer == NULL) {
        return;
    }

    // The length of the data to be saved by the user should not exceed the size of the global buffer
    int data_byte_length = 0;
    tag_datas_savecb_t fn_savecb = get_data_savecb_from_tag_type(tag_type);
    if (fn_savecb == NULL) {  // Make sure that there is a real estate process
        NRF_LOG_INFO("Tag data saver no impl.");
        return;
    }

    data_byte_length = fn_savecb(tag_type, buffer);
    // Make sure to save data, we can judge whether the data has changed through CRC
    if (data_byte_length <= 0) {
        NRF_LOG_INFO("Tag type %d data no save.", tag_type);
        return;
    }
    // Make sure that the data to be stored is not greater than the size of the current buffer area
    if (data_byte_length > buffer->length) {
        NRF_LOG_ERROR("Tag data save length overflow.", tag_type);
        return;
    }
    uint16_t crc;
    calc_14a_crc_lut(buffer->buffer, data_byte_length, (uint8_t *)&crc);
    // Determine whether the data has changed
    if (crc == *buffer->crc) {
        NRF_LOG_INFO("Tag slot data no change, length = %d", data_byte_length);
        return;
    }
    tag_sense_type_t sense_type = get_sense_type_from_tag_type(tag_type);
    // Get the special card slot FDS record information
    fds_slot_record_map_t map_info;
    get_fds_map_by_slot_sense_type_for_dump(slot, sense_type, &map_info);
    // Call the blocked FDS to write the function, and write the data of the specified field type of the card slot into the Flash
    bool ret = fds_write_sync(map_info.id, map_info.key, data_byte_length, buffer->buffer);
    if (ret) {
        NRF_LOG_INFO("Save tag slot data success.");
    } else {
        NRF_LOG_ERROR("Save tag slot data error.");
    }
    // After the preservation is completed, the CRC of the BUFFER in the corresponding memory
    *buffer->crc = crc;
}

/**
 * Delete data according to the type
 */
static void delete_data_by_tag_type(uint8_t slot, tag_sense_type_t sense_type) {
    if (sense_type == TAG_SENSE_NO) {
        return;
    }
    fds_slot_record_map_t map_info;
    get_fds_map_by_slot_sense_type_for_dump(slot, sense_type, &map_info);
    int count = fds_delete_sync(map_info.id, map_info.key);
    NRF_LOG_INFO("Slot %d delete sense type %d data, record count: %d", slot, sense_type, count);
}

/**
 * Load the emulation card data data. Note that loading is just data operation,
 * Start the analog card, please call tag_emulation_sense_run function, otherwise you will not sensor the field event
 */
void tag_emulation_load_data(void) {
    uint8_t slot = tag_emulation_get_slot();
    load_data_by_tag_type(slot, slotConfig.slots[slot].tag_hf);
    load_data_by_tag_type(slot, slotConfig.slots[slot].tag_lf);
}

/**
 *Save the emulated card configuration data. At the right time, this function should be called for data preservation of data
 */
void tag_emulation_save_data(void) {
    uint8_t slot = tag_emulation_get_slot();
    save_data_by_tag_type(slot, slotConfig.slots[slot].tag_hf);
    save_data_by_tag_type(slot, slotConfig.slots[slot].tag_lf);
}

/**
 * @brief Get the type of labeling of the emulation card from the corresponding card slot.
 *
 * @param slot Card slot
 * @param tag_type Label
 */
void tag_emulation_get_specific_types_by_slot(uint8_t slot, tag_slot_specific_type_t *tag_types) {
    tag_types->tag_hf = slotConfig.slots[slot].tag_hf;
    tag_types->tag_lf = slotConfig.slots[slot].tag_lf;
}

/**
 * Delete the data specified by a card slot, if it is the current activated card slot data, we also need to dynamically close the emulation of this card
 */
void tag_emulation_delete_data(uint8_t slot, tag_sense_type_t sense_type) {
    // delete data
    delete_data_by_tag_type(slot, sense_type);
    // Close the corresponding card type of the corresponding card slot
    switch (sense_type) {
        case TAG_SENSE_HF: {
            slotConfig.slots[slot].tag_hf = TAG_TYPE_UNDEFINED;
            slotConfig.slots[slot].enabled_hf = false;
        }
        break;
        case TAG_SENSE_LF: {
            slotConfig.slots[slot].tag_lf = TAG_TYPE_UNDEFINED;
            slotConfig.slots[slot].enabled_lf = false;
        }
        break;
        default:
            break;
    }
    // If the deleted card slot data is currently activated (being emulated), we also need to make dynamic shutdown
    if (slotConfig.active_slot == slot) {
        tag_emulation_sense_switch(sense_type, false);
    }
}

/**
 * Set the data of a card slot to the preset data from the factory
 */
bool tag_emulation_factory_data(uint8_t slot, tag_specific_type_t tag_type) {
    tag_datas_factory_t factory = get_data_factory_from_tag_type(tag_type);
    // The process of implementing the data formatting data!
    if (factory != NULL && factory(slot, tag_type)) {
        // If the current data card slot number currently set is the current activated card slot, then we need to update to the memory
        if (tag_emulation_get_slot() == slot) {
            load_data_by_tag_type(slot, tag_type);
        }
        return true;
    }
    return false;
}

/**
 * Switch field induction monitoring status
 * @param enable: Whether to make the field induction
 */
static void tag_emulation_sense_switch_all(bool enable) {
    uint8_t slot = tag_emulation_get_slot();
    // NRF_LOG_INFO("Slot %d tag type hf %d, lf %d", slot, slotConfig.slots[slot].tag_hf, slotConfig.slots[slot].tag_lf);
    if (enable && (slotConfig.slots[slot].enabled_hf) && (slotConfig.slots[slot].tag_hf != TAG_TYPE_UNDEFINED)) {
        nfc_tag_14a_sense_switch(true);
    } else {
        nfc_tag_14a_sense_switch(false);
    }
    if (enable && (slotConfig.slots[slot].enabled_lf) && (slotConfig.slots[slot].tag_lf != TAG_TYPE_UNDEFINED)) {
        lf_tag_125khz_sense_switch(true);
    } else {
        lf_tag_125khz_sense_switch(false);
    }
}

/**
 * Switch field induction monitoring status
 * @param type: Field sensor type
 * @param enable: Whether to enable this type of field induction
 */
void tag_emulation_sense_switch(tag_sense_type_t type, bool enable) {
    uint8_t slot = tag_emulation_get_slot();
    // Check the parameters, not allowed to switch non -normal field
    switch (type) {
        case TAG_SENSE_NO:
            APP_ERROR_CHECK(NRF_ERROR_INVALID_PARAM);
            break;
        case TAG_SENSE_HF:
            if (enable && (slotConfig.slots[slot].enabled_hf) &&
                    (slotConfig.slots[slot].tag_hf != TAG_TYPE_UNDEFINED)) {
                nfc_tag_14a_sense_switch(true);
            } else {
                nfc_tag_14a_sense_switch(false);
            }
            break;
        case TAG_SENSE_LF:
            if (enable && (slotConfig.slots[slot].enabled_lf) &&
                    (slotConfig.slots[slot].tag_lf != TAG_TYPE_UNDEFINED)) {
                lf_tag_125khz_sense_switch(true);
            } else {
                lf_tag_125khz_sense_switch(false);
            }
            break;
    }
}

static void tag_emulation_migrate_slot_config_v0_to_v8(void) {
    // Copy old slotConfig content
    uint8_t tmpbuf[sizeof(slotConfig)];
    memcpy(tmpbuf, (uint8_t *)&slotConfig, sizeof(tmpbuf));
    NRF_LOG_INFO("Migrating slotConfig v0...");
    NRF_LOG_HEXDUMP_INFO(tmpbuf, sizeof(tmpbuf));
    // Populate new slotConfig struct
    slotConfig.version = 8;  // v0->v8; the v8->v9 step runs in the cascade below
    slotConfig.active_slot = tmpbuf[0];
    for (uint8_t i = 0; i < ARRAYLEN(slotConfig.slots); i++) {
        bool enabled = tmpbuf[4 + (i * 4)] & 1;

        slotConfig.slots[i].tag_hf = tmpbuf[4 + (i * 4) + 2];
        for (uint8_t j = 0; j < ARRAYLEN(tag_specific_type_old2new_hf_values); j++) {
            if (slotConfig.slots[i].tag_hf == tag_specific_type_old2new_hf_values[j][0]) {
                slotConfig.slots[i].tag_hf = tag_specific_type_old2new_hf_values[j][1];
            }
        }
        slotConfig.slots[i].enabled_hf = slotConfig.slots[i].tag_hf != TAG_TYPE_UNDEFINED ? enabled : false;
        NRF_LOG_INFO("Slot %i HF: %02X->%04X enabled:%i", i, tmpbuf[4 + (i * 4) + 2], slotConfig.slots[i].tag_hf, slotConfig.slots[i].enabled_hf);

        slotConfig.slots[i].tag_lf = tmpbuf[4 + (i * 4) + 3];
        for (uint8_t j = 0; j < ARRAYLEN(tag_specific_type_old2new_lf_values); j++) {
            if (slotConfig.slots[i].tag_lf == tag_specific_type_old2new_lf_values[j][0]) {
                slotConfig.slots[i].tag_lf = tag_specific_type_old2new_lf_values[j][1];
            }
        }
        slotConfig.slots[i].enabled_lf = slotConfig.slots[i].tag_lf != TAG_TYPE_UNDEFINED ? enabled : false;
        NRF_LOG_INFO("Slot %i LF: %02X->%04X enabled:%i", i, tmpbuf[4 + (i * 4) + 3], slotConfig.slots[i].tag_lf, slotConfig.slots[i].enabled_lf);
    }
}

static void tag_emulation_migrate_slot_config_v8_to_v9(void) {
    // v8 (official v2.2.0) had no auto-poll fields; initialise them.
    NRF_LOG_INFO("Migrating slotConfig v8 -> v9 (auto-poll)...");
    slotConfig.version = 9;
    slotConfig.auto_poll_enable = 0;
    slotConfig.last_auth_slot = 0;
    slotConfig.auto_poll_interval_ms = AUTO_POLL_INTERVAL_DEFAULT_MS;
}

static void tag_emulation_migrate_slot_config_v9_to_v10(void) {
    // v10: auto-poll unified. Earlier v9 shipped with auto_poll_enable=0, which
    // meant users who only enabled Smart-select (wake) saw no multi-slot
    // rotation ("wake works, poll doesn't"). v10 turns ON the full feature by
    // default (wake-on-field + idle rotation) and standardises the rotation
    // interval on the validated 1000 ms cadence so it works with no extra CLI.
    NRF_LOG_INFO("Migrating slotConfig v9 -> v10 (auto-poll unified, default-on)...");
    slotConfig.version = 10;
    slotConfig.auto_poll_enable = AUTO_POLL_ALL;
    slotConfig.auto_poll_interval_ms = AUTO_POLL_INTERVAL_DEFAULT_MS;
}

static void tag_emulation_migrate_slot_config_v10_to_v11(void) {
    // v11: re-apply auto-poll defaults so devices that already ran the v9->v10
    // migration (which used the older 1000 ms cadence) pick up the new 350 ms
    // default interval on the next boot without a factory reset.
    NRF_LOG_INFO("Migrating slotConfig v10 -> v11 (auto-poll 350ms default)...");
    slotConfig.version = 11;
    slotConfig.auto_poll_enable = AUTO_POLL_ALL;
    slotConfig.auto_poll_interval_ms = AUTO_POLL_INTERVAL_DEFAULT_MS;
}

static void tag_emulation_migrate_slot_config_v11_to_v12(void) {
    // v12: split the previously shared last_auth_slot into per-sense
    // last_auth_hf / last_auth_lf so an HF wake and an LF wake each resume the
    // auto-poll cycle from the slot that band last engaged (the shared slot
    // could point at the other band and bias the start wrongly, e.g. an LF wake
    // from the default slot 3 never reaching slot 1).
    NRF_LOG_INFO("Migrating slotConfig v11 -> v12 (per-sense last_auth)...");
    slotConfig.version = 12;
    slotConfig.last_auth_hf = slotConfig.last_auth_slot;
    slotConfig.last_auth_lf = slotConfig.last_auth_slot;
}

static void tag_emulation_migrate_slot_config(void) {
    switch (slotConfig.version) {
        case 0:
        case 1:
        case 2:
        case 3:
        case 4:
        case 5:
        case 6:
        case 7:
            tag_emulation_migrate_slot_config_v0_to_v8();

            /*
             * Add new migration steps ABOVE THIS COMMENT
             * `tag_emulation_save_config()` and `break` statements should only be used on the last migration step, all the previous steps must fall
             * through to the next case.
             */

            tag_emulation_save_config();
        case 8:
            tag_emulation_migrate_slot_config_v8_to_v9();
            // fall through: intermediate step, save only on the last one
        case 9:
            tag_emulation_migrate_slot_config_v9_to_v10();
            // fall through
        case 10:
            tag_emulation_migrate_slot_config_v10_to_v11();
            // fall through
        case 11:
            tag_emulation_migrate_slot_config_v11_to_v12();
            tag_emulation_save_config();
        case TAG_SLOT_CONFIG_CURRENT_VERSION:
            break;
        default:
            NRF_LOG_ERROR("Unsupported slotConfig migration attempted! (%d -> %d)", slotConfig.version, TAG_SLOT_CONFIG_CURRENT_VERSION);
            break;
    }
}

/**
 * Load the emulated card configuration data, note that loading is just a card slot configuration
 */
static void tag_emulation_load_config(void) {
    uint16_t length = sizeof(slotConfig);
    // Read the card slot configuration data
    bool ret = fds_read_sync(FDS_EMULATION_CONFIG_FILE_ID, FDS_EMULATION_CONFIG_RECORD_KEY, &length, (uint8_t *)&slotConfig);
    if (ret) {
        // After the reading is completed, we will save a BCC of the current configuration. When it is stored later, it can be used as a reference for the contrast between changes.
        calc_14a_crc_lut((uint8_t *)&slotConfig, sizeof(slotConfig), (uint8_t *)&m_slot_config_crc);
        NRF_LOG_INFO("Load tag slot config done.");
        if (slotConfig.version < TAG_SLOT_CONFIG_CURRENT_VERSION) {  // old slotConfig, need to migrate
            tag_emulation_migrate_slot_config();
        }
    } else {
        NRF_LOG_INFO("Tag slot config does not exist.");
    }
}

/**
 * Save the emulated card configuration data
 */
static void tag_emulation_save_config(void) {
    // We are configured the card slot configuration, and we need to calculate the current card slot configuration CRC code to judge whether the data below is updated
    uint16_t new_calc_crc;
    calc_14a_crc_lut((uint8_t *)&slotConfig, sizeof(slotConfig), (uint8_t *)&new_calc_crc);
    if (new_calc_crc != m_slot_config_crc) {  // Before saving, make sure that the card slot configuration has changed
        NRF_LOG_INFO("Save tag slot config start.");
        bool ret = fds_write_sync(FDS_EMULATION_CONFIG_FILE_ID, FDS_EMULATION_CONFIG_RECORD_KEY, sizeof(slotConfig), (uint8_t *)&slotConfig);
        if (ret) {
            NRF_LOG_INFO("Save tag slot config success.");
            m_slot_config_crc = new_calc_crc;
        } else {
            NRF_LOG_ERROR("Save tag slot config error.");
        }
    } else {
        NRF_LOG_INFO("Tag slot config no change.");
    }
}

/**
 * Start tag emulation
 */
void tag_emulation_sense_run(void) {
    tag_emulation_sense_switch_all(true);
}

/**
 * Stop the tag emulation. Note that this function will absolutely block NFC-related events, including awakening MCU
 * If you still need to be awakened by NFC after the MCU is required, please do not call this function
 */
void tag_emulation_sense_end(void) {
    TAG_FIELD_LED_OFF();
    tag_emulation_sense_switch_all(false);
}

/**
 * Initialized tag emulation
 */
void tag_emulation_init(void) {
    tag_emulation_load_config();  // Configuration of loading the card slot of the emulation card
    tag_emulation_load_data();    // Load the data of the emulated card
}

/**
 * Save the tag data (written from RAM to Flash)
 */
void tag_emulation_save(void) {
    tag_emulation_save_config();  // Save the card slot configuration
    tag_emulation_save_data();    // Save card slot data
}

/**
 * Get the currently activated card slot index
 */
uint8_t tag_emulation_get_slot(void) {
    return slotConfig.active_slot;
}

/**
 * Set the currently activated card slot index
 */
void tag_emulation_set_slot(uint8_t index) {
    slotConfig.active_slot = index;  // Re -set to the new switched card slot
    rgb_marquee_reset();             // force animation color refresh according to new slot
}

/**
 * Switch to the card slot of the specified index, this function will automatically complete the data loading
 */
void tag_emulation_change_slot(uint8_t index, bool sense_disable) {
    if (sense_disable) {
        // Turn off the analog card to avoid triggering the emulation when switching the card slot
        tag_emulation_sense_end();
    }
    tag_emulation_save_data();      // Save the data of the current card, in case of there is a change
    g_is_tag_emulating = false;     // Reset the emulating flag
    tag_emulation_set_slot(index);  // Update the index of the activated card slot
    tag_emulation_load_data();      // Then reload the data of the card slot
    if (sense_disable) {
        // According to the configuration of the new card slot, the monitoring status of our update
        tag_emulation_sense_run();
    }
}

/**
 * Determine whether the specified card slot is enabled
 */
bool is_slot_enabled(uint8_t slot, tag_sense_type_t sense_type) {
    if (sense_type == TAG_SENSE_LF) {
        return slotConfig.slots[slot].enabled_lf;
    }
    if (sense_type == TAG_SENSE_HF) {
        return slotConfig.slots[slot].enabled_hf;
    }
    return false;
}

/**
 * Set whether the specified card slot is enabled
 */
void tag_emulation_slot_set_enable(uint8_t slot, tag_sense_type_t sense_type, bool enable) {
    // Set the capacity of the corresponding card slot directly
    if (sense_type == TAG_SENSE_LF) {
        slotConfig.slots[slot].enabled_lf = enable;
    }
    if (sense_type == TAG_SENSE_HF) {
        slotConfig.slots[slot].enabled_hf = enable;
    }
}

/**
 * Find the next valid card slot
 */
uint8_t tag_emulation_slot_find_next(uint8_t slot_now) {
    uint8_t start_slot = (slot_now + 1 == TAG_MAX_SLOT_NUM) ? 0 : slot_now + 1;
    for (uint8_t i = start_slot;;) {
        if (i == slot_now) return slot_now;                                              // No other activated card slots were found after a loop
        // Only consider slots enabled for the SENSE of the field that woke us,
        // so an HF reader polls only HF card slots and an LF reader only LF ones.
        if (is_slot_enabled(i, g_active_sense)) return i;
        i++;
        if (i == TAG_MAX_SLOT_NUM) {  // Continue the next cycle
            i = 0;
        }
    }
    return slot_now;  // If you cannot find it, the specified return value of the pass is returned by default
}

/**
 * Find the previous valid card slot
 */
uint8_t tag_emulation_slot_find_prev(uint8_t slot_now) {
    uint8_t start_slot = (slot_now == 0) ? (TAG_MAX_SLOT_NUM - 1) : slot_now - 1;
    for (uint8_t i = start_slot;;) {
        if (i == slot_now) return slot_now;                                              // No other activated card slots were found after a loop
        if (slotConfig.slots[i].enabled_hf || slotConfig.slots[i].enabled_lf) return i;  // Check whether the card slot that is currently traversed is enabled, so that the capacity determines that the current card slot is the card slot that can effectively enable capacity
        if (i == 0) {                                                                    // Continue the next cycle
            i = TAG_MAX_SLOT_NUM - 1;
        } else {
            i--;
        }
    }
    return slot_now;  // If you cannot find it, the specified return value of the pass is returned by default
}

/**
 * Set the card specified by the specified card slot card slot card type card to the specified type
 */
void tag_emulation_change_type(uint8_t slot, tag_specific_type_t tag_type) {
    tag_sense_type_t sense_type = get_sense_type_from_tag_type(tag_type);
    NRF_LOG_INFO("sense type = %d", sense_type);
    switch (sense_type) {
        case TAG_SENSE_LF: {
            slotConfig.slots[slot].tag_lf = tag_type;
            break;
        }
        case TAG_SENSE_HF: {
            slotConfig.slots[slot].tag_hf = tag_type;
            break;
        }
        default:
            break;  // never happen
    }
    NRF_LOG_INFO("tag type = %d", tag_type);
    // After the update is completed, we need to notify the relevant data in the update of the memory
    if (sense_type != TAG_SENSE_NO) {
        load_data_by_tag_type(slot, tag_type);
        NRF_LOG_INFO("reload data success.");
    }
}

/* =========================================================================
 * Auto-poll (Smart poll + multi-slot auto polling)
 * ========================================================================= */

uint8_t tag_emulation_get_auto_poll_enable(void) {
    return slotConfig.auto_poll_enable;
}

void tag_emulation_get_auto_poll(uint8_t *enable, uint16_t *interval_ms, uint8_t *last_auth_slot) {
    if (enable != NULL)        *enable = slotConfig.auto_poll_enable;
    if (interval_ms != NULL)   *interval_ms = slotConfig.auto_poll_interval_ms;
    if (last_auth_slot != NULL) *last_auth_slot = slotConfig.last_auth_slot;
}

void tag_emulation_set_auto_poll(uint8_t enable, uint16_t interval_ms) {
    slotConfig.auto_poll_enable = enable & (AUTO_POLL_SMART_SELECT | AUTO_POLL_TIMER_ROTATE);
    // Enabling multi-slot rotation explicitly exits manual (key-selected) mode:
    // the user wants cyclic polling again, so drop any sticky manual override
    // and let the next passive RF wake start a fresh cycle from the best slot.
    if (slotConfig.auto_poll_enable & AUTO_POLL_TIMER_ROTATE) {
        g_manual_slot_select = false;
        g_ap_cycling = false;
    }
    if (interval_ms < AUTO_POLL_INTERVAL_MIN_MS) interval_ms = AUTO_POLL_INTERVAL_MIN_MS;
    if (interval_ms > AUTO_POLL_INTERVAL_MAX_MS) interval_ms = AUTO_POLL_INTERVAL_MAX_MS;
    slotConfig.auto_poll_interval_ms = interval_ms;
    tag_emulation_save_config();
    NRF_LOG_INFO("Auto-poll set: enable=0x%02X interval=%u ms", slotConfig.auto_poll_enable, slotConfig.auto_poll_interval_ms);
}

/**
 * @brief Pick the best slot for a detected field of the given sense type.
 *        Prefers the last slot a reader engaged (Smart poll memory); falls
 *        back to the first slot enabled for that sense type.
 * @return slot index, or TAG_MAX_SLOT_NUM if none is enabled for that sense.
 */
static uint8_t tag_emulation_pick_slot_for_sense(tag_sense_type_t sense) {
    // Per-sense last-auth: an HF wake and an LF wake each resume from the slot
    // that band last engaged (the shared last_auth_slot could point at the
    // other band and wrongly bias the cycle start, e.g. an LF wake from the
    // default slot 3 never reaching slot 1).
    uint8_t pref = (sense == TAG_SENSE_LF) ? slotConfig.last_auth_lf
                                          : slotConfig.last_auth_hf;
    if (pref < TAG_MAX_SLOT_NUM && is_slot_enabled(pref, sense)) {
        return pref;
    }
    for (uint8_t i = 0; i < TAG_MAX_SLOT_NUM; i++) {
        if (is_slot_enabled(i, sense)) {
            return i;
        }
    }
    return TAG_MAX_SLOT_NUM;  // no enabled slot for this sense type
}

/**
 * @brief Called (from the HF/LF field-detect interrupt) when a reader field
 *        is detected. If Smart poll is enabled, switches the active slot to the
 *        best match for the field type and marks last_auth for persistence.
 *
 * ISR-SAFETY: this function performs NO Flash/FDS writes. The slot switch here
 * is a pure RAM + sense-toggle operation (flash reads in tag_emulation_load_data
 * are memory-mapped and safe from interrupt context). The last_auth persistence
 * is deferred to tag_emulation_smart_poll_process(), which runs in the main
 * loop, so the FDS write never happens inside the interrupt.
 */
void tag_emulation_smart_poll_on_field(tag_sense_type_t sense) {
    // Remember which field type woke us — auto-poll rotation uses this to stay
    // within the same band (HF reader polls only HF slots, LF only LF slots).
    g_active_sense = sense;

    // ---- Manual (key-selected) mode -------------------------------------
    // The user picked a slot with A/B; present THAT slot to the reader and
    // never auto-rotate. g_manual_slot_select is STICKY (set by cycle_slot /
    // any A/B press) and is only cleared when multi-slot rotation is re-enabled
    // in tag_emulation_set_auto_poll(), so a reader field blip can't silently
    // drop us back into cycling from slot 1.
    if (g_manual_slot_select) {
        return;
    }

    bool rotate = (slotConfig.auto_poll_enable & AUTO_POLL_TIMER_ROTATE) != 0;

    if (rotate) {
        if (!g_ap_cycling) {
            // Begin a fresh auto-poll cycle, starting from the best slot for
            // this sense type (last reader slot, else first enabled).
            uint8_t sel = tag_emulation_pick_slot_for_sense(sense);
            if (sel < TAG_MAX_SLOT_NUM) {
                if (sel != slotConfig.active_slot) {
                    tag_emulation_set_slot(sel);
                    tag_emulation_load_data();
                }
                if (((sense == TAG_SENSE_LF) ? slotConfig.last_auth_lf
                                            : slotConfig.last_auth_hf) != sel) {
                    if (sense == TAG_SENSE_LF) slotConfig.last_auth_lf = sel;
                    else                       slotConfig.last_auth_hf = sel;
                    slotConfig.last_auth_slot = sel;   // CLI/compat
                    g_smart_poll_dirty = true;
                }
                NRF_LOG_INFO("Auto poll: field=%s -> start cycle at slot %d",
                             sense == TAG_SENSE_HF ? "HF" : "LF", sel);
            }
            g_ap_cycling = true;
        }
        // KEY FIX: a field re-assert (blip) must NOT reset us to the first slot.
        // Just refresh the time guard and keep cycling from the current slot,
        // so the reader is offered 1,2,3,4 in turn instead of looping on slot 1.
        g_ap_cycle_end_ticks =
            app_timer_cnt_get() + APP_TIMER_TICKS(AUTO_POLL_CYCLE_GUARD_MS);
        return;
    }

    // ---- No timer-rotate: ordinary Smart-select -------------------------
    // Field wakeup simply selects the best slot; no multi-slot cycle.
    if (!(slotConfig.auto_poll_enable & AUTO_POLL_SMART_SELECT)) {
        return;
    }
    uint8_t sel = tag_emulation_pick_slot_for_sense(sense);
    if (sel >= TAG_MAX_SLOT_NUM) {
        return;  // nothing enabled for this field type
    }
    if (sel != slotConfig.active_slot) {
        // Only switch the active slot and reload its emulated data into RAM.
        // Do NOT call sense_end()/sense_run() here: the field-detect ISR that
        // invoked us is about to start emulation itself (the HF callback
        // re-inits NFC, the LF handler plays the PWM burst). Toggling sense
        // inside the ISR corrupts the LF modulation state and breaks LF
        // emulation. The caller's emulation start will transmit the freshly
        // loaded data for the new slot. No FDS write happens here.
        tag_emulation_set_slot(sel);
        tag_emulation_load_data();
        NRF_LOG_INFO("Smart poll: field=%s -> slot %d",
                     sense == TAG_SENSE_HF ? "HF" : "LF", sel);
    }
    // remember which slot the reader just engaged (persisted in main loop)
    if (((sense == TAG_SENSE_LF) ? slotConfig.last_auth_lf
                                : slotConfig.last_auth_hf) != sel) {
        if (sense == TAG_SENSE_LF) slotConfig.last_auth_lf = sel;
        else                       slotConfig.last_auth_hf = sel;
        slotConfig.last_auth_slot = sel;   // CLI/compat
        g_smart_poll_dirty = true;
    }
}

void tag_emulation_user_select_slot(void) {
    // The user explicitly chose a slot via A/B: take manual control and cancel
    // any running auto-poll cycle.
    g_manual_slot_select = true;
    g_ap_cycling = false;
}

void tag_emulation_on_field_lost(void) {
    // Manual (key-selected) mode is STICKY: a field blip or a full read-drop
    // must NOT cancel it, otherwise the reader's routine field toggles would
    // immediately drop us back into auto-poll cycling from slot 1. The user
    // leaves manual mode only by re-enabling multi-slot rotation (see
    // tag_emulation_set_auto_poll). The cycle session's own time guard keeps
    // it alive across blips, so there is nothing to tear down here.
}

/**
 * @brief Main-loop counterpart of tag_emulation_smart_poll_on_field().
 *        Persists the last_auth_slot update to Flash. MUST be called from the
 *        main loop, never from interrupt context.
 */
void tag_emulation_smart_poll_process(void) {
    if (g_smart_poll_dirty) {
        g_smart_poll_dirty = false;
        tag_emulation_save_config();
        NRF_LOG_INFO("Smart poll: persisted last_auth_slot=%d", slotConfig.last_auth_slot);
    }
}

/**
 * @brief Timer-driven multi-slot auto polling. Rotates the active slot among
 *        the card slots enabled for the SENSE of the field that woke the device.
 *        Gated on the auto-poll CYCLE SESSION (started and time-guarded in
 *        tag_emulation_smart_poll_on_field()), NOT on the momentary field flag
 *        g_is_tag_emulating. This way a field blip mid-poll does not freeze
 *        rotation, and the cycle keeps offering 1,2,3,4 in turn until its time
 *        guard expires.
 */
void tag_emulation_auto_poll_rotate(void) {
    if (!(slotConfig.auto_poll_enable & AUTO_POLL_TIMER_ROTATE)) {
        return;
    }
    if (get_device_mode() != DEVICE_MODE_TAG) {
        return;
    }
    // Cycle session gating (set by passive RF wake, not by a button press).
    if (!g_ap_cycling) {
        return;
    }
    // Time guard: once no field has re-asserted for AUTO_POLL_CYCLE_GUARD_MS,
    // end the cycle so we stop rotating.
    if (app_timer_cnt_diff_compute(g_ap_cycle_end_ticks, app_timer_cnt_get()) <= 0) {
        g_ap_cycling = false;
        NRF_LOG_INFO("Auto poll: cycle guard expired, stop");
        return;
    }
    uint8_t next = tag_emulation_slot_find_next(slotConfig.active_slot);
    if (next != slotConfig.active_slot) {
        if (g_active_sense == TAG_SENSE_LF) {
            // LF lightweight slot swap. The LF field is a continuous 125 kHz
            // carrier; the running pwm_handler EVT_STOPPED replay loop re-reads
            // the global m_pwm_seq on every field-confirmed burst, so we only
            // need to rebuild that sequence for the new slot. This avoids the
            // full teardown/re-init that tag_emulation_change_slot(..., true)
            // performs (nrfx_pwm_uninit + nrfx_lpcomp_uninit + HFXO release and
            // a blocking re-request), which could miss the narrow field window
            // and leave the rotated LF slot unplayed -- the bug where an LF wake
            // from the default slot 3 never reached slot 1.
            //
            // IMPORTANT: we must NOT call tag_emulation_load_data() here (timer
            // ISR, PWM still playing) -- rebuilding m_pwm_seq under a running
            // PWM hangs the replay state machine (device freeze / WDT reset).
            // We only update the slot index now and arm a deferred reload that
            // pwm_handler() applies at the next EVT_STOPPED, where the PWM is
            // guaranteed stopped and re-pointing the sequence is safe.
            tag_emulation_set_slot(next);     // RAM-only; safe from ISR
            g_lf_rotate_pending = true;       // reloaded at EVT_STOPPED
            if (slotConfig.last_auth_lf != next) {
                slotConfig.last_auth_lf = next;
                slotConfig.last_auth_slot = next;   // CLI/compat
                g_smart_poll_dirty = true;
            }
        } else {
            tag_emulation_change_slot(next, true);
        }
        NRF_LOG_INFO("Auto poll rotate -> slot %d", next);
    }
}

/**
 * @brief Apply a pending LF slot rotation. MUST be called from lf_tag_em.c's
 *        pwm_handler on NRFX_PWM_EVT_STOPPED, i.e. when the PWM is fully
 *        stopped and it is safe to rebuild the global m_pwm_seq. Rebuilding
 *        the sequence from the auto-poll timer ISR (while the PWM plays) would
 *        corrupt the replay state machine and freeze the device.
 */
void tag_emulation_lf_apply_pending_slot(void) {
    if (g_lf_rotate_pending) {
        g_lf_rotate_pending = false;
        tag_emulation_load_data();   // PWM stopped here -> safe to rebuild seq
    }
}

/**
 * @brief Drop any pending LF rotation (e.g. when the field is lost). Prevents a
 *        stale deferred reload from overwriting the slot a fresh field-detect
 *        selects in tag_emulation_smart_poll_on_field().
 */
void tag_emulation_lf_clear_pending_slot(void) {
    g_lf_rotate_pending = false;
}


/**
 * The factory initialization function of the card emulation.
 * Defaults to a dual-frequency card in slot 1, a hf M1 card in slot 2, and a lf em410x card in slot 3.
 */
void tag_emulation_factory_init(void) {
    fds_slot_record_map_t map_info;

    // Initialized a dual -frequency card in the card slot, if there is no historical record, it is a new state of factory.
    if (slotConfig.slots[0].enabled_hf && slotConfig.slots[0].tag_hf == TAG_TYPE_MIFARE_1024) {
        // Initialize a high -frequency M1 card in the card slot 1, if it does not exist.
        get_fds_map_by_slot_sense_type_for_dump(0, TAG_SENSE_HF, &map_info);
        if (!fds_is_exists(map_info.id, map_info.key)) {
            tag_emulation_factory_data(0, slotConfig.slots[0].tag_hf);
        }
    }

    if (slotConfig.slots[0].enabled_lf && slotConfig.slots[0].tag_lf == TAG_TYPE_EM410X) {
        // Initialize a low -frequency EM410X card in slot 1, if it does not exist.
        get_fds_map_by_slot_sense_type_for_dump(0, TAG_SENSE_LF, &map_info);
        if (!fds_is_exists(map_info.id, map_info.key)) {
            tag_emulation_factory_data(0, slotConfig.slots[0].tag_lf);
        }
    }

    if (slotConfig.slots[1].enabled_hf && slotConfig.slots[1].tag_hf == TAG_TYPE_MF0ICU1) {
        // Initialize a high -frequency M1 card in the card slot 2, if it does not exist.
        get_fds_map_by_slot_sense_type_for_dump(1, TAG_SENSE_HF, &map_info);
        if (!fds_is_exists(map_info.id, map_info.key)) {
            tag_emulation_factory_data(1, slotConfig.slots[1].tag_hf);
        }
    }

    if (slotConfig.slots[2].enabled_lf && slotConfig.slots[2].tag_lf == TAG_TYPE_EM410X) {
        // Initialize a low -frequency EM410X card in slot 3, if it does not exist.
        get_fds_map_by_slot_sense_type_for_dump(2, TAG_SENSE_LF, &map_info);
        if (!fds_is_exists(map_info.id, map_info.key)) {
            tag_emulation_factory_data(2, slotConfig.slots[2].tag_lf);
        }
    }
}
