#ifndef NFC_TAG_H
#define NFC_TAG_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "app_util.h"
#include "tag_base_type.h"
#include "utils.h"

// Up to eight card slots
#define TAG_MAX_SLOT_NUM 8

extern bool g_is_tag_emulating;
// Sense (HF/LF) of the field that most recently woke the device. Set by
// tag_emulation_smart_poll_on_field() from the field-detect ISR and used to
// keep auto-poll rotation within the same frequency band as the reader.
extern tag_sense_type_t g_active_sense;

// Tag data buffer
typedef struct {
    uint16_t length;
    uint8_t *buffer;
    uint16_t *crc;
} tag_data_buffer_t;

// Farming impact enable and closed energy switching function
typedef void (*tag_sense_switch_t)(bool enable);
// Flash data is notified to the registrar after loading to RAM
typedef int (*tag_datas_loadcb_t)(tag_specific_type_t type, tag_data_buffer_t *buffer);
// The data should be saved to the registered person before Flash
typedef int (*tag_datas_savecb_t)(tag_specific_type_t type, tag_data_buffer_t *buffer);
// Data factory initialization function
typedef bool (*tag_datas_factory_t)(uint8_t slot, tag_specific_type_t type);

// The data of the tag data loading and the recovery function of the preservation event mapping table
typedef struct {
    tag_sense_type_t sense_type;
    tag_specific_type_t tag_type;
    tag_datas_loadcb_t data_on_load;
    tag_datas_savecb_t data_on_save;
    tag_datas_factory_t data_factory;
    tag_data_buffer_t *data_buffer;
} tag_base_handler_map_t;

/**
 * The storage configuration of parameters such as the type of card emulated in the card slot
 * This configuration can be preserved by persistently to Flash
 * 4 bytes a word, keep in mind the entire word alignment
 */
#define TAG_SLOT_CONFIG_CURRENT_VERSION 11
// Intended struct size, for static assert
#define TAG_SLOT_CONFIG_CURRENT_SIZE 72

/* Auto-poll feature flags (bitmask stored in slotConfig.auto_poll_enable) */
#define AUTO_POLL_SMART_SELECT  (1u << 0)  // pick slot by detected field type + last_auth
#define AUTO_POLL_TIMER_ROTATE  (1u << 1)  // rotate active slot among enabled slots on a timer
/* Convenience mask: full auto-poll = wake-on-field + idle multi-slot rotation */
#define AUTO_POLL_ALL           (AUTO_POLL_SMART_SELECT | AUTO_POLL_TIMER_ROTATE)

/* Default rotation interval (ms) used when migrating / resetting config.
 * Matches the validated ZhaiRen "POLL1000ms" reference build. */
#define AUTO_POLL_INTERVAL_DEFAULT_MS 350
/* Clamp range for the rotation interval to keep the app_timer sane */
#define AUTO_POLL_INTERVAL_MIN_MS     200
#define AUTO_POLL_INTERVAL_MAX_MS     60000

typedef struct {
    // Basic configuration
    uint8_t version;      // struct version (U8 so map on old .activated<=7 field)
    uint8_t active_slot;  // Which slot is currently active
    uint32_t : 0;         // U32 align
    struct {              // 4-byte slot config + 2*2-byte tag_specific_types
        // Individual slot configuration
        uint32_t enabled_hf : 1;  // Whether to enable the HF card
        uint32_t enabled_lf : 1;  // Whether to enable the LF card
        uint32_t : 0;             // U32 align
        // Specific type of emulated card
        union {
            uint16_t U16_tag_hf;
            tag_specific_type_t tag_hf;
        };
        union {
            uint16_t U16_tag_lf;
            tag_specific_type_t tag_lf;
        };
    } slots[TAG_MAX_SLOT_NUM];
    // v9 additions: global auto-poll (Smart poll + multi-slot auto polling) settings
    uint8_t  auto_poll_enable;       // bitmask of AUTO_POLL_* flags
    uint8_t  last_auth_slot;         // last slot a reader engaged (Smart poll preference)
    uint16_t auto_poll_interval_ms;  // rotation interval for AUTO_POLL_TIMER_ROTATE
} PACKED tag_slot_config_t;

// Use the macro to check the struct size
STATIC_ASSERT(sizeof(tag_slot_config_t) == TAG_SLOT_CONFIG_CURRENT_SIZE);

// The most basic emulation card initialization program
void tag_emulation_init(void);
// Some of the data stored in RAM can be saved to Flash through this interface
void tag_emulation_save(void);

// Starting and ending of the emulation card
void tag_emulation_load_data(void);
void tag_emulation_sense_run(void);
void tag_emulation_sense_end(void);

// Farming response enable state switching package function
void tag_emulation_sense_switch(tag_sense_type_t type, bool enable);
// Delete the type of card specified in the card slot
void tag_emulation_delete_data(uint8_t slot, tag_sense_type_t sense_type);
// Initial data of the factory data of the specified card slot into the factory of the specified type of card
bool tag_emulation_factory_data(uint8_t slot, tag_specific_type_t tag_type);
// Change the type of the card that is being emulated
void tag_emulation_change_type(uint8_t slot, tag_specific_type_t tag_type);
// Load the data from the memory to the emulation card buffer
bool tag_emulation_load_by_buffer(tag_specific_type_t tag_type, bool update_crc);

tag_sense_type_t get_sense_type_from_tag_type(tag_specific_type_t type);
tag_data_buffer_t *get_buffer_by_tag_type(tag_specific_type_t type);

// Set the card slot currently used
void tag_emulation_set_slot(uint8_t index);
// Get the card slot currently used
uint8_t tag_emulation_get_slot(void);
// Switch the card slot to control whether the passing parameter control is closed during the switching period to listen to
void tag_emulation_change_slot(uint8_t index, bool sense_disable);
// Get the card slot to enable the state
bool is_slot_enabled(uint8_t slot, tag_sense_type_t sense_type);
// Set the card slot to enable
void tag_emulation_slot_set_enable(uint8_t slot, tag_sense_type_t sense_type, bool enable);
// Get the emulation card type of the corresponding card slot
void tag_emulation_get_specific_types_by_slot(uint8_t slot, tag_slot_specific_type_t *tag_types);
// Initialize some factory data
void tag_emulation_factory_init(void);

// In the direction, query any card slot that enable
uint8_t tag_emulation_slot_find_next(uint8_t slot_now);
uint8_t tag_emulation_slot_find_prev(uint8_t slot_now);
bool is_tag_specific_type_valid(tag_specific_type_t tag_type);

/* ---- Auto-poll (Smart poll + multi-slot auto polling) ---- */
// Returns the current auto_poll enable bitmask (AUTO_POLL_* flags)
uint8_t tag_emulation_get_auto_poll_enable(void);
// Set enable bitmask + rotation interval (interval is clamped). Persists to flash.
void tag_emulation_set_auto_poll(uint8_t enable, uint16_t interval_ms);
// Read the full auto-poll config (enable / interval / last_auth_slot)
void tag_emulation_get_auto_poll(uint8_t *enable, uint16_t *interval_ms, uint8_t *last_auth_slot);
// Called from the HF/LF field-detect interrupt. Selects the best slot for the
// given sense type (Smart poll) and records it as last_auth. ISR-safe: performs
// a Flash-free slot switch and defers persistence. No-op unless enabled.
void tag_emulation_smart_poll_on_field(tag_sense_type_t sense);
// Called from the main loop to persist Smart-poll state (last_auth_slot) to
// Flash. Must NOT be called from interrupt context.
void tag_emulation_smart_poll_process(void);
// Called by the auto-poll timer. Rotates the active slot among enabled slots
// (multi-slot auto polling). No-op unless enabled / in tag mode / cycling.
void tag_emulation_auto_poll_rotate(void);
// Called when the user picks a slot with A/B (key-wakeup). Marks manual mode so
// a following reader field emulates the chosen slot WITHOUT entering the
// auto-poll cycle. Also cancels any running cycle.
void tag_emulation_user_select_slot(void);
// Called from the HF/LF FIELD_LOST handler. Clears manual mode so the next
// passive RF wake can auto-cycle again. The auto-poll cycle itself is left
// running (time-guarded) so a field blip does not kill it.
void tag_emulation_on_field_lost(void);

#endif
