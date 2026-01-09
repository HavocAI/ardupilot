/*
  failsafe support
  Andrew Tridgell, December 2011
 */

#include "Rover.h"

#include <stdio.h>

/*
  our failsafe strategy is to detect main loop lockup and disarm.
 */

/*
  this failsafe_check function is called from the core timer interrupt
  at 1kHz.
 */
void Rover::failsafe_check()
{
    static uint16_t last_ticks;
    static uint32_t last_timestamp;
    const uint32_t tnow = AP_HAL::micros();

    const uint16_t ticks = scheduler.ticks();
    if (ticks != last_ticks) {
        // the main loop is running, all is OK
        last_ticks = ticks;
        last_timestamp = tnow;
        return;
    }

    if (tnow - last_timestamp > 200000) {
        // we have gone at least 0.2 seconds since the main loop
        // ran. That means we're in trouble, or perhaps are in
        // an initialisation routine or log erase. disarm the motors
        // To-Do: log error
        if (arming.is_armed()) {
            // disarm motors
            arming.disarm(AP_Arming::Method::CPUFAILSAFE);
        }
    }
}

/*
  called to set/unset a failsafe event.
 */
void Rover::failsafe_trigger(uint8_t failsafe_type, const char* type_str, bool on)
{
    uint8_t old_bits = failsafe.bits;
    if (on) {
        failsafe.bits |= failsafe_type;
    } else {
        failsafe.bits &= ~failsafe_type;
    }
    if ((old_bits & failsafe_type) == 0 && (failsafe.bits & failsafe_type) != 0) {
        // a failsafe event has started
        gcs().send_text(MAV_SEVERITY_WARNING, "%s Failsafe", type_str);

        RC_Channels::clear_overrides();

        if ((control_mode == &mode_manual || control_mode == &mode_acro || control_mode == &mode_steering)) {
            arming.disarm(AP_Arming::Method::FAILSAFE_ACTION_TERMINATE);
        }


    } else if (old_bits && failsafe.bits == 0) {
        gcs().send_text(MAV_SEVERITY_INFO, "Failsafe Cleared");
    }
}

void Rover::handle_battery_failsafe(const char* type_str, const int8_t action)
{
        switch ((FailsafeAction)action) {
            case FailsafeAction::None:
                break;
            case FailsafeAction::SmartRTL:
                if (set_mode(mode_smartrtl, ModeReason::BATTERY_FAILSAFE)) {
                    break;
                }
                FALLTHROUGH;
            case FailsafeAction::RTL:
                if (set_mode(mode_rtl, ModeReason::BATTERY_FAILSAFE)) {
                    break;
                }
                FALLTHROUGH;
            case FailsafeAction::Hold:
                set_mode(mode_hold, ModeReason::BATTERY_FAILSAFE);
                break;
            case FailsafeAction::SmartRTL_Hold:
                if (!set_mode(mode_smartrtl, ModeReason::BATTERY_FAILSAFE)) {
                    set_mode(mode_hold, ModeReason::BATTERY_FAILSAFE);
                }
                break;
            case FailsafeAction::Terminate:
#if AP_ROVER_ADVANCED_FAILSAFE_ENABLED
                char battery_type_str[17];
                snprintf(battery_type_str, 17, "%s battery", type_str);
                g2.afs.gcs_terminate(true, battery_type_str);
#else
                arming.disarm(AP_Arming::Method::BATTERYFAILSAFE);
#endif // AP_ROVER_ADVANCED_FAILSAFE_ENABLED
                break;
        }
}

#if AP_ROVER_ADVANCED_FAILSAFE_ENABLED
/*
   check for AFS failsafe check
 */
void Rover::afs_fs_check(void)
{
    // perform AFS failsafe checks
    g2.afs.check(failsafe.last_valid_rc_ms);
}
#endif
