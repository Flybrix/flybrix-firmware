#ifndef CONFIG_IMPL_H
#define CONFIG_IMPL_H

template <class Cursor>
void CONFIG_struct::writeTo(Cursor&& cursor) const {
    cursor.Append(*this);
}

template <class Cursor>
bool CONFIG_struct::readFrom(Cursor&& cursor) {
    return cursor.ParseInto(*this);
}

template <class Cursor>
bool CONFIG_struct::readPartialFrom(Cursor&& cursor) {
    uint16_t submask;
    if (!cursor.ParseInto(submask)) {
        return false;
    }
    if ((submask & CONFIG_struct::VERSION) && !cursor.ParseInto(version)) {
        return false;
    }
    if ((submask & CONFIG_struct::ID) && !cursor.ParseInto(id)) {
        return false;
    }
    if ((submask & CONFIG_struct::PCB) && !cursor.ParseInto(pcb)) {
        return false;
    }
    if ((submask & CONFIG_struct::MIX_TABLE) && !cursor.ParseInto(mix_table)) {
        return false;
    }
    if ((submask & CONFIG_struct::MAG_BIAS) && !cursor.ParseInto(mag_bias)) {
        return false;
    }
    if ((submask & CONFIG_struct::CHANNEL) && !cursor.ParseInto(channel)) {
        return false;
    }
    if ((submask & CONFIG_struct::PID_PARAMETERS) && !cursor.ParseInto(pid_parameters)) {
        return false;
    }
    if ((submask & CONFIG_struct::STATE_PARAMETERS) && !cursor.ParseInto(state_parameters)) {
        return false;
    }
    if (submask & CONFIG_struct::LED_STATES) {
        // split up LED states further, since the variable is giant
        uint16_t led_mask;
        if (!cursor.ParseInto(led_mask)) {
            return false;
        }
        for (size_t led_code = 0; led_code < 16; ++led_code) {
            if ((led_mask & (1 << led_code)) && !cursor.ParseInto(led_states.states[led_code])) {
                return false;
            }
        }
    }

    return true;
}

template <class Cursor>
void CONFIG_struct::writePartialTo(Cursor&& cursor, uint16_t submask, uint16_t led_mask) const {
    cursor.Append(submask);
    if (submask & CONFIG_struct::VERSION) {
        cursor.Append(version);
    }
    if (submask & CONFIG_struct::PCB) {
        cursor.Append(pcb);
    }
    if (submask & CONFIG_struct::MIX_TABLE) {
        cursor.Append(mix_table);
    }
    if (submask & CONFIG_struct::MAG_BIAS) {
        cursor.Append(mag_bias);
    }
    if (submask & CONFIG_struct::CHANNEL) {
        cursor.Append(channel);
    }
    if (submask & CONFIG_struct::PID_PARAMETERS) {
        cursor.Append(pid_parameters);
    }
    if (submask & CONFIG_struct::STATE_PARAMETERS) {
        cursor.Append(state_parameters);
    }
    if (submask & CONFIG_struct::LED_STATES) {
        cursor.Append(led_mask);
        for (size_t led_code = 0; led_code < 16; ++led_code) {
            if (led_mask & (1 << led_code)) {
                cursor.Append(led_states.states[led_code]);
            }
        }
    }
}

template <class Cursor>
bool CONFIG_struct::readMasks(Cursor&& cursor, uint16_t& submask, uint16_t& led_mask) {
    if (!cursor.ParseInto(submask)) {
        return false;
    }
    if (!(submask & CONFIG_struct::LED_STATES)) {
        return true;
    }
    return cursor.ParseInto(led_mask);
}

#endif /* CONFIG_IMPL_H */
