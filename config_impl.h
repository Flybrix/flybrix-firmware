#ifndef CONFIG_IMPL_H
#define CONFIG_IMPL_H

#include "config.h"

constexpr uint16_t fieldToMask(std::size_t field) {
    return 1 << field;
}

static_assert(4 == fieldToMask(Config::PCB), "Mask generation isn't static");

template <class T, std::size_t field>
struct FieldFunctor {
    static void Reset(T& data, uint16_t submask, uint16_t led_mask) {
        using FieldType = typename std::tuple_element<field, T>::type;
        if (submask & fieldToMask(field)) {
            std::get<field>(data) = FieldType();
        }
    }

    template <class Cursor>
    static bool ReadAll(T& data, Cursor&& cursor) {
        return cursor.ParseInto(std::get<field>(data));
    }

    template <class Cursor>
    static bool Read(T& data, Cursor&& cursor, uint16_t submask) {
        return (!(submask & fieldToMask(field))) || cursor.ParseInto(std::get<field>(data));
    }

    template <class Cursor>
    static void WriteAll(const T& data, Cursor&& cursor) {
        cursor.Append(std::get<field>(data));
    }

    template <class Cursor>
    static void WriteOrSkip(const T& data, Cursor&& cursor, uint16_t submask, uint16_t led_mask) {
        if (submask & fieldToMask(field)) {
            cursor.Append(std::get<field>(data));
        } else {
            cursor.Skip(sizeof(T));
        }
    }

    template <class Cursor>
    static void Write(const T& data, Cursor&& cursor, uint16_t submask, uint16_t led_mask) {
        if (submask & fieldToMask(field)) {
            cursor.Append(std::get<field>(data));
        }
    }
};

template <class T>
struct FieldFunctor<T, Config::LED_STATES> {
    static constexpr std::size_t FIELD = Config::LED_STATES;

    static void Reset(T& data, uint16_t submask, uint16_t led_mask) {
        if (!(submask & fieldToMask(FIELD))) {
            return;
        }
        LED::States default_states;
        for (size_t led_code = 0; led_code < 16; ++led_code) {
            if (led_mask & (1 << led_code)) {
                std::get<FIELD>(data).states[led_code] = default_states.states[led_code];
            }
        }
    }

    template <class Cursor>
    static bool ReadAll(T& data, Cursor&& cursor) {
        return cursor.ParseInto(std::get<FIELD>(data));
    }

    template <class Cursor>
    static bool Read(T& data, Cursor&& cursor, uint16_t submask) {
        if (!(submask & fieldToMask(FIELD))) {
            return true;
        }
        // split up LED states further, since the variable is giant
        uint16_t led_mask;
        if (!cursor.ParseInto(led_mask)) {
            return false;
        }
        for (size_t led_code = 0; led_code < 16; ++led_code) {
            if ((led_mask & (1 << led_code)) && !cursor.ParseInto(std::get<FIELD>(data).states[led_code])) {
                return false;
            }
        }
        return true;
    }

    template <class Cursor>
    static void WriteAll(const T& data, Cursor&& cursor) {
        cursor.Append(std::get<FIELD>(data));
    }

    template <class Cursor>
    static void Write(const T& data, Cursor&& cursor, uint16_t submask, uint16_t led_mask) {
        if (submask & fieldToMask(FIELD)) {
            cursor.Append(led_mask);
            for (size_t led_code = 0; led_code < 16; ++led_code) {
                if (led_mask & (1 << led_code)) {
                    cursor.Append(std::get<FIELD>(data).states[led_code]);
                }
            }
        }
    }

    template <class Cursor>
    static void WriteOrSkip(const T& data, Cursor&& cursor, uint16_t submask, uint16_t led_mask) {
        if (submask & fieldToMask(FIELD)) {
            for (size_t led_code = 0; led_code < 16; ++led_code) {
                if (led_mask & (1 << led_code)) {
                    cursor.Append(std::get<FIELD>(data).states[led_code]);
                } else {
                    cursor.Skip(sizeof(LED::StateCase));
                }
            }
        } else {
            cursor.Skip(sizeof(T));
        }
    }
};

template <std::size_t I = 0, typename... Tp>
inline typename std::enable_if<I == sizeof...(Tp), void>::type resetFields(std::tuple<Tp...>& t, uint16_t submask, uint16_t led_mask) {
}

template <std::size_t I = 0, typename... Tp>
    inline typename std::enable_if < I<sizeof...(Tp), void>::type resetFields(std::tuple<Tp...>& t, uint16_t submask, uint16_t led_mask) {
    FieldFunctor<std::tuple<Tp...>, I>::Reset(t, submask, led_mask);
    resetFields<I + 1, Tp...>(t, submask, led_mask);
}

template <std::size_t I = 0, class Cursor, typename... Tp>
inline typename std::enable_if<I == sizeof...(Tp), bool>::type readFieldsFrom(std::tuple<Tp...>& t, Cursor&& cursor, uint16_t submask) {
    return true;
}

template <std::size_t I = 0, class Cursor, typename... Tp>
    inline typename std::enable_if < I<sizeof...(Tp), bool>::type readFieldsFrom(std::tuple<Tp...>& t, Cursor&& cursor, uint16_t submask) {
    if (!FieldFunctor<std::tuple<Tp...>, I>::Read(t, cursor, submask)) {
        return false;
    }
    return readFieldsFrom<I + 1, Cursor, Tp...>(t, cursor, submask);
}

template <std::size_t I = 0, class Cursor, typename... Tp>
inline typename std::enable_if<I == sizeof...(Tp), bool>::type readAllFieldsFrom(std::tuple<Tp...>& t, Cursor&& cursor) {
    return true;
}

template <std::size_t I = 0, class Cursor, typename... Tp>
    inline typename std::enable_if < I<sizeof...(Tp), bool>::type readAllFieldsFrom(std::tuple<Tp...>& t, Cursor&& cursor) {
    if (!FieldFunctor<std::tuple<Tp...>, I>::ReadAll(t, cursor)) {
        return false;
    }
    return readAllFieldsFrom<I + 1, Cursor, Tp...>(t, cursor);
}

template <std::size_t I = 0, class Cursor, typename... Tp>
inline typename std::enable_if<I == sizeof...(Tp), void>::type writeFieldsTo(const std::tuple<Tp...>& t, Cursor&& cursor, uint16_t submask, uint16_t led_mask) {
}

template <std::size_t I = 0, class Cursor, typename... Tp>
    inline typename std::enable_if < I<sizeof...(Tp), void>::type writeFieldsTo(const std::tuple<Tp...>& t, Cursor&& cursor, uint16_t submask, uint16_t led_mask) {
    FieldFunctor<std::tuple<Tp...>, I>::Write(t, cursor, submask, led_mask);
    writeFieldsTo<I + 1, Cursor, Tp...>(t, cursor, submask, led_mask);
}

template <std::size_t I = 0, class Cursor, typename... Tp>
inline typename std::enable_if<I == sizeof...(Tp), void>::type writeOrSkipFieldsTo(const std::tuple<Tp...>& t, Cursor&& cursor, uint16_t submask, uint16_t led_mask) {
}

template <std::size_t I = 0, class Cursor, typename... Tp>
    inline typename std::enable_if < I<sizeof...(Tp), void>::type writeOrSkipFieldsTo(const std::tuple<Tp...>& t, Cursor&& cursor, uint16_t submask, uint16_t led_mask) {
    FieldFunctor<std::tuple<Tp...>, I>::WriteOrSkip(t, cursor, submask, led_mask);
    writeOrSkipFieldsTo<I + 1, Cursor, Tp...>(t, cursor, submask, led_mask);
}

template <std::size_t I = 0, class Cursor, typename... Tp>
inline typename std::enable_if<I == sizeof...(Tp), void>::type writeAllFieldsTo(const std::tuple<Tp...>& t, Cursor&& cursor) {
}

template <std::size_t I = 0, class Cursor, typename... Tp>
    inline typename std::enable_if < I<sizeof...(Tp), void>::type writeAllFieldsTo(const std::tuple<Tp...>& t, Cursor&& cursor) {
    FieldFunctor<std::tuple<Tp...>, I>::WriteAll(t, cursor);
    writeAllFieldsTo<I + 1, Cursor, Tp...>(t, cursor);
}

template <class Cursor>
bool Config::readPartialFrom(Cursor&& cursor) {
    uint16_t submask;
    if (!cursor.ParseInto(submask)) {
        return false;
    }
    return readFieldsFrom(data, cursor, submask);
}

template <class Cursor>
void Config::writePartialTo(Cursor&& cursor, uint16_t submask, uint16_t led_mask) const {
    cursor.Append(submask);
    writeFieldsTo(data, cursor, submask, led_mask);
}

template <class Cursor>
void Config::writeSkippableTo(Cursor&& cursor, uint16_t submask, uint16_t led_mask) const {
    cursor.Append(submask);
    writeOrSkipFieldsTo(data, cursor, submask, led_mask);
}

template <class Cursor>
bool Config::readMasks(Cursor&& cursor, uint16_t& submask, uint16_t& led_mask) {
    if (!cursor.ParseInto(submask)) {
        return false;
    }
    if (!(submask & fieldToMask(LED_STATES))) {
        return true;
    }
    return cursor.ParseInto(led_mask);
}

template <class Cursor>
void Config::writeTo(Cursor&& cursor) const {
    writeAllFieldsTo(data, cursor);
}

template <class Cursor>
bool Config::readFrom(Cursor&& cursor) {
    return readAllFieldsFrom(data, cursor);
}

#endif /* CONFIG_IMPL_H */
