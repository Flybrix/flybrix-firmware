#ifndef EEPROMCURSOR_H
#define EEPROMCURSOR_H

#include <Arduino.h>

class EEPROMCursor final {
   public:
    template <class T>
    void Append(T&& v) {
        uint8_t* v_begin{(uint8_t*)&v};
        uint8_t* v_end{v_begin + sizeof(T)};
        while (v_begin != v_end) {
            uint8_t value{*v_begin++};
            if (value != EEPROM.read(l)) {
                EEPROM.write(l, value);
            }
            ++l;
        }
    }

    template <class T, class... Targs>
    void Append(T&& v, Targs&&... vargs) {
        Append(v);
        Append(vargs...);
    }

    template <class T>
    void Skip(T&& v) {
        l += sizeof(T);
    }

    template <class T>
    void ParseInto(T& data) {
        uint8_t* v_begin{(uint8_t*)&data};
        uint8_t* v_end{v_begin + sizeof(T)};
        while (v_begin < v_end) {
            *v_begin++ = EEPROM.read(l++);
        }
    }

    template <class T, class... Targs>
    void ParseInto(T& data, Targs&... data_args) {
        ParseInto(data);
        ParseInto(data_args...);
    }

   private:
    std::size_t l{0};
};

#endif /* EEPROMCURSOR_H */
