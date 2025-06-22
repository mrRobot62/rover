#ifndef _DYNAMIC_ARRAY_H
#define _DYNAMIC_ARRAY_H

#include <Arduino.h>
#include <vector>

template <typename T>
class DynamicArray
{
private:
    std::vector<T> list;

public:
    void push(T value)
    {
        list.push_back(value);
    }

    void add(T value)
    {
        list.push_back(value);
    }

    T *to_array()
    {
        if (list.empty())
            return nullptr;
        return list.data();
    }

    bool empty()
    {
        return list.empty();
    }

    int size()
    {
        return list.size();
    }

    void clear()
    {
        list.clear();
    }

    // iteration über das Array mit callback aufruf. Die Callbackfunktion hat einen zusätlichen Parameter der genutzt werden kann
    template <typename T1>
    void iterate(void (*func)(T, T1 *), T1 *obj)
    {
        for (const auto &item : list)
        {
            func(item, obj);
        }
    }

    void print()
    {
        for (const auto &item : list)
        {
            Serial.print(item);
            Serial.print(" ");
        }
        Serial.println();
    }

    // **Iterator-Methoden hinzufügen**
    typename std::vector<T>::iterator begin() { return list.begin(); }
    typename std::vector<T>::iterator end() { return list.end(); }
    typename std::vector<T>::const_iterator begin() const { return list.begin(); }
    typename std::vector<T>::const_iterator end() const { return list.end(); }
};

#endif