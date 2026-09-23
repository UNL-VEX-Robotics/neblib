#pragma once

#include <cassert>
#include <new>
#include <utility>

namespace neblib::util
{
    template <typename T>
    class Optional
    {
    private:
        union Storage
        {
            T value;

            Storage() {}
            ~Storage() {}
        } storage;

        bool _hasValue;

    public:
        Optional() : _hasValue(false) {}

        Optional(const T &value) : _hasValue(false)
        {
            new (&storage.value) T(value);
            _hasValue = true;
        }

        ~Optional() { reset(); }

        template <typename... Args>
        T &emplace(Args &&...args)
        {
            reset();

            new (&storage.value) T(std::forward<Args>(args)...);
            _hasValue = true;

            return storage.value;
        }

        bool hasValue() const { return _hasValue; }

        T &value()
        {
            assert(_hasValue);
            return storage.value;
        }

        void reset()
        {
            if (_hasValue)
            {
                storage.value.~T();
                _hasValue = false;
            }
        }

        explicit operator bool() const noexcept { return _hasValue; }

        Optional(const Optional &) = delete;
        Optional &operator=(const Optional &) = delete;
    };
} // namespace neblib::util