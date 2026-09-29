#ifndef LOGGER_H
#define LOGGER_H

#include <Print.h>

/**
 * @class Logger
 *
 * @brief Provides utility methods for logging information in a consistent format
 */
class Logger {
    public:
        /**
         * @brief Constructs a Logger that writes to a given output stream
         *
         * @param out Stream to print log messages to (e.g. `Serial`, `SerialBT`).
         *            Defaults to `Serial`
        */
        explicit Logger(Print& out = Serial) : _out(out) {}

        /**
         * @brief Prints an informational message
         *
         * @note Used for displaying purely information messages.
                 Prepends `msg` with "[ INFO ]" before writing to stream `_out`.
         *
         * @param msg Message to be printed
        */
        void info(const char* msg) {
            _out.print("[ INFO ] ");
            _out.println(msg);
        }
        /**
         * @brief Prints a warning message
         *
         * @note Used for displaying minimal warnings that may require user attention.
                 Prepends `msg` with "[ WARN ]" before writing to stream `_out`.
         *
         * @param msg Message to be printed
        */
        void warn(const char* msg) {
            _out.print("[ WARN ] ");
            _out.println(msg);
        }
         /**
         * @brief Prints an error message
         *
         * @note Used for displaying significant errors that require user attention.
                 Prepends `msg` with "[ ERROR ]" before writing to stream `_out`.
         *
         * @param msg Message to be printed
        */
        void error(const char* msg) {
            _out.print("[ ERROR ] ");
            _out.println(msg);
        }
         /**
         * @brief Prints a debug message
         *
         * @note Used for displaying values or indicators to assist in debugging.
                 Prepends `msg` with "[ DEBUG ]" before writing to stream `_out`.
         *
         * @param msg Message to be printed
        */
        void debug(const char* msg) {
            _out.print("[ DEBUG ] ");
            _out.println(msg);
        }

    private:
        Print& _out; // Stream this logger writes to
};

#endif
