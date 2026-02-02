#ifndef RUNE_ERROR_HPP
#define RUNE_ERROR_HPP
#include <exception>
#include <string>

namespace rune
{

    class timeout_error : public std::exception
    {
    private:
        std::string message;

    public:
        // 构造函数接受一个字符串参数
        timeout_error(const std::string &msg) : message(msg) {}

        // 重写 what() 方法
        const char *what() const noexcept override
        {
            return message.c_str(); // 返回 C 风格字符串
        }
    };

    class timeout_error : public std::exception
    {
    private:
        std::string message;

    public:
        // 构造函数接受一个字符串参数
        timeout_error(const std::string &msg) : message(msg) {}

        // 重写 what() 方法
        const char *what() const noexcept override
        {
            return message.c_str(); // 返回 C 风格字符串
        }
    };

    class timeout_error : public std::exception
    {
    private:
        std::string message;

    public:
        // 构造函数接受一个字符串参数
        timeout_error(const std::string &msg) : message(msg) {}

        // 重写 what() 方法
        const char *what() const noexcept override
        {
            return message.c_str(); // 返回 C 风格字符串
        }
    };

    class timeout_error : public std::exception
    {
    private:
        std::string message;

    public:
        // 构造函数接受一个字符串参数
        timeout_error(const std::string &msg) : message(msg) {}

        // 重写 what() 方法
        const char *what() const noexcept override
        {
            return message.c_str(); // 返回 C 风格字符串
        }
    };
}

#endif