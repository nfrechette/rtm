# Handling asserts

This library uses a simple system to handle asserts. Asserts are fatal and must terminate otherwise the behavior is undefined if execution continues.

A total of 4 behaviors are supported:

*  We can print to `stderr` and `abort`
*  We can `throw` and exception
*  We can call a custom function
*  Do nothing and strip the check at compile time (**default behavior**)

Everything necessary is implemented in [**rtm/impl/error.h**](../includes/rtm/impl/error.h).

*Note: It is **NOT** possible to have different assert behaviors in two or more C++ object files. All the C++ files within your static or dynamic library that reference RTM must all use the same assert handling strategy.*

## Aborting

In order to enable the aborting behavior, simply define the macro `RTM_ON_ASSERT_ABORT`:

`#define RTM_ON_ASSERT_ABORT`

## Throwing

In order to enable the throwing behavior, simply define the macro `RTM_ON_ASSERT_THROW`:

`#define RTM_ON_ASSERT_THROW`

Note that the type of the exception thrown is `std::runtime_error`.

## Custom function

In order to enable the custom function calling behavior, define the macro `RTM_ON_ASSERT_CUSTOM` with the name of the function to call:

`#define RTM_ON_ASSERT_CUSTOM on_custom_assert_impl`

Note that the function signature is as follow: `void on_custom_assert_impl(const char* expression, int line, const char* file, const char* format, ...) {}`

You can also define your own assert implementation by defining the `RTM_ASSERT` macro as well:

```c++
#define RTM_ON_ASSERT_CUSTOM
#define RTM_ASSERT(expression, format, ...) checkf(expression, ANSI_TO_TCHAR(format), #__VA_ARGS__)
```

## No checks

By default if no macro mentioned above is defined, all asserts will be stripped at compile time.
