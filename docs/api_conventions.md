# API and other conventions

## C-style

C-style interfaces are generally preferred with a few supporting types. The simpler C-style interface leads to less intermediate code that the compiler needs to work with and it increases the likelihood that functions will be inlined. Inlining is a critical and integral part of performance for any math library. In general, code commonly used and that would benefit the most from being inlined uses a C-style API (vectors, quaternions, etc) while less critical code relies on various helper types (angles, helpers, etc).

## C++11

We restrict the code to use C++11 as it is the most widely used flavor on the platforms we currently support. *constexpr* in particular is used as much as possible. Despite the C-style API, everything lives under the *rtm* namespace and implementation details not meant to be consumed by clients is hidden inside the *rtm_impl* namespace as well as the *impl* header directory.

The library is 100% comprised of C++ headers and no linking is required. This makes for the easiest integration possible and it also gives us more freedom with what and when we can change things.

## Argument passing

This library supports many architectures, platforms, and compilers and sadly there is no consensus on how SIMD types should be passed by argument or returned by value. It is generally best to pass as many things by register, when possible, but usually only a certain number of registers can be used for it. Some platforms support aggregate types being passed by register (either by argument and/or by return value), others do not. To keep things as simple as possible, aliases are used for every type such as: *vector4f_arg0, vector4f_arg1, ..., vector4f_argn*.

RTM tries its best to do things optimally. Generally speaking, for code that isn't performance critical the difference will be very small and you are free to pass things by value or *const&* in your own code but using the argument aliases is encouraged.

## Matrix multiplication ordering

Whether you call it pre or post-multiplication, or left or right multiplication, it boils down to whether vectors are represented as rows or as columns. 

In the former case, multiplication with a matrix takes the form `v' = vM`  while in the later case we have `v' = Mv`.  Linear algebra typically treats vectors as *columns* and OpenGL opted to use that convention for [that reason](http://steve.hollasch.net/cgindex/math/matrix/column-vec.html). If you think of matrices as functions that modify an input and return an output it ends up reading like this: `result = object_to_world(local_to_object(input))`. This reads right-to-left as is common with nested function evaluation. Sadly, this somewhat breaks down when using multiplication operators: `result = object_to_world * local_to_object * input`. Regardless of how operator precedence works, the result will be identical but most modern programming languages (and western languages) read left-to-right.

On the other hand, DirectX uses *row* vectors and ends up with the much more natural: `result = input * local_to_object * object_to_world`. Your input is in local space, it gets transformed into object space before finally ending up in world space. Clean, clear, and readable. If you instead multiply the two matrices together on their own, you get the clear `local_to_world = local_to_object * object_to_world` instead of the awkward `local_to_world = object_to_world * local_to_object` you would get with OpenGL and *column* vectors.

An executive decision was made for all vectors to be row vectors for their improved readability.

## Coordinate frame

When relevant, a left-handed coordinate system is used:

* X axis = forward
* Y axis = right
* Z axis = up

