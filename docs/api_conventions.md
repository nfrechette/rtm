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


## Quaternion basis

A quaternion stores the vector part in the **[xyz]** components and the scalar part in the **[w]** component. The library uses the axis and angle form: a rotation of `angle` around the unit vector `axis` is `[axis * sin(angle / 2), cos(angle / 2)]`.

### Rotation direction

A positive angle turns the first axis of each pair onto the second axis:

| Rotation axis | Component | Positive rotation |
| ------------- | --------- | ----------------- |
| X (forward)   | [x]       | Y onto Z (right onto up) |
| Y (right)     | [y]       | Z onto X (up onto forward) |
| Z (up)        | [z]       | X onto Y (forward onto right) |

This is the right-hand rule on the coordinates. Because the coordinate system is left-handed, a positive rotation is clockwise when you look from the tip of the axis toward the origin.

To examine the convention of a library, apply these three quaternions to the X, Y, and Z basis vectors (here `s = sqrt(0.5)`). The table shows the result with RTM.

| Quaternion `[x, y, z, w]` | X    | Y    | Z    |
| ------------------------- | ---- | ---- | ---- |
| `[s, 0, 0, s]`            | X    | Z    | -Y   |
| `[0, s, 0, s]`            | -Z   | Y    | X    |
| `[0, 0, s, s]`            | Y    | -X   | Z    |

### Bivector basis

Geometric algebra represents a rotation with a rotor: a scalar part and a bivector part. The bivector `e_ab` is the plane of the axes `a` and `b`. The rotation with the rotor `R` is `v' = R v ~R`. When the `e_ab` coefficient of `R` is positive, the rotation turns the axis `b` onto the axis `a`. For example, `e12` turns Y onto X.

With this rule, the quaternion components map to these bivectors:

* [x] = e32 (the Y-Z plane, Y onto Z)
* [y] = e13 (the Z-X plane, Z onto X)
* [z] = e21 (the X-Y plane, X onto Y)
* [w] = the scalar part

Some libraries use the basis e23, e31, and e12. Each bivector of that basis is the negative of the bivector that RTM uses. To convert a rotor in that basis to an RTM quaternion, negate [x], [y], and [z]. This gives the conjugate.

### Multiplication

The components obey the Hamilton rule: `i^2 = j^2 = k^2 = ijk = -1` and `ij = k`. Here `i`, `j`, and `k` are the units of the [x], [y], and [z] components.

The library uses row vectors, thus `quat_mul(lhs, rhs)` applies `lhs` first and `rhs` second: `local_to_world = quat_mul(local_to_object, object_to_world)`. With the Hamilton notation, `quat_mul(lhs, rhs)` returns the product `rhs * lhs`. For example, `quat_mul(i, j)` returns `-k` and `quat_mul(j, i)` returns `k`.

`quat_mul_vector3(v, q)` returns the Hamilton product `q * v * conjugate(q)`. `matrix_from_quat(q)` returns the same rotation as a matrix for row vectors: `matrix_mul_vector3(v, matrix_from_quat(q))` is equal to `quat_mul_vector3(v, q)`.

## Floating point contraction

A compiler can merge a multiply and an add into one FMA instruction. This is floating point contraction. The FMA does not round the product, thus the result can change with the compiler, the flags, and the target. For the same input, the scalar code path gives the same result on all the targets only when the compiler does not merge these operations.

* With MSVC, the headers set `float_control(precise, on)`. Do not use `/fp:fast` or `/fp:contract`.
* With Clang, the headers disable contraction on x86, x64, AArch64 (Clang 16 and later, Apple Clang 15 and later), and RISC-V (Clang 18 and later). With `-ffp-contract=fast`, LLVM Clang ignores the pragma after it inlines a function. Use `-ffp-contract=on`, `-ffp-contract=off`, or `-ffp-contract=fast-honor-pragmas`.
* With GCC, the headers do not disable contraction. GCC uses `-ffp-contract=fast` by default in C++, and it merges the operations of a function after it inlines the function in the code of the caller. A pragma in the headers does not stop this. Also, GCC does not inline a function when its optimization options are not the same as those of the caller: such a pragma makes the code slower. Compile with `-ffp-contract=off` to get the same results on all the targets.

The NEON code path uses FMA instructions on purpose. Thus its results can differ in the last bits from the other code paths.
