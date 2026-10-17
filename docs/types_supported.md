# Types supported

Realtime Math supports the most commonly used types in video games and realtime applications in a SIMD friendly way with a strong focus on performance.

All types come in various flavors with a corresponding suffix for their width (e.g. vector4f):

*  **f** for *float32*
*  **d** for *float64*
*  **s** for *int16*
*  **i** for *int32*
*  **q** for *int64*

## Scalar

Both regular *float & double* types are supported for most operations and a special SIMD friendly scalar type as well: *scalarf & scalard*. Different architectures have an easier or harder time working with scalar floating point numbers. For example, older PowerPC processors had to write to memory and reload from it to transfer from one register file into another (e.g convert from a float to a SIMD vector). Modern processors handle this much better but inefficiencies remain, especially with SSE. While it is free to convert a SIMD scalar into a float with *_mm_cvtss_f32(..)* the reverse generally requires the compiler to fill the unused SIMD lanes with known values (either zero or the same). This introduces an extra instruction that isn't always required when only the first lane is used such as with *scalar_sqrt_reciprocal(..)*. By introducing a type for SIMD scalar values, each platform is free to make an optimal choice.

## Vector 4D

Lower dimensions vectors will not be supported with proper types but functions can support them with an appropriate suffix (e.g. *vector_dot3(..)*). If no suffix is specified, functions operate on the full width of the vector. When a function operates on a reduced number of SIMD lanes, the extra unused lanes will have undefined values.

*Vectors are row vectors in RTM and thus multiply on the left of matrices.*

## Mask 4D

A comparison mask used by vector selection/blending. Each SIMD lane consists of all ones (true) or zeroes (false) depending on the condition.

## Quaternion

Quaternions are 4D complex numbers commonly used to represent 3D rotations (when normalized). The **[xyz]** components are the real part while the **[w]** component is the imaginary part.

## VQ (vector-quaternion)

A VQ represents an affine transform in two distinct parts: a rotation quaternion and a vector3 translation. This type is commonly used in video games as it is very fast to work with and more compact than a full affine matrix.

Before 2.4, the name of this type was QV. The new name follows the order of operations from right to left (rotation, translation), as in the literature.

## VQS (vector-quaternion-scalar)

A VQS represents an affine transform in three distinct parts: a rotation quaternion, a vector4 translation, and a scalar uniform scale. This type is commonly used in video games as it is very fast to work with and more compact than a full affine matrix. It represents uniform positive/negative scale through a single scalar value. When multiplying transforms of this type, scale combines as it would with matrices.

Before 2.4, the name of this type was QVS. The new name follows the order of operations from right to left (scale, rotation, translation), as in the literature.

## VQV (vector-quaternion-vector)

A VQV represents an affine transform in three distinct parts: a rotation quaternion, a vector3 translation, and a vector3 non-uniform scale. This type is commonly used in video games as it is very fast to work with and more compact than a full affine matrix. It properly handles positive non-uniform scaling but negative scaling is a bit more problematic. A best effort is made by converting the quaternion to a matrix when necessary. If scale fidelity is important, consider using VQM or an affine matrix 3x4 instead. When multiplying transforms of this type, scale combines as it would with matrices.

Before 2.4, the name of this type was QVV. The new name follows the order of operations from right to left (scale, rotation, translation), as VQS does in the literature.

## VQM (vector-quaternion-matrix)

A VQM represents an affine transform in three distinct parts: a rotation quaternion, a vector3 translation, and a 3x3 scale/shear matrix. It transforms a point `p` with row vectors as follows: `p' = p * M * R + t`. The scale/shear matrix `M` applies first, then the rotation `R`, then the translation `t`. The name follows the order of operations from right to left, as VQV does.

VQM keeps the advantages of VQV: the rotation stays a normalized quaternion, and the translation and the rotation interpolate separately. Unlike VQV, VQM forms a group: the multiplication and the inverse do not discard information, and the multiplication is associative. When a multiplication rotates a non-uniform scale, the result contains shear. VQV discards that shear, but VQM keeps it in `M`. The scale/shear matrix can also contain a negative scale and a reflection.

*  `vqm_from_matrix` finds the rotation with the Gram-Schmidt process. More than one VQM gives the same matrix: the function returns the one with a pure rotation, and `M` contains the reflection, if any.
*  `vqm_get_scale` returns the diagonal of `M`. `vqm_get_scale_shear` returns the full matrix.
*  `vqm_lerp` interpolates `M` with a linear interpolation. `vqm_slerp` interpolates the scale (the diagonal of `M`) with an exponential interpolation (`vector_eerp`) and the remaining shear with a linear interpolation.
*  `vqm_inverse` has a variant with a fallback scale for a scale/shear part that is not invertible.

A VQM uses four SIMD registers: 16 components, or 64 bytes for `vqmf`. When the scale/shear part is exactly the identity, the transform holds a flag. Then `vqm_mul`, `vqm_mul_point3`, `vqm_mul_vector3`, `vqm_inverse`, `vqm_lerp`, and `vqm_slerp` skip the scale/shear part and use only the rotation and the translation. With a scale/shear part, a VQM costs more than a VQV because the multiplication computes three 3x3 matrix products.

VQM is experimental: include `rtm/experimental/vqmf.h` or `rtm/experimental/vqmd.h`. The API can change before the type moves out of the experimental directory.

For the theory, see *The VQM-Group and its Applications* by Michael Aristidou and Xin Li.

## Matrix 3x3

A generic 3x3 matrix. Suitable to represent rotations mixed with 3D scale or anything else that might fit.

## Matrix 3x4

An 3x4 affine matrix represents a 3D rotation, 3D translation, and 3D scale. It properly deals with skew/shear when present but once scale with mirroring is combined, it cannot be safely extracted back. Affine matrices are 4x4 but have their last row always equal to **[0, 0, 0, 1]** which is why it is named 3x4.

## Matrix 4x4

A generic 4x4 matrix. Suitable to represent 3D projection matrices and the likes.

## Unaligned and storage friendly types

When manipulating vectors of various width, it is often desirable to store them as an unaligned sequence of floats with no padding. For example, while a 3D mesh has a number of `float3` vertices, storing and manipulating them as `vector4f` would use 33% more memory. To that end, a number of types are provided to help with this: `float2f, float2d, float3f, float3d, float4f, float4d`. These types have no alignment requirement beyond the natural float/double alignment. Functions such as `vector_load3(const float3f* input)` can load them from memory and return a vector4 of the correct type.
